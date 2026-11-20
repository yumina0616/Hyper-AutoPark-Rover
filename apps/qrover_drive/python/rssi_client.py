"""RSSI 수신기 (App Lab 앱 안) — beacon/rssi_relay.py에 접속해서 RSSI를 받는다

    client = RssiClient()
    for slot_id, rssi in client.poll():   # 주기 루프에서 매번 호출 (기다리지 않음)
        ...

접속 주소를 모르므로 차례로 시도한다:
    1) 환경 변수 RSSI_RELAY_HOST (지정했으면)
    2) 127.0.0.1               (App Lab 앱이 보드와 네트워크를 같이 쓰는 경우)
    3) 컨테이너의 게이트웨이     (App Lab 앱이 별도 가상 네트워크인 경우, 보통 보드 쪽 주소)
끊기면 RETRY_SEC마다 다시 접속한다. 받은 시각은 쓰지 않고, 앱이 받은 순간의 time.time()을 쓴다.
"""

from __future__ import annotations

import os
import socket
import struct
import time
from typing import List, Optional, Tuple

DEFAULT_PORT = 8765
RETRY_SEC = 2.0


def default_gateway() -> Optional[str]:
    """Linux의 /proc/net/route에서 기본 게이트웨이 주소를 읽는다. 없으면 None."""
    try:
        with open("/proc/net/route") as f:
            next(f)
            for line in f:
                fields = line.split()
                if fields[1] == "00000000" and int(fields[3], 16) & 2:
                    return socket.inet_ntoa(struct.pack("<L", int(fields[2], 16)))
    except (OSError, StopIteration, ValueError, IndexError):
        pass
    return None


def parse_line(line: str) -> Optional[Tuple[int, float]]:
    """rx_scan.py 한 줄 `ISO시각, 칸번호, RSSI` → (칸, rssi). 형식이 아니면 None."""
    parts = [p.strip() for p in line.split(",")]
    if len(parts) != 3:
        return None
    try:
        return int(parts[1]), float(parts[2])
    except ValueError:
        return None


class RssiClient:
    def __init__(self, port: int = DEFAULT_PORT, hosts: Optional[List[str]] = None) -> None:
        self.port = port
        if hosts is None:
            hosts = [h for h in (os.environ.get("RSSI_RELAY_HOST"), "127.0.0.1", default_gateway()) if h]
        self.hosts = list(dict.fromkeys(hosts))  # 중복 제거, 순서 유지
        self.sock: Optional[socket.socket] = None
        self.connected_host: Optional[str] = None
        self._buf = b""
        self._next_try = 0.0

    def _connect(self) -> None:
        now = time.time()
        if now < self._next_try:
            return
        self._next_try = now + RETRY_SEC
        for host in self.hosts:
            try:
                s = socket.create_connection((host, self.port), timeout=0.3)
                s.setblocking(False)
                self.sock, self.connected_host, self._buf = s, host, b""
                print(f"[rssi] 중계기 접속: {host}:{self.port}", flush=True)
                return
            except OSError:
                continue

    def poll(self) -> List[Tuple[int, float]]:
        """지금까지 도착한 RSSI를 모두 꺼낸다. 접속 전이거나 없으면 빈 리스트."""
        if self.sock is None:
            self._connect()
            if self.sock is None:
                return []
        try:
            while True:
                chunk = self.sock.recv(4096)
                if not chunk:  # 중계기가 끊김
                    self._disconnect()
                    break
                self._buf += chunk
        except BlockingIOError:
            pass  # 지금 더 받을 게 없음
        except OSError:
            self._disconnect()

        out = []
        *lines, self._buf = self._buf.split(b"\n")
        for raw in lines:
            parsed = parse_line(raw.decode("utf-8", errors="ignore"))
            if parsed is not None:
                out.append(parsed)
        return out

    def _disconnect(self) -> None:
        if self.sock is not None:
            self.sock.close()
        print("[rssi] 중계기 연결 끊김, 다시 접속 시도", flush=True)
        self.sock, self.connected_host = None, None
