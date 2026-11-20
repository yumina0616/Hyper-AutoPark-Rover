"""RSSI 중계기 — App Lab 바깥 → App Lab 앱 안으로 RSSI를 넘기는 통로

왜 필요한가:
    App Lab 앱(Python) 안에서는 블루투스를 쓸 수 없어서, 비콘 수신(rx_scan.py, #21)은
    App Lab 바깥(UNO Q Linux)에서 돈다. 주행 판단은 App Lab 앱 안에서 하므로
    수신한 RSSI를 앱 안으로 넘겨 줄 통로가 필요하다.

동작:
    rx_scan.py 출력을 한 줄씩 받아서, TCP로 접속한 앱(rssi_client.py)에 그대로 보낸다.
    로버를 움직이지 않는다. 화면에도 그대로 출력한다.

실행 (UNO Q에서, App Lab 앱보다 먼저):
    python3 beacon/rx_scan.py | python3 beacon/rssi_relay.py

옵션:
    --port 8765   접속받을 포트 (앱의 rssi_client.py와 같아야 함)
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading

DEFAULT_PORT = 8765


class Relay:
    def __init__(self, port: int, host: str = "0.0.0.0") -> None:
        # 0.0.0.0: App Lab 컨테이너가 보드 내부 가상 네트워크로 접속할 수 있게 모든 주소에서 받는다
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((host, port))
        self.server.listen()
        self.port = self.server.getsockname()[1]
        self.clients: list[socket.socket] = []
        self.lock = threading.Lock()
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self) -> None:
        while True:
            try:
                conn, addr = self.server.accept()
            except OSError:
                return  # 서버를 닫았음
            with self.lock:
                self.clients.append(conn)
            print(f"[relay] 앱 접속: {addr[0]}", file=sys.stderr, flush=True)

    def send_line(self, line: str) -> None:
        data = (line.rstrip("\n") + "\n").encode("utf-8")
        with self.lock:
            alive = []
            for c in self.clients:
                try:
                    c.sendall(data)
                    alive.append(c)
                except OSError:
                    c.close()  # 앱이 끊김 → 목록에서 뺌
            self.clients = alive

    def close(self) -> None:
        self.server.close()
        with self.lock:
            for c in self.clients:
                c.close()
            self.clients = []


def main() -> None:
    p = argparse.ArgumentParser(description="rx_scan.py 출력을 App Lab 앱으로 중계")
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = p.parse_args()

    relay = Relay(args.port)
    print(f"[relay] 포트 {relay.port}에서 앱 접속 대기. rx_scan.py 출력을 기다리는 중...",
          file=sys.stderr, flush=True)
    try:
        for line in sys.stdin:
            print(line, end="", flush=True)  # 화면에도 그대로
            relay.send_line(line)
    except KeyboardInterrupt:
        pass
    finally:
        relay.close()


if __name__ == "__main__":
    main()
