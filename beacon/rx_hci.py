"""UNO Q iBeacon 수신기 — 블루투스 칩(HCI)을 직접 제어해서 광고를 하나도 거르지 않고 받는다.

rx_scan.py(bleak → BlueZ)는 칩의 "중복 광고 거르기"가 켜진 채로 스캔한다. 그래서 같은 비콘이
Linux 스캔 한 바퀴(약 10.24초)마다 한 번씩만 보고됐다 (2026-11-27 실측: 약 10초 간격).
이 스크립트는 칩에 직접
    - 수동 스캔(passive), 듣는 시간 100% (scan interval = window)
    - 중복 거르기 끔 (filter_duplicates = 0)
을 설정해서, ESP32가 0.1초마다 보내는 광고를 받은 그대로 출력한다.

실행 (UNO Q, App Lab 바깥, 칩을 직접 다뤄서 관리자 권한 필요):
    sudo python3 beacon/rx_hci.py
    sudo python3 beacon/rx_hci.py --stats          # 5초마다 초당 수신 횟수·최대 끊김을 함께 표시
    sudo python3 beacon/rx_hci.py | python3 beacon/rssi_relay.py

출력 형식은 rx_scan.py와 같다 (rssi_relay, peak_monitor, rssi_client가 그대로 읽는다):
    2026-11-27T16:07:22.666+09:00, 1, -72

주의
    - 같은 칩을 쓰는 다른 스캔(rx_scan.py, bluetoothctl scan on)은 꺼 두고 실행한다.
    - 칩이 블루투스 5 방식(확장 스캔)만 받으면 자동으로 그 방식으로 바꿔서 시도한다.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from datetime import datetime
from typing import Iterator, Optional

# esp32_tx.ino / rx_scan.py와 같은 값
BEACON_UUID = "7A1B2C3D-4E5F-6071-8293-A4B5C6D7E8F9"
EXPECTED_MINOR = 0
APPLE_IBEACON_PREFIX = b"\x4c\x00\x02\x15"   # 회사 ID 0x004C(리틀엔디언) + iBeacon 표시

# HCI 상수 (Bluetooth Core Spec Vol 4 Part E)
HCI_COMMAND_PKT = 0x01
HCI_EVENT_PKT = 0x04
EVT_CMD_COMPLETE = 0x0E
EVT_CMD_STATUS = 0x0F
EVT_LE_META = 0x3E
LE_ADV_REPORT = 0x02
LE_EXT_ADV_REPORT = 0x0D
OGF_LE = 0x08
OCF_SET_SCAN_PARAMS = 0x000B
OCF_SET_SCAN_ENABLE = 0x000C
OCF_SET_EXT_SCAN_PARAMS = 0x0041
OCF_SET_EXT_SCAN_ENABLE = 0x0042

SCAN_INTERVAL = 0x0010   # 16 × 0.625ms = 10ms
SCAN_WINDOW = 0x0010     # interval과 같게 → 쉬지 않고 계속 듣기


def opcode(ogf: int, ocf: int) -> int:
    return (ogf << 10) | ocf


def command_packet(ogf: int, ocf: int, params: bytes = b"") -> bytes:
    return struct.pack("<BHB", HCI_COMMAND_PKT, opcode(ogf, ocf), len(params)) + params


def hci_filter_bytes() -> bytes:
    """소켓이 받을 패킷 종류 (커널 struct hci_ufilter, 16바이트).

    이벤트 패킷 중 Command Complete/Status와 LE Meta만 받는다.
    C 구조체는 끝에 2바이트 빈칸이 있어서 16바이트다. 14바이트만 보내면 최신 커널이
    EINVAL(Invalid argument)로 거부한다 (2026-11-27 UNO Q에서 확인).
    """
    type_mask = 1 << HCI_EVENT_PKT
    ev_lo = (1 << EVT_CMD_COMPLETE) | (1 << EVT_CMD_STATUS)
    ev_hi = 1 << (EVT_LE_META - 32)
    return struct.pack("<IIIH2x", type_mask, ev_lo, ev_hi, 0)


# ── 광고 해석 (순수 함수, 칩 없이 테스트 가능) ─────────────────────────────

def parse_ibeacon(adv_data: bytes) -> Optional[tuple[str, int, int]]:
    """광고 데이터(AD 구조들)에서 iBeacon을 찾아 (UUID, major, minor)를 돌려준다."""
    i = 0
    while i < len(adv_data):
        length = adv_data[i]
        if length == 0 or i + 1 + length > len(adv_data):
            return None
        ad_type = adv_data[i + 1]
        body = adv_data[i + 2:i + 1 + length]
        if ad_type == 0xFF and body[:4] == APPLE_IBEACON_PREFIX and len(body) >= 25:
            raw = body[4:20].hex().upper()
            uuid = f"{raw[0:8]}-{raw[8:12]}-{raw[12:16]}-{raw[16:20]}-{raw[20:32]}"
            major = int.from_bytes(body[20:22], "big")
            minor = int.from_bytes(body[22:24], "big")
            return uuid, major, minor
        i += 1 + length
    return None


def iter_adv_reports(event: bytes) -> Iterator[tuple[bytes, int]]:
    """HCI 이벤트 패킷(맨 앞 0x04 포함)에서 (광고 데이터, RSSI)를 꺼낸다.

    리눅스 커널과 같은 방식으로 보고서를 앞에서부터 차례로 읽는다.
    """
    if len(event) < 4 or event[0] != HCI_EVENT_PKT or event[1] != EVT_LE_META:
        return
    sub = event[3]
    p = event[4:]
    if not p:
        return
    count, p = p[0], p[1:]
    for _ in range(count):
        if sub == LE_ADV_REPORT:
            # event_type(1) addr_type(1) addr(6) data_len(1) data(n) rssi(1)
            if len(p) < 9:
                return
            n = p[8]
            if len(p) < 9 + n + 1:
                return
            data = p[9:9 + n]
            rssi = struct.unpack("b", p[9 + n:10 + n])[0]
            p = p[10 + n:]
        elif sub == LE_EXT_ADV_REPORT:
            # event_type(2) addr_type(1) addr(6) phy1(1) phy2(1) sid(1) tx(1) rssi(1)
            # periodic_interval(2) direct_type(1) direct_addr(6) data_len(1) data(n)
            if len(p) < 24:
                return
            rssi = struct.unpack("b", p[13:14])[0]
            n = p[23]
            if len(p) < 24 + n:
                return
            data = p[24:24 + n]
            p = p[24 + n:]
        else:
            return
        if rssi == 127:   # 127 = RSSI 없음
            continue
        yield data, rssi


def beacon_lines(event: bytes, now: Optional[datetime] = None) -> list[str]:
    """이벤트 하나에서 우리 비콘만 골라 rx_scan.py와 같은 형식의 줄로 만든다."""
    lines = []
    for data, rssi in iter_adv_reports(event):
        beacon = parse_ibeacon(data)
        if beacon is None:
            continue
        uuid, slot_id, minor = beacon
        if uuid != BEACON_UUID or minor != EXPECTED_MINOR or not 0 <= slot_id <= 4:
            continue
        t = (now or datetime.now().astimezone()).isoformat(timespec="milliseconds")
        lines.append(f"{t}, {slot_id}, {rssi}")
    return lines


class GapStats:
    """초당 수신 횟수와 가장 긴 끊김 (--stats용)."""

    def __init__(self) -> None:
        self.reset(time.monotonic())

    def reset(self, now: float) -> None:
        self.started = now
        self.count = 0
        self.last: Optional[float] = None
        self.max_gap = 0.0

    def add(self, now: float) -> None:
        if self.last is not None:
            self.max_gap = max(self.max_gap, now - self.last)
        self.last = now
        self.count += 1

    def summary(self, now: float) -> str:
        span = max(now - self.started, 1e-6)
        return f"[stats] {span:.0f}초 동안 {self.count}회 (초당 {self.count / span:.1f}), 가장 긴 끊김 {self.max_gap:.2f}초"


# ── 칩 제어 ────────────────────────────────────────────────────────────

class HciScanner:
    def __init__(self, dev_id: int = 0) -> None:
        self.dev_id = dev_id
        self.sock: Optional[socket.socket] = None
        self.extended = False

    def open(self) -> None:
        steps = "소켓 만들기"
        try:
            s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
            steps = f"hci{self.dev_id}에 연결"
            s.bind((self.dev_id,))
            steps = "수신 필터 설정"
            sol_hci = getattr(socket, "SOL_HCI", 0)
            hci_filter = getattr(socket, "HCI_FILTER", 2)
            s.setsockopt(sol_hci, hci_filter, hci_filter_bytes())
        except OSError as exc:
            raise OSError(exc.errno, f"{steps} 단계에서 실패: {exc.strerror}") from exc
        s.settimeout(1.0)
        self.sock = s

    def _command(self, ocf: int, params: bytes = b"", timeout: float = 2.0) -> int:
        """명령을 보내고 그 명령의 결과 상태값(0 = 성공)을 돌려준다."""
        assert self.sock is not None
        self.sock.send(command_packet(OGF_LE, ocf, params))
        want = opcode(OGF_LE, ocf)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                pkt = self.sock.recv(260)
            except socket.timeout:
                continue
            if len(pkt) >= 7 and pkt[1] == EVT_CMD_COMPLETE:
                if struct.unpack("<H", pkt[4:6])[0] == want:
                    return pkt[6]
            elif len(pkt) >= 7 and pkt[1] == EVT_CMD_STATUS:
                if struct.unpack("<H", pkt[5:7])[0] == want:
                    return pkt[3]
        raise TimeoutError(f"HCI 명령 0x{want:04X} 응답 없음")

    def _set_enable(self, enable: bool) -> int:
        if self.extended:
            return self._command(OCF_SET_EXT_SCAN_ENABLE, struct.pack("<BBHH", int(enable), 0, 0, 0))
        return self._command(OCF_SET_SCAN_ENABLE, struct.pack("<BB", int(enable), 0))

    def start(self) -> None:
        # 1) 옛 방식(legacy) 시도: 혹시 켜져 있는 스캔을 끄고, 설정하고, 켠다
        self.extended = False
        self._set_enable(False)
        status = self._command(
            OCF_SET_SCAN_PARAMS, struct.pack("<BHHBB", 0x00, SCAN_INTERVAL, SCAN_WINDOW, 0x00, 0x00)
        )
        if status == 0:
            status = self._set_enable(True)
            if status == 0:
                return
        # 2) 칩이 블루투스 5 확장 스캔만 받는 경우 (상태값 0x01 / 0x0C 등)
        self.extended = True
        self._set_enable(False)
        params = struct.pack("<BBB", 0x00, 0x00, 0x01) + struct.pack(
            "<BHH", 0x00, SCAN_INTERVAL, SCAN_WINDOW
        )
        status = self._command(OCF_SET_EXT_SCAN_PARAMS, params)
        if status != 0:
            raise RuntimeError(f"스캔 설정 실패 (상태값 0x{status:02X})")
        status = self._set_enable(True)
        if status != 0:
            raise RuntimeError(
                f"스캔 시작 실패 (상태값 0x{status:02X}). 다른 스캔(rx_scan.py 등)을 끄고 다시 실행"
            )

    def events(self) -> Iterator[bytes]:
        assert self.sock is not None
        while True:
            try:
                yield self.sock.recv(260)
            except socket.timeout:
                yield b""

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self._set_enable(False)
        except Exception:
            pass
        self.sock.close()
        self.sock = None


def main() -> None:
    ap = argparse.ArgumentParser(description="UNO Q iBeacon 수신기 (HCI 직접 스캔, 중복 거르기 끔)")
    ap.add_argument("--dev", type=int, default=0, help="블루투스 장치 번호 (hci0 → 0)")
    ap.add_argument("--stats", action="store_true", help="5초마다 수신 횟수·최대 끊김을 표시 (stderr)")
    args = ap.parse_args()

    scanner = HciScanner(args.dev)
    try:
        scanner.open()
        scanner.start()
    except PermissionError:
        sys.exit("권한 없음: sudo python3 beacon/rx_hci.py 로 실행하세요")
    except (OSError, RuntimeError, TimeoutError) as exc:
        scanner.close()
        sys.exit(f"스캔 시작 실패: {exc}")

    mode = "확장(블루투스 5)" if scanner.extended else "기본"
    print("Starting BLE iBeacon Scanner (HCI direct, no duplicate filter)...")
    print(f"Target UUID: {BEACON_UUID}")
    print(f"Expected Minor: {EXPECTED_MINOR}")
    print(f"Scan mode: {mode}, passive, interval=window=10ms")
    print("Output: t_recv, slot_id, rssi")
    print("Press Ctrl+C to stop.\n", flush=True)

    stats = GapStats()
    last_report = time.monotonic()
    try:
        for event in scanner.events():
            for line in beacon_lines(event) if event else []:
                print(line, flush=True)
                stats.add(time.monotonic())
            if args.stats and time.monotonic() - last_report >= 5.0:
                now = time.monotonic()
                print(stats.summary(now), file=sys.stderr, flush=True)
                stats.reset(now)
                last_report = now
    except KeyboardInterrupt:
        print("\nBLE Scanner stopped.")
    finally:
        scanner.close()


if __name__ == "__main__":
    main()
