"""rx_scan.py(#21) 출력 ↔ 비콘 탐색 쪽 파서 호환 테스트

rx_scan.py가 찍는 줄을 중계기(rssi_client)와 확인 도구(peak_monitor)가 읽는다.
rx_scan.py의 출력 형식이 바뀌면 이 테스트가 실패해서 알려 준다.
(형식이 어긋나면 수신기가 모든 줄을 무시해서, 로버가 신호를 못 받은 것처럼 출발하지 않는다)

실제 블루투스 없이, 가짜 iBeacon 패킷을 rx_scan.detection_callback에 넣어서 확인한다.

실행 (레포 최상위 폴더에서):
    python -m pytest beacon/test_rx_scan_format.py
"""

import contextlib
import io
import sys
import types
import unittest
import uuid
from datetime import datetime
from pathlib import Path

try:
    import bleak  # noqa: F401
except ImportError:  # 노트북에 bleak이 없어도 콜백만 테스트할 수 있게
    sys.modules["bleak"] = types.SimpleNamespace(BleakScanner=object)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "qrover_drive" / "python"))

from beacon import rx_scan  # noqa: E402
from beacon.peak_monitor import parse_line as monitor_parse  # noqa: E402
from rssi_client import parse_line as client_parse  # noqa: E402

APPLE = 0x004C


def ibeacon(u=rx_scan.BEACON_UUID, major=1, minor=0, power=-59, prefix=b"\x02\x15"):
    """iBeacon 제조사 데이터 (Apple 회사 ID 뒤 부분, bleak이 주는 형태)."""
    return (prefix + uuid.UUID(u).bytes + major.to_bytes(2, "big")
            + minor.to_bytes(2, "big") + (power & 0xFF).to_bytes(1, "big"))


class Adv:
    def __init__(self, data, rssi=-61):
        self.manufacturer_data = {APPLE: data} if data is not None else {}
        self.rssi = rssi


def run_callback(adv):
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        rx_scan.detection_callback(None, adv)
    return buf.getvalue().strip()


class RxScanFilterTest(unittest.TestCase):
    def test_accepts_team_beacon(self):
        for slot in range(5):
            self.assertTrue(run_callback(Adv(ibeacon(major=slot))))

    def test_rejects_others(self):
        cases = {
            "다른 UUID": Adv(ibeacon(u="11111111-2222-3333-4444-555555555555")),
            "칸 번호 5": Adv(ibeacon(major=5)),
            "minor 1": Adv(ibeacon(minor=1)),
            "iBeacon 아님": Adv(ibeacon(prefix=b"\x10\x05")),
            "Apple 데이터 없음": Adv(None),
            "길이 다름": Adv(ibeacon()[:-1]),
        }
        for name, adv in cases.items():
            with self.subTest(name):
                self.assertEqual(run_callback(adv), "")


class FormatCompatibilityTest(unittest.TestCase):
    def test_relay_client_reads_rx_scan_line(self):
        line = run_callback(Adv(ibeacon(major=3), rssi=-67))
        measured_text = line.split(",", 1)[0]
        self.assertIsNotNone(datetime.fromisoformat(measured_text).utcoffset())
        sample = client_parse(line)
        self.assertEqual((sample.slot_id, sample.rssi), (3, -67.0))
        self.assertIsInstance(sample.t, float)

    def test_peak_monitor_reads_rx_scan_line(self):
        line = run_callback(Adv(ibeacon(major=2), rssi=-55))
        parsed = monitor_parse(line)
        self.assertIsNotNone(parsed)
        _, slot, rssi = parsed
        self.assertEqual((slot, rssi), (2, -55.0))

    def test_startup_messages_ignored(self):
        for msg in ["Starting BLE iBeacon Scanner...",
                    f"Target UUID: {rx_scan.BEACON_UUID}",
                    "Output: t_recv, slot_id, rssi",
                    "Duplicate advertisements: ENABLED"]:
            with self.subTest(msg):
                self.assertIsNone(client_parse(msg))
                self.assertIsNone(monitor_parse(msg))


if __name__ == "__main__":
    unittest.main()
