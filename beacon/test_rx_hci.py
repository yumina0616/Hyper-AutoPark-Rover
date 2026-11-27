"""beacon/rx_hci.py 테스트: 칩이 보내는 HCI 이벤트 바이트를 직접 만들어서 해석을 확인한다.

광고 내용은 esp32_tx.ino가 보내는 것과 같다: Flags(02 01 06) + 제조사 데이터(1A FF 4C 00 02 15 ...).
    python -m pytest beacon/test_rx_hci.py -q
"""

import struct
import unittest
from datetime import datetime, timedelta, timezone

from beacon.peak_monitor import parse_line as monitor_parse
from beacon.rx_hci import (
    BEACON_UUID,
    GapStats,
    LE_ADV_REPORT,
    LE_EXT_ADV_REPORT,
    beacon_lines,
    command_packet,
    hci_filter_bytes,
    iter_adv_reports,
    parse_ibeacon,
)

KST = timezone(timedelta(hours=9))
NOW = datetime(2026, 11, 27, 16, 7, 22, 666000, tzinfo=KST)


def esp32_adv(major=1, minor=0, uuid=BEACON_UUID, power=-59):
    uuid_bytes = bytes.fromhex(uuid.replace("-", ""))
    mfg = b"\x4c\x00\x02\x15" + uuid_bytes + major.to_bytes(2, "big") + minor.to_bytes(2, "big")
    mfg += struct.pack("b", power)
    return b"\x02\x01\x06" + bytes([len(mfg) + 1, 0xFF]) + mfg


def legacy_event(reports):
    """reports: [(adv_data, rssi)] → LE Advertising Report 이벤트 (커널과 같은 순서 배치)."""
    body = bytes([LE_ADV_REPORT, len(reports)])
    for data, rssi in reports:
        body += bytes([0x03, 0x00]) + b"\x11\x22\x33\x44\x55\x66" + bytes([len(data)]) + data
        body += struct.pack("b", rssi)
    return bytes([0x04, 0x3E, len(body)]) + body


def extended_event(data, rssi):
    body = bytes([LE_EXT_ADV_REPORT, 1])
    body += struct.pack("<H", 0x0010) + bytes([0x00]) + b"\x11\x22\x33\x44\x55\x66"
    body += bytes([0x01, 0x00, 0xFF, 0x7F]) + struct.pack("b", rssi)
    body += struct.pack("<H", 0) + bytes([0x00]) + b"\x00" * 6 + bytes([len(data)]) + data
    return bytes([0x04, 0x3E, len(body)]) + body


class ParseTest(unittest.TestCase):
    def test_parse_esp32_ibeacon(self):
        self.assertEqual(parse_ibeacon(esp32_adv(major=3, minor=0)), (BEACON_UUID, 3, 0))

    def test_non_ibeacon_and_broken_data(self):
        self.assertIsNone(parse_ibeacon(b"\x02\x01\x06"))
        self.assertIsNone(parse_ibeacon(b"\x1a\xff\x4c\x00"))   # 길이가 모자람
        self.assertIsNone(parse_ibeacon(b""))

    def test_legacy_report_rssi(self):
        reports = list(iter_adv_reports(legacy_event([(esp32_adv(), -65)])))
        self.assertEqual(len(reports), 1)
        self.assertEqual(reports[0][1], -65)

    def test_multiple_reports_in_one_event(self):
        ev = legacy_event([(esp32_adv(major=1), -60), (esp32_adv(major=2), -70)])
        lines = beacon_lines(ev, NOW)
        self.assertEqual([line.split(", ")[1:] for line in lines], [["1", "-60"], ["2", "-70"]])

    def test_extended_report(self):
        lines = beacon_lines(extended_event(esp32_adv(major=4), -58), NOW)
        self.assertEqual(lines, ["2026-11-27T16:07:22.666+09:00, 4, -58"])

    def test_filters_other_beacons(self):
        other_uuid = "11111111-2222-3333-4444-555555555555"
        ev = legacy_event([
            (esp32_adv(uuid=other_uuid), -50),   # 다른 UUID
            (esp32_adv(minor=1), -50),           # MINOR 1 (사본 버전)
            (esp32_adv(major=7), -50),           # 칸 번호 범위 밖
            (b"\x02\x01\x06", -50),              # iBeacon 아님
            (esp32_adv(major=0), 127),           # RSSI 없음
        ])
        self.assertEqual(beacon_lines(ev, NOW), [])

    def test_ignores_other_events_and_truncated(self):
        self.assertEqual(list(iter_adv_reports(b"\x04\x0e\x04\x01\x0b\x20\x00")), [])
        self.assertEqual(list(iter_adv_reports(legacy_event([(esp32_adv(), -60)])[:12])), [])

    def test_output_readable_by_existing_tools(self):
        line = beacon_lines(legacy_event([(esp32_adv(major=2), -61)]), NOW)[0]
        t, slot, rssi = monitor_parse(line)
        self.assertEqual((slot, rssi), (2, -61.0))
        self.assertEqual(t, NOW.timestamp())


class CommandTest(unittest.TestCase):
    def test_filter_matches_kernel_struct_size(self):
        # 커널 struct hci_ufilter는 16바이트 (14바이트면 EINVAL)
        import ctypes

        class HciUfilter(ctypes.Structure):
            _fields_ = [
                ("type_mask", ctypes.c_uint32),
                ("event_mask", ctypes.c_uint32 * 2),
                ("opcode", ctypes.c_uint16),
            ]

        self.assertEqual(len(hci_filter_bytes()), ctypes.sizeof(HciUfilter))
        self.assertEqual(hci_filter_bytes()[:4], (1 << 0x04).to_bytes(4, "little"))

    def test_scan_enable_packet_turns_off_duplicate_filter(self):
        # LE Set Scan Enable: opcode 0x200C, 파라미터 (enable=1, filter_duplicates=0)
        self.assertEqual(command_packet(0x08, 0x000C, b"\x01\x00"), b"\x01\x0c\x20\x02\x01\x00")


class StatsTest(unittest.TestCase):
    def test_gap_stats(self):
        s = GapStats()
        s.reset(0.0)
        for t in (0.1, 0.2, 0.3, 2.3, 2.4):
            s.add(t)
        self.assertEqual(s.count, 5)
        self.assertAlmostEqual(s.max_gap, 2.0)
        self.assertIn("초당 2.0", s.summary(2.5))


if __name__ == "__main__":
    unittest.main()
