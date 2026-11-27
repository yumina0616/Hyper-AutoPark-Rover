"""비콘 탐색 모드 연결 테스트 — 중계기·수신기는 노트북에서 실제 TCP로, 로버는 가상으로.

    rx_scan 형식 줄 → rssi_relay(TCP) → rssi_client → BeaconMode(BeaconSearch) → BridgeLink → (가짜 Bridge)

실행 (레포 최상위 폴더에서):
    python -m pytest apps/qrover_drive/tests/test_beacon_mode.py
"""

import random
import sys
import time
import unittest
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "apps" / "qrover_drive" / "python"))

from beacon.rssi_relay import Relay  # noqa: E402
from beacon_mode import BeaconMode  # noqa: E402
from bridge_link import BridgeLink, LinkConfig  # noqa: E402
from rssi_client import RssiClient, parse_line  # noqa: E402
from state_machine.beacon_search import BeaconSearch, SearchPhase  # noqa: E402
from state_machine.test_beacon_search import CM_PER_PWM, rssi_at  # noqa: E402


def rx_line(slot, rssi, t=None):
    """rx_scan.py와 같은 형식의 한 줄."""
    measured = datetime.now().astimezone() if t is None else datetime.fromtimestamp(t).astimezone()
    return f"{measured.isoformat(timespec='milliseconds')}, {slot}, {rssi}"


def poll_until(client, n, timeout=2.0):
    got, end = [], time.time() + timeout
    while len(got) < n and time.time() < end:
        got += client.poll()
        time.sleep(0.01)
    return got


class RelayClientTest(unittest.TestCase):
    def test_parse_line(self):
        sample = parse_line("2026-11-16T18:30:01.125, 1, -58")
        self.assertEqual((sample.slot_id, sample.rssi), (1, -58.0))
        self.assertEqual(sample.t, datetime.fromisoformat("2026-11-16T18:30:01.125").timestamp())
        self.assertIsNone(parse_line("Starting BLE iBeacon Scanner..."))
        self.assertIsNone(parse_line("a, b, c"))

    def test_timezone_offsets_represent_same_measurement_time(self):
        kst = parse_line("2026-11-16T18:30:01.125+09:00, 1, -58")
        utc = parse_line("2026-11-16T09:30:01.125+00:00, 1, -58")
        self.assertEqual(kst.t, utc.t)

    def test_relay_to_client_over_tcp(self):
        relay = Relay(port=0, host="127.0.0.1")
        client = None
        try:
            client = RssiClient(port=relay.port, hosts=["127.0.0.1"])
            client.poll()                      # 접속
            time.sleep(0.1)                    # relay가 접속을 받을 시간
            relay.send_line("Starting BLE iBeacon Scanner...")   # 안내 문구는 무시돼야 함
            relay.send_line(rx_line(2, -61))
            relay.send_line(rx_line(0, -75))
            samples = poll_until(client, 2)
            self.assertEqual([(s.slot_id, s.rssi) for s in samples], [(2, -61.0), (0, -75.0)])
        finally:
            if client is not None:
                client.close()
            relay.close()

    def test_client_without_relay_returns_empty(self):
        client = RssiClient(port=1, hosts=["127.0.0.1"])  # 아무도 안 듣는 포트
        self.assertEqual(client.poll(), [])


class FakeRssi:
    def __init__(self):
        self.queue = []

    def poll(self):
        out, self.queue = self.queue, []
        return out


class BeaconModeTest(unittest.TestCase):
    def test_waits_for_target_signal(self):
        rssi = FakeRssi()
        mode = BeaconMode(BeaconSearch(), rssi.poll, target_slot=3, log=lambda m: None)
        self.assertEqual(mode.step(0.0), (0, 0))
        rssi.queue = [(1, -60)]                  # 다른 칸 신호는 무시
        self.assertEqual(mode.step(0.1), (0, 0))
        self.assertFalse(mode.started)
        rssi.queue = [(3, -85)]
        left, right = mode.step(0.2)
        self.assertTrue(mode.started)
        self.assertGreater(left, 0)

    def test_does_not_start_during_start_delay(self):
        rssi = FakeRssi()
        mode = BeaconMode(BeaconSearch(), rssi.poll, target_slot=0, log=lambda m: None)
        rssi.queue = [(0, -80)]
        self.assertEqual(mode.step(0.0, allowed=False), (0, 0))
        self.assertFalse(mode.started)


class EndToEndTest(unittest.TestCase):
    def test_virtual_rover_returns_to_beacon(self):
        """중계기 TCP → 수신기 → 비콘 탐색 → BridgeLink까지 실제로 잇고, 가상 로버를 움직인다."""
        relay = Relay(port=0, host="127.0.0.1")
        sent = []
        client = None
        try:
            client = RssiClient(port=relay.port, hosts=["127.0.0.1"])
            client.poll()
            time.sleep(0.1)
            mode = BeaconMode(BeaconSearch(), client.poll, target_slot=2, log=lambda m: None)
            fake_t = [1000.0]
            link = BridgeLink(lambda name, l, r: sent.append((l, r)), clock=lambda: fake_t[0],
                              config=LinkConfig(start_delay_sec=0.0))

            rng = random.Random(5)
            x, dt = -120.0, 0.05
            epoch_base = time.time()
            for i in range(int(40 / dt)):
                t = i * dt
                if i % 2 == 0:  # 10Hz로 비콘 신호 (관계없는 칸 신호도 섞음)
                    relay.send_line(rx_line(2, round(rssi_at(x, rng)), epoch_base + t))
                    relay.send_line(rx_line(4, -70, epoch_base + t))
                time.sleep(0.001)  # TCP로 도착할 시간
                left, right = mode.step(epoch_base + t)
                link.send_drive(left, right)
                link.tick()
                fake_t[0] += dt
                x += (left + right) / 2 * CM_PER_PWM * dt
                if mode.finished:
                    break

            self.assertEqual(mode.search.phase, SearchPhase.DONE, mode.search.reason)
            self.assertLess(abs(x), 29.0, f"최종 위치 {x:.1f}cm")
            self.assertTrue(any(l < 0 and r < 0 for l, r in sent))   # 되돌아가는 명령이 나갔음
            self.assertEqual(sent[-1], (0, 0))                       # 마지막은 정지
        finally:
            if client is not None:
                client.close()
            relay.close()


if __name__ == "__main__":
    unittest.main()
