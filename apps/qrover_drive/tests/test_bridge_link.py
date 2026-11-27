"""bridge_link.py 테스트 — 진짜 Bridge 없이 가짜 notify와 가짜 시계로 확인한다.

실행 (레포 최상위 폴더에서):
    python -m pytest apps/qrover_drive/tests/test_bridge_link.py
"""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from bridge_link import (  # noqa: E402
    RANGE_ERROR,
    RANGE_OK,
    SAFETY_EMERGENCY,
    SAFETY_PASS,
    BridgeLink,
    LinkConfig,
)


class FakeClock:
    def __init__(self):
        self.now = 1000.0

    def __call__(self):
        return self.now


class FakeBridge:
    def __init__(self):
        self.sent = []

    def notify(self, name, *args):
        self.sent.append((name, *args))


def make(start_delay=0.0):
    clock = FakeClock()
    bridge = FakeBridge()
    link = BridgeLink(bridge.notify, clock, LinkConfig(start_delay_sec=start_delay))
    return link, bridge, clock


class StartDelayTest(unittest.TestCase):
    def test_sends_stop_during_start_delay(self):
        link, bridge, clock = make(start_delay=3.0)
        link.send_drive(100, 100)
        link.tick()
        self.assertEqual(bridge.sent[-1], ("drive", 0, 0))

    def test_sends_command_after_start_delay(self):
        link, bridge, clock = make(start_delay=3.0)
        link.send_drive(100, 100)
        link.tick()
        clock.now += 3.0
        link.tick()
        self.assertEqual(bridge.sent[-1], ("drive", 100, 100))


class SendTest(unittest.TestCase):
    def test_first_tick_sends(self):
        link, bridge, _ = make()
        self.assertEqual(link.tick(), (0, 0))
        self.assertEqual(bridge.sent, [("drive", 0, 0)])

    def test_change_sends_immediately(self):
        link, bridge, clock = make()
        link.tick()
        clock.now += 0.05
        link.send_drive(80, -80)
        self.assertEqual(link.tick(), (80, -80))

    def test_same_value_not_resent_before_heartbeat(self):
        link, bridge, clock = make()
        link.tick()
        clock.now += 0.1
        self.assertIsNone(link.tick())
        self.assertEqual(len(bridge.sent), 1)

    def test_heartbeat_resends_same_value(self):
        link, bridge, clock = make()
        link.send_drive(50, 50)
        link.tick()
        clock.now += 0.31  # 0.3을 더하면 소수점 오차로 0.2999…가 되어 경계에 안 걸림
        self.assertEqual(link.tick(), (50, 50))
        self.assertEqual(len(bridge.sent), 2)

    def test_heartbeat_keeps_watchdog_alive_for_long_run(self):
        # 3초 동안 20Hz로 tick → 보낸 간격이 MCU watchdog(1초)를 넘지 않아야 함
        link, bridge, clock = make()
        link.send_drive(60, 60)
        times = []
        for _ in range(60):
            if link.tick() is not None:
                times.append(clock.now)
            clock.now += 0.05
        gaps = [b - a for a, b in zip(times, times[1:])]
        self.assertLess(max(gaps), 1.0)

    def test_clamps_to_pwm_range(self):
        link, bridge, _ = make()
        link.send_drive(999, -999)
        link.tick()
        self.assertEqual(bridge.sent[-1], ("drive", 255, -255))


class DistanceTest(unittest.TestCase):
    def test_none_before_first_value(self):
        link, _, _ = make()
        self.assertIsNone(link.latest_distance())

    def test_fresh_value(self):
        link, _, clock = make()
        link.on_distance(250, RANGE_OK)
        d = link.latest_distance()
        self.assertEqual((d.dist_mm, d.status, d.t, d.stale), (250, RANGE_OK, clock.now, False))

    def test_stale_after_stale_sec(self):
        link, _, clock = make()
        link.on_distance(250, RANGE_OK)
        clock.now += link.cfg.stale_sec + 0.01
        self.assertTrue(link.latest_distance().stale)

    def test_keeps_status_code(self):
        link, _, _ = make()
        link.on_distance(-1, RANGE_ERROR)
        self.assertEqual(link.latest_distance().status, RANGE_ERROR)


class SafetyFeedbackTest(unittest.TestCase):
    def test_none_before_first_safety_feedback(self):
        link, _, _ = make()
        self.assertIsNone(link.latest_safety())

    def test_fresh_safety_feedback(self):
        link, _, clock = make()
        link.on_safety(SAFETY_PASS)
        reading = link.latest_safety()
        self.assertEqual((reading.reason, reading.t, reading.stale), (SAFETY_PASS, clock.now, False))

    def test_emergency_feedback_becomes_stale(self):
        link, _, clock = make()
        link.on_safety(SAFETY_EMERGENCY)
        clock.now += link.cfg.stale_sec + 0.01
        reading = link.latest_safety()
        self.assertEqual(reading.reason, SAFETY_EMERGENCY)
        self.assertTrue(reading.stale)


if __name__ == "__main__":
    unittest.main()
