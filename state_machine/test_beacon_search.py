"""주행 흐름 1단계(비콘 최댓값 감지 + 복귀) 테스트 — 가상 로버로 확인한다.

가상 로버: 속도 = PWM × 0.1 cm/s, 통로를 따라 x축으로만 움직인다.
비콘: x = 0, 통로에서 30cm 옆. RSSI = -59 - 20·log10(거리/1m) + 잡음, 10Hz, 10% 누락.

실행 (레포 최상위 폴더에서):
    python -m pytest state_machine/test_beacon_search.py
"""

import math
import random
import unittest

from beacon.peak_detector import PeakConfig, PeakDetector
from state_machine.beacon_search import BeaconSearch, SearchConfig, SearchPhase

CM_PER_PWM = 0.1      # 가상 로버: PWM 100 → 10 cm/s
LATERAL_CM = 30.0     # 비콘이 통로에서 떨어진 거리


def rssi_at(x_cm, rng, noise=2.0):
    d_m = math.hypot(x_cm, LATERAL_CM) / 100.0
    return -59.0 - 20.0 * math.log10(max(d_m, 0.05)) + rng.gauss(0, noise)


def simulate(search, start_x=-120.0, seed=1, dt=0.05, max_t=60.0, noise=2.0, beacon_on=True):
    """search를 가상 로버에 연결해서 끝날 때까지 돌린다. (마지막 위치, 마지막 출력) 반환."""
    rng = random.Random(seed)
    x, t = start_x, 0.0
    search.start(t)
    next_rssi = 0.0
    out = search.step(t)
    while t < max_t and out.phase not in (SearchPhase.DONE, SearchPhase.ABORT):
        if beacon_on and t >= next_rssi:
            next_rssi += 0.1
            if rng.random() > 0.1:
                search.on_rssi(t, rssi_at(x, rng, noise))
        out = search.step(t)
        x += (out.left + out.right) / 2 * CM_PER_PWM * dt
        t += dt
    return x, out


class PeakDetectorTest(unittest.TestCase):
    def feed_pass(self, det, seed=3):
        """10cm/s로 -100cm → +100cm 지나가기. 실제 최댓값은 t=10s."""
        rng = random.Random(seed)
        first = None
        for i in range(200):
            t = i * 0.1
            if det.update(t, rssi_at(-100 + 10 * t, rng)) and first is None:
                first = t
        return first

    def test_not_passed_while_approaching(self):
        det = PeakDetector()
        rng = random.Random(0)
        for i in range(90):  # t=0~8.9s, 아직 비콘 앞에 도착 전
            det.update(i * 0.1, rssi_at(-100 + i, rng))
        self.assertFalse(det.passed)

    def test_passes_after_beacon_and_peak_time_close(self):
        det = PeakDetector()
        passed_at = self.feed_pass(det)
        self.assertIsNotNone(passed_at)
        self.assertGreater(passed_at, 10.0)              # 지나간 뒤에야 판단
        self.assertLess(abs(det.peak_time() - 10.0), 1.0)  # 최댓값 시각 오차 1초 이내

    def test_short_dip_does_not_trigger(self):
        # "확인 시간" 장치 확인용: 평균 1초·6dB 창에서 0.3초 하락은 무시돼야 한다
        det = PeakDetector(PeakConfig(window_sec=1.0, drop_db=6.0, confirm_sec=0.6, min_samples=3))
        t = 0.0
        for _ in range(10):
            det.update(t, -50); t += 0.1
        for _ in range(3):   # 0.3초만 뚝 떨어졌다가
            det.update(t, -70); t += 0.1
        for _ in range(10):  # 다시 회복
            det.update(t, -50); t += 0.1
        self.assertFalse(det.passed)

    def test_default_ignores_brief_block_at_real_rate(self):
        # 2026-11-27 기본값 + 실제 수신 속도(초당 5회): 사람이 잠깐 가려 1개가 -15dB 떨어져도 무시
        det = PeakDetector()
        t = 0.0
        for _ in range(15):
            det.update(t, -72); t += 0.2
        det.update(t, -87); t += 0.2
        for _ in range(15):
            det.update(t, -72); t += 0.2
        self.assertFalse(det.passed)

    def test_default_detects_measured_pass(self):
        # 실측 모양: 조금 전 -78 → 칸 앞 -72 → 지나서 -78 (초당 5회)
        det = PeakDetector()
        t, passed_at = 0.0, None
        for r in [-78] * 10 + [-75] * 5 + [-72] * 10 + [-75] * 5 + [-78] * 15:
            if det.update(t, r) and passed_at is None:
                passed_at = t
            t += 0.2
        self.assertIsNotNone(passed_at)
        self.assertLess(abs(det.peak_time() - 4.0), 1.0)   # 칸 앞(-72) 구간 3.0~4.8초

    def test_weak_signal_never_passes(self):
        det = PeakDetector(PeakConfig(min_peak_rssi=-80))
        t = 0.0
        for r in [-90] * 10 + [-97] * 20:
            det.update(t, r); t += 0.1
        self.assertFalse(det.passed)

    def test_signal_lost(self):
        det = PeakDetector(PeakConfig(hold_sec=1.0))
        self.assertFalse(det.signal_lost(100.0))  # 아직 한 번도 못 받음
        det.update(0.0, -60)
        self.assertFalse(det.signal_lost(0.9))
        self.assertTrue(det.signal_lost(1.1))


class BeaconSearchTest(unittest.TestCase):
    def test_returns_near_beacon(self):
        for seed in range(5):
            x, out = simulate(BeaconSearch(), seed=seed)
            self.assertEqual(out.phase, SearchPhase.DONE, out.reason)
            self.assertLess(abs(x), 15.0, f"seed={seed}: 최종 위치 {x:.1f}cm")

    def test_noisy_rssi_stays_within_one_slot(self):
        # 잡음 ±6dB: 칸 간격(29cm)보다 크게 틀리면 안 됨 (정확한 칸은 다음 단계 ArUco가 맞춤)
        for seed in range(10):
            x, out = simulate(BeaconSearch(), seed=seed, noise=6.0)
            self.assertEqual(out.phase, SearchPhase.DONE, out.reason)
            self.assertLess(abs(x), 29.0, f"seed={seed}: 최종 위치 {x:.1f}cm")

    def test_returns_near_beacon_without_slowdown(self):
        x, out = simulate(BeaconSearch(config=SearchConfig(approach_rssi=None)), seed=2)
        self.assertEqual(out.phase, SearchPhase.DONE, out.reason)
        self.assertLess(abs(x), 15.0)

    def test_slows_down_near_beacon(self):
        search = BeaconSearch()
        search.start(0.0)
        for i in range(10):
            search.on_rssi(i * 0.1, -65)   # 접근 기준(-70)보다 셈
        out = search.step(1.0)
        self.assertEqual(out.phase, SearchPhase.APPROACH)
        self.assertEqual(out.left, SearchConfig().approach_pwm)

    def test_return_drives_backward(self):
        search = BeaconSearch()
        x, t, rng = -120.0, 0.0, random.Random(4)
        search.start(t)
        phases = []
        while t < 40:
            if round(t * 100) % 10 == 0:
                search.on_rssi(t, rssi_at(x, rng))
            out = search.step(t)
            phases.append((out.phase, out.left, out.right))
            x += (out.left + out.right) / 2 * CM_PER_PWM * 0.05
            t += 0.05
            if out.phase == SearchPhase.DONE:
                break
        returning = [p for p in phases if p[0] == SearchPhase.RETURN]
        self.assertTrue(returning)
        self.assertTrue(all(l < 0 and r < 0 for _, l, r in returning))

    def test_abort_when_no_beacon(self):
        _, out = simulate(BeaconSearch(config=SearchConfig(search_timeout=5.0)), beacon_on=False)
        self.assertEqual(out.phase, SearchPhase.ABORT)
        self.assertIn("시간 초과", out.reason)

    def test_abort_when_signal_lost(self):
        search = BeaconSearch()
        search.start(0.0)
        search.on_rssi(0.0, -80)
        out = search.step(2.0)   # hold_sec(1초) 넘게 신호 없음
        self.assertEqual(out.phase, SearchPhase.ABORT)
        self.assertIn("끊김", out.reason)

    def test_stopped_after_done(self):
        x, out = simulate(BeaconSearch(), seed=0)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_trim_applied(self):
        search = BeaconSearch(config=SearchConfig(trim_left=3, trim_right=-2, approach_rssi=None))
        search.start(0.0)
        search.on_rssi(0.0, -85)
        out = search.step(0.05)
        self.assertEqual((out.left, out.right), (103, 98))


if __name__ == "__main__":
    unittest.main()
