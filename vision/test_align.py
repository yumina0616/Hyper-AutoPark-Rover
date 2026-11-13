"""정렬 모듈(align.py) 테스트.

카메라나 로버 없이, 가짜 Marker 값을 넣어서 계산이 맞는지 확인한다.

실행 방법 (레포 최상위 폴더에서):
    python -m pytest vision/test_align.py
pytest가 없으면:
    cd vision
    python -m unittest test_align
"""

import unittest
from dataclasses import dataclass

from align import AlignConfig, Aligner, AlignStatus


@dataclass
class FakeMarker:
    """인식 모듈의 Marker 대신 쓰는 가짜 마커."""

    id: int
    err_x: float
    t: float


NOW = 1000.0   # 테스트용 "지금 시각"
TARGET = 3     # 목표 마커 번호


def marker(err_x, id=TARGET, age=0.0):
    """age초 전에 본 마커를 만든다."""
    return FakeMarker(id=id, err_x=err_x, t=NOW - age)


class LostTest(unittest.TestCase):
    """마커를 믿을 수 없으면 정지(0, 0)를 제안해야 한다."""

    def setUp(self):
        self.a = Aligner()

    def test_no_marker(self):
        r = self.a.step(None, TARGET, NOW)
        self.assertEqual(r.status, AlignStatus.LOST)
        self.assertEqual((r.left, r.right), (0, 0))

    def test_wrong_id(self):
        r = self.a.step(marker(0.3, id=2), TARGET, NOW)
        self.assertEqual(r.status, AlignStatus.LOST)
        self.assertEqual((r.left, r.right), (0, 0))

    def test_stale_marker(self):
        r = self.a.step(marker(0.3, age=1.0), TARGET, NOW)  # hold_sec(0.5)보다 오래됨
        self.assertEqual(r.status, AlignStatus.LOST)
        self.assertEqual((r.left, r.right), (0, 0))

    def test_lost_resets_aligned_count(self):
        cfg = AlignConfig(align_frames=2)
        a = Aligner(cfg)
        a.step(marker(0.0), TARGET, NOW)        # 1프레임 OK
        a.step(None, TARGET, NOW)               # 놓침 → 카운트 초기화
        r = a.step(marker(0.0), TARGET, NOW)    # 다시 1프레임 OK
        self.assertEqual(r.status, AlignStatus.ADJUSTING)


class InPlaceAlignTest(unittest.TestCase):
    """제자리 정렬: 두 바퀴를 반대로 돌려 방향만 맞춘다."""

    def setUp(self):
        self.a = Aligner()

    def test_marker_on_right_turns_clockwise(self):
        r = self.a.step(marker(0.5), TARGET, NOW)
        self.assertEqual(r.status, AlignStatus.ADJUSTING)
        self.assertGreater(r.left, 0)    # 왼쪽 바퀴 앞으로
        self.assertLess(r.right, 0)      # 오른쪽 바퀴 뒤로 → 시계 방향
        self.assertEqual(r.left, -r.right)

    def test_marker_on_left_turns_counterclockwise(self):
        r = self.a.step(marker(-0.5), TARGET, NOW)
        self.assertLess(r.left, 0)
        self.assertGreater(r.right, 0)

    def test_small_turn_raised_to_min_pwm(self):
        # kp=20, err=0.5 → 10인데, 최소 회전 PWM(60)보다 작으니 60으로 올려야 함
        r = self.a.step(marker(0.5), TARGET, NOW)
        self.assertEqual(r.left, AlignConfig().min_turn_pwm)

    def test_turn_limited_by_max_turn(self):
        cfg = AlignConfig(kp=1000, min_turn_pwm=0)
        r = Aligner(cfg).step(marker(1.0), TARGET, NOW)
        self.assertEqual(r.turn, cfg.max_turn)

    def test_deadband_gives_zero(self):
        r = self.a.step(marker(0.01), TARGET, NOW)   # align_tol(0.05)보다 작음
        self.assertEqual((r.left, r.right), (0, 0))
        self.assertEqual(r.status, AlignStatus.ADJUSTING)  # 아직 1프레임뿐

    def test_aligned_after_n_frames(self):
        cfg = AlignConfig(align_frames=3)
        a = Aligner(cfg)
        statuses = [a.step(marker(0.0), TARGET, NOW).status for _ in range(3)]
        self.assertEqual(statuses[:2], [AlignStatus.ADJUSTING] * 2)
        self.assertEqual(statuses[2], AlignStatus.ALIGNED)

    def test_off_center_frame_resets_count(self):
        cfg = AlignConfig(align_frames=2)
        a = Aligner(cfg)
        a.step(marker(0.0), TARGET, NOW)
        a.step(marker(0.5), TARGET, NOW)        # 벗어남 → 초기화
        r = a.step(marker(0.0), TARGET, NOW)
        self.assertEqual(r.status, AlignStatus.ADJUSTING)

    def test_reset(self):
        cfg = AlignConfig(align_frames=1)
        a = Aligner(cfg)
        self.assertEqual(a.step(marker(0.0), TARGET, NOW).status, AlignStatus.ALIGNED)
        a.reset()
        cfg.align_frames = 2
        self.assertEqual(a.step(marker(0.0), TARGET, NOW).status, AlignStatus.ADJUSTING)


class ReverseAlignTest(unittest.TestCase):
    """후진 정렬: 뒤로 가면서 좌우 속도 차이로 방향을 맞춘다."""

    def setUp(self):
        self.a = Aligner()

    def test_centered_goes_straight_back(self):
        r = self.a.step(marker(0.0), TARGET, NOW, mode="reverse")
        base = -AlignConfig().base_rev_pwm
        self.assertEqual((r.left, r.right), (base, base))

    def test_marker_on_right_turns_clockwise_while_reversing(self):
        r = self.a.step(marker(0.5), TARGET, NOW, mode="reverse")
        self.assertLess(r.left, 0)           # 둘 다 뒤로
        self.assertLess(r.right, 0)
        self.assertGreater(r.left, r.right)  # left - right > 0 → 시계 방향

    def test_reverse_keeps_moving_when_aligned(self):
        cfg = AlignConfig(align_frames=1)
        r = Aligner(cfg).step(marker(0.0), TARGET, NOW, mode="reverse")
        self.assertEqual(r.status, AlignStatus.ALIGNED)
        self.assertLess(r.left, 0)           # 정렬돼도 후진은 계속 (정지는 거리 센서 몫)

    def test_reverse_lost_stops(self):
        r = self.a.step(None, TARGET, NOW, mode="reverse")
        self.assertEqual((r.left, r.right), (0, 0))


class ScaleAndSignTest(unittest.TestCase):
    """err_x 범위(-100~100)와 부호 설정이 반영되는지."""

    def test_full_scale_100(self):
        a1 = Aligner(AlignConfig(min_turn_pwm=0))
        a100 = Aligner(AlignConfig(min_turn_pwm=0, err_x_full_scale=100))
        r1 = a1.step(marker(0.5), TARGET, NOW)
        r100 = a100.step(marker(50), TARGET, NOW)
        self.assertEqual((r1.left, r1.right), (r100.left, r100.right))

    def test_steer_sign_flips_direction(self):
        r = Aligner(AlignConfig(steer_sign=-1)).step(marker(0.5), TARGET, NOW)
        self.assertLess(r.left, 0)
        self.assertGreater(r.right, 0)

    def test_bad_mode(self):
        with self.assertRaises(ValueError):
            Aligner().step(marker(0.0), TARGET, NOW, mode="fly")


if __name__ == "__main__":
    unittest.main()
