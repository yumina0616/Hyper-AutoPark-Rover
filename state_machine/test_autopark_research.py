"""목표 ID 마커를 못 찾았을 때 재탐색 테스트.

왼쪽으로 marker_search_sec 동안 돌아도 목표 ID가 안 보이면
통로 방향으로 되돌아 회전 → 칸 단위로 앞/뒤 이동 → 다시 왼쪽으로 돌며 찾는다.
    python -m pytest state_machine/test_autopark_research.py -q
"""

import unittest
from dataclasses import dataclass

from state_machine.autopark import (
    SAFETY_PASS,
    AutoParkController,
    AutoParkPhase,
    SafetyObservation,
)
from state_machine.front_parking import DistanceObservation, RANGE_OK
from vision.aruco_auto_align import MarkerFrame

TARGET = 1


@dataclass(frozen=True)
class FakeMarker:
    id: int
    err_x: float
    size_px: float
    t: float


def step(ctrl, t, ids=(), seq=None):
    markers = [FakeMarker(i, 0.3, 80.0, t) for i in ids]
    frame = MarkerFrame(seq=seq if seq is not None else int(t * 100), t=t, markers=markers,
                        width_px=640)
    return ctrl.step(
        t,
        marker_frame=frame,
        distance=DistanceObservation(400, RANGE_OK, t),
        safety=SafetyObservation(SAFETY_PASS, t),
    )


def aligning(order=()):
    ctrl = AutoParkController(target_slot=TARGET)
    ctrl.cfg.marker_order = tuple(order)
    ctrl.phase = AutoParkPhase.ARUCO_ALIGN
    ctrl.align_stage.reset(10.0)
    return ctrl


def search_until_research(ctrl, ids=()):
    t = 10.0
    out = step(ctrl, t, ids)
    while out.phase == AutoParkPhase.ARUCO_ALIGN and t < 40:
        t += 0.1
        out = step(ctrl, t, ids)
    return t, out


class ResearchTest(unittest.TestCase):
    def test_searches_left_first(self):
        ctrl = aligning()
        out = step(ctrl, 10.0)
        p = ctrl.cfg.marker_search_pwm
        self.assertEqual((out.left, out.right), (-p, p))

    def test_not_found_turns_back_moves_forward_and_searches_again(self):
        ctrl = aligning()
        t, out = search_until_research(ctrl)
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        self.assertGreaterEqual(t - 10.0, ctrl.cfg.marker_search_sec)   # 정해진 시간만큼 돈 뒤
        p = ctrl.cfg.marker_search_pwm
        self.assertEqual((out.left, out.right), (p, -p))                 # 통로 방향(오른쪽)으로 되돌아 회전
        self.assertIn("목표 ID 1 못 찾음", out.reason)

        back_turn_end = t + ctrl.cfg.marker_search_sec
        move = step(ctrl, back_turn_end + 0.05)
        v = ctrl.cfg.realign_pwm
        self.assertEqual((move.left, move.right), (v, v))                # 앞으로 한 칸
        self.assertIn("앞으로 1칸", move.detail)

        move_sec = ctrl.cfg.slot_pitch_mm / ctrl.cfg.realign_mm_per_sec
        again = step(ctrl, back_turn_end + move_sec + 0.05)
        self.assertEqual(again.phase, AutoParkPhase.ARUCO_ALIGN)         # 다시 찾기
        self.assertEqual(ctrl.research_attempts, 1)

    def test_seen_marker_before_target_moves_forward_by_slot_difference(self):
        # 진행 방향 순서 (3, 4, 2, 0, 1) 에서 0을 봤다면 목표 1은 한 칸 앞
        ctrl = aligning(order=(3, 4, 2, 0, 1))
        search_until_research(ctrl, ids=(0,))
        self.assertIn("본 마커 [0]", ctrl.reason)
        self.assertIn("앞으로 1칸", ctrl._realign_plan[-1][3])

    def test_seen_marker_after_target_moves_backward(self):
        ctrl = aligning(order=(3, 4, 2, 1, 0))
        out = step(ctrl, 10.0, ids=(0,))
        self.assertEqual(ctrl.research_slots(), -1)
        _, out = search_until_research(ctrl, ids=(0,))
        last = ctrl._realign_plan[-1]
        v = ctrl.cfg.realign_pwm
        self.assertEqual((last[1], last[2]), (-v, -v))
        self.assertIn("뒤로 1칸", last[3])

    def test_unknown_order_defaults_to_one_slot_forward(self):
        ctrl = aligning()
        step(ctrl, 10.0, ids=(3,))
        self.assertEqual(ctrl.research_slots(), 1)

    def test_gives_up_after_max_attempts(self):
        ctrl = aligning()
        ctrl.cfg.max_research_attempts = 1
        search_until_research(ctrl)
        ctrl.phase = AutoParkPhase.ARUCO_ALIGN            # 이동을 마치고 다시 찾는다고 가정
        ctrl.align_stage.reset(50.0)
        t = 50.0
        out = step(ctrl, t)
        while out.phase == AutoParkPhase.ARUCO_ALIGN and t < 80:
            t += 0.1
            out = step(ctrl, t)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("못 찾음", out.reason)

    def test_target_found_does_not_research(self):
        ctrl = aligning()
        out = step(ctrl, 10.0, ids=(TARGET,))
        self.assertEqual(out.phase, AutoParkPhase.ARUCO_ALIGN)
        self.assertNotEqual(out.detail, "MARKER_MISSING")


if __name__ == "__main__":
    unittest.main()
