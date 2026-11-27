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
    ctrl.cfg.align_mode = "sweep"   # 돌며 찾기 방식 테스트
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
    def test_searches_left_in_short_pulses(self):
        ctrl = aligning()
        p = ctrl.cfg.search_pulse_pwm
        on, off = ctrl.cfg.search_pulse_on_sec, ctrl.cfg.search_pulse_off_sec
        out = step(ctrl, 10.0)
        self.assertEqual((out.left, out.right), (-p, p))                 # 짧게 왼쪽으로 돌고
        out = step(ctrl, 10.0 + on + 0.05)
        self.assertEqual((out.left, out.right), (0, 0))                  # 멈춰서 보기
        out = step(ctrl, 10.0 + on + off + 0.01)
        self.assertEqual((out.left, out.right), (-p, p))                 # 다시 짧게
        self.assertEqual(ctrl.search_pulses, 2)

    def test_not_found_turns_back_same_pulses_moves_forward_and_searches_again(self):
        ctrl = aligning()
        cfg = ctrl.cfg
        t, out = search_until_research(ctrl)
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        p = cfg.search_pulse_pwm
        self.assertEqual((out.left, out.right), (p, -p))                 # 통로 방향(오른쪽)으로 되돌아 회전
        self.assertIn("목표 ID 1 못 찾음", out.reason)

        # 탐색에서 돈 횟수와 같은 횟수만큼, 같은 세기·시간으로 반대로 돈다
        turn_back = [seg for seg in ctrl._realign_plan if seg[1] != 0 and seg[1] == -seg[2]]
        self.assertEqual(len(turn_back), cfg.search_max_pulses)
        self.assertTrue(all(seg[1] == p for seg in turn_back))

        back_turn_end = ctrl._realign_plan[-2][0]                       # 원상복구가 끝나는 시각
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

    def test_ignores_ids_outside_slots(self):
        # 배경에서 잘못 잡힌 17번 같은 번호는 재탐색 판단에 쓰지 않는다
        ctrl = aligning(order=(3, 4, 2, 0, 1))
        step(ctrl, 10.0, ids=(17, 0))
        self.assertEqual(ctrl._seen_other_ids, [0])

    def test_uses_nearest_marker_not_last_seen(self):
        # 실주행: 4, 3, 2, 1 순서로 봤지만 가장 크게 보인 건 3번 → 목표 0은 한 칸 앞
        ctrl = AutoParkController(target_slot=0)
        ctrl.cfg.align_mode = "sweep"
        ctrl.phase = AutoParkPhase.ARUCO_ALIGN
        ctrl.align_stage.reset(10.0)
        for t, mid, size in [(10.0, 4, 70.0), (11.0, 3, 120.0), (12.0, 2, 60.0), (13.0, 1, 40.0)]:
            frame = MarkerFrame(seq=int(t * 100), t=t, markers=[FakeMarker(mid, -0.5, size, t)],
                                width_px=640)
            ctrl.step(t, marker_frame=frame, distance=DistanceObservation(400, RANGE_OK, t),
                      safety=SafetyObservation(SAFETY_PASS, t))
        self.assertEqual(ctrl._seen_other_ids[-1], 1)
        self.assertEqual(ctrl.research_slots(), 1)      # 3번 기준 (1번 기준이면 3칸)

    def test_unknown_order_defaults_to_one_slot_forward(self):
        ctrl = aligning()
        ctrl.cfg.marker_order = ()
        step(ctrl, 10.0, ids=(3,))
        self.assertEqual(ctrl.research_slots(), 1)

    def test_default_test_site_order_target_0(self):
        # 시험장 배치 1, 2, 3, 0, 4 · 목표 0번
        ctrl = AutoParkController(target_slot=0)
        ctrl.phase = AutoParkPhase.ARUCO_ALIGN
        ctrl.align_stage.reset(10.0)
        for seen, slots in [(1, 3), (2, 2), (3, 1), (4, -1)]:
            ctrl._seen_other_ids = [seen]
            self.assertEqual(ctrl.research_slots(), slots, f"본 마커 {seen}")

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

    def test_centering_turns_in_pulses_big_then_small(self):
        ctrl = aligning()
        cfg, p = ctrl.cfg, ctrl.cfg.search_pulse_pwm

        def target(t, err):
            frame = MarkerFrame(seq=int(t * 100), t=t, markers=[FakeMarker(TARGET, err, 80.0, t)],
                                width_px=640)
            return ctrl.step(t, marker_frame=frame, distance=DistanceObservation(400, RANGE_OK, t),
                             safety=SafetyObservation(SAFETY_PASS, t))

        out = target(10.0, 0.6)                                          # 오른쪽으로 많이 벗어남
        self.assertEqual((out.left, out.right), (p, -p))
        self.assertEqual(ctrl._pulse_log[-1], (1, cfg.search_pulse_on_sec))
        out = target(10.0 + cfg.search_pulse_on_sec + 0.05, 0.6)
        self.assertEqual((out.left, out.right), (0, 0))                  # 멈춰서 보기
        t2 = 10.0 + cfg.search_pulse_on_sec + cfg.search_pulse_off_sec + 0.01
        out = target(t2, -0.1)                                           # 거의 가운데, 살짝 왼쪽
        self.assertEqual((out.left, out.right), (-p, p))
        self.assertEqual(ctrl._pulse_log[-1], (-1, cfg.align_fine_on_sec))

    def test_lost_after_seen_researches_and_replays_turns_in_reverse(self):
        ctrl = aligning()
        cfg, p = ctrl.cfg, ctrl.cfg.search_pulse_pwm
        ctrl._pulse_owner = ctrl.align_stage.started_at
        ctrl._pulse_log = [(-1, 0.1), (-1, 0.1), (1, 0.05)]             # 왼쪽 2번, 오른쪽 조금 1번
        ctrl.align_stage.has_seen_target = True
        out = ctrl._start_research(20.0)
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        turns = [(s[1], s[2]) for s in ctrl._realign_plan if s[1] == -s[2] and s[1] != 0]
        self.assertEqual(turns, [(-p, p), (p, -p), (p, -p)])            # 거꾸로: 왼쪽 조금, 오른쪽, 오른쪽

    def test_marker_missing_after_reacquire_fail_researches(self):
        ctrl = aligning()
        ctrl._pulse_owner = ctrl.align_stage.started_at
        ctrl.align_stage.has_seen_target = True
        ctrl.align_stage._reacquire_started_at = 10.0
        out = step(ctrl, 10.0 + ctrl.align_stage.cfg.lost_stop_sec + ctrl.align_stage.cfg.reacquire_sec + 0.1)
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)

    def test_beacon_return_gain_lengthens_return(self):
        ctrl = AutoParkController(target_slot=0)
        self.assertEqual(ctrl.beacon_search.cfg.return_gain, ctrl.cfg.beacon_return_gain)
        self.assertEqual(ctrl.beacon_search.cfg.search_pwm, ctrl.cfg.beacon_search_pwm)

    def test_target_found_does_not_research(self):
        ctrl = aligning()
        out = step(ctrl, 10.0, ids=(TARGET,))
        self.assertEqual(out.phase, AutoParkPhase.ARUCO_ALIGN)
        self.assertNotEqual(out.detail, "MARKER_MISSING")


if __name__ == "__main__":
    unittest.main()
