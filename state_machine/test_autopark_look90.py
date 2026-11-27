"""90° 보기 정렬(look90) 테스트.

벽 쪽으로 딱 90° 돌고 → 멈춰서 보고 → 벽 기준으로 방향을 바로잡고
→ 목표 마커가 옆으로 벗어난 만큼 통로를 따라 앞/뒤로 이동하는지 확인한다.
실제 카메라처럼 꼭짓점이 있는 5cm 마커를 합성한다 (test_autopark_realign.marker_at).
    python -m pytest state_machine/test_autopark_look90.py -q
"""

import unittest
from types import SimpleNamespace

try:
    import cv2  # noqa: F401
except ImportError:
    cv2 = None

from state_machine.autopark import (
    SAFETY_PASS,
    AutoParkController,
    AutoParkPhase,
    SafetyObservation,
)
from state_machine.front_parking import DistanceObservation, RANGE_OK
from vision.aruco_auto_align import MarkerFrame
from vision.pose import Pose

if cv2 is not None:
    from state_machine.test_autopark_realign import marker_at

W, H = 640, 480


def look_ctrl():
    ctrl = AutoParkController(target_slot=0)      # 시험장: 목표 0번, 순서 1,2,3,0,4
    ctrl.phase = AutoParkPhase.ARUCO_ALIGN
    ctrl.align_stage.reset(10.0)
    ctrl._enter_look("LOOK")
    return ctrl


def step(ctrl, t, markers=()):
    frame = MarkerFrame(seq=int(t * 100), t=t, markers=list(markers), width_px=W, height_px=H)
    return ctrl.step(t, marker_frame=frame, distance=DistanceObservation(400, RANGE_OK, t),
                     safety=SafetyObservation(SAFETY_PASS, t))


def look(ctrl, make_markers, t0=10.0):
    """look_sec 동안 프레임을 넣고, 판단이 나온 출력을 돌려준다."""
    t, out = t0, None
    while t <= t0 + ctrl.cfg.look_sec + 0.2:
        out = step(ctrl, t, make_markers(t))
        if out.phase != AutoParkPhase.ARUCO_ALIGN:
            break
        t += 0.1
    return out


class TurnInTest(unittest.TestCase):
    def test_beacon_done_turns_exactly_90_toward_wall(self):
        ctrl = AutoParkController(target_slot=0)
        ctrl.phase = AutoParkPhase.ARUCO_ALIGN
        ctrl._enter_look("TURN_IN")
        out = step(ctrl, 10.0)
        p = ctrl.cfg.search_pulse_pwm
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        self.assertEqual((out.left, out.right), (-p, p))                 # 왼쪽(벽 쪽)
        turns = [s for s in ctrl._realign_plan if s[1] != 0]
        self.assertEqual(len(turns), ctrl.cfg.pulses_per_90)             # 딱 90°만큼
        self.assertEqual(ctrl._after_plan_stage, "LOOK")


class WallDistanceCheckTest(unittest.TestCase):
    def test_compares_same_marker_after_a_complete_move(self):
        ctrl = look_ctrl()
        ctrl.pose_estimator = SimpleNamespace(estimate=lambda marker, *_: marker.pose)

        def marker(mid, wall_dist, size=80, offset=0):
            return SimpleNamespace(
                id=mid, size_px=size, corners=True, pose=Pose(wall_dist, offset, 0, 0)
            )

        move = look(ctrl, lambda t: [marker(1, 500), marker(2, 510, size=70)])
        self.assertEqual(move.phase, AutoParkPhase.REALIGN)
        now = ctrl._realign_plan[-1][0] + 0.01
        step(ctrl, now)         # 직진 끝
        step(ctrl, now + 0.1)   # 벽 쪽 회전 시작
        now = ctrl._realign_plan[-1][0] + 0.01
        step(ctrl, now)         # 벽 쪽 회전 끝

        # 회전 도중에 촬영된 프레임은 새 시퀀스여도 벽 거리 표본에 넣지 않는다.
        old = MarkerFrame(1, now - 0.1, [marker(2, 900)], W, H)
        ctrl.step(now + 0.1, marker_frame=old, safety=SafetyObservation(SAFETY_PASS, now + 0.1))
        self.assertEqual(ctrl._look_poses, {})
        for i in range(1, 4):
            out = step(ctrl, now + 0.5 * i + 0.11, [marker(2, 540, offset=120), marker(3, 550, size=90)])
        self.assertEqual((out.left, out.right), (0, 0))
        self.assertIn("510 → 540mm (변화 +30mm, 같은 2번", out.detail)
        self.assertIn("목표 +170mm, 주행 명령 1.23초", out.detail)
        self.assertIn("통로 이동 추정 +120mm", out.detail)
        self.assertNotIn("벽 거리 비교", step(ctrl, now + 1.7, [marker(3, 550)]).detail)

    def test_different_markers_are_labeled_and_insufficient_samples_are_skipped(self):
        ctrl = look_ctrl()
        before = {1: [Pose(mm, 0, 0, 0) for mm in (490, 500, 530)]}
        ctrl._wall_before_move = before, -290.0, 2.9
        ctrl._look_poses = {3: [Pose(mm, 0, 0, 0) for mm in (450, 460, 500)]}
        self.assertIn("변화 -40mm, 1번 → 3번, 같은 벽 가정", ctrl._decide_look(12.0).detail)

        ctrl._wall_before_move = before, 290.0, 2.9
        ctrl._look_poses = {1: [Pose(mm, 0, 0, 0) for mm in (480, 490)]}
        self.assertIn("비교 생략 (이동 후 마커 자세 부족", ctrl._decide_look(12.0).detail)


@unittest.skipIf(cv2 is None, "OpenCV 없음")
class LookDecisionTest(unittest.TestCase):
    def test_target_straight_in_front_starts_parking(self):
        ctrl = look_ctrl()
        out = look(ctrl, lambda t: [marker_at(5, 450, face_marker=False, t=t, marker_id=0)])
        self.assertEqual(out.phase, AutoParkPhase.FRONT_PARK)
        self.assertIn("정면", ctrl.last_look)

    def test_target_past_by_80mm_turns_to_aisle_and_backs_80mm(self):
        ctrl = look_ctrl()
        out = look(ctrl, lambda t: [marker_at(80, 450, face_marker=False, t=t, marker_id=0)])
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        p, v = ctrl.cfg.search_pulse_pwm, ctrl.cfg.realign_pwm
        self.assertEqual((out.left, out.right), (p, -p))                 # 통로 방향(오른쪽) 90°
        move = ctrl._realign_plan[-1]
        self.assertEqual((move[1], move[2]), (-v, -v))                   # 지나쳤으니 뒤로
        self.assertIn("뒤로 80mm", move[3])
        self.assertAlmostEqual(move[0] - ctrl._realign_plan[-2][0], 80 / 153)
        turns = [s for s in ctrl._realign_plan if s[1] != 0 and s[1] == -s[2]]
        self.assertEqual(len(turns), ctrl.cfg.right_pulses_per_90)
        self.assertEqual(ctrl._after_plan_stage, "TURN_IN")              # 이동 후 다시 90° 보기

    def test_target_not_reached_moves_forward(self):
        ctrl = look_ctrl()
        look(ctrl, lambda t: [marker_at(-100, 450, face_marker=False, t=t, marker_id=0)])
        move = ctrl._realign_plan[-1]
        self.assertGreater(move[1], 0)
        self.assertIn("앞으로 100mm", move[3])

    def test_tilted_to_wall_fixes_heading_first(self):
        # 90°가 덜 돌아서 벽과 15° 틀어짐 → 이동하기 전에 조금 돌아서 벽과 맞춘다
        ctrl = look_ctrl()
        out = look(ctrl, lambda t: [marker_at(0, 450, face_marker=False, turn_right_deg=-15,
                                              t=t, marker_id=0)])
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        p = ctrl.cfg.search_pulse_pwm
        self.assertEqual((out.left, out.right), (p, -p))                 # 오른쪽으로 바로잡기
        n = sum(1 for s in ctrl._realign_plan if s[1] != 0)
        self.assertEqual(n, 1)
        self.assertEqual(ctrl._after_plan_stage, "LOOK")                 # 바로잡고 다시 보기
        now = ctrl._realign_plan[-1][0] + 0.01
        step(ctrl, now)
        out = look(ctrl, lambda t: [marker_at(0, 450, face_marker=False, turn_right_deg=15,
                                              t=t, marker_id=0)], t0=now + 0.1)
        self.assertEqual((out.left, out.right), (-p, p))                 # 새 영상에서 반대 오차
        self.assertEqual(sum(1 for s in ctrl._realign_plan if s[1] != 0), 1)
        now = ctrl._realign_plan[-1][0] + 0.01
        step(ctrl, now)
        out = look(ctrl, lambda t: [marker_at(0, 450, face_marker=False, t=t, marker_id=0)],
                   t0=now + 0.1)
        self.assertEqual(out.phase, AutoParkPhase.FRONT_PARK)

    def test_only_other_marker_uses_order_and_its_offset(self):
        # 목표 0 안 보이고 3번이 30mm 옆에 보임. 순서 1,2,3,0,4 → 0은 3번보다 한 칸(170mm) 앞
        ctrl = look_ctrl()
        look(ctrl, lambda t: [marker_at(30, 450, face_marker=False, t=t, marker_id=3)])
        move = ctrl._realign_plan[-1]
        self.assertGreater(move[1], 0)
        self.assertIn("앞으로 140mm", move[3])                            # 170 - 30
        self.assertIn("3번 기준 +1칸", ctrl.last_look)

    def test_nothing_visible_peeks_left_and_right_before_moving(self):
        ctrl = look_ctrl()
        p, k = ctrl.cfg.search_pulse_pwm, ctrl.cfg.peek_pulses

        def finish_plan(t):
            end = ctrl._realign_plan[-1][0]
            out = step(ctrl, end + 0.01)
            self.assertEqual(out.phase, AutoParkPhase.ARUCO_ALIGN)
            return end + 0.02

        out = look(ctrl, lambda t: [])                                   # ① 벽 쪽으로 더 돌아 보기
        self.assertEqual((out.left, out.right), (-p, p))
        self.assertEqual(sum(1 for s in ctrl._realign_plan if s[1] != 0), k)
        t = finish_plan(10.0)
        out = look(ctrl, lambda t: [], t0=t)                             # ② 반대쪽으로 돌아 보기
        self.assertEqual((out.left, out.right), (p, -p))
        self.assertEqual(sum(1 for s in ctrl._realign_plan if s[1] != 0), 2 * k)
        t = finish_plan(t)
        look(ctrl, lambda t: [], t0=t)                                   # ③ 그래도 없으면 이동
        turns = [s for s in ctrl._realign_plan if s[1] != 0 and s[1] == -s[2]]
        self.assertEqual(len(turns), ctrl.cfg.right_pulses_per_90 - k)  # 직각에서 벗어난 만큼 덜 돈다
        self.assertIn("앞으로 170mm", ctrl._realign_plan[-1][3])

    def test_no_camera_frames_stops_instead_of_moving(self):
        # 영상이 한 장도 안 들어오면 "마커 없음"으로 보고 이동하면 안 된다
        ctrl = look_ctrl()
        t, out = 10.0, None
        while t < 10.0 + ctrl.cfg.look_sec + ctrl.cfg.look_frame_wait_sec + 0.5:
            out = ctrl.step(t, marker_frame=None, distance=DistanceObservation(400, RANGE_OK, t),
                            safety=SafetyObservation(SAFETY_PASS, t))
            self.assertEqual((out.left, out.right), (0, 0))
            if out.phase != AutoParkPhase.ARUCO_ALIGN:
                break
            t += 0.1
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("카메라 영상", out.reason)

    def test_old_frames_do_not_count(self):
        ctrl = look_ctrl()
        old = MarkerFrame(seq=1, t=0.0, markers=[], width_px=W, height_px=H)   # 10초 전 영상
        out = ctrl.step(12.0, marker_frame=old, distance=DistanceObservation(400, RANGE_OK, 12.0),
                        safety=SafetyObservation(SAFETY_PASS, 12.0))
        self.assertEqual(ctrl._look_frames, 0)

    def test_gives_up_after_max_moves(self):
        ctrl = look_ctrl()
        ctrl.look_moves = ctrl.cfg.max_look_moves
        ctrl._peek = 2                                                   # 좌우 확인까지 끝난 상태
        out = look(ctrl, lambda t: [])
        self.assertEqual(out.phase, AutoParkPhase.ABORT)


if __name__ == "__main__":
    unittest.main()
