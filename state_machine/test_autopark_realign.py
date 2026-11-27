"""자세 확인(기울기 감지)과 재정렬 반복 테스트.

실제 카메라처럼 꼭짓점이 있는 마커를 합성한다: 카메라를 벽 앞 (옆 오차, 거리, 회전)에 놓고
5cm 마커의 꼭짓점을 640x480 화면에 투영한다. 로버는 왼쪽 칸 배치에서 왼쪽으로 돌아 벽을 본
상태라, 벽을 볼 때 원래 진행 방향은 오른쪽이다.
    python -m pytest state_machine/test_autopark_realign.py -q
"""

import math
import unittest
from dataclasses import dataclass, field

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

from state_machine.autopark import (
    SAFETY_PASS,
    AutoParkController,
    AutoParkPhase,
    SafetyObservation,
)
from state_machine.front_parking import (
    DistanceObservation,
    FrontParkingConfig,
    FrontParkingController,
    MarkerObservation,
    ParkingState,
    RANGE_OK,
    STOP_MISALIGNED_NEAR,
)
from vision.aruco_auto_align import MarkerFrame

W, H = 640, 480
MARKER_MM = 50.0


@dataclass(frozen=True)
class PoseMarker:
    id: int
    err_x: float
    size_px: float
    t: float
    corners: list = field(default_factory=list)


def marker_at(lateral_mm, dist_mm, face_marker=True, turn_right_deg=0.0, t=0.0, marker_id=0):
    """카메라를 칸 중심선에서 lateral_mm, 벽에서 dist_mm 떨어진 곳에 두고 마커를 본다.

    face_marker=True면 마커가 화면 가운데 오도록 돌려 둔다 (정렬 단계가 끝난 상태).
    """
    from vision.pose import camera_matrix_from_fov, marker_object_points

    if face_marker:
        turn_right_deg = -math.degrees(math.atan2(lateral_mm, dist_mm))
    k = camera_matrix_from_fov(W, H, 45.0)
    psi = math.radians(turn_right_deg)
    z_cam = np.array([math.sin(psi), 0.0, -math.cos(psi)])
    x_cam = np.array([math.cos(psi), 0.0, math.sin(psi)])
    y_cam = np.array([0.0, -1.0, 0.0])
    r = np.column_stack([x_cam, y_cam, z_cam]).T
    tvec = -r @ np.array([lateral_mm, 0.0, dist_mm])
    rvec, _ = cv2.Rodrigues(r)
    pts, _ = cv2.projectPoints(marker_object_points(MARKER_MM), rvec, tvec, k, np.zeros(5))
    pts = pts.reshape(4, 2)
    sides = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
    err_x = (float(np.mean(pts[:, 0])) - W / 2) / (W / 2)
    return PoseMarker(marker_id, err_x, float(np.mean(sides)), t,
                      [(float(x), float(y)) for x, y in pts])


def mframe(seq, marker):
    return MarkerFrame(seq=seq, t=marker.t, markers=[marker], width_px=W, height_px=H)


def dist(t, mm=400):
    return DistanceObservation(dist_mm=mm, status=RANGE_OK, t=t)


def ok_safety(t):
    return SafetyObservation(reason=SAFETY_PASS, t=t)


def parking_ctrl(frames=3):
    ctrl = AutoParkController(target_slot=0)
    ctrl.cfg.align_mode = "sweep"   # 예전 재정렬(후진·회전·옆 이동) 테스트
    ctrl.cfg.pose_check_frames = frames
    ctrl.phase = AutoParkPhase.FRONT_PARK
    return ctrl


def step(ctrl, t, marker=None, seq=None, mm=400):
    frame = None if marker is None else mframe(seq if seq is not None else int(t * 100), marker)
    return ctrl.step(t, marker_frame=frame, distance=dist(t, mm), safety=ok_safety(t))


def run_pose_check(ctrl, lateral_mm, dist_mm=230, t0=1.0, frames=3):
    """확인 지점 도착 → 새 프레임 frames장으로 자세 확인. 마지막 출력을 돌려준다."""
    out = step(ctrl, t0, marker_at(lateral_mm, dist_mm, t=t0), seq=1)
    assert out.phase == AutoParkPhase.POSE_CHECK, out
    for i in range(frames):
        t = t0 + 0.05 * (i + 1)
        out = step(ctrl, t, marker_at(lateral_mm, dist_mm, t=t), seq=2 + i)
    return out


@unittest.skipIf(cv2 is None, "OpenCV 없음")
class PoseFromMarkerTest(unittest.TestCase):
    def test_target_marker_carries_pose(self):
        ctrl = AutoParkController(target_slot=0)
        m = ctrl._target_marker(mframe(1, marker_at(40, 300, t=1.0)))
        self.assertTrue(m.has_pose)
        self.assertAlmostEqual(m.lateral_mm, 40, delta=2)
        self.assertAlmostEqual(m.yaw_deg, math.degrees(math.atan2(40, 300)), delta=1)
        self.assertAlmostEqual(m.offset_px, 0, delta=2)   # 마커는 화면 가운데 (err_x로는 못 잡음)

    def test_marker_without_corners_has_no_pose(self):
        ctrl = AutoParkController(target_slot=0)
        m = ctrl._target_marker(mframe(1, PoseMarker(0, 0.1, 120.0, 1.0)))
        self.assertFalse(m.has_pose)


@unittest.skipIf(cv2 is None, "OpenCV 없음")
class PoseCheckTest(unittest.TestCase):
    def test_far_marker_does_not_trigger_check(self):
        ctrl = parking_ctrl()
        out = step(ctrl, 1.0, marker_at(0, 500, t=1.0), seq=1)   # 약 77px < 150px
        self.assertEqual(out.phase, AutoParkPhase.FRONT_PARK)
        self.assertGreater(out.left, 0)

    def test_straight_passes_and_keeps_parking(self):
        ctrl = parking_ctrl()
        out = run_pose_check(ctrl, lateral_mm=0)
        self.assertEqual(out.phase, AutoParkPhase.FRONT_PARK)
        self.assertIn("기울기 통과", out.detail)
        self.assertEqual((out.left, out.right), (0, 0))
        nxt = step(ctrl, 2.0, marker_at(0, 240, t=2.0), seq=50)
        self.assertEqual(nxt.phase, AutoParkPhase.FRONT_PARK)   # 다시 확인하지 않고 접근 계속
        self.assertGreater(nxt.left, 0)

    def test_tilted_past_slot_backs_up_turns_and_reverses(self):
        # 칸 중심선보다 60mm 진행 방향(오른쪽)으로 지나침 → 마커 보며 약 13.5° 기울어짐
        ctrl = parking_ctrl()
        out = run_pose_check(ctrl, lateral_mm=60)
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        self.assertEqual((out.left, out.right), (-80, -80))          # ① 벽에서 뒤로 빠짐
        self.assertIn("기울기", out.reason)
        t = 1.15
        back, rot = ctrl.cfg.realign_backup_sec, ctrl.cfg.realign_turn_sec
        turn = step(ctrl, t + back + 0.05, mm=400)
        p = ctrl.cfg.realign_turn_pwm
        self.assertEqual((turn.left, turn.right), (p, -p))            # ② 오른쪽(통로 방향)으로 회전
        shift = step(ctrl, t + back + rot + 0.05, mm=400)
        self.assertEqual((shift.left, shift.right), (-80, -80))       # ③ 지나쳤으니 후진
        self.assertIn("후진", shift.detail)
        done = step(ctrl, t + back + rot + 0.61, mm=400)              # 60mm / 100mm/s = 0.6초
        self.assertEqual(done.phase, AutoParkPhase.ARUCO_ALIGN)       # ④ 다시 마커 찾기
        self.assertEqual(ctrl.realign_attempts, 1)

    def test_tilted_before_slot_drives_forward(self):
        ctrl = parking_ctrl()
        run_pose_check(ctrl, lateral_mm=-60)
        shift = step(ctrl, 1.15 + ctrl.cfg.realign_backup_sec + ctrl.cfg.realign_turn_sec + 0.05, mm=400)
        self.assertEqual((shift.left, shift.right), (80, 80))
        self.assertIn("직진", shift.detail)

    def test_gives_up_after_max_attempts(self):
        ctrl = parking_ctrl()
        ctrl.cfg.max_realign_attempts = 1
        run_pose_check(ctrl, lateral_mm=60)
        self.assertEqual(ctrl.realign_attempts, 1)
        ctrl.phase = AutoParkPhase.FRONT_PARK   # 재정렬 뒤 다시 접근했다고 가정
        out = run_pose_check(ctrl, lateral_mm=60, t0=10.0)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("재정렬 1회 넘음", out.reason)

    def test_forward_shift_stops_for_obstacle(self):
        ctrl = parking_ctrl()
        run_pose_check(ctrl, lateral_mm=-60)
        out = step(ctrl, 1.15 + ctrl.cfg.realign_backup_sec + ctrl.cfg.realign_turn_sec + 0.05, mm=100)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("장애물", out.reason)

    def test_pose_check_times_out_without_pose(self):
        ctrl = parking_ctrl(frames=5)
        step(ctrl, 1.0, marker_at(0, 250, t=1.0), seq=1)
        out = step(ctrl, 4.5)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("자세 확인 시간 초과", out.reason)


class FrontParkingPoseAlignTest(unittest.TestCase):
    def test_close_with_pixel_offset_but_straight_pose_parks(self):
        # 2026-11-24 실측 상황: 정지 위치에서 err_x -0.26(약 83px)인데 실제로는 똑바름
        ctrl = FrontParkingController(FrontParkingConfig(required_stop_samples=3))
        for i in range(3):
            t = 1.0 + i * 0.1
            out = ctrl.update(
                MarkerObservation(0, 340.0, -83.0, t, yaw_deg=-1.7, lateral_mm=1.0),
                DistanceObservation(120, RANGE_OK, t),
                now=t,
            )
        self.assertEqual(out.state, ParkingState.PARKED)

    def test_close_and_tilted_stops_with_realign_reason(self):
        ctrl = FrontParkingController()
        out = ctrl.update(
            MarkerObservation(0, 340.0, 0.0, 1.0, yaw_deg=14.0, lateral_mm=30.0),
            DistanceObservation(120, RANGE_OK, 1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.SAFETY_STOP)
        self.assertEqual(ctrl.stop_reason, STOP_MISALIGNED_NEAR)

    def test_tof_close_and_marker_slightly_small_still_parks(self):
        # ToF와 카메라 위치 차이로 마커가 기준(300px)보다 조금 작아도(290px) 정지 인정
        ctrl = FrontParkingController(FrontParkingConfig(required_stop_samples=1))
        out = ctrl.update(
            MarkerObservation(0, 290.0, 0.0, 1.0, yaw_deg=1.0, lateral_mm=2.0),
            DistanceObservation(125, RANGE_OK, 1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.PARKED)

    def test_tof_close_but_marker_much_smaller_is_sensor_conflict(self):
        ctrl = FrontParkingController()
        out = ctrl.update(
            MarkerObservation(0, 150.0, 0.0, 1.0, yaw_deg=1.0, lateral_mm=2.0),
            DistanceObservation(120, RANGE_OK, 1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.SAFETY_STOP)
        self.assertNotEqual(ctrl.stop_reason, STOP_MISALIGNED_NEAR)

    def test_tilt_within_10deg_is_ok(self):
        ctrl = FrontParkingController()
        self.assertTrue(ctrl.is_aligned(MarkerObservation(0, 340.0, 0.0, 1.0, 9.5, 10.0)))
        self.assertFalse(ctrl.is_aligned(MarkerObservation(0, 340.0, 0.0, 1.0, 10.5, 10.0)))


@unittest.skipIf(cv2 is None, "OpenCV 없음")
class StopPositionRealignTest(unittest.TestCase):
    def test_tilted_at_stop_position_realigns_instead_of_abort(self):
        ctrl = parking_ctrl()
        ctrl._pose_checked = True   # 확인 지점은 이미 통과했다고 가정
        out = step(ctrl, 1.0, marker_at(40, 120, t=1.0), seq=1, mm=120)   # 약 18° 기울어짐
        self.assertEqual(out.phase, AutoParkPhase.REALIGN)
        self.assertEqual((out.left, out.right), (-80, -80))
        self.assertIn("정지 위치에서 기울어짐", out.reason)


if __name__ == "__main__":
    unittest.main()
