"""vision/pose.py 테스트: 답을 아는 합성 장면으로 자세 추정과 부호 약속을 확인한다.

카메라를 벽 앞 (옆 오차, 거리, 틀어진 각도)에 놓고 마커 꼭짓점을 화면에 투영한 뒤,
그 꼭짓점만으로 estimate_pose가 원래 자세를 되찾는지 본다.
    python -m pytest vision/test_pose.py -q
"""

import math
import unittest

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None

if cv2 is not None:
    from vision.pose import (
        PoseEstimator,
        camera_matrix_from_fov,
        estimate_pose,
        marker_object_points,
    )

W, H = 640, 480
MARKER_MM = 100.0


def project_corners(offset_mm, wall_dist_mm, turn_right_deg, k):
    """카메라 위치·방향을 정해 마커 꼭짓점의 화면 좌표를 만든다.

    turn_right_deg: 로버가 벽 정면에서 오른쪽(시계 방향)으로 돈 각도
    마커 좌표: x 오른쪽(벽을 바라볼 때), y 위, z 벽에서 앞쪽
    """
    psi = math.radians(turn_right_deg)
    z_cam = np.array([math.sin(psi), 0.0, -math.cos(psi)])   # 정면
    x_cam = np.array([math.cos(psi), 0.0, math.sin(psi)])    # 오른쪽
    y_cam = np.array([0.0, -1.0, 0.0])                       # 아래
    r_cam_to_marker = np.column_stack([x_cam, y_cam, z_cam])
    r = r_cam_to_marker.T                                    # 마커 → 카메라
    c = np.array([offset_mm, 0.0, wall_dist_mm])             # 카메라 위치 (마커 좌표)
    t = -r @ c
    rvec, _ = cv2.Rodrigues(r)
    pts, _ = cv2.projectPoints(marker_object_points(MARKER_MM), rvec, t, k, np.zeros(5))
    return pts.reshape(4, 2)


@unittest.skipIf(cv2 is None, "OpenCV 없음")
class PoseTests(unittest.TestCase):
    def setUp(self):
        self.k = camera_matrix_from_fov(W, H, 45.0)

    def estimate(self, offset, dist, turn):
        corners = project_corners(offset, dist, turn, self.k)
        pose = estimate_pose(corners.tolist(), self.k, marker_mm=MARKER_MM)
        self.assertIsNotNone(pose)
        return pose

    def test_straight_in_front(self):
        p = self.estimate(0, 400, 0)
        self.assertAlmostEqual(p.wall_dist_mm, 400, delta=1)
        self.assertAlmostEqual(p.offset_mm, 0, delta=1)
        self.assertAlmostEqual(p.yaw_deg, 0, delta=0.5)
        self.assertAlmostEqual(p.bearing_deg, 0, delta=0.5)

    def test_shifted_right_and_turned_left_to_face_marker(self):
        # 칸 중심선보다 50mm 오른쪽에서 마커를 정면으로 바라봄 → 왼쪽으로 약 7.1° 돌아 있음
        turn = -math.degrees(math.atan2(50, 400))
        p = self.estimate(50, 400, turn)
        self.assertAlmostEqual(p.offset_mm, 50, delta=1)
        self.assertAlmostEqual(p.wall_dist_mm, 400, delta=1)
        # 칸 방향(벽에 수직)으로 맞추려면 오른쪽으로 돌아야 함 → yaw +
        self.assertAlmostEqual(p.yaw_deg, -turn, delta=0.5)
        # 마커는 화면 정중앙 (err_x = 0인데도 옆 오차가 있음을 잡아냄)
        self.assertAlmostEqual(p.bearing_deg, 0, delta=0.5)

    def test_turned_right_at_center(self):
        p = self.estimate(0, 300, 10)
        self.assertAlmostEqual(p.yaw_deg, -10, delta=0.5)   # 왼쪽으로 돌아야 함
        self.assertAlmostEqual(p.offset_mm, 0, delta=1)
        self.assertLess(p.bearing_deg, 0)                    # 마커는 화면 왼쪽에 보임

    def test_shifted_left_looking_straight(self):
        # 카메라가 중심선 왼쪽(-80)에서 벽을 똑바로 보면 마커는 화면 오른쪽 → bearing +
        p = self.estimate(-80, 500, 0)
        self.assertAlmostEqual(p.offset_mm, -80, delta=1)
        self.assertAlmostEqual(p.yaw_deg, 0, delta=0.5)
        self.assertGreater(p.bearing_deg, 0)

    def test_frontal_scenes_pick_correct_solution(self):
        # OpenCV IPPE가 잘못된 해를 냈던 정면 장면들 (재투영 오차로 걸러야 함)
        for offset, dist, turn in [(0, 400, 0), (0, 600, 0), (50, 400, -7.125)]:
            p = self.estimate(offset, dist, turn)
            self.assertAlmostEqual(p.wall_dist_mm, dist, delta=1)
            self.assertAlmostEqual(p.offset_mm, offset, delta=1)
            self.assertAlmostEqual(p.yaw_deg, -turn, delta=0.5)

    def test_big_turn_20deg(self):
        p = self.estimate(-40, 250, 20)
        self.assertAlmostEqual(p.yaw_deg, -20, delta=0.5)
        self.assertAlmostEqual(p.offset_mm, -40, delta=1)

    def test_garbage_corners_rejected(self):
        # 정사각형 마커로 설명할 수 없는 꼭짓점 → 재투영 오차가 커서 None
        bad = [(100, 100), (400, 120), (150, 300), (380, 330)]
        self.assertIsNone(estimate_pose(bad, self.k))

    def test_close_range_150mm(self):
        p = self.estimate(10, 150, 3)
        self.assertAlmostEqual(p.wall_dist_mm, 150, delta=1)
        self.assertAlmostEqual(p.offset_mm, 10, delta=1)
        self.assertAlmostEqual(p.yaw_deg, -3, delta=0.5)

    def test_pixel_noise_keeps_reasonable_accuracy(self):
        rng = np.random.default_rng(0)
        corners = project_corners(30, 400, -5, self.k) + rng.normal(0, 0.5, (4, 2))
        p = estimate_pose(corners.tolist(), self.k, marker_mm=MARKER_MM)
        self.assertAlmostEqual(p.wall_dist_mm, 400, delta=10)
        self.assertAlmostEqual(p.offset_mm, 30, delta=10)

    def test_invalid_corners(self):
        self.assertIsNone(estimate_pose([], self.k))
        self.assertIsNone(estimate_pose([(0, 0)] * 3, self.k))

    def test_estimator_uses_marker_corners(self):
        class M:
            corners = project_corners(0, 400, 0, camera_matrix_from_fov(W, H, 45.0)).tolist()

        p = PoseEstimator(marker_mm=MARKER_MM, hfov_deg=45.0).estimate(M(), W, H)
        self.assertAlmostEqual(p.wall_dist_mm, 400, delta=1)
        self.assertIsNone(PoseEstimator().estimate(object(), W, H))


if __name__ == "__main__":
    unittest.main()
