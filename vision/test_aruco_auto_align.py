"""ArUco 제자리 정렬 단계: 카메라/Bridge/하드웨어 없는 단위 테스트.

    python -m unittest vision.test_aruco_auto_align -v
"""

import unittest
from dataclasses import dataclass

from vision.align import AlignConfig
from vision.aruco_auto_align import ArucoAlignStage, MarkerFrame, StageConfig


@dataclass(frozen=True)
class FakeMarker:
    id: int
    err_x: float
    t: float


def frame(seq, err_x=None, t=100.0, marker_id=0):
    markers = [] if err_x is None else [FakeMarker(marker_id, err_x, t)]
    return MarkerFrame(seq, t, markers)


class StageTests(unittest.TestCase):
    def setUp(self):
        self.stage = ArucoAlignStage(target_id=0)
        self.stage.reset(now=100.0)

    def test_marker_right_proposes_right_turn_as_integer_pwm(self):
        r = self.stage.step(frame(1, 0.5), now=100.0)
        self.assertEqual(r.state, "TURNING")
        self.assertIsInstance(r.left, int)
        self.assertGreater(r.left, 0)
        self.assertEqual(r.left, -r.right)
        self.assertLessEqual(r.left, 255)

    def test_marker_left_proposes_left_turn(self):
        r = self.stage.step(frame(1, -0.5), now=100.0)
        self.assertLess(r.left, 0)
        self.assertGreater(r.right, 0)

    def test_proportional_turn_has_consistent_maximum_and_minimum(self):
        low = self.stage.step(frame(1, 0.5), now=100.0)
        high = self.stage.step(frame(2, 1.0, t=100.1), now=100.1)
        self.assertEqual(low.left, 60)
        self.assertEqual(high.left, 100)

    def test_bad_motor_pwm_config_is_rejected(self):
        with self.assertRaises(ValueError):
            ArucoAlignStage(0, align_config=AlignConfig(min_turn_pwm=90, max_turn=40))

    def test_centered_five_distinct_frames_then_latch(self):
        for seq in range(1, 5):
            r = self.stage.step(frame(seq, 0.01, t=100 + seq / 10), now=100 + seq / 10)
            self.assertEqual(r.state, "CENTERING_CHECK")
            self.assertEqual((r.left, r.right), (0, 0))
        r = self.stage.step(frame(5, 0.01, t=100.5), now=100.5)
        self.assertEqual(r.state, "ALIGNED")
        self.assertEqual(self.stage.step(frame(6, 0.9, t=100.6), now=100.6).state, "ALIGNED")

    def test_same_camera_frame_cannot_falsely_finish_alignment(self):
        one = frame(1, 0.0)
        for i in range(5):
            r = self.stage.step(one, now=100.0 + i * 0.05)
            self.assertEqual(r.state, "CENTERING_CHECK")

    def test_wrong_target_id_stops(self):
        r = self.stage.step(frame(1, 0.8, marker_id=2), now=100.0)
        self.assertEqual(r.state, "WRONG_MARKER")
        self.assertEqual((r.left, r.right), (0, 0))

    def test_wrong_id_does_not_trigger_opt_in_search(self):
        stage = ArucoAlignStage(0, StageConfig(search_if_missing=True))
        stage.reset(100.0)
        r = stage.step(frame(1, 0.5, marker_id=1), now=100.0)
        self.assertEqual(r.state, "WRONG_MARKER")
        self.assertEqual((r.left, r.right), (0, 0))

    def test_missing_after_seen_stops_and_does_not_search(self):
        stage = ArucoAlignStage(0, StageConfig(search_if_missing=True))
        stage.reset(100.0)
        stage.step(frame(1, 0.5), now=100.0)
        r = stage.step(frame(2, None, t=100.1), now=100.1)
        self.assertEqual(r.state, "MARKER_MISSING")
        self.assertEqual((r.left, r.right), (0, 0))

    def test_search_disabled_by_default(self):
        r = self.stage.step(frame(1, None), now=100.0)
        self.assertEqual((r.left, r.right), (0, 0))

    def test_opt_in_search_limited_in_time(self):
        stage = ArucoAlignStage(0, StageConfig(search_if_missing=True, search_sec=1.0))
        stage.reset(100.0)
        r = stage.step(frame(1, None), now=100.0)
        self.assertEqual((r.left, r.right), (60, -60))
        self.assertEqual(stage.step(frame(2, None, t=101.1), now=101.1).state, "MARKER_MISSING")

    def test_camera_stale_stops_even_if_previous_command_was_turning(self):
        self.stage.step(frame(1, 0.9), now=100.0)
        r = self.stage.step(frame(1, 0.9), now=100.6)
        self.assertEqual(r.state, "CAMERA_STALE")
        self.assertEqual((r.left, r.right), (0, 0))

    def test_camera_error_latches_stop(self):
        r = self.stage.step(None, now=100.1, camera_error="USB camera unavailable")
        self.assertEqual(r.state, "CAMERA_ERROR")
        self.assertEqual(self.stage.step(frame(1, 0.9), now=100.2).state, "CAMERA_ERROR")

    def test_camera_not_ready_deadline(self):
        self.assertEqual(self.stage.step(None, now=101.0).state, "WAITING_FOR_CAMERA")
        self.assertEqual(self.stage.step(None, now=103.0).state, "CAMERA_ERROR")

    def test_timeout_stops(self):
        r = self.stage.step(frame(1, 0.9, t=115.0), now=115.0)
        self.assertEqual(r.state, "TIMEOUT")
        self.assertEqual((r.left, r.right), (0, 0))

    def test_lost_frames_reset_stability_counter(self):
        stage = ArucoAlignStage(0, align_config=AlignConfig(align_frames=2, kp=100, max_turn=100))
        stage.reset(100.0)
        stage.step(frame(1, 0.01), now=100.0)
        stage.step(frame(2, None, t=100.1), now=100.1)
        r = stage.step(frame(3, 0.01, t=100.2), now=100.2)
        self.assertEqual(r.state, "CENTERING_CHECK")


if __name__ == "__main__":
    unittest.main()
