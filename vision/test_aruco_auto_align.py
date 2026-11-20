"""vision/aruco_auto_align.py 제어 로직 단위 테스트 (카메라·모터 없이 실행)."""

import unittest

from vision.aruco_auto_align import AlignConfig, ArucoAlignController


class ControllerTests(unittest.TestCase):
    def test_marker_right_turn_right(self):
        c = ArucoAlignController()
        cmd = c.update(500, 640, now=0.0)
        self.assertEqual(cmd.state, "TURNING_RIGHT")
        self.assertGreater(cmd.left, 0)
        self.assertLess(cmd.right, 0)

    def test_marker_left_turn_left(self):
        c = ArucoAlignController()
        cmd = c.update(120, 640, now=0.0)
        self.assertEqual(cmd.state, "TURNING_LEFT")
        self.assertLess(cmd.left, 0)
        self.assertGreater(cmd.right, 0)

    def test_five_stable_center_frames_stop_and_latch(self):
        c = ArucoAlignController()
        for i in range(4):
            cmd = c.update(325, 640, now=float(i))
            self.assertEqual(cmd.state, "CENTERING_CHECK")
            self.assertEqual((cmd.left, cmd.right), (0.0, 0.0))
        cmd = c.update(325, 640, now=4.0)
        self.assertEqual(cmd.state, "ALIGNED")
        self.assertEqual(c.update(600, 640, now=5.0).state, "ALIGNED")

    def test_lost_after_seen_stops(self):
        c = ArucoAlignController(AlignConfig(search_if_missing=True))
        self.assertEqual(c.update(None, 640, now=0.0).state, "SEARCHING")
        c.update(500, 640, now=1.0)
        cmd = c.update(None, 640, now=2.0)
        self.assertEqual((cmd.left, cmd.right), (0.0, 0.0))

    def test_timeout_stops(self):
        c = ArucoAlignController()
        c.update(500, 640, now=0.0)
        cmd = c.update(500, 640, now=15.0)
        self.assertEqual(cmd.state, "TIMEOUT")
        self.assertEqual((cmd.left, cmd.right), (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
