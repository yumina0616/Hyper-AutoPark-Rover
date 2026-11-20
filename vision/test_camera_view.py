"""vision/camera_view.py 테스트.

숫자 계산은 OpenCV 없이, 화면 처리 전체는 가짜 카메라(마커를 그린 합성 이미지)로 확인한다.
    python -m pytest vision/test_camera_view.py -q
"""

import threading
import unittest
import urllib.request
from dataclasses import dataclass, field
from http.server import ThreadingHTTPServer

from vision.aruco_auto_align import StageResult
from vision.camera_view import FpsMeter, marker_line, stage_line, status_lines

try:
    import cv2
    import numpy as np
except ImportError:  # OpenCV 없는 환경에서는 통합 테스트만 건너뜀
    cv2 = None


@dataclass
class FakeMarker:
    id: int
    err_x: float
    size_px: float
    t: float = 0.0
    corners: list = field(default_factory=list)


class FpsMeterTests(unittest.TestCase):
    def test_ten_frames_per_second(self):
        m = FpsMeter(window_sec=1.0)
        for i in range(11):
            m.tick(i * 0.1)
        self.assertAlmostEqual(m.fps, 10.0, places=6)

    def test_old_frames_drop_out_of_window(self):
        m = FpsMeter(window_sec=1.0)
        for i in range(5):
            m.tick(i * 0.5)  # 2 fps
        for i in range(1, 31):
            m.tick(2.0 + i * 0.05)  # 이후 1.5초 동안 20 fps
        self.assertAlmostEqual(m.fps, 20.0, places=6)

    def test_not_enough_frames(self):
        m = FpsMeter()
        self.assertEqual(m.fps, 0.0)
        m.tick(1.0)
        self.assertEqual(m.fps, 0.0)


class TextTests(unittest.TestCase):
    def test_marker_line_without_corners(self):
        text = marker_line(FakeMarker(2, 0.25, 140.0))
        self.assertIn("ID 2", text)
        self.assertIn("err_x +0.250", text)
        self.assertIn("size 140.0px", text)
        self.assertIn("skew n/a", text)

    def test_marker_line_with_square_corners_has_zero_skew(self):
        corners = [(0, 0), (100, 0), (100, 100), (0, 100)]
        text = marker_line(FakeMarker(0, 0.0, 100.0, corners=corners))
        self.assertIn("skew +0.000", text)

    def test_stage_line(self):
        text = stage_line(StageResult(81, -81, "TURNING", err_x=0.4))
        self.assertEqual(text, "align stage: TURNING  L=+81 R=-81  err +0.400")

    def test_status_lines_put_target_first(self):
        markers = [FakeMarker(3, 0.1, 50.0), FakeMarker(1, -0.2, 60.0)]
        lines = status_lines(12.0, 8.5, 640, 480, 1, markers, StageResult(0, 0, "TURNING"))
        self.assertTrue(lines[0].startswith("FPS 12.0"))
        self.assertIn("target ID 1", lines[0])
        self.assertTrue(lines[2].startswith("ID 1"))
        self.assertTrue(lines[3].startswith("ID 3"))

    def test_status_lines_no_marker(self):
        lines = status_lines(0.0, 0.0, 640, 480, 0, [], StageResult(0, 0, "MARKER_MISSING"))
        self.assertEqual(lines[-1], "no marker")


class _FakeCapture:
    """마커 한 장이 그려진 같은 이미지를 계속 돌려주는 가짜 카메라."""

    def __init__(self, frame):
        self.frame = frame

    def read(self):
        return True, self.frame.copy()

    def release(self):
        pass


def _synthetic_frame(marker_id: int, x: int) -> "np.ndarray":
    """640x480 흰 화면의 (x, 140) 위치에 200px 마커를 그린다."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "generateImageMarker"):
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, 200)
    else:
        img = cv2.aruco.drawMarker(dictionary, marker_id, 200)
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)
    frame[140:340, x:x + 200] = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return frame


@unittest.skipIf(cv2 is None, "OpenCV 없음")
class CameraViewIntegrationTests(unittest.TestCase):
    def _run_view(self, marker_id, x, target_id=0):
        from vision.camera_view import CameraView, ViewConfig, make_handler

        view = CameraView(ViewConfig(target_id=target_id))
        view._open_camera = lambda: _FakeCapture(_synthetic_frame(marker_id, x))
        view.start()
        self.addCleanup(view.stop)
        return view, make_handler

    def test_marker_on_right_proposes_right_turn(self):
        view, _ = self._run_view(marker_id=0, x=400)  # 마커 중심 x=500 → 화면 오른쪽
        seq, jpeg = view.wait_jpeg(-1)
        self.assertIsNone(view.error)
        self.assertTrue(jpeg.startswith(b"\xff\xd8"))  # JPEG 시작 표시
        # 같은 프레임이라도 seq가 매번 달라 정렬 단계가 계속 진행된다
        for _ in range(3):
            seq, _ = view.wait_jpeg(seq)
        r = view.stage._last
        self.assertEqual(r.state, "TURNING")
        self.assertGreater(r.left, 0)
        self.assertLess(r.right, 0)

    def test_centered_marker_reaches_aligned(self):
        view, _ = self._run_view(marker_id=0, x=220)  # 중심 x=320 → 정중앙
        seq = -1
        for _ in range(10):
            seq, _ = view.wait_jpeg(seq)
        self.assertEqual(view.stage.finished_state, "ALIGNED")

    def test_wrong_marker_stops(self):
        view, _ = self._run_view(marker_id=3, x=400, target_id=0)
        seq = -1
        for _ in range(3):
            seq, _ = view.wait_jpeg(seq)
        self.assertEqual(view.stage._last.state, "WRONG_MARKER")

    def test_web_page_and_stream(self):
        view, make_handler = self._run_view(marker_id=0, x=220)
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(view))
        server.daemon_threads = True
        threading.Thread(target=server.serve_forever, daemon=True).start()
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f"http://127.0.0.1:{server.server_address[1]}"

        page = urllib.request.urlopen(base + "/", timeout=5).read().decode("utf-8")
        self.assertIn('src="/stream"', page)

        with urllib.request.urlopen(base + "/stream", timeout=5) as resp:
            self.assertIn("multipart/x-mixed-replace", resp.headers["Content-Type"])
            head = resp.read(200)
        self.assertIn(b"Content-Type: image/jpeg", head)

        req = urllib.request.Request(base + "/reset", method="POST")
        self.assertIn("초기화", urllib.request.urlopen(req, timeout=5).read().decode("utf-8"))


if __name__ == "__main__":
    unittest.main()
