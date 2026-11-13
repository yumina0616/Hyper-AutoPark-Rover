from dataclasses import dataclass, field
import time
import cv2
import numpy as np

# TODO: 실험 및 #6 마커 규격 이슈에서 최종 사전 결정
DEFAULT_DICT_TYPE = cv2.aruco.DICT_4X4_50


@dataclass
class Marker:
    id: int
    err_x: float
    size_px: float
    t: float
    # 마커 네 꼭짓점의 픽셀 좌표 (x, y). 순서: 마커 기준 좌상 → 우상 → 우하 → 좌하
    # "마커 기준"이라 마커를 돌려 붙이면 화면 위치가 달라짐 → 마커는 정방향으로 부착할 것
    # 마커가 기울어진 정도(skew) 계산용. 기본값이 있어서 corners 없이 만든 Marker도 그대로 동작
    corners: list[tuple[float, float]] = field(default_factory=list)


class ArucoDetector:
    def __init__(self, dict_type: int = DEFAULT_DICT_TYPE):
        self.dict_type = dict_type
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_type)

        # OpenCV 4.7+ / 5.x 와 구버전(4.6 이하, UNO Q 환경 대비) 모두 호환되도록 처리
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
            self._use_new_api = True
        else:
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self._use_new_api = False

    def detect_markers(self, frame: np.ndarray) -> list[Marker]:
        if frame is None or frame.size == 0:
            return []

        _, width = frame.shape[:2]
        center_frame_x = width / 2.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.parameters
            )

        results: list[Marker] = []
        if ids is None:
            return results

        now = time.time()
        for corner, marker_id in zip(corners, ids.flatten()):
            pts = corner[0]  # (4, 2): 좌상, 우상, 우하, 좌하 꼭짓점 좌표

            # 좌우 오차: -1.0(화면 왼쪽 끝) ~ 0.0(가운데) ~ +1.0(오른쪽 끝)
            marker_center_x = float(np.mean(pts[:, 0]))
            err_x = (marker_center_x - center_frame_x) / center_frame_x
            err_x = float(np.clip(err_x, -1.0, 1.0))

            # 마커 한 변의 평균 픽셀 길이
            side_lengths = [
                np.linalg.norm(pts[0] - pts[1]),
                np.linalg.norm(pts[1] - pts[2]),
                np.linalg.norm(pts[2] - pts[3]),
                np.linalg.norm(pts[3] - pts[0]),
            ]
            size_px = float(np.mean(side_lengths))

            results.append(
                Marker(
                    id=int(marker_id),
                    err_x=round(err_x, 4),
                    size_px=round(size_px, 2),
                    t=now,
                    corners=[(float(x), float(y)) for x, y in pts],
                )
            )

        return results


_default_detector = ArucoDetector()


def detect_markers(frame: np.ndarray) -> list[Marker]:
    """docs/interfaces.md 5장 규격에 맞춘 기본 ArUco 검출 함수"""
    return _default_detector.detect_markers(frame)
