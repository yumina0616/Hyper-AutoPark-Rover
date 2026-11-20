"""전면 주차 실행 모드 (apps/qrover_drive/python/front_parking_mode.py)

카메라(ArUco)와 BridgeLink(ToF 거리)의 최신 관측값을 state_machine/front_parking.py 에 넘기고,
계산된 바퀴 명령(left, right)을 link.send_drive(left, right)로 전송합니다.
"""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Optional

import cv2
import numpy as np

# 저장소 루트(state_machine/, vision/)를 import 경로에 추가
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from state_machine.front_parking import (  # noqa: E402
    DistanceObservation,
    FrontParkingConfig,
    FrontParkingController,
    MarkerObservation,
    ParkingState,
)
from bridge_link import BridgeLink  # noqa: E402

try:
    from vision.aruco import detect_markers as _repo_detect_markers
except ImportError:
    _repo_detect_markers = None

TARGET_MARKER_ID = 0     # 목표 주차칸 마커 ID (0~4)
CAMERA_INDEX = 0         # Logitech C270 (/dev/video0)
FRAME_WIDTH = 640
FRAME_HEIGHT = 480


class _FallbackArucoDetector:
    """vision/aruco.py 가 앱 폴더에 복사되지 않은 환경을 위한 보조 검출기."""

    def __init__(self) -> None:
        aruco = cv2.aruco
        self._dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        if hasattr(aruco, "ArucoDetector"):
            self._detector = aruco.ArucoDetector(self._dict, aruco.DetectorParameters())
            self._new_api = True
        else:
            self._params = aruco.DetectorParameters_create()
            self._detector = None
            self._new_api = False

    def detect_target(self, frame: np.ndarray, target_id: int, now: float) -> Optional[MarkerObservation]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._new_api:
            corners_list, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners_list, ids, _ = cv2.aruco.detectMarkers(gray, self._dict, parameters=self._params)
        if ids is None:
            return None

        center_x_frame = frame.shape[1] / 2.0
        for corners, detected_id in zip(corners_list, ids.flatten()):
            if int(detected_id) != target_id:
                continue
            pts = corners.reshape((4, 2))
            side_lengths = [float(np.linalg.norm(pts[i] - pts[(i + 1) % 4])) for i in range(4)]
            side_px = float(np.mean(side_lengths))
            offset_px = float(np.mean(pts[:, 0])) - center_x_frame
            return MarkerObservation(
                marker_id=target_id,
                side_px=side_px,
                offset_px=offset_px,
                t=now,
            )
        return None


class FrontParkingRunner:
    """qrover_drive 메인 루프에서 호출하는 전면 주차 실행기."""

    def __init__(
        self,
        target_marker_id: int = TARGET_MARKER_ID,
        camera_index: int = CAMERA_INDEX,
        config: Optional[FrontParkingConfig] = None,
    ) -> None:
        cfg = config or FrontParkingConfig(target_marker_id=target_marker_id)
        cfg.target_marker_id = target_marker_id
        self.controller = FrontParkingController(cfg)
        self.camera_index = camera_index
        self._cap: Optional[cv2.VideoCapture] = None
        self._fallback = _FallbackArucoDetector()
        self._last_marker: Optional[MarkerObservation] = None
        self._last_state: Optional[ParkingState] = None
        self._last_log_t: float = 0.0

    def _ensure_camera(self) -> Optional[cv2.VideoCapture]:
        if self._cap is not None and self._cap.isOpened():
            return self._cap
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            cap.release()
            return None
        self._cap = cap
        print(
            f"[front_parking] 카메라 열림 (index={self.camera_index}, target_id={self.controller.cfg.target_marker_id})",
            flush=True,
        )
        return self._cap

    def release(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def _read_marker(self, now: float) -> Optional[MarkerObservation]:
        cap = self._ensure_camera()
        if cap is None:
            return self._last_marker

        ok, frame = cap.read()
        if not ok or frame is None:
            # 새 프레임 읽기 실패 시 이전 시각(t)을 유지해 600ms 후 MARKER_LOST로 안전 정지
            return self._last_marker

        target_id = self.controller.cfg.target_marker_id
        obs: Optional[MarkerObservation] = None

        if _repo_detect_markers is not None:
            markers = _repo_detect_markers(frame)
            half_w = frame.shape[1] / 2.0
            for m in markers:
                if m.id == target_id:
                    obs = MarkerObservation(
                        marker_id=m.id,
                        side_px=float(m.size_px),
                        offset_px=float(m.err_x * half_w),
                        t=float(m.t),
                    )
                    break
        else:
            obs = self._fallback.detect_target(frame, target_id, now)

        if obs is not None:
            self._last_marker = obs
        return obs

    def step(self, link: BridgeLink) -> ParkingState:
        """qrover_drive의 주기 루프(loop)에서 매번 호출."""
        now = time.time()

        if self.controller.state in (ParkingState.PARKED, ParkingState.SAFETY_STOP):
            link.send_drive(0, 0)
            self.release()
            return self.controller.state

        marker_obs = self._read_marker(now)
        raw_dist = link.latest_distance()
        dist_obs = (
            DistanceObservation(
                dist_mm=raw_dist.dist_mm,
                status=raw_dist.status,
                t=raw_dist.t,
                stale=raw_dist.stale,
            )
            if raw_dist is not None
            else None
        )

        out = self.controller.update(marker_obs, dist_obs, now)
        link.send_drive(out.left, out.right)

        if out.state != self._last_state or (now - self._last_log_t) >= 0.5:
            dist_str = f"{raw_dist.dist_mm}mm(st={raw_dist.status})" if raw_dist else "None"
            side_str = f"{marker_obs.side_px:.1f}px" if marker_obs else "None"
            off_str = f"{marker_obs.offset_px:+.1f}px" if marker_obs else "None"
            print(
                f"[front_parking] state={out.state.value} "
                f"samples={out.parking_samples}/{self.controller.cfg.required_stop_samples} "
                f"marker=({side_str}, {off_str}) tof={dist_str} -> L={out.left} R={out.right}",
                flush=True,
            )
            self._last_state = out.state
            self._last_log_t = now

        return out.state
