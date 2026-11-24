"""비콘 탐색 → ArUco 정렬 → 전면 주차를 App Lab I/O와 연결한다."""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Callable, Optional

# App Lab 밖에서 테스트할 때 저장소 공용 모듈을 찾는다. App Lab에서는
# scripts/sync_app_modules.py가 같은 python/ 아래로 복사한 모듈을 사용한다.
_REPO_ROOT = Path(__file__).resolve().parents[3]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from bridge_link import BridgeLink  # noqa: E402
from state_machine.autopark import (  # noqa: E402
    AutoParkController,
    AutoParkOutput,
)
from state_machine.front_parking import DistanceObservation  # noqa: E402
from vision.aruco_auto_align import ArucoCamera  # noqa: E402


class AutoParkRunner:
    """하나의 카메라와 Bridge를 전체 자동주차 상태 기계에 연결한다."""

    def __init__(
        self,
        target_slot: int,
        poll_rssi: Callable[[], list],
        controller: Optional[AutoParkController] = None,
        camera: Optional[ArucoCamera] = None,
        log: Callable[[str], None] = print,
    ) -> None:
        self.target_slot = int(target_slot)
        self.poll_rssi = poll_rssi
        self.controller = controller or AutoParkController(self.target_slot)
        self.camera = camera or ArucoCamera(camera_index=0)
        self.log = log
        self._camera_started = False
        self._last_log_key: Optional[tuple[str, str]] = None

    def step(self, link: BridgeLink, allowed: bool = True) -> AutoParkOutput:
        if not self._camera_started and not self.controller.terminal:
            self.camera.start()
            self._camera_started = True

        now = time.time()
        target_rssi = [
            rssi for slot, rssi in self.poll_rssi() if int(slot) == self.target_slot
        ]
        if self._camera_started:
            marker_frame, camera_error = self.camera.snapshot()
        else:
            marker_frame, camera_error = None, None
        raw_dist = link.latest_distance()
        distance = (
            DistanceObservation(
                dist_mm=raw_dist.dist_mm,
                status=raw_dist.status,
                t=raw_dist.t,
                stale=raw_dist.stale,
            )
            if raw_dist is not None
            else None
        )

        out = self.controller.step(
            now=now,
            target_rssi=target_rssi,
            marker_frame=marker_frame,
            distance=distance,
            camera_error=camera_error,
            allowed=allowed,
        )
        link.send_drive(out.left, out.right)

        log_key = (out.phase.value, out.detail)
        if log_key != self._last_log_key:
            suffix = f" — {out.reason}" if out.reason else ""
            self.log(
                f"[autopark] {out.phase.value}: {out.detail}{suffix} "
                f"-> L={out.left} R={out.right}"
            )
            self._last_log_key = log_key

        if self.controller.terminal:
            self.close()
        return out

    def close(self) -> None:
        if self._camera_started:
            self.camera.close()
            self._camera_started = False
