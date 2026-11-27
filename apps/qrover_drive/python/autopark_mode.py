"""비콘 탐색 → ArUco 정렬 → 전면 주차를 App Lab I/O와 연결한다."""

from __future__ import annotations

from pathlib import Path
import sys
import time
from typing import Callable, Optional

# App Lab 밖에서 테스트할 때 저장소 공용 모듈을 찾는다. App Lab에서는
# scripts/sync_app_modules.py가 같은 python/ 아래로 복사한 모듈을 사용한다.
# App Lab 컨테이너에서는 이 파일이 /app/python/에 있어서 위로 3단계 폴더가 없다
# (parents[3] → IndexError로 앱이 바로 꺼졌음, 2026-11-27). 그때는 복사본을 쓰므로 건너뛴다.
_PARENTS = Path(__file__).resolve().parents
if len(_PARENTS) > 3 and (_PARENTS[3] / "state_machine").is_dir():
    if str(_PARENTS[3]) not in sys.path:
        sys.path.insert(0, str(_PARENTS[3]))

from bridge_link import BridgeLink  # noqa: E402
from state_machine.autopark import (  # noqa: E402
    AutoParkController,
    AutoParkOutput,
    RssiObservation,
    SafetyObservation,
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
        on_status: Optional[Callable[[AutoParkOutput], None]] = None,
        auto_start: bool = True,
        camera_index: Optional[int] = None,
        beacon_slot: Optional[int] = None,
    ) -> None:
        self.target_slot = int(target_slot)
        # 따라갈 비콘의 칸 번호 (송신기 BEACON_MAJOR). 없으면 목표 마커 번호와 같다고 본다.
        self.beacon_slot = self.target_slot if beacon_slot is None else int(beacon_slot)
        self.poll_rssi = poll_rssi
        self.controller = controller or AutoParkController(self.target_slot)
        if camera is None and camera_index is None:
            raise ValueError("camera 또는 camera_index를 지정해야 합니다")
        self.camera = (
            camera
            if camera is not None
            else ArucoCamera(camera_index=int(camera_index))
        )
        self.log = log
        self.on_status = on_status
        self._enabled = auto_start
        self._camera_started = False
        self._last_log_key: Optional[tuple[str, str, str]] = None
        self.last_output: Optional[AutoParkOutput] = None

    def step(self, link: BridgeLink, allowed: bool = True) -> AutoParkOutput:
        if self.controller.terminal:
            detail = "주차 완료" if self.controller.phase.value == "PARKED" else "중단"
            out = AutoParkOutput(
                self.controller.phase, 0, 0, detail, self.controller.reason
            )
            link.send_drive(0, 0)
            self._publish(out)
            return out
        if not self._enabled:
            out = AutoParkOutput(self.controller.phase, 0, 0, "수동 시작 대기")
            link.send_drive(0, 0)
            self._publish(out)
            return out

        if not self._camera_started and not self.controller.terminal:
            self.camera.start()
            self._camera_started = True

        now = time.time()
        target_rssi = []
        for sample in self.poll_rssi():
            if hasattr(sample, "slot_id"):
                slot, rssi, measured_t = sample.slot_id, sample.rssi, sample.t
            else:
                slot, rssi = sample
                measured_t = now
            if int(slot) == self.beacon_slot:
                target_rssi.append(RssiObservation(float(measured_t), float(rssi)))
        if self._camera_started:
            marker_frame, camera_error = self.camera.snapshot()
        else:
            marker_frame, camera_error = None, None
        raw_dist = link.latest_distance()
        raw_safety = link.latest_safety()
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
        safety = (
            SafetyObservation(
                reason=raw_safety.reason,
                t=raw_safety.t,
                stale=raw_safety.stale,
            )
            if raw_safety is not None
            else None
        )

        out = self.controller.step(
            now=now,
            target_rssi=target_rssi,
            marker_frame=marker_frame,
            distance=distance,
            safety=safety,
            camera_error=camera_error,
            allowed=allowed,
        )
        link.send_drive(out.left, out.right)

        self._publish(out)
        self._log_beacon(now)

        if self.controller.terminal:
            self._enabled = False
            self.close()
        return out

    def _log_beacon(self, now: float) -> None:
        """비콘 탐색 중 0.5초마다 평균 계산 상태를 보여 준다."""
        if self.controller.phase.value != "BEACON_SEARCH":
            return
        if now - getattr(self, "_last_beacon_log", 0.0) < 0.5:
            return
        self._last_beacon_log = now
        bs = getattr(self.controller, "beacon_search", None)
        det = getattr(bs, "detector", None)
        if det is None:
            return
        samples = list(getattr(det, "_samples", []))
        last = f"{samples[-1][1]:.0f}" if samples else "-"
        avg = f"{det.filtered:.1f}" if det.filtered is not None else "-"
        peak = f"{det.peak_rssi:.1f}" if det.peak_rssi is not None else "-"
        below = ""
        if det.filtered is not None and det.peak_rssi is not None:
            below = f", 최대보다 {det.peak_rssi - det.filtered:.1f}dB 낮음 (기준 {det.cfg.drop_db:g})"
        self.log(f"[beacon] 최근 {last} | 평균({det.cfg.window_sec:g}초 {len(samples)}개) {avg}"
                 f" | 최대 {peak}{below} | {getattr(bs.phase, 'value', bs.phase)}")

    def _publish(self, out: AutoParkOutput) -> None:
        self.last_output = out
        log_key = (out.phase.value, out.detail, out.reason)
        if log_key != self._last_log_key:
            suffix = f" — {out.reason}" if out.reason else ""
            self.log(
                f"[autopark] {out.phase.value}: {out.detail}{suffix} "
                f"-> L={out.left} R={out.right}"
            )
            self._last_log_key = log_key
            if self.on_status is not None:
                self.on_status(out)

    def start(self) -> None:
        if self.controller.terminal:
            self.controller.reset()
        self._enabled = True

    def cancel(self) -> None:
        self.controller.abort("사용자 취소")
        self._enabled = False
        self.close()

    def reset(self) -> None:
        self.close()
        self.controller.reset()
        self._enabled = False
        self._last_log_key = None
        self.last_output = None

    def set_target_slot(self, target_slot: int) -> None:
        can_change_before_motion = (
            not self.controller.beacon_started
            and self.controller.phase.value in ("SENSOR_READY", "BEACON_SEARCH")
        )
        if self._enabled and not self.controller.terminal and not can_change_before_motion:
            raise RuntimeError("주행 중에는 목표 칸을 바꿀 수 없습니다")
        self.controller.set_target_slot(target_slot)
        self.target_slot = int(target_slot)

    def status(self) -> str:
        if self.last_output is None:
            return f"{self.controller.phase.value}:아직 실행 전"
        return f"{self.last_output.phase.value}:{self.last_output.detail}:{self.last_output.reason}"

    def close(self) -> None:
        if self._camera_started:
            self.camera.close()
            self._camera_started = False
