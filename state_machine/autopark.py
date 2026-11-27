"""비콘 탐색부터 전면 주차까지 연결하는 상위 상태 기계.

각 하위 모듈은 관측값으로 바퀴 명령을 *제안*하고, 이 클래스가 현재 단계에
맞는 제안 하나만 선택한다. 실제 Bridge 전송은 App Lab 실행기에서 담당한다.

흐름:
    SENSOR_READY -> BEACON_SEARCH -> ARUCO_ALIGN -> FRONT_PARK -> PARKED
    각 동작 단계의 오류/시간 초과 -> ABORT
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence

from state_machine.beacon_search import BeaconSearch, SearchPhase
from state_machine.front_parking import (
    DistanceObservation,
    FrontParkingConfig,
    FrontParkingController,
    MarkerObservation,
    ParkingState,
    RANGE_ERROR,
    RANGE_OK,
)
from vision.aruco_auto_align import ArucoAlignStage, MarkerFrame, StageConfig


class AutoParkPhase(str, Enum):
    SENSOR_READY = "SENSOR_READY"
    BEACON_SEARCH = "BEACON_SEARCH"
    ARUCO_ALIGN = "ARUCO_ALIGN"
    FRONT_PARK = "FRONT_PARK"
    PARKED = "PARKED"
    ABORT = "ABORT"


@dataclass
class AutoParkConfig:
    camera_width_px: float = 640.0
    # 주차칸이 통로 진행 방향 왼쪽이고 전면 주차하는 배치의 기본값.
    # 실제 카메라/모터 방향을 바퀴를 띄운 상태에서 반드시 확인할 것.
    marker_search_direction: str = "left"
    marker_search_pwm: int = 60
    marker_search_sec: float = 6.0
    align_timeout_sec: float = 15.0
    parking_timeout_sec: float = 20.0
    sensor_ready_timeout_sec: float = 5.0
    beacon_wait_timeout_sec: float = 10.0
    beacon_obstacle_stop_mm: int = 120
    parking_marker_hold_sec: float = 0.3
    max_parking_reacquire_attempts: int = 1
    rssi_future_tolerance_sec: float = 0.5


@dataclass(frozen=True)
class AutoParkOutput:
    phase: AutoParkPhase
    left: int
    right: int
    detail: str
    reason: str = ""


@dataclass(frozen=True)
class RssiObservation:
    t: float
    rssi: float


@dataclass(frozen=True)
class SafetyObservation:
    reason: int
    t: float
    stale: bool = False


SAFETY_PASS = 0
SAFETY_NO_COMMAND = 1
SAFETY_WATCHDOG = 2
SAFETY_SENSOR_ERROR = 3
SAFETY_EMERGENCY = 4


class AutoParkController:
    """RSSI, 카메라, 거리 관측값을 받아 전체 자동주차 단계를 진행한다."""

    def __init__(
        self,
        target_slot: int,
        config: Optional[AutoParkConfig] = None,
        beacon_search: Optional[BeaconSearch] = None,
        align_stage: Optional[ArucoAlignStage] = None,
        parking_controller: Optional[FrontParkingController] = None,
    ) -> None:
        self.target_slot = int(target_slot)
        if not 0 <= self.target_slot <= 4:
            raise ValueError("target_slot은 주차칸 범위 0~4여야 합니다")
        self.cfg = config or AutoParkConfig()
        if (
            self.cfg.camera_width_px <= 0
            or self.cfg.parking_timeout_sec <= 0
            or self.cfg.sensor_ready_timeout_sec <= 0
            or self.cfg.beacon_wait_timeout_sec <= 0
            or self.cfg.beacon_obstacle_stop_mm <= 0
            or self.cfg.max_parking_reacquire_attempts < 0
        ):
            raise ValueError("해상도·시간 제한·안전 거리는 양수이고 재시도 횟수는 0 이상이어야 합니다")

        self.beacon_search = beacon_search or BeaconSearch()
        self.align_stage = align_stage or ArucoAlignStage(
            target_id=self.target_slot,
            config=StageConfig(
                timeout_sec=self.cfg.align_timeout_sec,
                search_if_missing=True,
                search_direction=self.cfg.marker_search_direction,
                search_pwm=self.cfg.marker_search_pwm,
                search_sec=self.cfg.marker_search_sec,
                continue_on_wrong_marker=True,
                reacquire_if_lost=True,
            ),
        )
        self.parking_controller = parking_controller or FrontParkingController(
            FrontParkingConfig(target_marker_id=self.target_slot)
        )
        self.reset()

    def reset(self) -> None:
        self.phase = AutoParkPhase.SENSOR_READY
        self.reason = ""
        self.beacon_started = False
        self._beacon_wait_started_at: Optional[float] = None
        self._sensor_wait_started_at: Optional[float] = None
        self._parking_started_at: Optional[float] = None
        self._parking_marker_missing_since: Optional[float] = None
        self._parking_reacquire_attempts = 0
        self.parking_controller.reset()

    def step(
        self,
        now: float,
        target_rssi: Sequence[object] = (),
        marker_frame: Optional[MarkerFrame] = None,
        distance: Optional[DistanceObservation] = None,
        safety: Optional[SafetyObservation] = None,
        camera_error: Optional[str] = None,
        allowed: bool = True,
    ) -> AutoParkOutput:
        if self.phase == AutoParkPhase.PARKED:
            return self._stop("주차 완료")
        if self.phase == AutoParkPhase.ABORT:
            return self._stop("중단", self.reason)
        if safety is not None and safety.stale:
            return self._abort("MCU 안전상태 피드백이 오래됨")
        if distance is not None and (distance.stale or distance.status == RANGE_ERROR):
            return self._abort("거리 센서 오류 또는 오래된 측정값")
        if self.phase == AutoParkPhase.SENSOR_READY:
            return self._step_sensor_ready(now, distance, safety, allowed)
        if safety is None:
            return self._abort("MCU 안전상태 피드백 끊김")
        if safety.reason != SAFETY_PASS:
            return self._abort(self._safety_reason(safety.reason))
        if self.phase == AutoParkPhase.BEACON_SEARCH:
            return self._step_beacon(now, target_rssi, distance)
        if self.phase == AutoParkPhase.ARUCO_ALIGN:
            return self._step_align(now, marker_frame, distance, camera_error)
        return self._step_front_park(now, marker_frame, distance, camera_error)

    def _step_sensor_ready(
        self,
        now: float,
        distance: Optional[DistanceObservation],
        safety: Optional[SafetyObservation],
        allowed: bool,
    ) -> AutoParkOutput:
        if not allowed:
            return self._stop("시작 안전 대기")
        if self._sensor_wait_started_at is None:
            self._sensor_wait_started_at = now
        if now - self._sensor_wait_started_at > self.cfg.sensor_ready_timeout_sec:
            return self._abort("거리 센서 또는 MCU 안전 피드백 준비 시간 초과")
        if distance is None:
            return self._stop("거리 센서 첫 측정 대기")
        if safety is None or safety.reason == SAFETY_NO_COMMAND:
            return self._stop("MCU 안전상태 첫 피드백 대기")
        if safety.reason != SAFETY_PASS:
            return self._abort(self._safety_reason(safety.reason))
        self.phase = AutoParkPhase.BEACON_SEARCH
        self._beacon_wait_started_at = now
        return self._stop("센서 준비 완료, 목표 비콘 신호 대기")

    def _step_beacon(
        self,
        now: float,
        target_rssi: Sequence[object],
        distance: Optional[DistanceObservation],
    ) -> AutoParkOutput:
        if not self.beacon_started:
            if not target_rssi:
                if (
                    self._beacon_wait_started_at is not None
                    and now - self._beacon_wait_started_at > self.cfg.beacon_wait_timeout_sec
                ):
                    return self._abort("목표 비콘 첫 신호 대기 시간 초과")
                return self._stop("목표 비콘 신호 대기")
            self.beacon_search.start(now)
            self.beacon_started = True

        for sample in target_rssi:
            if isinstance(sample, RssiObservation):
                measured_t, rssi = sample.t, sample.rssi
            elif isinstance(sample, tuple) and len(sample) == 2:
                measured_t, rssi = sample
            else:
                measured_t, rssi = now, sample
            if float(measured_t) > now + self.cfg.rssi_future_tolerance_sec:
                return self._abort("현재보다 미래인 RSSI 측정 시각 수신")
            self.beacon_search.on_rssi(float(measured_t), float(rssi))

        out = self.beacon_search.step(now)
        if out.phase == SearchPhase.DONE:
            self.phase = AutoParkPhase.ARUCO_ALIGN
            self.align_stage.reset(now)
            return self._stop("비콘 위치 복귀 완료, 마커 탐색 시작")
        if out.phase == SearchPhase.ABORT:
            return self._abort(f"비콘 탐색 실패: {out.reason}")
        if (
            distance is not None
            and distance.status == RANGE_OK
            and distance.dist_mm <= self.cfg.beacon_obstacle_stop_mm
            and (out.left + out.right) > 0
        ):
            return self._abort("비콘 주행 중 전방 장애물 감지")
        return AutoParkOutput(self.phase, out.left, out.right, out.phase.value, out.reason)

    def _step_align(
        self,
        now: float,
        marker_frame: Optional[MarkerFrame],
        distance: Optional[DistanceObservation],
        camera_error: Optional[str],
    ) -> AutoParkOutput:
        out = self.align_stage.step(marker_frame, now=now, camera_error=camera_error)
        if (
            distance is not None
            and distance.status == RANGE_OK
            and distance.dist_mm <= self.parking_controller.cfg.park_distance_mm
            and (out.left != 0 or out.right != 0)
        ):
            return self._abort("벽과 가까워 정렬 회전을 중단")
        if out.state == "ALIGNED":
            self.phase = AutoParkPhase.FRONT_PARK
            if self._parking_started_at is None:
                self._parking_started_at = now
            self._parking_marker_missing_since = None
            self.parking_controller.reset()
            return self._stop("마커 중앙 정렬 완료, 전면 접근 시작")
        if out.state in ("TIMEOUT", "CAMERA_ERROR"):
            return self._abort(f"ArUco 정렬 실패: {out.reason or out.state}")
        return AutoParkOutput(self.phase, out.left, out.right, out.state, out.reason)

    def _step_front_park(
        self,
        now: float,
        marker_frame: Optional[MarkerFrame],
        distance: Optional[DistanceObservation],
        camera_error: Optional[str],
    ) -> AutoParkOutput:
        if camera_error:
            return self._abort(f"주차 중 카메라 오류: {camera_error}")
        if self._parking_started_at is None:
            self._parking_started_at = now
        if now - self._parking_started_at > self.cfg.parking_timeout_sec:
            return self._abort("전면 주차 시간 초과")

        marker = self._target_marker(marker_frame)
        marker_missing = (
            marker is None
            or (now - marker.t) > self.parking_controller.cfg.marker_timeout_sec
        )
        if marker_missing:
            if self._parking_marker_missing_since is None:
                self._parking_marker_missing_since = now
            missing_for = now - self._parking_marker_missing_since
            if missing_for < self.cfg.parking_marker_hold_sec:
                return self._stop("접근 중 목표 마커 일시 유실")
            far_enough_to_turn = (
                distance is not None
                and (
                    distance.status != RANGE_OK
                    or distance.dist_mm > self.parking_controller.cfg.slowdown_distance_mm
                )
            )
            if (
                far_enough_to_turn
                and self._parking_reacquire_attempts < self.cfg.max_parking_reacquire_attempts
            ):
                self._parking_reacquire_attempts += 1
                self.phase = AutoParkPhase.ARUCO_ALIGN
                self.align_stage.reset(now)
                return self._stop("접근 중 마커 유실, 제한 재탐색 시작")
            return self._abort("접근 중 목표 마커 재탐색 불가")

        self._parking_marker_missing_since = None
        out = self.parking_controller.update(marker, distance, now)
        if out.state == ParkingState.PARKED:
            self.phase = AutoParkPhase.PARKED
            return self._stop("주차 완료")
        if out.state == ParkingState.SAFETY_STOP:
            return self._abort("전면 주차 안전 정지")
        return AutoParkOutput(self.phase, out.left, out.right, out.state.value)

    def _target_marker(self, frame: Optional[MarkerFrame]) -> Optional[MarkerObservation]:
        if frame is None:
            return None
        target = next((m for m in frame.markers if m.id == self.target_slot), None)
        if target is None or not hasattr(target, "size_px"):
            return None
        width_px = frame.width_px or self.cfg.camera_width_px
        return MarkerObservation(
            marker_id=target.id,
            side_px=float(target.size_px),
            offset_px=float(target.err_x) * width_px / 2.0,
            t=float(target.t),
        )

    def _abort(self, reason: str) -> AutoParkOutput:
        self.phase = AutoParkPhase.ABORT
        self.reason = reason
        return self._stop("중단", reason)

    def abort(self, reason: str = "사용자 취소") -> AutoParkOutput:
        return self._abort(reason)

    def set_target_slot(self, target_slot: int) -> None:
        if not 0 <= int(target_slot) <= 4:
            raise ValueError("target_slot은 주차칸 범위 0~4여야 합니다")
        self.target_slot = int(target_slot)
        self.align_stage.target_id = self.target_slot
        self.parking_controller.cfg.target_marker_id = self.target_slot

    @staticmethod
    def _safety_reason(reason: int) -> str:
        names = {
            SAFETY_NO_COMMAND: "MCU가 아직 주행 명령을 받지 못함",
            SAFETY_WATCHDOG: "MCU watchdog 정지",
            SAFETY_SENSOR_ERROR: "MCU 거리 센서 오류 정지",
            SAFETY_EMERGENCY: "MCU 전방 비상정지",
        }
        return names.get(reason, f"알 수 없는 MCU 안전상태({reason})")

    def _stop(self, detail: str, reason: str = "") -> AutoParkOutput:
        return AutoParkOutput(self.phase, 0, 0, detail, reason)

    @property
    def terminal(self) -> bool:
        return self.phase in (AutoParkPhase.PARKED, AutoParkPhase.ABORT)
