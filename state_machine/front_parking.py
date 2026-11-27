"""전면 주차 상태 기계 (MPU / Python)

기존 front_parking_uno_q/sketch/sketch.ino 의 판단 로직(ALIGN / APPROACH / PARKED)을
Python(state_machine)으로 옮기고, 리뷰 피드백을 반영한 모듈입니다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

# apps/qrover_drive/python/bridge_link.py 와 동일한 거리 상태 코드
RANGE_OK = 0
RANGE_OUT_OF_RANGE = 1
RANGE_ERROR = 2


class ParkingState(str, Enum):
    WAIT = "WAIT"
    ALIGN = "ALIGN"
    APPROACH = "APPROACH"
    PARKED = "PARKED"
    SAFETY_STOP = "SAFETY_STOP"
    MARKER_LOST = "MARKER_LOST"
    SENSOR_ERROR = "SENSOR_ERROR"


@dataclass
class FrontParkingConfig:
    target_marker_id: int = 0
    # 2026-11-24 camera_view 실측 (마커 5cm, 640x480): "적당한 정지 위치"가 마커 약 348px,
    # 카메라-벽 약 113mm. 멈추면서 밀리는 거리를 생각해 조금 앞(약 130mm)에서 멈추기 시작한다.
    park_distance_mm: int = 130          # ToF 정지 거리 조건 (TODO: 같은 위치의 ToF 값 실측 후 보정)
    emergency_distance_mm: int = 90      # 무조건 안전 정지 거리 (TODO: 실물 보정)
    marker_stop_side_px: float = 300.0   # 640x480·마커 5cm 기준 정지 크기 (약 130mm, TODO: 실측 보정)
    # ToF와 카메라는 붙은 위치가 달라 정지 거리에서 몇 %씩 어긋난다. ToF가 정지 거리 안일 때
    # 마커가 기준 크기의 이 비율 이상이면 "두 센서가 맞다"고 본다. 이보다 작으면 센서 불일치.
    vision_agree_ratio: float = 0.8      # TODO: 실험으로 결정
    center_tolerance_px: float = 45.0    # 화면 중심 허용 오차 (px). 자세 정보가 없을 때만 사용
    # 자세(vision/pose.py)가 있으면 화면 픽셀 대신 칸 기준 기울기·옆 오차로 "가운데 맞음"을 판정한다.
    # 근거리에서는 카메라 장착 오차 때문에 err_x가 크게 나와도(실측 -0.26) 로버는 똑바를 수 있다.
    tilt_tol_deg: float = 10.0           # 허용 기울기 (팀 결정 2026-11-27, ±5~10 검토 → 10)
    lateral_tol_mm: float = 30.0         # 허용 옆 오차 (TODO: 로버 폭·칸 폭으로 계산해 확정)
    slowdown_distance_mm: int = 300      # 감속 시작 거리 (mm)
    drive_speed: int = 115               # 일반 접근 속도 (-255 ~ 255)
    slow_speed: int = 95                 # 근접 감속 속도
    align_speed: int = 90                # 제자리 정렬 회전 속도
    approach_steer_kp: float = 0.5       # 접근 중 offset_px → 좌우 PWM 보정
    approach_max_turn: int = 45          # 접근 중 보정 상한
    pivot_offset_px: float = 120.0       # 이보다 크게 벗어나면 전진하지 않고 제자리 정렬
    marker_timeout_sec: float = 0.6      # 마커 미수신 허용 시간 (600ms)
    required_stop_samples: int = 3       # 정지 조건 연속 충족 횟수
    parking_timeout_sec: float = 20.0    # 마커를 처음 본 뒤 이 시간 안에 못 끝내면 안전 정지


@dataclass
class MarkerObservation:
    marker_id: int
    side_px: float
    offset_px: float   # +면 화면 오른쪽, -면 화면 왼쪽
    t: float           # 프레임 측정 시각 (초)
    # 자세 추정값 (vision/pose.py). 계산 못 하면 None → 픽셀 기준으로 판정
    yaw_deg: Optional[float] = None       # + = 오른쪽으로 돌아야 칸과 똑바름
    lateral_mm: Optional[float] = None    # + = 카메라가 칸 중심선보다 오른쪽

    @property
    def has_pose(self) -> bool:
        return self.yaw_deg is not None and self.lateral_mm is not None


@dataclass
class DistanceObservation:
    dist_mm: int
    status: int        # RANGE_OK(0), RANGE_OUT_OF_RANGE(1), RANGE_ERROR(2)
    t: float           # ToF 수신 시각 (초)
    stale: bool = False


@dataclass
class StepOutput:
    left: int
    right: int
    state: ParkingState
    parking_samples: int


# SAFETY_STOP 이유 (상위 상태 기계가 "다시 정렬"과 "중단"을 구분할 때 쓴다)
STOP_EMERGENCY = "EMERGENCY"            # ToF 비상 거리
STOP_TIMEOUT = "TIMEOUT"                # 주차 시간 초과
STOP_SENSOR_CONFLICT = "SENSOR_CONFLICT"  # 영상과 ToF가 서로 안 맞음
STOP_MISALIGNED_NEAR = "MISALIGNED_NEAR"  # 벽 가까이에서 기울거나 옆으로 벗어남 → 뒤로 빠져 재정렬 가능


class FrontParkingController:
    def __init__(self, config: Optional[FrontParkingConfig] = None) -> None:
        self.cfg = config or FrontParkingConfig()
        self.reset()

    def reset(self) -> None:
        self.state: ParkingState = ParkingState.WAIT
        self.stop_reason: str = ""
        self.parking_samples: int = 0
        self._ever_received_marker: bool = False
        self._active_started_at: Optional[float] = None
        self._last_counted_marker_t: Optional[float] = None
        self._last_counted_dist_t: Optional[float] = None

    def update(
        self,
        marker: Optional[MarkerObservation],
        distance: Optional[DistanceObservation],
        now: float,
    ) -> StepOutput:
        # 1. 완료(PARKED) 또는 안전 래치(SAFETY_STOP) 상태는 리셋 전까지 정지 유지
        if self.state in (ParkingState.PARKED, ParkingState.SAFETY_STOP):
            return StepOutput(0, 0, self.state, self.parking_samples)

        # 2. 거리 센서(ToF) 상태 확인 (0: OK, 1: 범위 밖, 2: 오류)
        if distance is None:
            self.state = ParkingState.WAIT
            return StepOutput(0, 0, self.state, self.parking_samples)

        if distance.stale or distance.status == RANGE_ERROR:
            self.state = ParkingState.SENSOR_ERROR
            self.parking_samples = 0
            return StepOutput(0, 0, self.state, self.parking_samples)

        tof_ok = distance.status == RANGE_OK
        if tof_ok and distance.dist_mm <= self.cfg.emergency_distance_mm:
            return self._safety_stop(STOP_EMERGENCY)

        # 3. 마커 수신 및 타임아웃 확인
        if marker is not None and marker.marker_id == self.cfg.target_marker_id:
            self._ever_received_marker = True
            if self._active_started_at is None:
                self._active_started_at = now

        if not self._ever_received_marker:
            self.state = ParkingState.WAIT
            return StepOutput(0, 0, self.state, self.parking_samples)

        if (
            self._active_started_at is not None
            and now - self._active_started_at > self.cfg.parking_timeout_sec
        ):
            return self._safety_stop(STOP_TIMEOUT)

        if (
            marker is None
            or marker.marker_id != self.cfg.target_marker_id
            or (now - marker.t) > self.cfg.marker_timeout_sec
        ):
            self.state = ParkingState.MARKER_LOST
            self.parking_samples = 0
            self._last_counted_marker_t = None
            self._last_counted_dist_t = None
            return StepOutput(0, 0, self.state, self.parking_samples)

        # 4. 정렬 / 마커 크기 / ToF 거리 조건 판정
        centered = self.is_aligned(marker)
        vision_close = marker.side_px >= self.cfg.marker_stop_side_px
        vision_agrees = marker.side_px >= self.cfg.marker_stop_side_px * self.cfg.vision_agree_ratio
        # RANGE_OUT_OF_RANGE(1)일 때는 tof_ok == False 이므로 tof_close = False (정상 주행 계속)
        tof_close = tof_ok and (distance.dist_mm <= self.cfg.park_distance_mm)

        # 5. 정지 조건 만족 시: 즉시 정지하고 새 프레임 + 새 ToF 값일 때만 카운트
        if centered and vision_agrees and tof_close:
            has_new_marker = marker.t != self._last_counted_marker_t
            has_new_dist = distance.t != self._last_counted_dist_t
            if has_new_marker and has_new_dist:
                self.parking_samples += 1
                self._last_counted_marker_t = marker.t
                self._last_counted_dist_t = distance.t

            if self.parking_samples >= self.cfg.required_stop_samples:
                self.state = ParkingState.PARKED
            else:
                self.state = ParkingState.APPROACH
            return StepOutput(0, 0, self.state, self.parking_samples)

        # 조건이 깨지면 연속 카운트 초기화
        self.parking_samples = 0
        self._last_counted_marker_t = None
        self._last_counted_dist_t = None

        # 6. 두 센서가 서로 모순되거나 이미 벽 가까이에서 정렬이 틀어졌으면 정지한다.
        # 근거리 제자리 회전은 로버 모서리가 벽에 닿을 수 있고, MCU 거리 안전장치도
        # 제자리 회전에는 적용되지 않으므로 여기서 허용하지 않는다.
        if vision_close and not tof_ok:
            return self._safety_stop(STOP_SENSOR_CONFLICT)

        if tof_close and vision_agrees and not centered:
            return self._safety_stop(STOP_MISALIGNED_NEAR)

        # ToF만 정지 거리 안인데 마커가 훨씬 작으면 안전 정지.
        # 단 기울어져 있으면 비스듬히 봐서 작게 보인 것일 수 있으니 재정렬 대상으로 본다.
        if tof_close and not vision_agrees:
            if marker.has_pose and not centered:
                return self._safety_stop(STOP_MISALIGNED_NEAR)
            return self._safety_stop(STOP_SENSOR_CONFLICT)

        # 7. 중심 정렬(ALIGN) 또는 전진 접근(APPROACH) 명령 생성
        slowdown = (
            (tof_ok and distance.dist_mm <= self.cfg.slowdown_distance_mm)
            or (marker.side_px >= self.cfg.marker_stop_side_px * 0.75)
        )
        speed = self.cfg.slow_speed if slowdown else self.cfg.drive_speed

        if abs(marker.offset_px) > self.cfg.pivot_offset_px:
            if tof_ok and distance.dist_mm <= self.cfg.slowdown_distance_mm:
                return self._safety_stop(STOP_MISALIGNED_NEAR)
            self.state = ParkingState.ALIGN
            if marker.offset_px > 0:
                # 마커가 화면 오른쪽에 있음 -> 시계 방향(우회전)
                return StepOutput(self.cfg.align_speed, -self.cfg.align_speed, self.state, 0)
            else:
                # 마커가 화면 왼쪽에 있음 -> 반시계 방향(좌회전)
                return StepOutput(-self.cfg.align_speed, self.cfg.align_speed, self.state, 0)

        self.state = ParkingState.APPROACH
        turn = int(round(self.cfg.approach_steer_kp * marker.offset_px))
        turn = max(-self.cfg.approach_max_turn, min(self.cfg.approach_max_turn, turn))
        return StepOutput(_clamp_pwm(speed + turn), _clamp_pwm(speed - turn), self.state, 0)

    def is_aligned(self, marker: MarkerObservation) -> bool:
        """칸과 똑바로 마주 보는지. 자세가 있으면 기울기·옆 오차, 없으면 화면 픽셀로 본다."""
        if marker.has_pose:
            return (
                abs(marker.yaw_deg) <= self.cfg.tilt_tol_deg
                and abs(marker.lateral_mm) <= self.cfg.lateral_tol_mm
            )
        return abs(marker.offset_px) <= self.cfg.center_tolerance_px

    def _safety_stop(self, reason: str) -> StepOutput:
        self.state = ParkingState.SAFETY_STOP
        self.stop_reason = reason
        self.parking_samples = 0
        return StepOutput(0, 0, self.state, self.parking_samples)


def _clamp_pwm(value: int) -> int:
    return max(-255, min(255, int(value)))
