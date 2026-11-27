"""비콘 탐색부터 전면 주차까지 연결하는 상위 상태 기계.

각 하위 모듈은 관측값으로 바퀴 명령을 *제안*하고, 이 클래스가 현재 단계에
맞는 제안 하나만 선택한다. 실제 Bridge 전송은 App Lab 실행기에서 담당한다.

흐름:
    SENSOR_READY -> BEACON_SEARCH -> ARUCO_ALIGN -> FRONT_PARK -> PARKED
    각 동작 단계의 오류/시간 초과 -> ABORT

자세 확인과 재정렬 (기울기 감지, vision/pose.py):
    FRONT_PARK 중 마커가 확인 지점 크기(pose_check_side_px)가 되면 멈추고 POSE_CHECK
      -> 기울기·옆 오차가 허용 범위 안이면 FRONT_PARK 계속
      -> 넘으면 REALIGN: 뒤로 빠짐 -> 통로 방향으로 회전 -> 옆 오차만큼 직진/후진
                        -> ARUCO_ALIGN(다시 마커 찾기) -> FRONT_PARK -> POSE_CHECK ...
    정지 위치에서 기울어져 멈춘 경우(MISALIGNED_NEAR)도 같은 REALIGN으로 다시 시도한다.
    재정렬은 max_realign_attempts번까지, 넘으면 ABORT.

재탐색 (목표 ID 마커를 못 찾음):
    ARUCO_ALIGN에서 marker_search_sec 동안 돌아도 목표 ID가 안 보이면
      -> 통로 방향으로 되돌아 회전 -> 칸 단위로 앞/뒤 이동 -> ARUCO_ALIGN 다시
    다른 번호가 보였고 marker_order(실제 배치 순서)가 있으면 그 차이만큼, 없으면 앞으로 한 칸.
    max_research_attempts번까지, 넘으면 ABORT.
    엔코더가 없어 이동·회전량은 시간으로 정하고, 매번 카메라로 다시 확인해서 오차를 줄인다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from statistics import median
from typing import Optional, Sequence

from state_machine.beacon_search import BeaconSearch, SearchConfig, SearchPhase
from state_machine.front_parking import (
    DistanceObservation,
    FrontParkingConfig,
    FrontParkingController,
    MarkerObservation,
    ParkingState,
    RANGE_ERROR,
    RANGE_OK,
    STOP_MISALIGNED_NEAR,
)
from vision.align import AlignConfig
from vision.aruco_auto_align import ArucoAlignStage, MarkerFrame, StageConfig
from vision.pose import DEFAULT_HFOV_DEG, DEFAULT_MARKER_MM, Pose, PoseEstimator


class AutoParkPhase(str, Enum):
    SENSOR_READY = "SENSOR_READY"
    BEACON_SEARCH = "BEACON_SEARCH"
    ARUCO_ALIGN = "ARUCO_ALIGN"
    FRONT_PARK = "FRONT_PARK"
    POSE_CHECK = "POSE_CHECK"
    REALIGN = "REALIGN"
    PARKED = "PARKED"
    ABORT = "ABORT"


@dataclass
class AutoParkConfig:
    camera_width_px: float = 640.0
    # 주차칸이 통로 진행 방향 왼쪽이고 전면 주차하는 배치의 기본값.
    # 실제 카메라/모터 방향을 바퀴를 띄운 상태에서 반드시 확인할 것.
    marker_search_direction: str = "left"
    # 2026-11-27 실주행: 탐색 회전 60은 너무 빨라 마커를 못 잡고 정렬 시간 초과 → 회전 속도 절반.
    # 느려진 만큼 탐색·정렬 제한 시간을 늘렸다. 30이 바닥에서 안 돌면 올릴 것 (TODO: MIN_PWM 실측)
    marker_search_pwm: int = 30                 # (방향만 쓴다. 실제 세기는 search_pulse_pwm)
    marker_search_sec: float = 3.5              # (예전 연속 회전용, 지금은 search_max_pulses가 정한다)
    # 탐색·정렬 회전은 모두 "짧게 돌고 → 멈춰서 보기"를 반복한다 (2026-11-27 실주행).
    # 계속 돌리면 각도가 매번 달랐고, 카메라가 늦어 마커를 지나쳤다.
    # 돈 것은 하나하나 기록해서, 원상복구할 때 거꾸로 똑같이 되돌린다.
    search_pulse_pwm: int = 50                  # 한 번 돌 때 세기 TODO: 실측
    search_pulse_on_sec: float = 0.1            # 한 번 크게 도는 시간 TODO: 실측
    align_fine_on_sec: float = 0.05             # 마커가 거의 가운데일 때 조금 도는 시간 TODO: 실측
    align_fine_err: float = 0.25                # |err_x|가 이 이하이면 조금만 돈다
    search_pulse_off_sec: float = 1.0           # 멈춰서 카메라로 보는 시간
    search_max_pulses: int = 20                 # 이만큼 돌아도 목표 ID가 없으면 재탐색 TODO: 실측(각도)
    replay_pause_sec: float = 0.3               # 원상복구 때 한 번 돌고 쉬는 시간
    align_timeout_sec: float = 60.0
    # 재탐색: 목표 ID를 못 찾았을 때 몇 칸 앞/뒤로 갈지 정하는 정보
    # marker_order = 진행 방향으로 지나가는 실제 마커 순서 (예: (2, 3, 4, 0, 1)). 비어 있으면
    # 다른 번호를 봐도 방향을 알 수 없으니 앞으로 한 칸씩 간다.
    marker_order: tuple = (1, 2, 3, 0, 4)       # 2026-11-27 시험장: 별표 출발 기준 지나가는 순서
    slot_pitch_mm: float = 170.0                # 시험장 마커 중심 간 실측 거리
    max_research_attempts: int = 4
    # 마커를 화면 가운데로 맞추는 회전 (P제어). 예전 kp 100 / 최대 100 / 최소 60의 절반
    align_kp: float = 50.0
    align_max_turn: int = 50
    align_min_turn_pwm: int = 30
    parking_timeout_sec: float = 20.0
    sensor_ready_timeout_sec: float = 5.0
    beacon_wait_timeout_sec: float = 10.0
    beacon_obstacle_stop_mm: int = 120
    parking_marker_hold_sec: float = 0.3
    max_parking_reacquire_attempts: int = 1
    rssi_future_tolerance_sec: float = 0.5

    # 비콘 직진. 2026-11-27: 55로 낮췄더니 후진이 거의 안 됐다(모터가 걸림) → 70으로 앞뒤 같게.
    # 시뮬레이션(벽까지 50cm, 채널 섞임)에서는 속도 50~80 사이 정확도 차이가 거의 없었다.
    # 비콘은 "근처까지"만 데려오고, 정확한 위치는 아래 90° 보기 + 앞뒤 보정이 맞춘다.
    beacon_search_pwm: int = 70                 # TODO: 실측
    beacon_approach_pwm: int = 70               # 감속 없음 (후진도 이 세기로 한다)
    beacon_return_gain: float = 1.25            # 후진이 덜 돌아와서 늘림 TODO: 실측
    beacon_search_timeout_sec: float = 45.0
    # 2026-11-27 실주행: 목표 근처에 가면 -69가 찍히기 시작 → 최근 1.5초에 4번 이상이면 도착으로 본다
    beacon_arrive_rssi: Optional[float] = -69.0   # TODO: 실측 (None이면 끔)
    beacon_arrive_count: int = 4

    # 마커 정렬 방식
    #   "look90": 벽 쪽으로 딱 90° 돌아서 보고 → 벽(마커) 기준으로 방향을 바로잡고
    #             → 옆으로 벗어난 만큼 통로를 따라 앞/뒤로 이동 (기본)
    #   "sweep" : 돌면서 찾고 가운데로 맞추기 (예전 방식)
    align_mode: str = "look90"
    pulses_per_90: int = 18                     # 왼쪽 90° 끊어 돌기 (시험 바닥 기준)
    right_pulses_per_90: int = 17               # 오른쪽 90° 끊어 돌기 (시험 바닥 기준)
    turn_pause_sec: float = 0.3                 # 90° 돌 때 한 번 돌고 쉬는 시간
    look_sec: float = 1.5                       # 멈춰서 마커를 보는 시간 (이 동안의 평균으로 판단)
    look_min_frames: int = 3                    # 자세를 믿으려면 최소 이만큼 봐야 함
    look_frame_wait_sec: float = 3.0            # 보는 동안 새 영상이 하나도 없으면 이만큼 더 기다리고 중단
    look_frame_max_age_sec: float = 1.0         # 이보다 오래된 영상은 안 본 것으로 친다
    wall_yaw_tol_deg: float = 6.0               # 벽과 이 이상 틀어지면 조금 돌아서 바로잡음
    approach_lateral_tol_mm: float = 50.0      # 전면 주차 접근을 시작할 때 허용하는 옆 오차
    max_yaw_fixes: int = 5                      # 벽을 볼 때 한 번씩 돌고 확인하는 최대 횟수
    max_look_moves: int = 6                     # 앞뒤로 옮겨서 다시 보기 최대 횟수
    peek_pulses: int = 3                        # 아무것도 안 보이면 좌우로 이만큼(약 15°) 돌아서 더 본다

    # 자세 추정 (vision/pose.py)
    camera_hfov_deg: float = DEFAULT_HFOV_DEG   # TODO: 카메라 보정으로 교체
    marker_mm: float = DEFAULT_MARKER_MM        # 마커 검은 부분 한 변 (5cm)
    # 자세 확인: 마커가 이 크기가 되면(5cm 마커·화각 45° 기준 약 26cm) 멈추고 기울기를 확인한다.
    # 멀면 기울기 값이 ±8~11°씩 흔들려서(2026-11-24 실측) 가까이에서 평균으로 본다.
    pose_check_side_px: float = 150.0           # TODO: 실험으로 결정
    pose_check_frames: int = 10                 # 평균 낼 새 프레임 수
    pose_check_timeout_sec: float = 3.0         # 이 안에 다 못 모으면 중단
    # 재정렬 동작 (엔코더 없음 → 시간 기반). 허용 기울기·옆 오차는 FrontParkingConfig 값을 쓴다.
    max_realign_attempts: int = 3
    realign_pwm: int = 80                       # 후진·옆 이동 속도 TODO: 실험으로 결정 (MIN_PWM 이상)
    realign_backup_sec: float = 0.8             # 벽에서 떨어지는 후진 시간 TODO: 실측
    realign_turn_pwm: int = 40                  # 회전 속도 (예전 80의 절반, 2026-11-27)
    realign_turn_sec: float = 1.8               # realign_turn_pwm으로 약 90° 회전 시간 TODO: 실측
    realign_mm_per_sec: float = 138.0           # PWM 80 전진 1초 실측
    realign_reverse_mm_per_sec: float = 153.0   # PWM 80 후진 1초 실측
    realign_shift_min_sec: float = 0.2
    realign_shift_max_sec: float = 3.0


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

        self.beacon_search = beacon_search or BeaconSearch(
            config=SearchConfig(
                search_pwm=self.cfg.beacon_search_pwm,
                approach_pwm=self.cfg.beacon_approach_pwm,
                approach_rssi=None,
                return_gain=self.cfg.beacon_return_gain,
                search_timeout=self.cfg.beacon_search_timeout_sec,
                arrive_rssi=self.cfg.beacon_arrive_rssi,
                arrive_count=self.cfg.beacon_arrive_count,
            )
        )
        self.align_stage = align_stage or ArucoAlignStage(
            target_id=self.target_slot,
            config=StageConfig(
                timeout_sec=self.cfg.align_timeout_sec,
                search_if_missing=True,
                search_direction=self.cfg.marker_search_direction,
                search_pwm=self.cfg.marker_search_pwm,
                search_sec=self.cfg.align_timeout_sec,   # 탐색 종료는 끊어 돌기 횟수로 정한다
                continue_on_wrong_marker=True,
                reacquire_if_lost=True,
            ),
            align_config=AlignConfig(
                kp=self.cfg.align_kp,
                max_turn=self.cfg.align_max_turn,
                min_turn_pwm=self.cfg.align_min_turn_pwm,
            ),
        )
        self.parking_controller = parking_controller or FrontParkingController(
            FrontParkingConfig(target_marker_id=self.target_slot)
        )
        if self.cfg.max_realign_attempts < 0 or self.cfg.pose_check_frames < 1:
            raise ValueError("재정렬 횟수는 0 이상, 자세 확인 프레임은 1 이상이어야 합니다")
        self.pose_estimator = PoseEstimator(
            marker_mm=self.cfg.marker_mm, hfov_deg=self.cfg.camera_hfov_deg
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
        self._pose_checked = False
        self._pose_check_started_at: Optional[float] = None
        self._pose_samples: list[tuple[float, float]] = []   # (yaw_deg, lateral_mm)
        self._pose_last_seq: Optional[int] = None
        self._pose_frames_seen = 0
        self._pose_target_seen = 0
        self._pose_reacquire_used = False
        self._target_frame_seq: Optional[int] = None
        self._target_bearing: Optional[float] = None
        self._turn_since_target = 0
        self._target_recovery: Optional[tuple[int, int]] = None  # 첫 방향, 회전 횟수
        self.realign_attempts = 0
        self.research_attempts = 0
        self._seen_other_ids: list[int] = []   # 정렬 단계에서 본 다른 번호 마커 (최근 순)
        self._seen_sizes: dict[int, float] = {}  # 번호별 가장 크게 보인 크기(px) = 가까운 정도
        self.search_pulses = 0                  # 이번 탐색에서 찾으며 돈 횟수
        self._pulse_log: list[tuple[int, float]] = []   # 돈 기록 (방향 +1 오른쪽/-1 왼쪽, 시간)
        self._pulse_owner: Optional[float] = None       # 어느 정렬 단계의 기록인지
        self._pulse_sign = 0
        self._pulse_on_until: Optional[float] = None
        self._pulse_off_until: Optional[float] = None
        # 90° 보기
        self._look_stage = "TURN_IN"            # TURN_IN(벽 쪽으로 90°) / LOOK(멈춰서 보기)
        self._after_plan_stage: Optional[str] = None
        self._look_started: Optional[float] = None
        self._look_seq: Optional[int] = None
        self._look_obs: list[tuple[int, float, Optional[float], Optional[float]]] = []  # (id, 크기, yaw, 옆)
        self._look_poses: dict[int, list[Pose]] = {}
        self._look_yaws: list[float] = []
        self._look_target_visible = False
        self._wall_before_move: Optional[tuple[dict[int, list[Pose]], float, float]] = None
        self._look_frames = 0                   # 보는 동안 받은 새 영상 수
        self._yaw_fixes = 0
        self.look_moves = 0
        self.last_look: Optional[str] = None    # 마지막 판단 내용 (로그·테스트용)
        self._peek = 0                          # 안 보일 때 좌우 확인 단계 (0: 안 함, 1: 벽쪽, 2: 반대쪽)
        self._peek_offset = 0                   # 직각에서 벽 쪽(+)으로 더 돈 횟수
        self._realign_plan: list[tuple[float, int, int, str]] = []  # (끝 시각, L, R, 설명)
        self.last_pose: Optional[tuple[float, float]] = None        # 마지막 자세 확인 평균
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
        if self.phase == AutoParkPhase.SENSOR_READY:
            # 출발 전에는 늦거나 오류인 값을 "아직 준비 중"으로 보고 기다린다.
            # 앱 시작 직후 카메라 스레드가 OpenCV를 여는 동안 MCU 신호가 잠깐 밀려
            # 출발도 하기 전에 ABORT된 적이 있다 (2026-11-27). 준비 시간 초과면 그때 중단.
            return self._step_sensor_ready(now, distance, safety, allowed)
        if safety is not None and safety.stale:
            return self._abort("MCU 안전상태 피드백이 오래됨")
        if distance is not None and (distance.stale or distance.status == RANGE_ERROR):
            return self._abort("거리 센서 오류 또는 오래된 측정값")
        if safety is None:
            return self._abort("MCU 안전상태 피드백 끊김")
        if safety.reason != SAFETY_PASS:
            return self._abort(self._safety_reason(safety.reason))
        if self.phase == AutoParkPhase.BEACON_SEARCH:
            return self._step_beacon(now, target_rssi, distance)
        if self.cfg.align_mode == "look90":
            self._remember_target(now, marker_frame)
        if self.phase == AutoParkPhase.ARUCO_ALIGN:
            return self._step_align(now, marker_frame, distance, camera_error)
        if self.phase == AutoParkPhase.POSE_CHECK:
            return self._step_pose_check(now, marker_frame, camera_error, distance)
        if self.phase == AutoParkPhase.REALIGN:
            if self._target_recovery is not None and camera_error:
                return self._abort(f"목표 마커 재탐색 중 카메라 오류: {camera_error}")
            if self._target_recovery is not None and not self._recovery_clearance(distance):
                return self._abort("목표 마커 재탐색 중 거리 확인 불가 또는 벽에 너무 가까움")
            return self._step_realign(now, distance)
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
        if distance is None or distance.stale or distance.status == RANGE_ERROR:
            return self._stop("거리 센서 정상 측정 대기")
        if safety is None or safety.stale or safety.reason == SAFETY_NO_COMMAND:
            return self._stop("MCU 안전상태 피드백 대기")
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
            self._enter_look("TURN_IN")
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
        if self.cfg.align_mode == "look90":
            return self._step_look(now, marker_frame, distance, camera_error)
        out = self.align_stage.step(marker_frame, now=now, camera_error=camera_error)
        if marker_frame is not None:
            for m in marker_frame.markers:
                mid = int(getattr(m, "id", -1))
                # 배경에서 잘못 잡힌 번호(예: 17)는 버린다. 칸 번호는 0~4뿐이다.
                if mid != self.target_slot and 0 <= mid <= 4:
                    if mid in self._seen_other_ids:
                        self._seen_other_ids.remove(mid)
                    self._seen_other_ids.append(mid)
                    size = float(getattr(m, "size_px", 0.0))
                    self._seen_sizes[mid] = max(size, self._seen_sizes.get(mid, 0.0))
        if (
            distance is not None
            and distance.status == RANGE_OK
            and distance.dist_mm <= self.parking_controller.cfg.park_distance_mm
            and (out.left != 0 or out.right != 0)
        ):
            return self._abort("벽과 가까워 정렬 회전을 중단")
        if self._pulse_owner != self.align_stage.started_at:   # 새 정렬 단계 → 기록 새로 시작
            self._pulse_owner = self.align_stage.started_at
            self._pulse_log = []
            self.search_pulses = 0
            self._pulse_on_until = self._pulse_off_until = None
        if out.state in ("TIMEOUT", "MARKER_MISSING"):
            # 못 찾았거나, 봤다가 놓치고 다시 못 찾음 → 원상복구 후 이동해서 다시 찾기
            return self._start_research(now)
        if out.state != "ALIGNED" and out.state != "CAMERA_ERROR":
            return self._align_pulse(now, out)
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
                self._enter_look("LOOK")        # 벽 쪽을 보고 있으니 바로 다시 본다
                return self._stop("접근 중 마커 유실, 제한 재탐색 시작")
            return self._abort("접근 중 목표 마커 재탐색 불가")

        self._parking_marker_missing_since = None

        # 확인 지점에 오면 한 번 멈추고 기울기를 평균으로 확인한다
        if (
            not self._pose_checked
            and marker.has_pose
            and marker.side_px >= self.cfg.pose_check_side_px
        ):
            self.phase = AutoParkPhase.POSE_CHECK
            self._pose_check_started_at = now
            self._pose_samples = []
            self._pose_last_seq = None
            self._pose_frames_seen = 0
            self._pose_target_seen = 0
            return self._stop("확인 지점 도착, 기울기 확인")

        out = self.parking_controller.update(marker, distance, now)
        if out.state == ParkingState.PARKED:
            self.phase = AutoParkPhase.PARKED
            return self._stop("주차 완료")
        if out.state == ParkingState.SAFETY_STOP:
            if (
                self.parking_controller.stop_reason == STOP_MISALIGNED_NEAR
                and marker.has_pose
            ):
                # 벽 가까이에서 기울어짐 → 포기하지 않고 뒤로 빠져 다시 정렬
                return self._start_realign(
                    now, marker.lateral_mm, f"정지 위치에서 기울어짐 ({marker.yaw_deg:+.1f}°)"
                )
            return self._abort(
                f"전면 주차 안전 정지 ({self.parking_controller.stop_reason or '이유 없음'})"
            )
        self._remember_motion(out.left, out.right)
        return AutoParkOutput(self.phase, out.left, out.right, out.state.value)

    # ── 자세 확인 ─────────────────────────────────────────────
    def _step_pose_check(
        self,
        now: float,
        marker_frame: Optional[MarkerFrame],
        camera_error: Optional[str],
        distance: Optional[DistanceObservation] = None,
    ) -> AutoParkOutput:
        if camera_error:
            return self._abort(f"자세 확인 중 카메라 오류: {camera_error}")
        started = self._pose_check_started_at if self._pose_check_started_at is not None else now
        if now - started > self.cfg.pose_check_timeout_sec:
            if (
                self.cfg.align_mode == "look90"
                and not self._pose_reacquire_used
                and self._pose_frames_seen > self._pose_target_seen
                and self._target_bearing is not None
                and marker_frame is not None
                and now - marker_frame.t <= self.cfg.look_frame_max_age_sec
                and not any(m.id == self.target_slot for m in marker_frame.markers)
                and self._recovery_clearance(distance)
            ):
                self._pose_reacquire_used = True
                return self._begin_target_recovery(now)
            return self._abort(f"자세 확인 시간 초과 ({self._pose_check_counts()})")

        if marker_frame is not None and marker_frame.seq != self._pose_last_seq:
            self._pose_last_seq = marker_frame.seq
            self._pose_frames_seen += 1
            self._pose_target_seen += int(any(m.id == self.target_slot for m in marker_frame.markers))
            marker = self._target_marker(marker_frame)
            if marker is not None and marker.has_pose:
                self._pose_samples.append((marker.yaw_deg, marker.lateral_mm))

        n = len(self._pose_samples)
        if n < self.cfg.pose_check_frames:
            return self._stop(f"기울기 확인 중 ({self._pose_check_counts()})")

        yaw = sum(y for y, _ in self._pose_samples) / n
        lateral = sum(x for _, x in self._pose_samples) / n
        self.last_pose = (yaw, lateral)
        pcfg = self.parking_controller.cfg
        if abs(yaw) <= pcfg.tilt_tol_deg and abs(lateral) <= pcfg.lateral_tol_mm:
            self._pose_checked = True
            self.phase = AutoParkPhase.FRONT_PARK
            return self._stop(f"기울기 통과 ({yaw:+.1f}°, 옆 {lateral:+.0f}mm), 주차 계속")
        return self._start_realign(now, lateral, f"기울기 {yaw:+.1f}°, 옆 {lateral:+.0f}mm")

    def _pose_check_counts(self) -> str:
        valid = len(self._pose_samples)
        return (
            f"유효 자세 {valid}/{self.cfg.pose_check_frames}, "
            f"새 영상 {self._pose_frames_seen}장, "
            f"{self.target_slot}번 미검출 {self._pose_frames_seen - self._pose_target_seen}장, "
            f"검출 후 자세 계산 실패 {self._pose_target_seen - valid}장"
        )

    # ── 재정렬 ────────────────────────────────────────────────
    def _start_realign(self, now: float, lateral_mm: float, why: str) -> AutoParkOutput:
        if self.realign_attempts >= self.cfg.max_realign_attempts:
            return self._abort(f"재정렬 {self.cfg.max_realign_attempts}회 넘음: {why}")
        self.realign_attempts += 1
        if self.cfg.align_mode == "look90":
            # 벽에서 뒤로 빠진 뒤, 벽을 본 채로 다시 보기 → 방향·옆 오차를 90° 방식으로 바로잡는다
            back = self.cfg.realign_pwm
            self._realign_plan = [(now + self.cfg.realign_backup_sec, -back, -back, "벽에서 뒤로 빠짐")]
            self._after_plan_stage = "LOOK"
            self.phase = AutoParkPhase.REALIGN
            self._pose_checked = False
            self._parking_started_at = None
            self.parking_controller.reset()
            self.reason = why
            return self._step_realign(now, None)

        pwm = self.cfg.realign_pwm
        # 마커를 찾을 때 돈 방향의 반대로 돌면 원래 진행 방향(통로 방향)을 본다
        to_aisle = 1 if self.cfg.marker_search_direction == "left" else -1   # +1 = 오른쪽 회전
        # 왼쪽 칸 배치에서 벽을 보면 원래 진행 방향은 오른쪽 → lateral +면 이미 칸을 지나친 것
        ahead = lateral_mm * (1 if self.cfg.marker_search_direction == "left" else -1) > 0
        shift_dir = -1 if ahead else 1
        shift_sec = self._realign_travel_sec(shift_dir * abs(lateral_mm))
        shift_sec = min(self.cfg.realign_shift_max_sec, max(self.cfg.realign_shift_min_sec, shift_sec))

        t = now
        plan = []
        t += self.cfg.realign_backup_sec
        plan.append((t, -pwm, -pwm, "벽에서 뒤로 빠짐"))
        t += self.cfg.realign_turn_sec
        turn = self.cfg.realign_turn_pwm
        plan.append((t, to_aisle * turn, -to_aisle * turn, "통로 방향으로 회전"))
        t += shift_sec
        label = "칸을 지나쳐서 후진" if ahead else "칸 앞까지 직진"
        plan.append((t, shift_dir * pwm, shift_dir * pwm, f"{label} ({abs(lateral_mm):.0f}mm)"))
        self._realign_plan = plan

        self.phase = AutoParkPhase.REALIGN
        self._pose_checked = False
        self._parking_started_at = None
        self._parking_marker_missing_since = None
        self.parking_controller.reset()
        self.reason = why
        return self._step_realign(now, None)

    # ── 90° 보기 (look90) ─────────────────────────────────────
    def _enter_look(self, stage: str) -> None:
        self._look_stage = stage
        self._look_started = None
        self._look_seq = None
        self._look_obs = []
        self._look_poses = {}
        self._look_yaws = []
        self._look_target_visible = False
        self._look_frames = 0
        if stage == "TURN_IN":
            self._yaw_fixes = 0
            self._peek = 0
            self._peek_offset = 0

    def _to_wall(self) -> int:
        """벽(칸) 쪽으로 도는 방향. +1 = 오른쪽 회전."""
        return -1 if self.cfg.marker_search_direction == "left" else 1

    def _pulses_for_90(self, sign: int) -> int:
        return self.cfg.right_pulses_per_90 if sign > 0 else self.cfg.pulses_per_90

    def _travel_sign(self) -> int:
        """벽을 볼 때 자세의 옆 오차(+ = 카메라가 오른쪽)를 진행 방향 앞(+)으로 바꾸는 부호."""
        return 1 if self.cfg.marker_search_direction == "left" else -1

    def _remember_target(self, now: float, frame: Optional[MarkerFrame]) -> None:
        if (
            frame is None
            or frame.seq == self._target_frame_seq
            or now - frame.t > self.cfg.look_frame_max_age_sec
        ):
            return
        self._target_frame_seq = frame.seq
        for marker in frame.markers:
            if marker.id == self.target_slot:
                self._target_bearing = getattr(marker, "err_x", 0.0)
                self._turn_since_target = 0
                break

    def _remember_motion(self, left: int, right: int) -> None:
        if left + right:
            # 이동 후의 시야는 이동 전 회전 기록으로 복구할 수 없다.
            self._target_bearing = None
            self._turn_since_target = 0
        elif left and self._target_bearing is not None and not self._turn_since_target:
            self._turn_since_target = 1 if left > 0 else -1

    def _recovery_clearance(self, distance: Optional[DistanceObservation]) -> bool:
        return bool(
            distance is not None
            and distance.status == RANGE_OK
            and distance.dist_mm > self.parking_controller.cfg.park_distance_mm
        )

    def _begin_target_recovery(self, now: float) -> AutoParkOutput:
        along_view = 1 if self._target_bearing > 0 else -1
        direction = -self._turn_since_target or along_view
        self._target_recovery = (direction, 0)
        self.phase = AutoParkPhase.ARUCO_ALIGN
        self._yaw_fixes = 0
        self._enter_look("LOOK")
        return self._stop(f"목표 {self.target_slot}번 유실, 마지막 검출 방향으로 재탐색")

    def _step_target_recovery(self, now: float, distance: Optional[DistanceObservation]) -> AutoParkOutput:
        first_direction, attempts = self._target_recovery
        poses = self._look_poses.get(self.target_slot, [])
        direction = None
        if self._look_target_visible and len(poses) >= self.cfg.look_min_frames:
            yaw = median(p.yaw_deg for p in poses)
            offset = median(p.offset_mm for p in poses)
            self.last_look = f"목표 {self.target_slot}번 재검출 (각도 {yaw:+.1f}°, 옆 {offset:+.0f}mm)"
            if abs(offset) > self.parking_controller.cfg.lateral_tol_mm:
                if not self._recovery_clearance(distance):
                    return self._abort("목표 마커 재검출 후 이동 여유 부족")
                pitch = self.cfg.slot_pitch_mm
                move = max(-pitch, min(pitch, -self._travel_sign() * offset))
                self._target_recovery = None
                self._pose_checked = False
                self._parking_started_at = None
                return self._look_move(now, move, wall_yaw=yaw)
            if abs(yaw) <= self.parking_controller.cfg.tilt_tol_deg:
                self._target_recovery = None
                self._pose_checked = False
                self._parking_started_at = None
                return self._start_front_park(now, self.last_look)
            # 옆 위치가 맞으면 최신 목표 자세로 방향부터 보정하고 다시 확인한다.
            direction = 1 if yaw > 0 else -1
        # 먼저 두 번 되돌아가고, 놓쳤으면 반대쪽도 한 번씩 확인한다.
        if attempts >= 6 or not self._recovery_clearance(distance):
            return self._abort(f"목표 {self.target_slot}번 재탐색 중단 ({attempts}회)")
        direction = direction or (first_direction if attempts < 2 else -first_direction)
        self._target_recovery = (first_direction, attempts + 1)
        plan, _ = self._pulse_plan(now, direction, 1, "목표 마커 재탐색 회전")
        return self._run_plan(now, plan, "LOOK", f"목표 {self.target_slot}번 재탐색 {attempts + 1}/6")

    def _pulse_plan(self, now: float, sign: int, count: int, label: str, t: Optional[float] = None):
        p = self.cfg.search_pulse_pwm
        t = now if t is None else t
        plan = []
        for i in range(count):
            t += self.cfg.search_pulse_on_sec
            plan.append((t, sign * p, -sign * p, f"{label} {i + 1}/{count}"))
            t += self.cfg.turn_pause_sec
            plan.append((t, 0, 0, f"{label} {i + 1}/{count}"))
        return plan, t

    def _run_plan(self, now: float, plan, after: str, why: str) -> AutoParkOutput:
        self._realign_plan = plan
        self._after_plan_stage = after
        self.phase = AutoParkPhase.REALIGN
        self.reason = why
        return self._step_realign(now, None)

    def _step_look(self, now, marker_frame, distance, camera_error) -> AutoParkOutput:
        if camera_error:
            return self._abort(f"ArUco 정렬 실패: {camera_error}")
        if self._look_stage == "TURN_IN":
            sign = self._to_wall()
            plan, _ = self._pulse_plan(now, sign, self._pulses_for_90(sign), "벽 쪽으로 90° 회전")
            return self._run_plan(now, plan, "LOOK", "벽 쪽으로 90° 회전")

        # LOOK: 멈춰서 look_sec 동안 마커를 모은다
        if self._look_started is None:
            self._look_started = now
            self._look_obs = []
            self._look_seq = None
            self._look_frames = 0
        fresh = (
            marker_frame is not None
            and marker_frame.seq != self._look_seq
            and marker_frame.t >= self._look_started
            and now - marker_frame.t <= self.cfg.look_frame_max_age_sec
        )
        if fresh:
            self._look_seq = marker_frame.seq
            self._look_frames += 1
            frame_poses = {}
            for m in marker_frame.markers:
                mid = int(getattr(m, "id", -1))
                if not 0 <= mid <= 4:
                    continue
                yaw = lat = None
                if getattr(m, "corners", None):
                    w = marker_frame.width_px or int(self.cfg.camera_width_px)
                    h = getattr(marker_frame, "height_px", None) or int(w * 3 / 4)
                    try:
                        pose = self.pose_estimator.estimate(m, int(w), int(h))
                    except Exception:
                        pose = None
                    if pose is not None:
                        yaw, lat = pose.yaw_deg, pose.offset_mm
                        frame_poses[mid] = pose
                self._look_obs.append((mid, float(getattr(m, "size_px", 0.0)), yaw, lat))
            for mid, pose in frame_poses.items():
                self._look_poses.setdefault(mid, []).append(pose)
            self._look_target_visible = self.target_slot in frame_poses
            if frame_poses:
                self._look_yaws.append(median(pose.yaw_deg for pose in frame_poses.values()))
        waited = now - self._look_started
        if waited < self.cfg.look_sec:
            return self._stop(f"멈춰서 마커 보기 (영상 {self._look_frames}장, 마커 {len(self._look_obs)}개)")
        if self._look_frames == 0:
            # 영상이 안 들어오는데 "마커가 없다"고 판단하면 엉뚱하게 이동한다 (2026-11-27 실주행)
            if waited < self.cfg.look_sec + self.cfg.look_frame_wait_sec:
                return self._stop("카메라 영상 기다리는 중")
            return self._abort("카메라 영상이 안 들어옴 (마커를 볼 수 없어 멈춤)")
        if self._target_recovery is not None:
            if marker_frame is None or now - marker_frame.t > self.cfg.look_frame_max_age_sec:
                if waited < self.cfg.look_sec + self.cfg.look_frame_wait_sec:
                    return self._stop("목표 재탐색 영상 대기")
                return self._abort("목표 재탐색 중 새 카메라 영상 없음")
            return self._step_target_recovery(now, distance)
        if (
            self._turn_since_target
            and marker_frame is not None
            and now - marker_frame.t <= self.cfg.look_frame_max_age_sec
            and not any(m.id == self.target_slot for m in marker_frame.markers)
            and self._recovery_clearance(distance)
        ):
            return self._begin_target_recovery(now)
        if self._yaw_fixes and len(self._look_yaws) < self.cfg.look_min_frames:
            if waited < self.cfg.look_sec + self.cfg.look_frame_wait_sec:
                return self._stop("보정 회전 후 마커 각도 확인 대기")
            return self._abort("보정 회전 후 마커 각도를 다시 확인할 수 없음")
        return self._decide_look(now)

    def _decide_look(self, now: float) -> AutoParkOutput:
        obs = self._look_obs
        yaws = self._look_yaws

        # 1) 벽과 틀어졌으면 (어느 마커든 벽은 같은 면) 조금 돌아서 바로잡는다
        if len(yaws) >= self.cfg.look_min_frames:
            yaw = sum(yaws) / len(yaws)
            if abs(yaw) > self.cfg.wall_yaw_tol_deg:
                if self._yaw_fixes >= self.cfg.max_yaw_fixes:
                    return self._abort(f"회전 보정 후에도 벽과 {yaw:+.1f}° 틀어짐")
                self._yaw_fixes += 1
                sign = 1 if yaw > 0 else -1          # yaw + = 오른쪽으로 돌아야 똑바름
                self.last_look = f"벽과 {yaw:+.1f}° 틀어짐 → 1번 돌고 다시 확인"
                plan, _ = self._pulse_plan(now, sign, 1, "벽과 방향 맞추기")
                return self._run_plan(now, plan, "LOOK", self.last_look)

        if self._wall_before_move is not None:
            return self._finish_wall_check()

        # 2) 목표 마커가 보이면 옆 오차만 본다
        tgt = [lat for mid, _, _, lat in obs if mid == self.target_slot]
        tgt_lats = [x for x in tgt if x is not None]
        if tgt:
            if len(tgt_lats) < self.cfg.look_min_frames:
                self.last_look = "목표 마커 보임 (자세 정보 부족) → 접근"
                return self._start_front_park(now, self.last_look)
            lat = sum(tgt_lats) / len(tgt_lats)
            if abs(lat) <= self.cfg.approach_lateral_tol_mm:
                self.last_look = f"목표 마커 정면 (옆 {lat:+.0f}mm) → 접근"
                return self._start_front_park(now, self.last_look)
            move = -self._travel_sign() * lat
            self.last_look = f"목표 마커가 옆으로 {lat:+.0f}mm → 통로 따라 {move:+.0f}mm 이동"
            return self._look_move(now, move)

        # 3) 다른 번호만 보이면: 가장 가까운(크게 보인) 마커 기준으로 칸 차이 + 그 마커와의 옆 오차
        others: dict[int, list] = {}
        for mid, size, _, lat in obs:
            others.setdefault(mid, []).append((size, lat))
        order = list(self.cfg.marker_order)
        if others:
            nearest = max(others, key=lambda k: max(s for s, _ in others[k]))
            lats = [x for _, x in others[nearest] if x is not None]
            here = self._travel_sign() * (sum(lats) / len(lats)) if lats else 0.0
            if self.target_slot in order and nearest in order:
                diff = order.index(self.target_slot) - order.index(nearest)
            else:
                diff = 1
            move = diff * self.cfg.slot_pitch_mm - here
            move = max(-self.cfg.slot_pitch_mm, min(self.cfg.slot_pitch_mm, move))
            self.last_look = (f"목표 안 보임, 가장 가까운 {nearest}번 기준 {diff:+d}칸"
                              f" → 통로 따라 {move:+.0f}mm 이동")
            return self._look_move(now, move)

        # 4) 아무것도 안 보이면: 벽을 제대로 보고 있으면 마커가 최소 하나는 보여야 한다
        #    (50cm에서 화면 폭 약 41cm, 마커 간격 17cm). 90°가 빗나갔을 수 있으니
        #    먼저 좌우로 조금씩 돌아서 더 본다. 그래도 없으면 직각으로 돌아온 뒤 앞으로 한 칸.
        k = self.cfg.peek_pulses
        if self._peek == 0:
            self._peek, self._peek_offset = 1, k
            self.last_look = f"마커 안 보임 → 벽 쪽으로 {k}번 더 돌아서 보기"
            plan, _ = self._pulse_plan(now, self._to_wall(), k, "벽 쪽으로 조금 더 돌아 보기")
            return self._run_plan(now, plan, "LOOK", self.last_look)
        if self._peek == 1:
            self._peek, self._peek_offset = 2, -k
            self.last_look = f"마커 안 보임 → 반대쪽으로 {2 * k}번 돌아서 보기"
            plan, _ = self._pulse_plan(now, -self._to_wall(), 2 * k, "반대쪽으로 돌아 보기")
            return self._run_plan(now, plan, "LOOK", self.last_look)
        self.last_look = "좌우로 봐도 마커 안 보임 → 앞으로 한 칸"
        return self._look_move(now, self.cfg.slot_pitch_mm)

    def _finish_wall_check(self) -> AutoParkOutput:
        before, move_mm, sec = self._wall_before_move
        self._wall_before_move = None
        after = {
            mid: samples for mid, samples in self._look_poses.items()
            if len(samples) >= self.cfg.look_min_frames
        }
        command = f"목표 {move_mm:+.0f}mm, 주행 명령 {sec:.2f}초"
        if not before or not after:
            missing = "전후" if not before and not after else "이동 전" if not before else "이동 후"
            return self._stop(f"벽 거리 비교 생략 ({missing} 마커 자세 부족, {command})")

        common = before.keys() & after.keys()
        travel = ""
        if common:
            before_id = after_id = max(sorted(common), key=lambda mid: min(len(before[mid]), len(after[mid])))
            markers = f"같은 {before_id}번"
            start_offset = median(p.offset_mm for p in before[before_id])
            end_offset = median(p.offset_mm for p in after[after_id])
            travel = f", 통로 이동 추정 {(end_offset - start_offset) * self._travel_sign():+.0f}mm"
        else:
            before_id = max(before, key=lambda mid: len(before[mid]))
            after_id = max(after, key=lambda mid: len(after[mid]))
            markers = f"{before_id}번 → {after_id}번, 같은 벽 가정"
        start = median(p.wall_dist_mm for p in before[before_id])
        end = median(p.wall_dist_mm for p in after[after_id])
        return self._stop(
            f"벽 거리 비교: {start:.0f} → {end:.0f}mm "
            f"(변화 {end - start:+.0f}mm, {markers}, {command}{travel})"
        )

    def _realign_travel_sec(self, move_mm: float) -> float:
        speed = self.cfg.realign_mm_per_sec if move_mm >= 0 else self.cfg.realign_reverse_mm_per_sec
        return abs(move_mm) / speed

    def _look_move(self, now: float, move_mm: float, wall_yaw: Optional[float] = None) -> AutoParkOutput:
        if self.look_moves >= self.cfg.max_look_moves:
            return self._abort(f"목표 ID {self.target_slot} 마커 앞으로 {self.look_moves}번 옮겨도 못 맞춤")
        self.look_moves += 1
        to_aisle = -self._to_wall()
        # 좌우 확인으로 직각에서 벗어난 만큼(_peek_offset) 더하거나 빼서 정확히 통로 방향을 본다
        count = max(0, self._pulses_for_90(to_aisle) + self._peek_offset)
        if wall_yaw is not None:
            angle = 90.0 + to_aisle * wall_yaw
            count = max(0, round(angle * self._pulses_for_90(to_aisle) / 90.0))
        self._peek, self._peek_offset = 0, 0
        plan, t = self._pulse_plan(now, to_aisle, count, "통로 방향으로 90° 회전")
        v = self.cfg.realign_pwm * (1 if move_mm > 0 else -1)
        sec = min(6.0, max(self.cfg.realign_shift_min_sec, self._realign_travel_sec(move_mm)))
        before = {
            mid: samples for mid, samples in self._look_poses.items()
            if len(samples) >= self.cfg.look_min_frames
        }
        self._wall_before_move = before, move_mm, sec
        where = "앞으로" if move_mm > 0 else "뒤로"
        plan.append((t + sec, v, v, f"통로 따라 {where} {abs(move_mm):.0f}mm"))
        return self._run_plan(now, plan, "TURN_IN", self.last_look or "")

    def _start_front_park(self, now: float, why: str) -> AutoParkOutput:
        self.phase = AutoParkPhase.FRONT_PARK
        if self._parking_started_at is None:
            self._parking_started_at = now
        self._parking_marker_missing_since = None
        self.parking_controller.reset()
        self.reason = why
        return self._stop("마커 정면 확인, 전면 접근 시작", why)

    # ── 끊어 돌기 (탐색·정렬 공통) ──────────────────────────────
    def _align_pulse(self, now: float, out) -> AutoParkOutput:
        p = self.cfg.search_pulse_pwm
        if self._pulse_on_until is not None and now < self._pulse_on_until:
            s = self._pulse_sign
            return AutoParkOutput(self.phase, s * p, -s * p, out.state, "끊어 돌기")
        if self._pulse_off_until is not None and now < self._pulse_off_until:
            return AutoParkOutput(self.phase, 0, 0, out.state, "멈춰서 마커 확인")
        rotating = out.left != 0 and out.left == -out.right
        if not rotating:
            return AutoParkOutput(self.phase, 0, 0, out.state, out.reason)

        if out.state.startswith("SEARCHING"):
            if self.search_pulses >= self.cfg.search_max_pulses:
                return self._start_research(now)
            self.search_pulses += 1
            on = self.cfg.search_pulse_on_sec
            what = f"찾으며 끊어 돌기 {self.search_pulses}/{self.cfg.search_max_pulses}"
        else:   # TURNING(가운데로 맞추기), REACQUIRING(놓친 쪽으로 다시 찾기)
            err = abs(out.err_x) if out.err_x is not None else 1.0
            fine = err <= self.cfg.align_fine_err
            on = self.cfg.align_fine_on_sec if fine else self.cfg.search_pulse_on_sec
            what = "가운데로 조금 돌기" if fine else "가운데로 끊어 돌기"
        sign = 1 if out.left > 0 else -1        # +1 = 오른쪽
        self._pulse_log.append((sign, on))
        self._pulse_sign = sign
        self._pulse_on_until = now + on
        self._pulse_off_until = now + on + self.cfg.search_pulse_off_sec
        return AutoParkOutput(self.phase, sign * p, -sign * p, out.state, what)

    # ── 재탐색 (목표 ID를 못 찾음) ─────────────────────────────
    def research_slots(self) -> int:
        """몇 칸 이동할지. + = 진행 방향(앞), - = 뒤. 본 마커 번호와 실제 순서로 정한다.

        한 번 돌면 여러 마커가 보이므로, 가장 크게 보인(= 로버와 가장 가까운) 마커를 기준으로
        한다. 2026-11-27 실주행: 4,3,2,1을 다 보고도 마지막에 본 1번 기준으로 3칸을 가 버렸다.
        """
        order = list(self.cfg.marker_order)
        if self.target_slot in order:
            sized = [(size, mid) for mid, size in self._seen_sizes.items() if mid in order]
            if sized:
                nearest = max(sized)[1]
                diff = order.index(self.target_slot) - order.index(nearest)
                if diff != 0:
                    return diff
            for seen in reversed(self._seen_other_ids):   # 크기 정보가 없을 때
                if seen in order:
                    diff = order.index(self.target_slot) - order.index(seen)
                    if diff != 0:
                        return diff
        return 1

    def _start_research(self, now: float) -> AutoParkOutput:
        if self.research_attempts >= self.cfg.max_research_attempts:
            return self._abort(f"목표 ID {self.target_slot} 마커를 {self.research_attempts}번 다시 찾아도 못 찾음")
        self.research_attempts += 1
        slots = self.research_slots()
        seen = list(self._seen_other_ids)
        self._seen_other_ids = []
        self._seen_sizes = {}

        turn = self.cfg.search_pulse_pwm
        move = self.cfg.realign_pwm * (1 if slots > 0 else -1)
        move_sec = self._realign_travel_sec(slots * self.cfg.slot_pitch_mm)
        move_sec = min(6.0, max(self.cfg.realign_shift_min_sec, move_sec))

        t = now
        plan = []
        log = list(reversed(self._pulse_log))
        n = len(log)
        for i, (sign, on) in enumerate(log):   # 돈 것을 거꾸로, 같은 세기·같은 시간으로 되돌린다
            label = f"통로 방향으로 원상복구 {i + 1}/{n}"
            t += on
            plan.append((t, -sign * turn, sign * turn, label))
            t += self.cfg.replay_pause_sec
            plan.append((t, 0, 0, label))
        self._pulse_log = []
        self.search_pulses = 0
        self._pulse_on_until = self._pulse_off_until = None
        t += move_sec
        where = "앞으로" if slots > 0 else "뒤로"
        plan.append((t, move, move, f"{where} {abs(slots)}칸 이동"))
        self._realign_plan = plan

        self.phase = AutoParkPhase.REALIGN
        self._pose_checked = False
        self._parking_started_at = None
        self.parking_controller.reset()
        seen_txt = f", 본 마커 {seen}" if seen else ""
        self.reason = f"목표 ID {self.target_slot} 못 찾음 → 재탐색 {self.research_attempts}회째{seen_txt}"
        return self._step_realign(now, None)

    def _step_realign(
        self, now: float, distance: Optional[DistanceObservation]
    ) -> AutoParkOutput:
        while self._realign_plan and now >= self._realign_plan[0][0]:
            self._realign_plan.pop(0)
        if not self._realign_plan:
            self.phase = AutoParkPhase.ARUCO_ALIGN
            self.align_stage.reset(now)
            self._enter_look(self._after_plan_stage or "TURN_IN")
            self._after_plan_stage = None
            return self._stop("이동 완료, 마커 다시 보기")
        _, left, right, label = self._realign_plan[0]
        if (
            left + right > 0
            and distance is not None
            and distance.status == RANGE_OK
            and distance.dist_mm <= self.cfg.beacon_obstacle_stop_mm
        ):
            return self._abort("재정렬 직진 중 전방 장애물 감지")
        self._remember_motion(left, right)
        return AutoParkOutput(self.phase, left, right, label, self.reason)

    def _target_marker(self, frame: Optional[MarkerFrame]) -> Optional[MarkerObservation]:
        if frame is None:
            return None
        target = next((m for m in frame.markers if m.id == self.target_slot), None)
        if target is None or not hasattr(target, "size_px"):
            return None
        width_px = frame.width_px or self.cfg.camera_width_px
        height_px = getattr(frame, "height_px", None) or width_px * 3 / 4
        yaw = lateral = None
        if getattr(target, "corners", None):
            try:
                pose = self.pose_estimator.estimate(target, int(width_px), int(height_px))
            except Exception:  # OpenCV 계산 실패 → 자세 없이 픽셀 기준으로
                pose = None
            if pose is not None:
                yaw, lateral = pose.yaw_deg, pose.offset_mm
        return MarkerObservation(
            marker_id=target.id,
            side_px=float(target.size_px),
            offset_px=float(target.err_x) * width_px / 2.0,
            t=float(target.t),
            yaw_deg=yaw,
            lateral_mm=lateral,
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
