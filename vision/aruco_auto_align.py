"""Q-Rover ArUco 제자리 중앙 정렬 단계 (#23 개선안).

기존 ``vision.aruco.ArucoDetector`` 와 ``vision.align.Aligner`` 를 재사용한다.
이 모듈은 *모터에 명령을 전송하지 않는다*. 상태 기계/주행 앱이
``StageResult.left`` 와 ``right`` 를 ``BridgeLink.send_drive`` 에 전달한다.

연결 순서:
    ArucoCamera.start()  # 카메라 읽기는 별도 스레드에서 실행
    stage = ArucoAlignStage(target_id=0)
    stage.reset(now=time.time())
    # 앱의 기존 20Hz loop 에서:
    observation, error = camera.snapshot()
    proposal = stage.step(observation, now=time.time(), camera_error=error)
    link.send_drive(proposal.left, proposal.right)
    link.tick()  # Bridge heartbeat 는 기존 앱이 계속 담당

기본적으로 마커가 없으면 정지한다. 탐색 회전은 명시적으로 켰을 때만
최대 search_sec 동안 제안하며, 실제 구동 전에 회전 방향을 검증해야 한다.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional, Sequence

from vision.align import AlignConfig, Aligner, AlignStatus, MarkerLike


@dataclass(frozen=True)
class MarkerFrame:
    """새로 촬영해 검출한 프레임 한 장 (같은 seq는 같은 프레임)."""

    seq: int
    t: float                 # time.time() 기준 수신/촬영 시각
    markers: Sequence[MarkerLike]
    width_px: Optional[int] = None  # 실제 프레임 폭. 없으면 실행기 설정값을 사용


@dataclass(frozen=True)
class StageConfig:
    timeout_sec: float = 15.0         # TODO: 실험으로 결정
    frame_stale_sec: float = 0.5      # TODO: 실험으로 결정
    camera_ready_timeout_sec: float = 3.0
    search_if_missing: bool = False  # 기본은 마커 미검출 시 정지
    search_direction: str = "right"  # TODO: 후방 카메라/배선 실물 검증
    search_pwm: int = 60             # TODO: MIN_PWM 실측 후 조정
    search_sec: float = 5.0          # TODO: 회전 공간 및 속도 실험 후 조정


@dataclass(frozen=True)
class StageResult:
    left: int                 # Bridge 규약 -255 ~ 255
    right: int
    state: str
    reason: str = ""
    err_x: Optional[float] = None


STOP = (0, 0)


class ArucoAlignStage:
    """관측값 → 바퀴 보정 *제안*. 카메라/Bridge 없이 단위 테스트 가능."""

    TERMINAL = {"ALIGNED", "TIMEOUT", "CAMERA_ERROR"}

    def __init__(
        self,
        target_id: int,
        config: Optional[StageConfig] = None,
        align_config: Optional[AlignConfig] = None,
    ) -> None:
        self.target_id = target_id
        self.cfg = config or StageConfig()
        if self.cfg.timeout_sec <= 0 or self.cfg.frame_stale_sec <= 0:
            raise ValueError("timeout_sec, frame_stale_sec는 양수여야 합니다")
        if self.cfg.search_direction not in ("left", "right"):
            raise ValueError("search_direction은 left/right만 가능합니다")
        if not 0 <= self.cfg.search_pwm <= 255 or self.cfg.search_sec < 0:
            raise ValueError("search_pwm은 0~255, search_sec는 0 이상이어야 합니다")
        # 최소 PWM이 최대 PWM보다 크면 제어 이득을 바꿔도 사실상 일정한
        # 속도로만 회전한다. 실물 캘리브레이션 전에도 설정 모순은 막는다.
        # 이 단계에서만 호환성 있는 PWM 범위를 사용한다.
        # 기존 align.py 의 reverse 동작 설정은 변경하지 않는다.
        cfg = align_config or AlignConfig(kp=100.0, max_turn=100, min_turn_pwm=60)
        if cfg.max_turn < cfg.min_turn_pwm:
            raise ValueError("정렬 최대 회전 PWM은 최소 회전 PWM 이상이어야 합니다")
        self.aligner = Aligner(cfg)
        self.reset()

    def reset(self, now: Optional[float] = None) -> None:
        """정렬 단계에 새로 진입할 때 반드시 호출한다."""
        self.started_at = time.time() if now is None else now
        self.last_seq: Optional[int] = None
        self.has_seen_target = False
        self.finished_state: Optional[str] = None
        self._last = StageResult(*STOP, "WAITING_FOR_CAMERA")
        self.aligner.reset()

    def step(
        self,
        observation: Optional[MarkerFrame],
        now: Optional[float] = None,
        camera_error: Optional[str] = None,
    ) -> StageResult:
        now = time.time() if now is None else now

        if self.finished_state is not None:
            return StageResult(*STOP, self.finished_state)

        if camera_error:
            self.finished_state = "CAMERA_ERROR"
            return StageResult(*STOP, "CAMERA_ERROR", camera_error)

        if now - self.started_at >= self.cfg.timeout_sec:
            self.finished_state = "TIMEOUT"
            return StageResult(*STOP, "TIMEOUT", "정렬 제한 시간 초과")

        if observation is None:
            if now - self.started_at >= self.cfg.camera_ready_timeout_sec:
                self.finished_state = "CAMERA_ERROR"
                return StageResult(*STOP, "CAMERA_ERROR", "카메라 첫 프레임 시간 초과")
            return StageResult(*STOP, "WAITING_FOR_CAMERA")

        if now - observation.t > self.cfg.frame_stale_sec or observation.t > now + 0.1:
            self.aligner.reset()
            self._last = StageResult(*STOP, "CAMERA_STALE", "새 프레임 없음")
            return self._last

        # 20Hz 주행 루프가 같은 카메라 프레임을 여러 번 읽더라도
        # 연속 정렬 프레임 수를 중복 계산하지 않는다.
        if observation.seq == self.last_seq:
            return self._last
        self.last_seq = observation.seq

        target = next((m for m in observation.markers if m.id == self.target_id), None)
        if target is None:
            self.aligner.reset()
            if observation.markers:
                # 다른 칸의 마커가 보인 상황에서는 자동 회전을 제안하지 않는다.
                # 목표 ID가 틀릴 때에는 정지 우선 (AGENTS.md 6장).
                self._last = StageResult(*STOP, "WRONG_MARKER", "목표 ID가 아닌 마커만 검출")
            elif (
                self.cfg.search_if_missing
                and not self.has_seen_target
                and now - self.started_at < self.cfg.search_sec
            ):
                pwm = self.cfg.search_pwm
                sign = 1 if self.cfg.search_direction == "right" else -1
                self._last = StageResult(sign * pwm, -sign * pwm, "SEARCHING")
            else:
                self._last = StageResult(*STOP, "MARKER_MISSING", "목표 ID 검출 안 됨")
            return self._last

        self.has_seen_target = True
        r = self.aligner.step(target, self.target_id, now, mode="align")
        if r.status == AlignStatus.LOST:
            self.aligner.reset()
            self._last = StageResult(*STOP, "MARKER_MISSING", r.reason)
        elif r.status == AlignStatus.ALIGNED:
            self.finished_state = "ALIGNED"
            self._last = StageResult(*STOP, "ALIGNED", err_x=r.err)
        else:
            state = "CENTERING_CHECK" if (r.left, r.right) == STOP else "TURNING"
            self._last = StageResult(r.left, r.right, state, err_x=r.err)
        return self._last

    @property
    def finished(self) -> bool:
        return self.finished_state is not None


class ArucoCamera:
    """App Lab의 20Hz heartbeat 를 막지 않도록 백그라운드에서 프레임을 읽음.

    OpenCV는 start()한 뒤 작업 스레드에서만 불러온다. 보드에 설치되어
    있지 않거나 카메라 접근에 실패하면 error로 알린다(모터 제안 0).
    """

    def __init__(self, camera_index: int = 0) -> None:
        self.camera_index = camera_index
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._frame: Optional[MarkerFrame] = None
        self._error: Optional[str] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._read_loop, daemon=True, name="aruco-camera")
        self._thread.start()

    def snapshot(self) -> tuple[Optional[MarkerFrame], Optional[str]]:
        with self._lock:
            return self._frame, self._error

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=0.5)  # 블로킹 카메라 드라이버는 daemon 스레드로 남을 수 있음

    def _read_loop(self) -> None:
        cap = None
        try:
            import cv2
            from vision.aruco import ArucoDetector

            detector = ArucoDetector()
            cap = cv2.VideoCapture(self.camera_index)
            if not cap.isOpened():
                raise RuntimeError(f"카메라 {self.camera_index}을(를) 열 수 없음")
            seq = 0
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError("카메라 프레임 읽기 실패")
                captured_t = time.time()
                markers = detector.detect_markers(frame)
                seq += 1
                observation = MarkerFrame(seq, captured_t, markers, int(frame.shape[1]))
                with self._lock:
                    self._frame = observation
        except Exception as exc:
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
        finally:
            if cap is not None:
                cap.release()
