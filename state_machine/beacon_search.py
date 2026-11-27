"""주행 흐름 1단계: 비콘으로 목표 칸 찾기 (탐색 → 접근 → 복귀)

기본 주행 동작(직진·후진·정지)을 조합해서 "RSSI가 가장 셌던 지점"으로 돌아간다.

    SEARCH    통로를 search_pwm으로 직진하며 RSSI 수집
    APPROACH  RSSI가 approach_rssi 이상이면 approach_pwm으로 감속 (덜 지나치게)
    RETURN    "지나쳤다"가 확정되면, 최댓값 이후 보낸 명령을 거꾸로 같은 시간만큼 실행
    DONE      복귀 끝, 정지 → 다음 단계(회전하며 ArUco 정렬)로
    ABORT     시간 초과 / 비콘 신호 끊김 → 정지

엔코더가 없어서 거리를 모르므로, "같은 PWM으로 같은 시간 반대로 가면 제자리"라고 가정한다.
중간에 속도가 바뀌어도 구간별로 순서를 뒤집어 실행하면 되므로 cm/s 측정값이 없어도 동작한다.
(출발 보정·가속 차이 때문에 오차가 생긴다 → 정확한 위치는 다음 단계 ArUco가 맞춘다)

이 모듈은 바퀴 값만 계산한다. 실제 전송은 Bridge 연결 모듈(#19, apps/qrover_drive)이 한다.

    search = BeaconSearch()
    search.start(now)
    search.on_rssi(t, rssi)        # RSSI를 받을 때마다
    out = search.step(now)         # 주기 루프에서 매번 → out.left, out.right를 drive로 전송
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

from beacon.peak_detector import PeakDetector


class SearchPhase(str, Enum):
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    RETURN = "RETURN"
    DONE = "DONE"
    ABORT = "ABORT"


@dataclass
class SearchConfig:
    search_pwm: int = 100           # 탐색 직진 속도          TODO: 실험으로 결정 (MIN_PWM 이상)
    approach_pwm: int = 80          # 접근 감속 속도          TODO: 실험으로 결정
    approach_rssi: Optional[float] = -76.0  # 이 이상이면 감속 (실측: 칸 조금 전 약 -78, 칸 앞 약 -72). None이면 감속 안 함   TODO: 실측 확정
    search_timeout: float = 30.0    # 출발 후 이 시간 안에 못 찾으면 중단 (#7 SEARCH_TIMEOUT)   TODO
    return_timeout: float = 10.0    # 복귀 시간이 이보다 길면 이상한 것 → 중단 (#7 RETURN_TIMEOUT) TODO
    trim_left: int = 0              # 직진할 때 휘는 것 보정 (#9 TRIM)   TODO: 실험으로 결정
    trim_right: int = 0


@dataclass
class SearchOutput:
    phase: SearchPhase
    left: int
    right: int
    reason: str = ""


class BeaconSearch:
    def __init__(self, detector: Optional[PeakDetector] = None,
                 config: Optional[SearchConfig] = None) -> None:
        self.detector = detector or PeakDetector()
        self.cfg = config or SearchConfig()
        self.phase = SearchPhase.IDLE
        self.reason = ""
        self._start_t = 0.0
        self._segments: List[Tuple[float, int]] = []   # (시작 시각, pwm) — 직진 명령 기록
        self._return_plan: List[Tuple[float, int]] = []  # (끝나는 시각, pwm)
        self.return_duration = 0.0

    # ── 입력 ─────────────────────────────────────────
    def start(self, now: float) -> None:
        self.detector.reset()
        self.phase = SearchPhase.SEARCH
        self.reason = ""
        self._start_t = now
        self._segments = []
        self._return_plan = []
        self.return_duration = 0.0

    def on_rssi(self, t: float, rssi: float) -> None:
        if self.phase in (SearchPhase.SEARCH, SearchPhase.APPROACH):
            self.detector.update(t, rssi)

    # ── 주기 루프 ────────────────────────────────────
    def step(self, now: float) -> SearchOutput:
        if self.phase in (SearchPhase.SEARCH, SearchPhase.APPROACH):
            return self._step_search(now)
        if self.phase == SearchPhase.RETURN:
            return self._step_return(now)
        return SearchOutput(self.phase, 0, 0, self.reason)  # IDLE / DONE / ABORT → 정지

    def _step_search(self, now: float) -> SearchOutput:
        det = self.detector
        if det.passed:
            self._begin_return(now)
            return self._step_return(now)
        if now - self._start_t > self.cfg.search_timeout:
            return self._abort("탐색 시간 초과 (최댓값을 못 찾음)")
        if det.signal_lost(now):
            return self._abort("비콘 신호 끊김")

        approach = (self.cfg.approach_rssi is not None and det.filtered is not None
                    and det.filtered >= self.cfg.approach_rssi)
        self.phase = SearchPhase.APPROACH if approach else SearchPhase.SEARCH
        pwm = self.cfg.approach_pwm if approach else self.cfg.search_pwm
        self._record(now, pwm)
        return self._drive(pwm)

    def _step_return(self, now: float) -> SearchOutput:
        for end_t, pwm in self._return_plan:
            if now < end_t:
                return self._drive(pwm)
        self.phase = SearchPhase.DONE
        self.reason = "복귀 완료"
        return SearchOutput(self.phase, 0, 0, self.reason)

    # ── 내부 ─────────────────────────────────────────
    def _record(self, now: float, pwm: int) -> None:
        if not self._segments or self._segments[-1][1] != pwm:
            self._segments.append((now, pwm))

    def _begin_return(self, now: float) -> None:
        """최댓값 시각 이후의 직진 구간을 거꾸로 뒤집어 복귀 계획을 만든다."""
        peak_t = max(self.detector.peak_time() or now, self._start_t)
        pieces: List[Tuple[float, int]] = []  # (구간 길이, pwm)
        for i, (seg_start, pwm) in enumerate(self._segments):
            seg_end = self._segments[i + 1][0] if i + 1 < len(self._segments) else now
            overlap = min(seg_end, now) - max(seg_start, peak_t)
            if overlap > 0:
                pieces.append((overlap, pwm))

        self.return_duration = sum(d for d, _ in pieces)
        self.phase = SearchPhase.RETURN
        if self.return_duration > self.cfg.return_timeout:
            self._abort(f"복귀 시간이 너무 김 ({self.return_duration:.1f}초)")
            return

        t = now
        self._return_plan = []
        for duration, pwm in reversed(pieces):  # 마지막에 간 구간부터 반대로
            t += duration
            self._return_plan.append((t, -pwm))

    def _drive(self, pwm: int) -> SearchOutput:
        if pwm == 0:
            return SearchOutput(self.phase, 0, 0, self.reason)
        sign = 1 if pwm > 0 else -1
        left = pwm + sign * self.cfg.trim_left
        right = pwm + sign * self.cfg.trim_right
        return SearchOutput(self.phase, _clamp(left), _clamp(right), self.reason)

    def _abort(self, reason: str) -> SearchOutput:
        self.phase = SearchPhase.ABORT
        self.reason = reason
        return SearchOutput(self.phase, 0, 0, reason)


def _clamp(v: int) -> int:
    return max(-255, min(255, int(v)))
