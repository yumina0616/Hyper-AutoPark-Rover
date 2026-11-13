"""ArUco 기반 정렬 모듈 (비주얼 서보잉).

인식 모듈이 찾은 마커 위치(err_x)를 보고, 마커가 화면 가운데에 오도록
"바퀴를 어떻게 돌리면 좋을지" 보정 제안(left, right)을 계산한다.

역할 경계 (AGENTS.md 6장)
    인식 모듈:  frame  -> Marker(id, err_x, size_px, t)
    정렬 모듈:  Marker -> 보정 제안 (left, right) + 상태      <- 이 파일
    상태 기계:  보정 제안을 받아 Bridge drive(left, right)로 MCU에 전송

이 모듈은 모터를 직접 움직이지 않는다. 숫자만 계산해서 돌려준다.
그래서 카메라나 로버 없이도 가짜 Marker 값으로 테스트할 수 있다.

관련 이슈: #12 (정렬 모듈), #5 (인식 모듈), #2 (인터페이스), #7 (상태 기계)
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Protocol


# ─────────────────────────────────────────────────────────────
#  입력: 인식 모듈의 Marker
# ─────────────────────────────────────────────────────────────
class MarkerLike(Protocol):
    """정렬 모듈이 Marker에서 쓰는 값.

    인식 모듈(#5)의 Marker 클래스를 직접 불러오지 않고,
    아래 세 속성만 있으면 어떤 객체든 받는다.
    (인식 모듈과 독립적으로 개발·테스트하기 위해서)
    """

    id: int        # 마커 번호
    err_x: float   # 화면 가운데에서 좌우로 벗어난 정도 (+ = 화면 오른쪽)
    t: float       # 검출 시각, time.time() 기준 (초)


# ─────────────────────────────────────────────────────────────
#  설정값: 모두 실험으로 정할 값이라 한 곳에 모아 둔다
# ─────────────────────────────────────────────────────────────
@dataclass
class AlignConfig:
    # err_x의 "끝" 값. 인식 모듈이 -1.0~1.0으로 주면 1.0, -100~100으로 주면 100.
    # 범위는 이슈 #2에서 결정. 여기만 바꾸면 나머지 계산은 그대로 동작한다.
    err_x_full_scale: float = 1.0

    # P제어 이득: err_x가 화면 끝(1.0)일 때 좌우 바퀴에 줄 보정량(PWM).
    # 세미나 코드는 -100~100 오차에 KP=0.2 → 화면 끝에서 20. 같은 느낌으로 20에서 시작.
    kp: float = 20.0  # TODO: 실험으로 결정

    # 보정량 상한 (PWM). 갑자기 휙 도는 것을 막는다. 세미나 코드 MAX_TURN=40.
    max_turn: int = 40  # TODO: 실험으로 결정

    # 데드밴드: |err_x|(정규화 후)가 이 값보다 작으면 "가운데"로 보고 보정하지 않는다.
    # 아주 작은 오차까지 고치려 하면 좌우로 흔들린다.
    align_tol: float = 0.05  # TODO: 실험으로 결정 (#7의 ALIGN_TOL)

    # 정렬 완료 판단: 허용 오차 안에 이 프레임 수만큼 "연속"으로 있어야 ALIGNED.
    align_frames: int = 5  # TODO: 실험으로 결정 (#7의 ALIGN_FRAMES)

    # 마커를 마지막으로 본 뒤 이 시간(초)이 지나면 오래된 값으로 보고 LOST.
    hold_sec: float = 0.5  # 세미나 코드 HOLD_SEC 값

    # 제자리 정렬 때 최소 회전 PWM. 엔코더 없는 TT 모터는 PWM이 너무 낮으면
    # 정지 마찰 때문에 아예 안 돈다. 보정이 필요할 때는 최소 이 값으로 올린다.
    min_turn_pwm: int = 60  # TODO: 실험으로 결정 (MIN_PWM 측정 후, #9)

    # 후진 정렬 때 기본 후진 속도 (PWM, 양수로 적고 계산할 때 뒤로 보냄).
    base_rev_pwm: int = 80  # TODO: 실험으로 결정

    # 조향 부호. 이론상 +1이 맞다 (아래 "부호 설명" 참고).
    # 카메라 화면이 좌우 반전돼 있거나 모터 배선이 반대면 -1로 바꾼다.
    # 실물 로버에서 한 번 확인할 것.
    steer_sign: int = 1  # TODO: 실물에서 확인


# ─────────────────────────────────────────────────────────────
#  출력
# ─────────────────────────────────────────────────────────────
class AlignStatus(str, Enum):
    ADJUSTING = "ADJUSTING"  # 맞추는 중
    ALIGNED = "ALIGNED"      # 허용 오차 안에 align_frames 프레임 연속 → 정렬 완료
    LOST = "LOST"            # 마커 없음 / 다른 ID / 오래된 값 → 정지 제안


@dataclass
class AlignResult:
    left: int             # 왼쪽 바퀴 제안값 (-255 ~ 255, + = 앞으로)
    right: int            # 오른쪽 바퀴 제안값
    status: AlignStatus
    err: float            # 정규화한 오차 (-1.0 ~ 1.0). 마커가 없으면 0.0
    turn: int             # 좌우 차이 보정량 (+ = 시계 방향/오른쪽으로 돌기)
    reason: str = ""      # LOST일 때 이유 (로그용)


# ─────────────────────────────────────────────────────────────
#  부호 설명
# ─────────────────────────────────────────────────────────────
# turn > 0 이면  left = +turn, right = -turn  → 위에서 볼 때 시계 방향(오른쪽)으로 돈다.
#
# 로버에 고정된 카메라는 앞을 보든 뒤를 보든, 로버가 시계 방향으로 돌면
# 카메라도 "자기 기준 오른쪽"으로 고개를 돌린다.
# 그래서 마커가 화면 오른쪽(err_x > 0)에 있으면 시계 방향으로 돌면 된다 → steer_sign = +1.
#
# 후진할 때도 같다. 후진 중 left - right > 0 이면 차체가 시계 방향으로 돌고,
# 진행 방향(뒤쪽)이 후방 카메라 기준 오른쪽으로 휘어진다.
#
# 단, 카메라 영상이 거울처럼 좌우 반전돼 나오는 제품도 있어서 실물 확인이 필요하다.


class Aligner:
    """정렬 보정값 계산기.

    "몇 프레임 연속으로 맞았는지"를 기억해야 해서 클래스로 만들었다.
    상태 기계가 정렬 단계에 들어갈 때 reset()을 부르고,
    카메라 프레임마다 step()을 부르면 된다.

    사용 예:
        aligner = Aligner()
        aligner.reset()
        result = aligner.step(marker, target_id=3, now=time.time(), mode="align")
        # 상태 기계가 result.left, result.right를 drive()로 보낸다
    """

    def __init__(self, config: Optional[AlignConfig] = None) -> None:
        self.cfg = config or AlignConfig()
        self._ok_frames = 0  # 허용 오차 안에 연속으로 있었던 프레임 수

    def reset(self) -> None:
        """정렬 단계를 새로 시작할 때 부른다."""
        self._ok_frames = 0

    def step(
        self,
        marker: Optional[MarkerLike],
        target_id: int,
        now: float,
        mode: str = "align",
    ) -> AlignResult:
        """마커 하나를 보고 보정 제안을 계산한다.

        marker:    인식 모듈이 찾은 마커 (목표 마커가 없으면 None)
        target_id: 찾아야 하는 마커 번호 (= 목표 칸 번호)
        now:       지금 시각 (time.time())
        mode:      "align"   = 제자리에서 방향만 맞추기
                   "reverse" = 후진하면서 방향 맞추기
        """
        if mode not in ("align", "reverse"):
            raise ValueError(f"mode는 'align' 또는 'reverse'여야 합니다: {mode!r}")

        # ① 마커를 믿을 수 없으면 정지 제안 (정지 우선, AGENTS.md 6장)
        lost_reason = self._lost_reason(marker, target_id, now)
        if lost_reason:
            self._ok_frames = 0
            return AlignResult(0, 0, AlignStatus.LOST, 0.0, 0, lost_reason)

        # ② 오차를 -1.0 ~ 1.0으로 맞추고 부호 적용
        err = _clamp(marker.err_x / self.cfg.err_x_full_scale, -1.0, 1.0)
        err *= self.cfg.steer_sign

        # ③ 데드밴드 안이면 보정 0, 연속 프레임 수 세기
        if abs(err) < self.cfg.align_tol:
            self._ok_frames += 1
            turn = 0
        else:
            self._ok_frames = 0
            turn = int(round(_clamp(self.cfg.kp * err, -self.cfg.max_turn, self.cfg.max_turn)))

        aligned = self._ok_frames >= self.cfg.align_frames

        if mode == "align":
            return self._align_in_place(err, turn, aligned)
        return self._align_reverse(err, turn, aligned)

    # ── 내부 함수 ────────────────────────────────────────────
    def _lost_reason(self, marker: Optional[MarkerLike], target_id: int, now: float) -> str:
        if marker is None:
            return "마커 없음"
        if marker.id != target_id:
            return f"다른 마커 (본 것 {marker.id}, 목표 {target_id})"
        if now - marker.t > self.cfg.hold_sec:
            return f"오래된 값 ({now - marker.t:.2f}초 전)"
        return ""

    def _align_in_place(self, err: float, turn: int, aligned: bool) -> AlignResult:
        """제자리 정렬: 두 바퀴를 서로 반대로 돌린다."""
        if aligned:
            return AlignResult(0, 0, AlignStatus.ALIGNED, err, 0)

        if turn != 0 and abs(turn) < self.cfg.min_turn_pwm:
            # 너무 약하면 바퀴가 안 돈다 → 최소 PWM으로 올림
            turn = self.cfg.min_turn_pwm if turn > 0 else -self.cfg.min_turn_pwm

        left, right = _limit(turn, -turn)
        return AlignResult(left, right, AlignStatus.ADJUSTING, err, turn)

    def _align_reverse(self, err: float, turn: int, aligned: bool) -> AlignResult:
        """후진 정렬: 뒤로 가면서 좌우 속도 차이로 방향을 맞춘다.

        후진에서는 정렬이 끝나도 멈추지 않는다 (멈추는 건 거리 센서의 몫).
        status는 "지금 잘 맞고 있는지" 알려주는 용도.
        """
        base = -self.cfg.base_rev_pwm
        left, right = _limit(base + turn, base - turn)
        status = AlignStatus.ALIGNED if aligned else AlignStatus.ADJUSTING
        return AlignResult(left, right, status, err, turn)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _limit(left: float, right: float) -> tuple[int, int]:
    """바퀴 값을 -255 ~ 255 범위의 정수로 자른다."""
    return int(_clamp(left, -255, 255)), int(_clamp(right, -255, 255))
