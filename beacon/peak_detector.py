"""비콘 RSSI 최댓값(가장 가까웠던 지점) 감지 — 주행 흐름 1단계

로버가 통로를 지나가면 목표 칸 비콘의 RSSI가 커졌다가 작아진다.
"지금이 최댓값인지"는 미래를 모르므로 바로 알 수 없다.
그래서 **최댓값보다 충분히 떨어진 상태가 일정 시간 이어지면 "지나쳤다"**고 판단하고,
그때까지 기억해 둔 최댓값 시각을 돌려준다.

    det = PeakDetector()
    det.update(t, rssi)   # RSSI를 받을 때마다 (t: 받은 시각, 초)
    det.passed            # 지나쳤으면 True (한 번 True가 되면 계속 True)
    det.peak_time()       # 최댓값이었던 시각 (필터 지연 보정 포함)

RSSI가 어디서 오는지(UNO Q bleak, ESP32 수신기 등)와 상관없는 순수 계산이라
가짜 데이터나 녹화한 CSV로 테스트할 수 있다.

한계 (docs, #7):
    - RSSI 최댓값 지점이 칸 정중앙이라는 보장은 없다 → 정확한 칸은 다음 단계(ArUco)가 맞춘다
    - 평균을 내면 최댓값이 늦게 잡힌다 → 창 길이의 절반만큼 앞당겨 보정한다
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple


@dataclass
class PeakConfig:
    # 초기값 근거: 가상 로버 시뮬레이션(state_machine/test_beacon_search.py)에서 RSSI 잡음 ±2/4/6dB로
    # 비교했을 때 가장 안정적이었던 값 (±6dB에서도 30번 중 26번이 비콘 앞 15cm 이내, 최대 26cm).
    # 0.5초 / 4dB / 0.6초는 잡음 ±4dB만 돼도 비콘 도착 전에 "지나쳤다"고 착각했다. 실물로 다시 확인할 것.

    # 이 시간(초) 동안 받은 RSSI를 평균 낸다. 길수록 덜 흔들리지만 반응이 늦다
    window_sec: float = 1.0        # TODO: 실물 RSSI로 결정
    # 평균을 내려면 창 안에 최소 이만큼의 샘플이 있어야 한다 (너무 적으면 판단 보류)
    min_samples: int = 3           # TODO: 실험으로 결정
    # 최댓값보다 이만큼(dB) 떨어지면 "지나치는 중"으로 본다
    drop_db: float = 6.0           # TODO: 실물 RSSI로 결정
    # 위 상태가 이 시간(초) 동안 이어져야 "지나쳤다"로 확정 (순간적인 흔들림 무시)
    confirm_sec: float = 1.0       # TODO: 실물 RSSI로 결정
    # 최댓값이 이보다 약하면 아직 비콘 근처가 아니라고 보고 판단하지 않는다
    min_peak_rssi: float = -90.0   # TODO: 실험으로 결정
    # 마지막 RSSI 이후 이 시간(초)이 지나면 "신호 끊김" (#7 BEACON_HOLD_SEC)
    hold_sec: float = 1.0          # TODO: 실험으로 결정


class PeakDetector:
    def __init__(self, config: Optional[PeakConfig] = None) -> None:
        self.cfg = config or PeakConfig()
        self.reset()

    def reset(self) -> None:
        self._samples: Deque[Tuple[float, float]] = deque()  # (t, rssi)
        self.filtered: Optional[float] = None   # 지금의 평균 RSSI
        self.peak_rssi: Optional[float] = None  # 지금까지 가장 컸던 평균 RSSI
        self._peak_t_raw: Optional[float] = None
        self._below_since: Optional[float] = None
        self.last_t: Optional[float] = None
        self.passed: bool = False

    def update(self, t: float, rssi: float) -> bool:
        """RSSI 하나를 넣는다. 이번에 처음으로 '지나쳤다'가 되면 True."""
        self.last_t = t
        self._samples.append((t, float(rssi)))
        while self._samples and self._samples[0][0] < t - self.cfg.window_sec:
            self._samples.popleft()

        if len(self._samples) < self.cfg.min_samples:
            return False
        self.filtered = sum(r for _, r in self._samples) / len(self._samples)

        if self.passed:
            return False  # 이미 확정됐으면 최댓값을 더 갱신하지 않는다

        if self.peak_rssi is None or self.filtered > self.peak_rssi:
            self.peak_rssi = self.filtered
            self._peak_t_raw = t
            self._below_since = None
            return False

        if self.peak_rssi < self.cfg.min_peak_rssi:
            return False  # 아직 너무 약함 → 먼 곳의 흔들림일 수 있음

        if self.filtered <= self.peak_rssi - self.cfg.drop_db:
            if self._below_since is None:
                self._below_since = t
            if t - self._below_since >= self.cfg.confirm_sec:
                self.passed = True
                return True
        else:
            self._below_since = None  # 다시 올라왔으면 처음부터 다시 확인
        return False

    def peak_time(self) -> Optional[float]:
        """최댓값이었던 시각. 평균 창 때문에 늦게 잡힌 만큼(창 절반) 앞당긴다."""
        if self._peak_t_raw is None:
            return None
        return self._peak_t_raw - self.cfg.window_sec / 2

    def signal_lost(self, now: float) -> bool:
        """마지막 RSSI 이후 hold_sec이 지났으면 True. 한 번도 못 받았으면 False."""
        return self.last_t is not None and now - self.last_t > self.cfg.hold_sec
