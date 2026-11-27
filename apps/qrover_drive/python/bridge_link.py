"""MCU와 Bridge로 주고받는 부분 (Python 쪽) — #19

상태 기계는 Bridge를 직접 다루지 않고 이 모듈의 함수만 쓴다.
    link.send_drive(left, right)   바퀴 값 요청 (실제 전송은 tick()이 함)
    link.latest_distance()         가장 최근 거리값 + 받은 시각 + 오래됐는지
    link.tick()                    주기 루프에서 매번 호출 (heartbeat 전송)

Bridge 함수(notify)와 시계(clock)를 밖에서 넣어 주는 구조라,
진짜 Bridge 없이 가짜 함수로 테스트할 수 있다. (tests/test_bridge_link.py)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

# 거리 센서 상태 코드 (MCU safety.h와 같은 값)
RANGE_OK = 0
RANGE_OUT_OF_RANGE = 1
RANGE_ERROR = 2

# MCU safety.h의 SafetyReason과 같은 값
SAFETY_PASS = 0
SAFETY_NO_COMMAND = 1
SAFETY_WATCHDOG = 2
SAFETY_SENSOR_ERROR = 3
SAFETY_EMERGENCY = 4


@dataclass
class LinkConfig:
    heartbeat_sec: float = 0.3    # 값이 그대로여도 이 주기로 다시 보냄 (MCU watchdog 1초보다 짧게)
    # 이 시간 넘게 새 값이 없으면 "오래된 값". 0.3초는 UNO Q에서 카메라 스레드가 바쁠 때
    # 정상 신호도 넘겨서(2026-11-27) 0.8초로 늘림. 바퀴를 세우는 안전장치(watchdog 1초,
    # 비상정지)는 MCU가 자체적으로 지킨다.   TODO: 실험으로 결정
    stale_sec: float = 0.8
    start_delay_sec: float = 3.0  # 앱 시작 후 이 시간 동안은 정지 명령만 보냄 (로버에서 손 뗄 시간)


@dataclass
class DistanceReading:
    dist_mm: int      # 거리 (mm). 상태가 OK가 아니면 -1
    status: int       # RANGE_OK / RANGE_OUT_OF_RANGE / RANGE_ERROR
    t: float          # Python이 받은 시각 (time.time())
    stale: bool       # 오래된 값인지 (지금 - t > stale_sec)


@dataclass
class SafetyReading:
    reason: int       # SAFETY_PASS / NO_COMMAND / WATCHDOG / SENSOR_ERROR / EMERGENCY
    t: float          # Python이 받은 시각
    stale: bool


class BridgeLink:
    def __init__(
        self,
        notify: Callable[..., None],
        clock: Callable[[], float] = time.time,
        config: Optional[LinkConfig] = None,
    ) -> None:
        """notify: Bridge.notify 같은 함수. notify("drive", left, right) 형태로 부른다."""
        self._notify = notify
        self._clock = clock
        self.cfg = config or LinkConfig()

        self._start_t = clock()
        self._wanted = (0, 0)                    # 상태 기계가 원하는 바퀴 값
        self._last_sent: Optional[tuple[int, int]] = None
        self._last_sent_t = 0.0
        self._distance: Optional[tuple[int, int, float]] = None  # (dist_mm, status, t)
        self._safety: Optional[tuple[int, float]] = None          # (reason, t)

    # ── MCU → Python ──────────────────────────────────
    def on_distance(self, dist_mm: int, status: int) -> None:
        """Bridge.provide("distance", link.on_distance)로 등록한다."""
        self._distance = (int(dist_mm), int(status), self._clock())

    def latest_distance(self) -> Optional[DistanceReading]:
        """아직 한 번도 못 받았으면 None."""
        if self._distance is None:
            return None
        dist_mm, status, t = self._distance
        stale = (self._clock() - t) > self.cfg.stale_sec
        return DistanceReading(dist_mm, status, t, stale)

    def on_safety(self, reason: int) -> None:
        """Bridge.provide("safety", link.on_safety)로 등록한다."""
        self._safety = (int(reason), self._clock())

    def latest_safety(self) -> Optional[SafetyReading]:
        if self._safety is None:
            return None
        reason, t = self._safety
        stale = (self._clock() - t) > self.cfg.stale_sec
        return SafetyReading(reason, t, stale)

    # ── Python → MCU ──────────────────────────────────
    def send_drive(self, left: int, right: int) -> None:
        """원하는 바퀴 값을 기록한다. 실제 전송은 tick()에서."""
        self._wanted = (_clamp(left), _clamp(right))

    def tick(self) -> Optional[tuple[int, int]]:
        """주기 루프에서 매번 부른다. 이번에 보냈으면 보낸 값을, 아니면 None을 돌려준다.

        보내는 조건: 값이 바뀌었을 때 + heartbeat_sec마다.
        시작 후 start_delay_sec 동안은 원하는 값과 상관없이 (0, 0)을 보낸다.
        """
        now = self._clock()
        cmd = (0, 0) if self.in_start_delay() else self._wanted

        if cmd != self._last_sent or now - self._last_sent_t >= self.cfg.heartbeat_sec:
            self._notify("drive", cmd[0], cmd[1])
            self._last_sent = cmd
            self._last_sent_t = now
            return cmd
        return None

    def in_start_delay(self) -> bool:
        return self._clock() - self._start_t < self.cfg.start_delay_sec


def _clamp(v: int) -> int:
    return max(-255, min(255, int(v)))
