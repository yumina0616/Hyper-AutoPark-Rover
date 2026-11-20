"""비콘 탐색 모드 (주행 흐름 1단계)를 Bridge 주행과 연결한다.

    RSSI 수신기(rssi_client) ──RSSI──▶ BeaconSearch(판단) ──(left, right)──▶ BridgeLink(drive 전송) ──▶ MCU

출발 조건: 목표 칸 비콘 신호를 한 번이라도 받은 뒤에만 탐색을 시작한다.
           (중계기 미접속, 비콘 꺼짐 상태에서 로버가 달려 나가지 않게)
끝나면(DONE/ABORT) 계속 정지 명령을 보낸다. 다음 단계(회전하며 ArUco 정렬)는 이후에 붙인다.

BeaconSearch, RssiClient, BridgeLink를 밖에서 넣어 주는 구조라 가짜 객체로 테스트할 수 있다.
"""

from __future__ import annotations

from typing import Callable, Optional

from state_machine.beacon_search import BeaconSearch, SearchPhase


class BeaconMode:
    def __init__(self, search: BeaconSearch, poll_rssi: Callable[[], list], target_slot: int,
                 log: Callable[[str], None] = print) -> None:
        self.search = search
        self.poll_rssi = poll_rssi
        self.target_slot = target_slot
        self.log = log
        self.started = False
        self._last_phase: Optional[SearchPhase] = None

    def step(self, now: float, allowed: bool = True):
        """주기 루프에서 매번 부른다. (left, right)를 돌려준다.

        allowed: False면 출발하지 않는다 (예: BridgeLink 시작 대기 3초 동안).
        """
        target_rssi = [rssi for slot, rssi in self.poll_rssi() if slot == self.target_slot]

        if not self.started:
            if allowed and target_rssi:
                self.search.start(now)
                self.started = True
                self.log(f"[beacon] 칸 {self.target_slot} 신호 확인 → 탐색 시작")
            else:
                return 0, 0

        for rssi in target_rssi:
            self.search.on_rssi(now, rssi)

        out = self.search.step(now)
        if out.phase != self._last_phase:
            msg = f"[beacon] {out.phase.value}"
            if out.phase == SearchPhase.RETURN:
                msg += f" (약 {self.search.return_duration:.2f}초 되돌아감)"
            if out.reason:
                msg += f" — {out.reason}"
            self.log(msg)
            self._last_phase = out.phase
        return out.left, out.right

    @property
    def finished(self) -> bool:
        return self.search.phase in (SearchPhase.DONE, SearchPhase.ABORT)
