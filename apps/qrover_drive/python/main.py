"""Q-ROVER 주행 앱 (MPU 쪽) — #19, 주행 흐름 1단계

MODE로 동작을 고른다.
    "idle"           정지 명령만 보내고 거리값만 출력 (기본값, 바퀴 안 움직임)
    "test_sequence"  정해 둔 순서대로 전진·후진·회전 (바퀴를 들고 방향 확인용)
    "beacon_search"  비콘 RSSI가 가장 셌던 지점 찾기 → 지나치면 되돌아오기 (1단계)

⚠️ 바퀴가 움직이는 모드는 처음에는 로버를 들어서 바퀴가 바닥에 닿지 않게 하고 확인한다.

beacon_search 모드 준비 (UNO Q):
    1) python scripts/sync_app_modules.py      (공용 모듈을 앱 폴더로 복사, 레포 최상위에서)
    2) python3 beacon/rx_scan.py | python3 beacon/rssi_relay.py    (App Lab 바깥, 먼저 실행)
    3) 이 앱 실행
"""

import time

from arduino.app_utils import App, Bridge

from bridge_link import RANGE_ERROR, RANGE_OK, RANGE_OUT_OF_RANGE, BridgeLink

# ★ 동작 선택: "idle" / "test_sequence" / "beacon_search"
MODE = "idle"

# beacon_search: 목표 칸 번호 (0~4). 나중에 웹 화면에서 선택하도록 바꿈 (#7)
TARGET_SLOT = 0

# test_sequence: (초, 왼쪽, 오른쪽, 설명) — 시작 대기 3초 뒤 순서대로 실행하고, 끝나면 정지
TEST_SEQUENCE = [
    (2.0, 100, 100, "전진"),
    (1.0, 0, 0, "정지"),
    (2.0, -100, -100, "후진"),
    (1.0, 0, 0, "정지"),
    (2.0, 120, -120, "제자리 회전 (시계 방향)"),
    (1.0, 0, 0, "정지"),
]

STATUS_NAME = {RANGE_OK: "OK", RANGE_OUT_OF_RANGE: "범위 밖", RANGE_ERROR: "오류"}

link = BridgeLink(notify=Bridge.notify)

# 이 문자열이 스케치의 Bridge.notify("distance", ...)와 정확히 같아야 한다
Bridge.provide("distance", link.on_distance)

beacon = None
if MODE == "beacon_search":
    from beacon_mode import BeaconMode
    from rssi_client import RssiClient
    from state_machine.beacon_search import BeaconSearch

    rssi_client = RssiClient()
    beacon = BeaconMode(BeaconSearch(), rssi_client.poll, TARGET_SLOT,
                        log=lambda m: print(m, flush=True))

seq_start = None
last_print = 0.0
last_label = None


def current_step(elapsed):
    """시퀀스 시작 후 elapsed초일 때의 (왼쪽, 오른쪽, 설명). 끝났으면 정지."""
    t = 0.0
    for duration, left, right, label in TEST_SEQUENCE:
        t += duration
        if elapsed < t:
            return left, right, label
    return 0, 0, "시퀀스 끝 (정지)"


def decide(now):
    """MODE에 따라 이번 주기의 (왼쪽, 오른쪽, 설명)을 정한다."""
    global seq_start
    if MODE == "test_sequence":
        if link.in_start_delay():
            return 0, 0, "시작 대기"
        if seq_start is None:
            seq_start = now
        return current_step(now - seq_start)

    if MODE == "beacon_search":
        left, right = beacon.step(now, allowed=not link.in_start_delay())
        label = "비콘 탐색 끝 (정지)" if beacon.finished else (
            "비콘 탐색" if beacon.started else "비콘 신호 기다리는 중")
        return left, right, label

    return 0, 0, "정지 (idle)"


def loop():
    global last_print, last_label
    now = time.time()

    left, right, label = decide(now)
    link.send_drive(left, right)
    link.tick()

    if label != last_label:
        print(f"[drive] {label}: L={left} R={right}", flush=True)
        last_label = label

    if now - last_print >= 0.5:
        d = link.latest_distance()
        if d is None:
            print("[distance] 아직 못 받음", flush=True)
        else:
            stale = " (오래된 값!)" if d.stale else ""
            print(f"[distance] {d.dist_mm} mm, {STATUS_NAME.get(d.status, d.status)}{stale}", flush=True)
        last_print = now

    time.sleep(0.05)  # 20Hz


App.run(user_loop=loop)
