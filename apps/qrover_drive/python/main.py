"""Q-ROVER 주행 실행부 확인 앱 (MPU 쪽) — #19

지금 단계에서 하는 일
    - MCU가 보내는 거리값(distance)을 받아서 0.5초마다 출력
    - 상태 기계 대신, 정해 둔 순서대로 drive 명령을 보내는 "확인용 시퀀스"

⚠️ 기본값은 바퀴를 움직이지 않는다 (RUN_TEST_SEQUENCE = False → 계속 정지 명령).
   바퀴 방향을 확인하려면 로버를 들어서 바퀴가 바닥에 닿지 않게 한 뒤 True로 바꾼다.

상태 기계(#7, 2주차)가 생기면 TEST_SEQUENCE 부분을 상태 기계로 바꾼다.
"""

import time

from arduino.app_utils import App, Bridge

from bridge_link import RANGE_ERROR, RANGE_OK, RANGE_OUT_OF_RANGE, BridgeLink

# ★ 바퀴를 들고 확인할 때만 True
RUN_TEST_SEQUENCE = False

# (초, 왼쪽, 오른쪽, 설명) — 시작 대기 3초가 지난 뒤 순서대로 실행하고, 끝나면 정지
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


def loop():
    global seq_start, last_print, last_label
    now = time.time()

    if RUN_TEST_SEQUENCE and not link.in_start_delay():
        if seq_start is None:
            seq_start = now
        left, right, label = current_step(now - seq_start)
    else:
        left, right, label = 0, 0, "정지 (테스트 시퀀스 꺼짐)" if not RUN_TEST_SEQUENCE else "시작 대기"

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
