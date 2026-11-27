"""Q-ROVER 주행 앱 (MPU 쪽) — 개별 시험 모드 + 전체 자동주차

MODE로 동작을 고른다.
    "idle"           정지 명령만 보내고 거리값만 출력 (기본값, 바퀴 안 움직임)
    "test_sequence"  정해 둔 순서대로 전진·후진·회전 (바퀴를 들고 방향 확인용)
    "beacon_search"  비콘 RSSI가 가장 셌던 지점 찾기 → 지나치면 되돌아오기 (1단계)
    "front_parking"  ArUco 마커 정렬 후 ToF 거리·마커 크기 조건으로 전면 주차
    "autopark"       비콘 탐색 → 마커 탐색·정렬 → 전면 주차를 연속 실행

⚠️ 바퀴가 움직이는 모드는 처음에는 로버를 들어서 바퀴가 바닥에 닿지 않게 하고 확인한다.

beacon_search 모드 준비 (UNO Q):
    1) python scripts/sync_app_modules.py      (공용 모듈을 앱 폴더로 복사, 레포 최상위에서)
    2) python3 beacon/rx_scan.py | python3 beacon/rssi_relay.py    (App Lab 바깥, 먼저 실행)
    3) 이 앱 실행

front_parking 모드 준비 (UNO Q):
    1) python scripts/sync_app_modules.py      (공용 모듈을 앱 폴더로 복사, 레포 최상위에서)
    2) Logitech C270 카메라 연결 확인 (/dev/video2) 후 MODE = "front_parking" 으로 변경하여 실행
"""

import time

from arduino.app_utils import App, Bridge

from bridge_link import RANGE_ERROR, RANGE_OK, RANGE_OUT_OF_RANGE, BridgeLink

# ★ 동작 선택: "idle" / "test_sequence" / "beacon_search" / "front_parking" / "autopark"
MODE = "idle"

# beacon_search / front_parking / autopark: 목표 칸 번호 (0~4). 나중에 웹 화면에서 선택하도록 바꿈 (#7)
TARGET_SLOT = 0

# UNO Q에서 Logitech C270은 /dev/video2. video0·1은 qcom-venus 코덱 장치다.
CAMERA_INDEX = 2

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
# MCU가 실제 명령을 통과/차단했는지 Python 상태 기계에 알려준다
Bridge.provide("safety", link.on_safety)

beacon = None
if MODE == "beacon_search":
    from beacon_mode import BeaconMode
    from rssi_client import RssiClient
    from state_machine.beacon_search import BeaconSearch

    rssi_client = RssiClient()
    beacon = BeaconMode(BeaconSearch(), rssi_client.poll, TARGET_SLOT,
                        log=lambda m: print(m, flush=True))

front_parking = None
if MODE == "front_parking":
    from front_parking_mode import FrontParkingRunner

    front_parking = FrontParkingRunner(
        target_marker_id=TARGET_SLOT,
        camera_index=CAMERA_INDEX,
    )

autopark = None
if MODE == "autopark":
    from autopark_mode import AutoParkRunner
    from rssi_client import RssiClient

    rssi_client = RssiClient()
    autopark = AutoParkRunner(
        target_slot=TARGET_SLOT,
        poll_rssi=rssi_client.poll,
        camera_index=CAMERA_INDEX,
        log=lambda m: print(m, flush=True),
    )
    # 향후 버튼/웹 UI가 이 이름으로 시작·취소·초기화·목표칸 변경을 요청할 수 있다.
    Bridge.provide("autopark_start", lambda: autopark.start())
    Bridge.provide("autopark_cancel", lambda: autopark.cancel())
    Bridge.provide("autopark_reset", lambda: autopark.reset())
    Bridge.provide("autopark_target", lambda slot: autopark.set_target_slot(int(slot)))
    Bridge.provide("autopark_status", lambda: autopark.status())

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

    if MODE == "autopark":
        out = autopark.step(link, allowed=not link.in_start_delay())
        label = f"전체 자동주차 ({out.phase.value}: {out.detail})"
        link.tick()

        if label != last_label:
            print(f"[drive] {label}: L={out.left} R={out.right}", flush=True)
            last_label = label
    elif MODE == "front_parking":
        if link.in_start_delay():
            link.send_drive(0, 0)
            label = "시작 대기"
        else:
            state = front_parking.step(link)
            label = f"전면 주차 ({state.value})"
        link.tick()

        if label != last_label:
            print(f"[drive] {label}", flush=True)
            last_label = label
    else:
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
