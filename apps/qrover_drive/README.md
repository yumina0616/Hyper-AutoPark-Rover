# qrover_drive — 주행 실행부 (#19)

Python(MPU)이 Bridge로 바퀴 값을 보내면, MCU가 **안전장치를 거쳐** 실행하는 App Lab 앱입니다.
MCU는 스스로 판단하지 않습니다. 구조는 [`docs/architecture.md`](../../docs/architecture.md) "판단·실행 분리"를 참고하세요.

> **상태:** 구현 (2026-11-16). **실물 검증 안 됨.** App Lab 빌드·업로드, 실제 로버 동작은 아직 확인하지 않았습니다.

## 폴더 구조

```
qrover_drive/
├── app.yaml               App Lab 앱 설정
├── sketch/                MCU (C++)
│   ├── sketch.ino         메인 루프: 거리 읽기 → 명령 확인 → 안전장치 → 실행
│   ├── config.h           핀·안전 설정값
│   ├── sensor.*           VL53L0X, 상태 코드 3종
│   ├── command.*          Bridge "drive" 수신
│   ├── safety.*           안전장치 (하드웨어 없이 테스트 가능)
│   ├── motor.*            바퀴 구동, 출발 보정
│   └── sketch.yaml        라이브러리 목록
├── python/                MPU (Python)
│   ├── main.py            MODE로 동작 선택: 개별 시험 / 전체 자동주차
│   ├── bridge_link.py     Bridge 송수신, heartbeat, 시작 대기, 오래된 값 판단
│   ├── beacon_mode.py     비콘 탐색(1단계)을 Bridge 주행과 연결
│   ├── autopark_mode.py   비콘 → ArUco → 전면 주차 전체 실행기
│   ├── rssi_client.py     RSSI 중계기에 접속해서 RSSI 받기
│   └── beacon/, state_machine/, vision/   ← scripts/sync_app_modules.py가 복사 (커밋 안 됨)
└── tests/                 노트북에서 돌리는 테스트
    ├── test_bridge_link.py
    ├── test_beacon_mode.py
    └── test_safety.cpp
```

## 주행 흐름 1단계: 비콘 탐색 (`MODE = "beacon_search"`)

목표 칸 비콘의 RSSI가 가장 셌던 지점을 찾아 되돌아옵니다.

```
[App Lab 바깥]  rx_scan.py ──▶ rssi_relay.py ──TCP──┐
[App Lab 앱 안]                                      ▼
                rssi_client → BeaconSearch(판단) → bridge_link ──Bridge drive──▶ MCU
```

App Lab 앱 안에서는 블루투스를 쓸 수 없어서, 비콘 수신은 바깥에서 하고 **중계기(`beacon/rssi_relay.py`)가 TCP로 앱에 넘깁니다.**

흐름: 직진 탐색 → RSSI가 세지면 감속 → 최댓값보다 6dB 떨어진 상태가 1초 이어지면 "지나쳤다" → 최댓값 이후 보낸 명령을 거꾸로 같은 시간만큼 후진 → 정지.
목표 칸 신호를 한 번 받은 뒤에만 출발합니다. 시간 초과(30초)나 신호 끊김(1초)이면 정지합니다.
판단 로직: [`state_machine/beacon_search.py`](../../state_machine/beacon_search.py), [`beacon/peak_detector.py`](../../beacon/peak_detector.py)

**실행 순서 (UNO Q)**

1. 공용 모듈을 앱 폴더로 복사 (레포 최상위에서, 원본을 고칠 때마다 다시)
   ```bash
   python3 scripts/sync_app_modules.py
   ```
2. App Lab 바깥에서 수신 + 중계 (먼저 실행, 계속 켜 둠)
   ```bash
   python3 beacon/rx_scan.py | python3 beacon/rssi_relay.py
   ```
3. `python/main.py`에서 `MODE = "beacon_search"`, `TARGET_SLOT = 비콘을 둔 칸 번호`로 바꾸고 앱 실행
4. 로그에 `[rssi] 중계기 접속` → `[beacon] 칸 N 신호 확인 → 탐색 시작` → `RETURN` → `DONE`이 나오는지 확인

**로버를 움직이지 않고 감지만 확인하기**: `python3 beacon/rx_scan.py | python3 -m beacon.peak_monitor`

**확인 안 된 것**
- 앱(컨테이너) 안에서 중계기에 접속되는지. `127.0.0.1` → 컨테이너 게이트웨이 순서로 시도합니다. 안 되면 보드 IP를 `RSSI_RELAY_HOST` 환경 변수로 지정합니다
- 실제 RSSI 흔들림에 맞는 감지 설정값 (지금은 가상 로버 시뮬레이션 기준)
- 같은 PWM·같은 시간으로 되돌아갈 때의 실제 위치 오차

`motor`, `sensor`, `config`는 [`apps/qrover_obstacle_avoidance`](../qrover_obstacle_avoidance) (#17)에서 가져와 고쳤습니다.

## 전체 자동주차 (`MODE = "autopark"`)

`autopark` 모드는 아래 단계를 앱 재시작 없이 연속 실행합니다.

1. 목표 비콘 RSSI 최댓값을 지나친 뒤 최댓값 위치로 복귀
2. 마커가 없으면 저속 제자리 회전, 목표 ID를 찾으면 화면 중앙 정렬
3. 중앙 정렬을 여러 새 프레임에서 확인한 뒤 전면 접근
4. 중앙 정렬 + 마커 크기 + ToF 거리 조건을 3회 연속 확인하면 `PARKED`
5. 비콘·카메라·거리 센서 오류나 각 단계 시간 초과 시 `ABORT`로 정지 유지

실행 전 `python3 scripts/sync_app_modules.py`를 실행하고, `python/main.py`에서
`MODE = "autopark"`, `TARGET_SLOT = 목표 칸 번호`로 설정합니다. 기본값은 안전을
위해 계속 `idle`입니다.

현재 자동 탐색 회전의 기본 방향은 **왼쪽 전면 주차 배치** 기준입니다. 실제 모터
배선과 카메라 좌우 방향을 로버를 들어 올린 상태에서 확인한 뒤
`AutoParkConfig.marker_search_direction`을 조정해야 합니다.

## Bridge 메시지

| 이름 | 방향 | 값 | 주기 |
|---|---|---|---|
| `drive` | Python → MCU | `left`, `right` (-255 ~ 255, + = 앞으로) | 바뀔 때 + 0.3초마다 (heartbeat) |
| `distance` | MCU → Python | `dist_mm`, `status` | 약 50ms마다 |

거리 상태 코드: `0` 정상 / `1` 범위 밖(앞이 비어 있음) / `2` 오류(측정 실패·센서 없음)

## 안전장치 (MCU, 위에서부터 먼저 확인)

| 순서 | 조건 | 결과 |
|---|---|---|
| 1 | 부팅 후 첫 명령 전, 또는 마지막 명령 후 1초 경과 | 정지 |
| 2 | 거리 센서 오류 | 정지 |
| 3 | 전진 중(왼쪽+오른쪽 > 0) 앞 거리 < 80mm | 정지 (후진·제자리 회전은 막지 않음) |
| 4 | 그 외 | Python 명령 그대로 |

정지 이유가 바뀔 때마다 시리얼 모니터에 `safety: WATCHDOG` 같은 줄이 찍힙니다.

## 실행

1. App Lab에서 이 폴더를 앱으로 열고 Run
2. 기본값은 **바퀴를 움직이지 않습니다** (`python/main.py`의 `RUN_TEST_SEQUENCE = False`). 로그에 거리값이 0.5초마다 찍히는지 확인합니다
3. 바퀴 방향을 확인할 때만:
   - **로버를 들어서 바퀴가 바닥에 닿지 않게** 합니다
   - `RUN_TEST_SEQUENCE = True`로 바꾸고 Run → 시작 대기 3초 후 전진 → 정지 → 후진 → 정지 → 시계 방향 회전 → 정지

App Lab이 레포 폴더를 바로 열 수 있는지는 아직 확인하지 않았습니다 (#1). 안 되면 App Lab 앱 폴더로 복사해서 실행합니다.

## 테스트 (노트북, 로버 없이)

```bash
python -m pytest apps/qrover_drive/tests/test_bridge_link.py
```

```bash
python -m unittest state_machine.test_autopark -v
python apps/qrover_drive/tests/test_autopark_mode.py
```

```bash
g++ -std=c++11 -I apps/qrover_drive/sketch apps/qrover_drive/tests/test_safety.cpp apps/qrover_drive/sketch/safety.cpp -o test_safety && ./test_safety
```

## 확인 안 된 것

- App Lab에서 여러 `.cpp/.h` 파일로 나뉜 스케치가 빌드되는지 (#1). 안 되면 `sketch.ino` 하나로 합칩니다
- 실제 로버에서 방향, watchdog, 비상정지, 센서 선 뽑기 동작 (바퀴 들고 확인하는 별도 테스트)
- VL53L0X `RangeStatus` 1~3(신호 약함 등)을 오류로 보고 정지하는데, 실제로 너무 자주 나와서 로버가 자꾸 멈추는지
- 출발 보정 중 120ms 동안은 루프가 멈춥니다 (거리값 전송도 잠깐 멈춤)
