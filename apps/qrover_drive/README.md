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
│   ├── main.py            확인용 앱 (거리 출력 + 확인용 주행 순서)
│   └── bridge_link.py     Bridge 송수신, heartbeat, 시작 대기, 오래된 값 판단
└── tests/                 노트북에서 돌리는 테스트
    ├── test_bridge_link.py
    └── test_safety.cpp
```

`motor`, `sensor`, `config`는 [`apps/qrover_obstacle_avoidance`](../qrover_obstacle_avoidance) (#17)에서 가져와 고쳤습니다.

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
g++ -std=c++11 -I apps/qrover_drive/sketch apps/qrover_drive/tests/test_safety.cpp apps/qrover_drive/sketch/safety.cpp -o test_safety && ./test_safety
```

## 확인 안 된 것

- App Lab에서 여러 `.cpp/.h` 파일로 나뉜 스케치가 빌드되는지 (#1). 안 되면 `sketch.ino` 하나로 합칩니다
- 실제 로버에서 방향, watchdog, 비상정지, 센서 선 뽑기 동작 (바퀴 들고 확인하는 별도 테스트)
- VL53L0X `RangeStatus` 1~3(신호 약함 등)을 오류로 보고 정지하는데, 실제로 너무 자주 나와서 로버가 자꾸 멈추는지
- 출발 보정 중 120ms 동안은 루프가 멈춥니다 (거리값 전송도 잠깐 멈춤)
