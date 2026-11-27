# 시스템 아키텍처

> **상태 (2026-11-10 기준):** 설계 초안입니다. 부품 주문 전이며 구현·검증된 것은 없습니다.
> 실선 = 현재 로버에 있거나 1주차 전반에 만들 부분, 점선 = 부품 도착 후(1주차 후반~) 또는 선택 기능.

```mermaid
flowchart LR
    subgraph SLOT["주차칸 ×5 (진행 방향 왼쪽)"]
        TX["송신 XIAO ESP32S3 (목표 칸)<br/>BLE 비콘 · 목표 칸 번호"]
        MARKER["ArUco 마커 ×5<br/>칸마다 다른 ID"]
    end

    subgraph ROVER["로버"]
        RX["수신 XIAO ESP32S3<br/>ID · RSSI 수신"]
        CAM_F["전방 카메라<br/>C270 USB 웹캠"]
        CAM_R["후방 USB 카메라"]
        HUB["USB-C 허브"]

        subgraph UNOQ["Arduino UNO Q"]
            subgraph MPU["MPU · Linux (Python)"]
                BEACON_PY["beacon<br/>RSSI 필터 · 최댓값 추적"]
                VISION_PY["vision<br/>ArUco 인식 (OpenCV)<br/>id · err_x · size_px"]
                AI_PY["주차 금지 마커 검출<br/>(Edge Impulse 모델)"]
                SM["state_machine<br/>단계 판단 · 바퀴 값 결정<br/>수신 시각으로 오래된 값 판단"]
            end

            subgraph MCU["MCU · Arduino 스케치 (C++)"]
                SENS_MCU["sensors<br/>ToF 읽기<br/>dist_mm · status"]
                MOTION_MCU["motion<br/>drive · brake · coast · startDrive"]
                SAFETY["안전장치<br/>비상정지 · watchdog"]
            end
        end

        TOF_F["전방 ToF<br/>VL53L0X"]
        TOF_R["후방 ToF<br/>VL53L0X 후보"]
        DRV["DRV8833<br/>모터 드라이버"]
        MOTORS["TT 모터 ×2<br/>엔코더 없음"]
        BATT["AA ×4<br/>모터 전원"]
        PB["보조배터리<br/>보드 전원"]
    end

    TX -. "BLE 광고" .-> RX
    RX -. "USB 시리얼" .-> HUB
    MARKER -. "영상" .-> CAM_R
    CAM_F -- "USB" --> HUB
    CAM_R -. "USB" .-> HUB
    HUB --> MPU

    HUB -. "ID · RSSI" .-> BEACON_PY
    HUB -- "영상" --> VISION_PY
    HUB -. "영상" .-> AI_PY

    BEACON_PY -. "접근 · 최댓값 판단" .-> SM
    VISION_PY -- "마커 정보" --> SM
    AI_PY -. "금지 여부" .-> SM

    TOF_F -- "I2C" --> SENS_MCU
    TOF_R -. "I2C" .-> SENS_MCU
    SENS_MCU -- "Bridge: distance(dist_mm, status)" --> SM
    SAFETY -- "Bridge: safety(reason)" --> SM
    SM -- "Bridge: drive(left, right)" --> MOTION_MCU
    SENS_MCU --> SAFETY
    SAFETY -- "정지 우선" --> MOTION_MCU
    MOTION_MCU -- "PWM D3·D5·D6·D9" --> DRV
    DRV --> MOTORS
    BATT --> DRV
    PB --> UNOQ
```

## 역할 분리

| 위치 | 담당 | 이유 |
|---|---|---|
| MPU (Python) | 판단: 비콘·ArUco·거리 정보를 모아 단계를 정하고 바퀴 값을 결정 | 영상 처리처럼 무거운 계산과 전체 흐름 관리 |
| MCU (C++) | 실행: 받은 바퀴 값으로 모터 구동, 거리 센서 읽기 | 빠르고 일정한 타이밍 |
| MCU 안전장치 | 비상정지, watchdog. Python 판단보다 항상 우선 | Python이 멈추거나 통신이 끊겨도 로버는 멈춰야 함 |

두 쪽은 [인터페이스 문서](interfaces.md)에 정의한 Bridge 메시지로만 연결합니다.

## 판단·실행 분리 (모듈 단위)

> **상태:** 선택 방향 (2026-11-16). 팀 합의 후 확정합니다.

```mermaid
flowchart LR
    CAM[카메라]
    HW_M[DRV8833 · TT 모터 ×2]
    TOF[VL53L0X ToF]

    subgraph MPU["MPU · Python — 판단"]
        VISION["vision<br/>ArUco · 정렬 제안"]
        BEACON["beacon<br/>RSSI (예정)"]
        SM["<b>state_machine</b><br/>단계 판단 · 바퀴 값 결정"]
        IO["Bridge 송수신<br/>수신 시각 · heartbeat"]
        VISION --> SM
        BEACON -.-> SM
        SM <--> IO
    end

    subgraph MCU["MCU · 스케치 — 실행"]
        CMD["command<br/>drive 수신"]
        SENSOR["sensor<br/>상태 코드 3종"]
        SAFETY["<b>safety</b><br/>watchdog · 비상정지 · 오류"]
        MOTOR["motor<br/>setWheels · 출발 보정"]
        CMD --> SAFETY
        SENSOR --> SAFETY
        SAFETY --> MOTOR
    end

    CAM --> VISION
    IO -- "drive(L, R)" --> CMD
    SENSOR -- "distance" --> IO
    MOTOR --> HW_M
    TOF --> SENSOR
```

- **명령은 `state_machine` 한 곳에서만 나갑니다.** vision·beacon은 제안만 하고 MCU로 직접 보내지 않습니다.
- **MCU의 `safety`가 항상 마지막에 확인합니다.** 위험하면 Python 명령과 상관없이 `(0, 0)`으로 바꿉니다.
- Bridge 메시지는 `drive`(Python → MCU)와 `distance`(MCU → Python) 두 개뿐입니다.

### 모듈별 역할

| 위치 | 모듈 | 하는 일 |
|---|---|---|
| MPU | `vision` | 마커 검출(`detect_markers`), 정렬 보정 제안(`Aligner`), 기울어짐(`marker_skew`) |
| MPU | `beacon` | 목표 비콘 RSSI 필터·최댓값 추적 (1주차 후반부터) |
| MPU | `state_machine` | 현재 단계(탐색·정렬·후진 등)를 판단하고 `drive(left, right)` 결정 ([상태 기계 문서](state_machine.md)) |
| MPU | Bridge 송수신 | `distance`를 받은 시각과 함께 기록, `drive`를 바뀔 때 + `HEARTBEAT_SEC`마다 전송 |
| MCU | `command` | `drive`를 받아 `cmdLeft`, `cmdRight`, 받은 시각(`lastCmdMs`) 저장 |
| MCU | `sensor` | ToF 읽기, 상태 코드(OK / 범위 밖 / 오류)와 함께 `distance` 전송 |
| MCU | `safety` | watchdog, 비상정지, 센서 오류 처리 → 최종 바퀴 값 결정 |
| MCU | `motor` | `setWheels()`: 정지↔주행이 바뀔 때만 출발 보정, 그 외 `drive()` |

### MCU `safety` 우선순위

| 순서 | 조건 | 결과 |
|---|---|---|
| 1 | 마지막 `drive` 이후 `WATCHDOG_MS`(1초) 경과, 또는 부팅 후 첫 명령 전 | 정지 |
| 2 | 거리 센서 상태가 오류(`RANGE_ERROR`) | 정지 |
| 3 | 앞 거리 `< EMERGENCY_MM`(80mm)이고 전진 중 | 정지 |
| 4 | 위에 해당 없음 | Python 명령 그대로 실행 |

MCU 메인 루프는 긴 `delay()` 없이 20~50ms 주기로 반복합니다. 회전·후진 중에도 센서를 계속 읽어야 하기 때문입니다.

### 기존 코드 재사용 (거리·주행 팀 장애물 회피 코드, [#17](https://github.com/yumina0616/Hyper-AutoPark-Rover/pull/17))

| 기존 파일 | 처리 |
|---|---|
| `config.h` | 그대로 사용 + `WATCHDOG_MS`, `EMERGENCY_MM` 추가 |
| `sensor.*` | 그대로 사용 + 상태 코드 3종 반환 |
| `motor.*` | `drive`, `brake`, `startDrive`는 그대로. `moveForward`, `turnAround`는 제거 (판단은 Python이 함) |
| `behavior.*` | 제거 → `command.*`, `safety`로 대체 |

## 주차 배치와 센서 역할 (2026-11-10 결정)

- 주차칸 5개는 **진행 방향 왼쪽**에 둡니다.
- ArUco 마커는 **5개, 칸마다 다른 ID**로 각 칸 뒷벽에 붙입니다.
- 로버는 후진 주차를 하므로, 후면이 칸을 향하도록 **오른쪽(시계 방향)으로 회전**합니다.
- 회전은 정해진 각도가 아니라 **목표 ID 마커가 보이면 정지**합니다. 처음 자세가 비스듬해도 칸을 향할 수 있게 하기 위해서입니다.

| 센서 | 역할 |
|---|---|
| 비콘 | 목표 칸 번호를 알려주고, 달리는 동안 "이 근처다"를 판단 (대략적인 위치) |
| ArUco 마커 | 가까이에서 맞는 칸인지 ID로 확인하고 정렬 (정확한 위치) |
| 후방 ToF | 후진할 때 남은 거리로 감속·정지 |

비콘이 칸 하나를 정확히 구분하지 못하더라도 마커 ID가 나머지를 해결합니다.
엔코더가 없어 이동량은 시간으로 추정하므로([#9](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/9)), 정밀함이 필요한 회전·정렬·후진은 모두 센서 피드백으로 처리합니다.
상태 흐름은 [#7](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/7)에서 논의합니다.

## 하드웨어 출처

모터(TT 모터), DRV8833, 핀 배치, AA 배터리 4개, C270 웹캠, USB-C 허브는 이전 세미나 자료
[emjdp/hyper-q-rover](https://github.com/emjdp/hyper-q-rover) 1주차 슬라이드를 기준으로 했습니다.

## 확인되지 않은 연결

- 수신 XIAO → USB 시리얼 → UNO Q 연결은 아직 검증하지 않았습니다.
- 후방 카메라와 수신 보드까지 USB 허브에 꽂았을 때 포트 수와 전원이 충분한지 확인해야 합니다.
- 후방 ToF를 추가하면 전방 센서와 I2C 주소(`0x29`)가 겹칩니다. XSHUT 핀으로 주소를 바꾸는 처리가 필요합니다.
- App Lab의 객체 인식 기능과 OpenCV ArUco가 같은 카메라를 동시에 쓸 수 있는지 확인해야 합니다.
