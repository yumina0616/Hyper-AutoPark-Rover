# 모듈 인터페이스

> **상태 (2026-11-10 기준):** 초안입니다. 값은 이전 세미나 코드
> [emjdp/hyper-q-rover](https://github.com/emjdp/hyper-q-rover)에서 가져온 초기값이며, 팀 합의 후 "확정"으로 바꿉니다.
> 구조는 [아키텍처 문서](architecture.md)를 참고하세요.

모듈끼리 **무엇을, 어떤 이름과 단위로, 어느 방향으로** 주고받는지 정하는 문서입니다.
여기 적힌 이름과 단위가 코드와 다르면 코드가 아니라 이 문서를 먼저 고치고 팀에 알립니다.

## 결정된 사항

| 항목 | 결정 | 상태 |
|---|---|---|
| 바퀴 값 계산 위치 | Python 상태 기계가 `drive(left, right)`로 바퀴 값을 보내고, MCU는 실행만 함 | 확정 (2026-11-10) |
| 안전장치 위치 | 비상정지·watchdog은 MCU가 항상 우선 적용 | 확정 (2026-11-10) |
| 측정 시각 | Python이 값을 **받은 순간** `time.time()`으로 기록. 오래된 값은 이 시각으로 판단 | 선택 방향 |
| 거리값 상태 | 거리값과 상태 코드(`status`)를 따로 보냄 | 선택 방향 |
| 회전 방식 | 연속 회전 `spin`, 목표 마커가 보이면 정지. 최대 회전 시간은 MCU가 지킴. 기본 방향은 오른쪽(시계) | 선택 방향 ([#8](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/8)) |
| 마커 | 5개, 칸마다 다른 ID. 비콘이 목표 칸 번호를 알려줌 | 확정 (2026-11-10) |
| 이동량 추정 | 엔코더 없음. 시간 기반으로 추정하고 `Odometry` 모듈로 분리 | 선택 방향 ([#9](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/9)) |

---

## 1. 모터 (motion, MCU)

### 핀과 속도 규약

| 항목 | 값 | 비고 |
|---|---|---|
| 왼쪽 모터 | `AIN1=5`, `AIN2=3` | UNO Q의 PWM 핀은 3·5·6·9뿐이라 변경 불가 |
| 오른쪽 모터 | `BIN1=6`, `BIN2=9` | |
| 속도 범위 | `-255 ~ 255` | `+` = 앞으로, `0` = 힘 빼기 |
| PWM 방식 | 반전 PWM (slow decay) | |

### 함수

| 함수 | 동작 |
|---|---|
| `drive(left, right)` | 양쪽 바퀴 속도 지정 |
| `coast()` | 힘을 빼서 굴러가게 둠 |
| `brake()` | 네 핀 모두 HIGH로 100ms 급제동 후 `coast()` |
| `startDrive(left, right)` | 출발할 때 120ms 동안 ±255로 밀고 나서 목표 속도로 |
| `setWheels(left, right)` | 정지↔주행이 바뀌는 순간에만 `brake()`/`startDrive()` 호출 |
| `spin(direction, pwm)` | 제자리 연속 회전. 정지는 `drive(0, 0)`. 이름·인자는 [#8](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/8)에서 확정 |

### 튜닝 값 (실험으로 결정)

| 이름 | 초기값 | 비고 |
|---|---|---|
| `SPEED` | `100` | 주차 상황에 맞춰 조정 예정. `TODO: 실험으로 결정` |
| `KICK_MS` | `120` | 출발 보정 시간 |
| `MIN_PWM` | 미측정 | 처음 굴러가기 시작하는 PWM. 직진과 제자리 회전을 따로 측정 |

---

## 2. 거리 센서 (sensors, MCU)

| 항목 | 값 |
|---|---|
| 센서 | VL53L0X, I2C 주소 `0x29` |
| I2C | 100kHz (400kHz면 초기화 실패), SDA=D20, SCL=D21 |
| 단위 | `mm` (int) |
| 측정 주기 | 약 10~20Hz |

### 상태 코드 (`status`)

| 값 | 이름 | 뜻 | `dist_mm` | 판단 |
|---|---|---|---|---|
| `0` | `RANGE_OK` | 정상 측정 | 측정값 | 사용 |
| `1` | `RANGE_OUT_OF_RANGE` | 범위 밖 (약 2m 이상, 센서 `RangeStatus==4`) | `-1` | 앞이 비어 있음 |
| `2` | `RANGE_ERROR` | 측정 실패 (센서 `RangeStatus` 1~3, 응답 없음) | `-1` | 믿지 말고 정지 |

이전 코드는 `-1` 하나로 "범위 밖"만 표시했습니다. 센서 고장을 "비어 있음"으로 착각하지 않도록 상태를 나눕니다.

---

## 3. Bridge 메시지 (MCU ↔ Python)

받는 쪽이 `Bridge.provide("이름", 함수)`로 등록하고, 보내는 쪽이 `Bridge.notify("이름", 값...)`으로 호출합니다.
**이름 철자가 한 글자라도 다르면 에러 없이 조용히 동작하지 않습니다.**

| 이름 | 방향 | 인자 | 주기 | 상태 |
|---|---|---|---|---|
| `distance` | MCU → Python | `dist_mm`(int), `status`(int) | 측정할 때마다 | 선택 방향 |
| `drive` | Python → MCU | `left`, `right` (`-255 ~ 255`) | 바뀔 때 + `HEARTBEAT_SEC`마다 | 선택 방향 |
| `spin` | Python → MCU | `direction`, `pwm`, `max_ms` (예시) | 회전 시작할 때 | 미정 ([#8](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/8)) |

`drive`는 값이 그대로여도 `HEARTBEAT_SEC`마다 다시 보냅니다. 그러지 않으면 MCU watchdog이 "Python이 멈췄다"고 판단해 정지합니다.

이전 코드의 `set_vision_block`, `set_target`, `telemetry`는 이 프로젝트에서 쓰지 않습니다.

---

## 4. 안전장치

| 이름 | 초기값 | 위치 | 동작 |
|---|---|---|---|
| `EMERGENCY_MM` | `80` | MCU | 전방 거리가 이 값 미만이면 무엇을 하든 정지 |
| `WATCHDOG_MS` | `1000` | MCU | 마지막 `drive` 명령 후 이 시간이 지나면 정지 |
| `HEARTBEAT_SEC` | `0.3` | Python | `drive` 재전송 주기 |
| `START_DELAY` | `3.0` | Python | 앱 시작 후 출발 전 대기 (로버에서 손 떼는 시간) |
| `STALE_SEC` | `0.3` | Python | 거리값을 이 시간 넘게 못 받으면 오래된 값으로 보고 정지. `TODO: 실험으로 결정` |

후방 ToF가 추가되면 `EMERGENCY_MM`을 후진 방향에도 적용할지 정해야 합니다.

---

## 5. ArUco (vision, Python)

이전 코드에 없는 부분이라 새로 정합니다.

```python
detect_markers(frame) -> list[Marker]
```

| 필드 | 타입 | 뜻 |
|---|---|---|
| `id` | int | 마커 번호 |
| `err_x` | float | 좌우 오차. `-1.0`(화면 왼쪽 끝) ~ `0`(가운데) ~ `+1.0`(오른쪽 끝) |
| `size_px` | float | 마커 한 변의 픽셀 길이 (가까울수록 큼) |
| `t` | float | 검출 시각 (`time.time()`) |

| 항목 | 값 | 상태 |
|---|---|---|
| 카메라 | C270 USB 웹캠 (전방, 1주차 개발용) | 이전 로버 기준 |
| 마커 개수 | 5개, 칸마다 다른 ID. 목표 ID가 아닌 마커는 무시 | 확정 (2026-11-10) |
| 마커 사전 | 미정 (예: `DICT_4X4_50`) | [#6](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/6)에서 결정 |
| 마커 크기·높이 | 미정 | [#6](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/6)에서 결정 |
| 오차 표현 | 이전 코드는 정수 `-100 ~ +100`. 여기서는 `-1.0 ~ +1.0` 제안 | 팀 합의 필요 |

---

## 6. 이동량 추정 (Odometry, Python)

엔코더가 없어 지금은 **속도 × 시간**으로 추정합니다. 나중에 엔코더 모터로 바꿔도 상태 기계 코드를 고치지 않도록 구현을 분리합니다. ([#9](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/9))

```python
class Odometry:
    def reset(self): ...
    def distance_cm(self) -> float: ...   # 지금 구현: CM_PER_SEC × 주행 시간
    def heading_deg(self) -> float: ...   # 지금 구현: DEG_PER_SEC × 회전 시간
```

| 이름 | 초기값 | 비고 |
|---|---|---|
| `CM_PER_SEC` | 미측정 | 직진 속도. `TODO: 실험으로 결정` |
| `DEG_PER_SEC` | 미측정 | 제자리 회전 속도. `TODO: 실험으로 결정` |
| `TRIM_LEFT`, `TRIM_RIGHT` | `0` | 직진할 때 휘어짐을 보정하는 값. `TODO: 실험으로 결정` |

---

## 7. 비콘 (beacon, Python)

1주차 후반 부품 도착 후 정합니다.

- 비콘은 **목표 칸 번호**를 함께 보냅니다 (예: iBeacon의 major/minor 값). 로버는 이 번호와 같은 ID의 마커를 찾습니다.
- 수신 XIAO → USB 시리얼 → Python으로 들어오는 형식(ID, RSSI, 칸 번호)은 미정입니다.

---

## 8. 아직 정하지 않은 것

- [ ] 회전 함수·메시지의 이름과 인자 ([#8](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/8))
- [ ] 비콘 메시지 형식 (1주차 후반, 부품 도착 후)
- [ ] ArUco 마커 사전·크기·높이 ([#6](https://github.com/yumina0616/Hyper-AutoPark-Rover/issues/6))
- [ ] 오차 표현 범위 (`-1.0 ~ 1.0` vs `-100 ~ 100`)
- [ ] 후방 ToF 추가 시 Bridge 메시지 구분 방법 (예: `distance`에 센서 위치 인자 추가)
