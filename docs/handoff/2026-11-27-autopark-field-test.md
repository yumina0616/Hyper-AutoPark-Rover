# 인수인계: 자율 전면 주차 실주행 테스트 (2026-11-24 ~ 11-27)

> 다른 AI 에이전트(또는 팀원)가 이어서 작업할 수 있게, 실주행 테스트 동안 한 **모든 시도·문제·원인·수정**을 시간 순서로 정리한 문서입니다.
> 작업 규칙은 [`AGENTS.md`](../../AGENTS.md)를 먼저 읽으세요. 이 문서의 숫자는 모두 **실험 변수**이고, "검증"이라고 적은 것 외에는 실물에서 확정되지 않았습니다.

---

## 0. 한눈에 보기

| 항목 | 상태 |
|---|---|
| 작업 브랜치 | **`feat/state-machine-pose-realign`** (GitHub에 push됨, **PR 아직 없음**). 최신 커밋 `459b704` |
| 이 브랜치에 합쳐진 다른 브랜치 | `feat/vision-camera-check`(카메라 웹 도구·자세 추정), `feat/beacon-hci-scan`(새 비콘 수신기) |
| main 기준 | `352a3d2` (#27 머지까지). #23·#24·#26·#27 머지됨 |
| 테스트 | Python **212개 + 하위 10개 통과**, C++ safety **18개 통과** (깨끗한 폴더 기준, 5장 참고) |
| 실주행 결과 | 비콘 탐색 → 90° 회전 → 마커 보기까지 동작 확인. **아직 주차 완료(PARKED)까지 간 적 없음** |
| 마지막으로 막힌 곳 | 카메라 `프레임 읽기 실패` → 재시도 코드 넣고 push함(`459b704`), **실물 재확인 전** |

---

## 1. 프로젝트·하드웨어 요약

- **목표**: Arduino UNO Q 로버가 BLE 비콘 → ArUco 마커 → ToF 거리로 목표 칸에 **전면 주차**.
- **보드**: Arduino UNO Q (호스트명 `hyperq2`, SSH `arduino@hyperq2.local`)
  - MCU (STM32U585, Zephyr): 모터·ToF·안전장치. 스케치 `apps/qrover_drive/sketch/`
  - MPU (Linux, Debian, 커널 7.0): Python 판단. App Lab 컨테이너 안에서 `/app/python/`로 실행
  - 둘 사이 Bridge(RPC): Python → `drive(left, right)`, MCU → `distance(mm, status)`, `safety(reason)`
- **카메라**: Logitech C270, USB 허브 경유. **보드에서 `/dev/video2`** (`video0·1`은 qcom-venus 코덱 장치)
- **거리**: VL53L0X (전면). 상태 코드 0 정상 / 1 범위 밖 / 2 오류 (MCU가 분류)
- **비콘**: ESP32 송신기, 공식 코드 `beacon/esp32_tx/esp32_tx.ino` (**MAJOR 1, MINOR 0**, 100ms). 노트북 USB 전원(COM9)
- **모터**: TT 모터 2개, DRV8833, **엔코더 없음** → 이동·회전은 시간 기반
- **시험장 배치** (2026-11-27)
  - 별표에서 출발해 직진, **마커 벽은 진행 방향 왼쪽**, 직진 경로~벽 거리 **약 50cm**
  - 마커 순서(지나가는 순서): **1 → 2 → 3 → 0 → 4**, 마커 한 변 **5cm** (`DICT_4X4_50`)
  - **목표 마커 ID 0**, **비콘은 1번(MAJOR 1)** → `main.py`에 `TARGET_SLOT = 0`, `BEACON_SLOT = 1`로 따로 설정
  - 칸 간격(가정) 290mm = 칸 24cm + 간격 5cm (실측 안 함)

---

## 2. 보드에서 실행하는 법 (매번 이 순서)

SSH 창 2개를 씁니다. 사용자 비밀번호는 **사람이 직접** 입력합니다(에이전트가 입력 금지).

### 창 1: 비콘 수신 + 중계기 (먼저 켜기)
```bash
cd ~/Hyper-AutoPark-Rover
sudo -v                      # 비밀번호 먼저 입력 (파이프 출력에 프롬프트가 가려지는 문제 때문)
sudo python3 beacon/rx_hci.py | python3 beacon/rssi_relay.py
```
- `시각, 1, -7x` 줄이 계속 찍혀야 정상. 앱이 붙으면 `[relay] 앱 접속: 172.18.0.2`
- 0회만 나오면 `sudo btmgmt power off; sudo btmgmt power on` 후 다시 (8장 참고)
- `rx_scan.py`(예전 수신기)나 `bluetoothctl scan on`은 동시에 켜지 말 것
- **`rx_hci.py --stats`만 따로 켜면 앱으로 신호가 안 간다** (실제로 이 실수로 "첫 신호 대기 시간 초과" 여러 번 발생)

### 창 2: 주행 앱
```bash
cd ~/Hyper-AutoPark-Rover
arduino-app-cli app stop ~/Hyper-AutoPark-Rover/apps/qrover_drive
git stash && git pull && git stash pop        # main.py의 MODE 변경을 보존
python3 scripts/sync_app_modules.py           # 공용 모듈을 앱 폴더로 복사 (빼먹으면 예전 코드로 돈다)
arduino-app-cli app start ~/Hyper-AutoPark-Rover/apps/qrover_drive
docker logs -f hyper-autopark-rover-apps-qrover_drive-main-1 2>&1 | grep -E "autopark|beacon|rssi"
```
- 처음 한 번: `sed -i 's/^MODE = .*/MODE = "autopark"/' apps/qrover_drive/python/main.py`
- `main.py`를 바꾼 업데이트가 오면 `stash pop`이 충돌할 수 있음 → `git checkout -- apps/qrover_drive/python/main.py && git pull` 후 `sed` 다시
- 앱 로그는 경로로 본다: `arduino-app-cli app logs ~/Hyper-AutoPark-Rover/apps/qrover_drive` (이름 `user:qrover-drive`는 `~/ArduinoApps` 앱 전용이라 안 됨)
- 정지: `arduino-app-cli app stop ~/Hyper-AutoPark-Rover/apps/qrover_drive`
- 판단 기록만 보기: `docker logs hyper-autopark-rover-apps-qrover_drive-main-1 2>&1 | grep -E "REALIGN|ARUCO_ALIGN|FRONT_PARK|ABORT" | tail -30`
- **카메라는 한 프로그램만** 쓸 수 있다. `camera_view`가 켜져 있으면 앱이 못 읽음: `pgrep -af camera_view`
- 앱 시작 시 MCU 스케치를 다시 컴파일·업로드한다(몇 분). `ERRO ... Stopped decode loop: EOF`는 무시해도 됨

### 카메라 화면 보기 (주행과 별도)
```bash
python3 -m vision.camera_view --camera 2      # 브라우저 http://hyperq2.local:8080
```
마커 크기·자세(dist/offset/yaw)·기울기 판정을 화면에 그려 준다. **주행 앱과 동시에 쓰지 말 것.**

---

## 3. 현재 주행 흐름 (`state_machine/autopark.py`, `align_mode = "look90"`)

```
SENSOR_READY ─▶ BEACON_SEARCH ─▶ ARUCO_ALIGN(90° 보기) ─▶ FRONT_PARK ─▶ POSE_CHECK ─▶ FRONT_PARK ─▶ PARKED
                                      ▲   │ (이동이 필요하면)          │ (기울면)
                                      └── REALIGN ◀───────────────────┘
어느 단계든 위험·시간 초과 ─▶ ABORT
```

### 3-1. SENSOR_READY (출발 전 점검)
- 시작 3초는 정지 명령만(`start_delay_sec`). 그 뒤 ToF 정상값 + MCU safety `PASS`가 오면 다음 단계.
- **늦거나 오류인 값은 "기다림"으로 처리**, 5초 넘으면 ABORT (11-27 수정, 4장 #9).

### 3-2. BEACON_SEARCH (비콘 따라 직진)
- PWM **70** 직진(감속 없음). 목표 칸 RSSI가 처음 들어와야 출발, 10초 안에 안 오면 ABORT.
- **도착 판단 두 가지**
  1. **최근 1.5초에 −69dBm 이상이 4번 이상** → 그 자리에서 멈추고 도착(되돌아오기 없음) (`beacon_arrive_rssi`)
  2. 1.5초 이동평균이 최댓값보다 **4dB** 떨어진 상태가 1초 → "지나쳤다" → 최댓값 이후 달린 시간 × **1.25**만큼 후진
- 최댓값 인정 최소 −80dBm, 샘플 5개 이상부터 판단, 신호 1.5초 끊기면 ABORT, 탐색 45초 제한.
- 0.5초마다 로그: `[beacon] 최근 -78 | 평균(1.5초 7개) -76.3 | 최대 -72.4, 최대보다 3.9dB 낮음 (기준 4) | SEARCH`

### 3-3. ARUCO_ALIGN — 90° 보기 (`_step_look`)
1. **TURN_IN**: 벽 쪽(왼쪽)으로 **끊어 돌기 18번 = 약 90°** (`pulses_per_90`, 영상 추정 5°/번)
   - 끊어 돌기 1번 = PWM 50으로 0.1초 회전 + 0.3초 정지
2. **LOOK**: 멈춰서 **1.5초** 동안 새 영상의 마커를 모은다 (1초보다 오래된 영상 제외)
   - 영상이 한 장도 없으면 3초 더 기다리고 `카메라 영상이 안 들어옴`으로 ABORT
3. 판단 (`_decide_look`)
   1. 마커 자세(yaw) 평균이 **벽과 6° 넘게 틀어짐** → `round(|yaw|/5°)`번 제자리 회전해 벽과 직각 맞추기 → 다시 LOOK (최대 3번)
   2. **목표 0번 보임**: 옆 오차 ±30mm 이내 → **FRONT_PARK**. 넘으면 → 통로 쪽 90° → 옆 오차만큼 앞/뒤 이동 → TURN_IN부터 다시
   3. **0번 안 보이고 다른 번호 보임**: 가장 크게 보인(가까운) 마커 기준 `(칸 차이 × 290mm) − 그 마커와의 옆 오차`만큼 이동
   4. **아무것도 안 보임**: 벽 쪽으로 3번 더 → 반대쪽 6번 → 그래도 없으면 직각 복귀 후 앞으로 한 칸 (`peek_pulses`)
   - 이동(통로 따라 앞뒤)은 PWM 80, 거리 ÷ 100mm/s(추정) 시간, 0.2~6초. 최대 6번(`max_look_moves`)
   - 옆 오차 부호: 왼쪽 칸 배치에서 벽을 볼 때 `offset +` = 카메라가 오른쪽 = **진행 방향으로 지나침** → 뒤로
- 예전 방식(`align_mode = "sweep"`: 돌며 찾기 + P제어 정렬 + 돈 기록 거꾸로 복구)도 코드에 남아 있음

### 3-4. FRONT_PARK / POSE_CHECK / PARKED (`front_parking.py`)
- 접근 PWM **70**, ToF ≤300mm 또는 마커 ≥225px이면 **55**, 화면 오프셋으로 좌우 ±25 보정
- 마커 **150px(약 26cm)** 되면 멈추고 **자세 10프레임 평균** → 기울기 ±10°·옆 ±30mm 이내면 계속, 아니면 REALIGN(look90에선 0.8초 후진 후 다시 LOOK)
- **정지 조건**: ToF ≤ **130mm** AND 마커 ≥ 300px의 80%(=240px) AND 자세 정렬 → 새 프레임+새 ToF **3회 연속** → PARKED
- 안전정지: ToF ≤90mm, 시간 20초, 센서 불일치(ToF만 가깝고 마커가 훨씬 작음). 근거리에서 기울어 멈추면 ABORT 대신 재정렬
- 공통: MCU safety가 PASS 아니거나 신호 0.8초 이상 끊기면 ABORT. MCU는 별도로 watchdog 1초, 전진 중 80mm 미만 비상정지

---

## 4. 시간순 작업 기록 (시도 → 문제 → 원인 → 수정)

### 11-20 ~ 11-24: 준비
1. **카메라 웹 도구 `vision/camera_view.py`** (`be66d26`): SSH로는 `imshow` 창이 안 떠서 MJPEG 웹 스트리밍으로 만듦. 첫 프레임 전 브라우저 접속 시 CPU 헛도는 버그 고침.
2. **카메라 번호**: 0번 실패 → `/sys/class/video4linux/*/name`으로 확인하니 C270은 `video2`. 이후 USB가 빠져 `video2`가 사라진 적 있음(허브·전원 문제, 재연결로 해결).
3. **자세 추정 `vision/pose.py`** (`91e4e60`): solvePnP로 벽까지 거리·옆 오차·yaw. **OpenCV 4.14 IPPE가 정면 장면에서 틀린 해(재투영 오차 수백 px)**를 내서, IPPE_SQUARE 후보 + ITERATIVE 중 **재투영 오차 최소 해** 선택, 5px 넘으면 버림. 0.5px 잡음에서 40cm ±10mm/±1.5°, 60cm ±28mm/±2.7°.
4. 마커를 **10cm → 5cm**로 교체(사용자) → 기본값 50mm (`7f1ce4f`). camera_view에 기울기 판정 줄 추가 (`7551821`).
5. **실측(11-24, camera_view)**: "적당한 정지 위치"에서 마커 **약 348px, 카메라 거리 약 113mm**, 기울기 −1.7°, 옆 +1mm, `err_x` −0.26. 거리별 size×dist가 일정해 화각 45° 추정이 맞음. 사용자 판단: **기울기 ±10°까지 허용**, "그 위치보다 조금 먼 곳에서 정지".
6. 같은 위치에서 `err_x`가 −0.26(약 83px)인데 자세는 똑바름 → 화면 픽셀 기준 중심 판정이면 SAFETY_STOP이 남. → 자세 기준으로 판정하기로.

### 11-27 오전: 코드 반영
7. **`cefb2b1` 정지 기준·자세 판정·재정렬**: 정지 300px/130mm, 자세 있으면 기울기·옆 오차로 중심 판정, 26cm 지점 자세 확인, 기울면 후진·회전·옆 이동 반복(sweep 방식 REALIGN). 테스트 중 발견: 299px vs 300px 같은 작은 차이로 "센서 불일치" ABORT → `vision_agree_ratio 0.8` 추가.
8. **비콘 수신 문제**: `rx_scan.py`(bleak/BlueZ)가 **약 10초에 1번**만 받음(07:07:22→32→42). 원인: 리눅스 LE 탐색 한 바퀴(10.24초)마다 **칩의 중복 거르기**가 같은 비콘을 1번만 보고. `DuplicateData` 필터는 BlueZ 단계라 칩 필터를 못 끔.
   - **`beacon/rx_hci.py`** (`b8b051a`): HCI 소켓으로 칩에 직접 수동 스캔·duty 100%·중복 거르기 끔. 출력 형식은 `rx_scan.py`와 동일.
   - 첫 실행 `Errno 22 Invalid argument` → **HCI 필터 구조체가 16바이트인데 14바이트를 보냄**(커널 7.0은 크기 부족 시 거부) → `<IIIH2x`로 수정 (`5c6fa8b`).
   - 결과: **5초에 27회(초당 5.4), 최대 끊김 0.52초** (검증). 이후 한때 "확장(블루투스 5) 스캔" 모드로 시작하며 0회 → `btmon`으로 보니 칩은 20초에 5911회 수신 중. `btmgmt power off/on` 후 정상화. **확장 모드 0회의 근본 원인은 미해결**.
9. **비콘 실측**(사용자): 칸 안(주차 위치) 약 −65, 칸 정면 −70 초반(약 −72), 조금 전 −70 후반(약 −78), 같은 자리 ±5dB 흔들림. 채널 3개 때문에 −72/−83 두 묶음이 섞여 나옴.
   - `80a6d60`: 평균 1.0→1.5초, 하락 6→4dB, 최소 최대 −90→−80, 최소 샘플 3→5, 끊김 허용 1.0→1.5초, 감속 −70→−76. 시뮬레이션(80회) 예전 최대 150cm 빗나감 → 20cm 이내 72/80. 중앙값 필터는 오히려 나빠 평균 유지. 실험 기록 `experiments/2026-11-27-beacon-rssi.md`.

### 11-27 오후: 실주행 반복 (문제가 나올 때마다 수정·push)
| # | 증상 (로그) | 원인 | 수정 (커밋) |
|---|---|---|---|
| 1 | 앱 시작 직후 꺼짐 `IndexError: 3` (`autopark_mode.py` 12줄) | 컨테이너에서 파일이 `/app/python/`이라 `parents[3]` 없음 | 레포 폴더가 실제 있을 때만 경로 추가 (`65ddab2`) |
| 2 | 직진하다 `ABORT: 거리 센서 오류` | VL53L0X가 앞이 비었거나 가장자리에서 RangeStatus 1·2를 냄 → 오류로 분류 | MCU `classifyRange`: 1·2·4 → 범위 밖, 3(너무 가까움) → 0mm, 5 이상만 오류 (`2a23f29`). 손 대기 시험으로 확인 |
| 3 | 출발 전 `ABORT: MCU 안전상태 피드백이 오래됨` | 앱 시작 직후 카메라 스레드(OpenCV)가 CPU를 잡아 신호가 0.3초 넘게 밀림(추정) | SENSOR_READY에선 늦은 값은 기다림, `stale_sec` 0.3→0.8 (`b2b72b3`) |
| 4 | 마커 찾기 회전이 너무 빨라 못 찾음, 접근 115가 너무 빠름 | 설정값 | 회전 전부 절반, 접근 115→70, 감속 95→55 (`42ff787`) |
| 5 | **회전 방향이 반대** (왼쪽 명령에 오른쪽 회전) | 좌우 모터 배선이 코드와 반대 (직진은 정상) | MCU 핀 교체: 왼쪽 6·9 / 오른쪽 5·3 (`1dfd8c0`) |
| 6 | 목표 ID 못 찾으면 그냥 중단 | 재탐색 기능 없음 | 못 찾으면 되돌아 회전 → 한 칸 이동 → 다시 찾기 (`1dfd8c0`) |
| 7 | 로버가 엉뚱한 마커(1번)를 찾음 | `TARGET_SLOT` 하나로 비콘 번호·마커 번호를 같이 씀. 실제는 마커 0 / 비콘 1 | `BEACON_SLOT` 분리 (`5a3683d`) |
| 8 | 재탐색할수록 직선에서 벗어남(영상) | PWM 30 연속 회전이 걸렸다 미끄러져 각도가 매번 다름, "같은 시간" 되돌리기로는 복구 안 됨. ID 17 오인식 | 끊어 돌기(PWM 50, 0.15초 돌기·0.35초 정지)·횟수 기록·같은 횟수 복구, 0~4만 인정 (`39666dd`), 배치 순서 1,2,3,0,4 (`9e1684e`) |
| 9 | 목표 마커를 봤다가 놓치고 ABORT | 정렬은 여전히 연속 P제어 → 카메라 지연으로 지나침. 본 뒤 놓치면 재탐색 없음 | **MCU 출발 킥이 120ms 최대 출력**이라 짧은 회전도 크게 돎 → 40ms로. 정렬도 끊어 돌기(0.1초/거의 맞으면 0.05초, 1초 정지), 모든 회전 기록 후 거꾸로 재생, 놓쳐도 재탐색, 비콘 55/복귀 1.25배 (`aaaa369`) |
| 10 | 4,3,2,1을 다 보고 "앞으로 3칸" → 장애물 ABORT | 마지막에 본(1번) 기준으로 칸 수 계산 | 가장 크게 보인(가까운) 마커 기준 (`52e8a98`) |
| 11 | 비콘 복귀가 거의 안 됨, 평균이 들쭉날쭉 | PWM 55 후진에서 모터가 걸림(추정). 시뮬레이션상 속도 50~80 정확도 차이 거의 없음 | 비콘 앞뒤 PWM 70, 평균 로그 추가, **look90 방식 도입** (`d2dbb01`) |
| 12 | 비콘 근처에서 −69가 찍히기 시작(사용자 관찰), 마커 안 보이면 무조건 직진 | — | −69 이상 4번/1.5초면 도착, 안 보이면 좌우 확인 후 이동 (`de6be6d`) |
| 13 | "마커 안 보임 → 앞으로"를 반복하며 지나침, 마지막 `RuntimeError: 카메라 프레임 읽기 실패` | 카메라 스레드가 첫 읽기 실패에 죽음 → 영상 0장을 "마커 없음"으로 판단 | 영상 없으면 이동 안 하고 3초 대기 후 ABORT (`b127dba`), 읽기 실패 20회 재시도 + 2번 다시 열기 (`459b704`). **실물 미확인** |

### 실주행에서 관찰된 것 (사용자 영상·로그 기준)
- 끊어 돌기 1번 ≈ **5°**, 18번 ≈ 90° (11-27 영상, 30~50초 구간. 추정)
- 비콘 RSSI는 시작점(별표)에서도 −72/−83이 섞여 나옴. 시작점 평균 약 −77
- 비콘 판단은 2초쯤 늦어 그만큼 지나침 → 되돌아오기 필요, 그러나 후진이 덜 됨
- 0번 마커가 바로 근처인데도 안 잡힌 적 있음 → **0번 마커 인식 자체를 camera_view로 확인 필요** (인쇄 상태·반사·사전)

---

## 5. 코드 위치와 테스트

| 파일 | 역할 |
|---|---|
| `state_machine/autopark.py` | 전체 상태 기계. 설정 `AutoParkConfig`. look90(`_step_look`, `_decide_look`, `_look_move`), sweep(`_align_pulse`, `_start_research`), 자세 확인·재정렬 |
| `state_machine/front_parking.py` | 접근·정지 조건, `is_aligned`, `stop_reason` |
| `state_machine/beacon_search.py` | 비콘 직진·도착·복귀 (`arrive_rssi`, `return_gain`) |
| `beacon/peak_detector.py` | RSSI 이동평균·최댓값·지나침 판단 (`filter`: mean/median/upper) |
| `beacon/rx_hci.py` | HCI 직접 수신기 (`--stats`) |
| `vision/pose.py` | solvePnP 자세 추정 |
| `vision/aruco_auto_align.py` | `ArucoAlignStage`(sweep용), `ArucoCamera`(영상 스레드, 재시도) |
| `vision/camera_view.py` | 웹 카메라 화면 도구 |
| `apps/qrover_drive/python/main.py` | `MODE`, `TARGET_SLOT=0`, `BEACON_SLOT=1`, `CAMERA_INDEX=2` |
| `apps/qrover_drive/python/autopark_mode.py` | 앱 실행기, 비콘 평균 로그 `_log_beacon` |
| `apps/qrover_drive/python/bridge_link.py` | Bridge, `stale_sec 0.8` |
| `apps/qrover_drive/sketch/` | MCU: `config.h`(핀·킥 40ms·watchdog 1s·비상 80mm), `safety.cpp`(`applySafety`, `classifyRange`) |
| `scripts/sync_app_modules.py` | 공용 모듈을 앱 폴더로 복사 (pose.py 포함) |

**테스트 돌리는 법** (중요): 로컬에 예전 sync 복사본(`apps/qrover_drive/python/beacon|state_machine|vision/`)이 있으면 **한 번에 `pytest`하면 원본이 가려져 수집 오류**가 난다. 깨끗한 폴더에서 돌릴 것:
```bash
git worktree add --detach <임시폴더> HEAD && cd <임시폴더> && python -m pytest -q
g++ -std=c++11 -I apps/qrover_drive/sketch apps/qrover_drive/tests/test_safety.cpp apps/qrover_drive/sketch/safety.cpp -o ts && ./ts
```
(근본 해결: 앱 테스트가 `sys.path`에 앱 폴더를 레포보다 먼저 넣는 순서를 고치기 — 미해결)

주요 테스트 파일: `test_autopark_look90.py`(90° 보기, 합성 꼭짓점), `test_autopark_research.py`(sweep 재탐색·끊어 돌기), `test_autopark_realign.py`(자세 확인), `test_beacon_search.py`(도착·복귀 배율), `beacon/test_rx_hci.py`(HCI 패킷 해석), `vision/test_pose.py`.

---

## 6. 실측이 필요한 값 (모두 `TODO`)

| 값 | 지금 | 재는 법 |
|---|---|---|
| `pulses_per_90` | 18 | 바닥에서 끊어 돌기 N번 후 각도 측정 |
| `realign_mm_per_sec` (PWM 80 직진 속도) | 100 | 1초 직진 거리 |
| `slot_pitch_mm` | 290 | 마커 중심 간 거리 실측 |
| `beacon_return_gain` | 1.25 | 지나친 거리 vs 후진 거리 |
| `beacon_arrive_rssi` | −69 (4회/1.5초) | 목표 앞·조금 전·지나친 위치에서 분포 기록 |
| `park_distance_mm` | 130 | **같은 위치의 ToF 값** (카메라 거리로 추정한 값임) |
| `marker_stop_side_px` | 300 | 정지 위치 size 재확인 |
| `lateral_tol_mm` | 30 | 로버 폭·칸 폭으로 계산 |
| `MOTOR_START_TIME` | 40ms | 바닥에서 출발 실패하면 늘림 |
| 최소 PWM (회전 30·직진 55·후진) | ? | 바닥에서 안 움직이는 최저값 |
| `camera_hfov_deg` | 45 | 체스판 카메라 보정 |
| 카메라·ToF가 로버 중심에서 떨어진 거리 | ? | 자로 재서 offset 보정 |

---

## 7. 지금 바로 해야 할 일 (우선순위)

1. **카메라 안정성 확인**: 최신(`459b704`) 받아서 주행. `pgrep -af camera_view`로 카메라 점유 확인. 그래도 `프레임 읽기 실패`면 허브·전원(보조배터리) 문제일 가능성. `docker exec ... ls /dev/video*`로 컨테이너 안에서 장치 보이는지 확인
2. **0번 마커 인식 확인** (`camera_view`)
3. **`pulses_per_90` 실측** — look90 정확도의 핵심
4. 끝까지(PARKED) 한 번 가 보기 → 결과를 `experiments/YYYY-MM-DD-*.md`로 기록
5. 안정되면 PR 정리: `feat/state-machine-pose-realign`은 기능이 많이 섞여 있음(카메라 도구·자세·비콘 수신기·look90·MCU 수정). 팀 리뷰를 위해 나눌지 사용자와 상의

## 8. 알려진 문제·주의점

- `rx_hci.py`가 "확장(블루투스 5)" 모드로 시작하면 0회 수신되던 현상: `btmgmt power off/on`으로 우회, 근본 원인 미해결 (legacy 경로에서만 검증)
- `sudo ... | python3 rssi_relay.py`: 비밀번호 프롬프트가 중계기 출력에 가려짐 → `sudo -v` 먼저
- App Lab 컨테이너는 별도 네트워크: 앱 → 중계기 `172.18.0.1:8765`(게이트웨이)로 접속됨 (검증)
- App Lab에서 여러 `.cpp/.h` 스케치 빌드는 **동작 확인됨** (이슈 #1 핵심 항목)
- 보드 시계는 UTC. RSSI 시각은 `+00:00` 시간대 포함 출력(#27)
- MCU `setWheels`는 정지→주행마다 최대 출력 킥(지금 40ms) → 짧은 회전 각도에 영향
- 로컬 미커밋 파일: `beacon/esp32_rx/`, `beacon/esp32_tx/` PlatformIO 사본(이 사본은 MINOR 1이라 수신기가 무시함, 쓰지 말 것)
- 이전 대화에서 잘못 설명된 것: 이슈 #9·PR #11 문서·노션에 "후진 시 조향 부호 반대" 서술(틀림, 전면 주차로 바뀌어 무관)

## 9. 작업 규칙 (사용자 선호, 꼭 지킬 것)

- **항상 한국어**, 쉬운 말. 영어 문장 금지 (사용자가 여러 번 지적)
- git 명령 실행 전 무슨 명령인지 한 줄 설명 (AGENTS.md)
- 커밋 메시지 `타입(영역): 한국어 설명`, **Claude 공동 작성자 표시 금지**
- main 직접 커밋·force push·머지 금지. 실주행 테스트 중에는 **기능 브랜치 push는 묻지 않고 바로**(사용자 허락, 2026-11-27)
- PR 본문은 초안을 보여 주고 사용자가 올리는 쪽을 선호
- 보드 명령은 사용자가 SSH로 직접 실행 → 에이전트는 **복사해 붙일 명령을 순서대로** 준다. 비밀번호는 절대 대신 입력하지 않음
- 바퀴 들기 같은 안전 안내를 반복하지 말 것 (사용자가 바닥에서 시험하길 원함). 단 위험한 변경은 한 번은 알림
