# Arduino UNO Q 전면 주차 - App Lab 프로젝트

## 파일 위치
- `sketch/sketch.ino`: MCU(STM32), VL53L0X + IR + DRV8833 + Bridge
- `python/main.py`: MPU(Linux), C270 + ArUco + Bridge

## 연결
- DRV8833: AIN1=D5, AIN2=D3, BIN1=D6, BIN2=D9
- VL53L0X: SDA=D20, SCL=D21, I2C 0x29
- 전방 IR: 좌=D2, 우=D4 (LOW=감지)
- 후방 IR: 좌=D7, 우=D8 (전면 주차에서 읽기만 할당; 사용하지 않음)
- Logitech C270: USB 허브를 통해 UNO Q Linux에 연결

## 설치와 실행
1. Arduino App Lab에서 새 App을 만들고 `sketch.ino`, `main.py` 내용을 해당 위치에 붙여 넣습니다.
2. MCU: Adafruit VL53L0X, Arduino_RouterBridge 라이브러리 설치/확인.
3. Linux Python: `cv2.aruco` 사용 가능한 OpenCV와 numpy 설치/확인.
4. `main.py`: `TARGET_MARKER_ID` (0~4), `CAMERA_INDEX` 설정.
5. 주행 전에 바퀴를 바닥에서 띄우고 방향 및 IR 감지 극성을 확인합니다.
6. 적절한 제동 거리 확보 후 `sketch.ino`에서 `PARK_DISTANCE_MM`, `EMERGENCY_DISTANCE_MM`, `MARKER_STOP_SIDE_PX`를 실측값으로 조정합니다.
7. 앱을 실행합니다. 마커가 보이지 않으면 출발하지 않습니다.

## 판단 규칙
- 목표 마커를 보고 중앙에 정렬한 뒤 접근합니다.
- 목표 마커의 네 변 평균 픽셀 길이 ≥ 140, ToF ≤ 150mm, 중심 오차 ≤ 45픽셀이 3회 연속이면 주차 완료.
- IR 장애물 감지, ToF ≤ 90mm, ToF 오류: 즉시 래치 정지.
- 마커 미검출 또는 600ms 이상 업데이트 없음: 모터 브레이크. 마커 재인식 시 재개 가능.
- ToF ≤ 150mm인데 비전 조건 불일치: 안전 정지 래치.
- 주차 완료/안전 래치 이후 재시작하려면 MCU를 리셋해야 합니다.

## 주의
- 위 픽셀/거리 임계값은 **예시**이며 실제 센서 배치와 정지 관성에 맞게 보정해야 합니다.
- `PARK_DISTANCE_MM`는 차체 범퍼가 아닌 VL53L0X 센서에서 측정한 거리입니다.
- 광학식 VL53L0X 거리 센서와 단순 IR 디지털 장애물 센서를 혼동하지 마십시오.
- C270의 카메라 중심과 로봇 중심은 달라질 수 있습니다. 카메라/마커 높이와 각도도 조정해야 합니다.
- 현재 코드는 정밀한 측면 주차나 조향 경로 계획이 아닌 정면 마커에 대한 직진 접근 및 회전 정렬입니다.
- Linux 영상 프레임 지연, RPC, 센서 읽기에 걸리는 시간만큼 차량이 이동할 수 있으므로 저속으로 시험합니다.
