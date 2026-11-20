"""Q-Rover: ArUco 마커 중앙 정렬(제자리 회전) 제어.

이 파일은 기본적으로 모터를 실제 구동하지 않는 미리보기 모드입니다.
실제 구동에는 QroverMotor.set_wheels()를 하드웨어에 맞게 구현해야 합니다.

사용:
    python aruco_auto_align.py --camera 0 --target-id 0
    python aruco_auto_align.py --self-test
"""

from __future__ import annotations

import argparse
import time
import unittest
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class AlignConfig:
    # 화면 반쪽 너비를 1로 정규화: 640px 영상에서 0.05 => 중심에서 ±16px
    tolerance_ratio: float = 0.05
    stable_frames: int = 5
    proportional_gain: float = 0.32
    min_turn_speed: float = 0.12  # 실제 모터 특성에 맞춰 조정할 값 (0~1)
    max_turn_speed: float = 0.25
    max_duration_sec: float = 15.0

    # 카메라에서 마커가 아직 한 번도 보이지 않을 때 탐색 회전할지 선택.
    # 장애물과 모터 회전 방향을 점검하기 전에는 False 유지.
    search_if_missing: bool = False
    search_direction: str = "right"
    search_turn_speed: float = 0.12


@dataclass(frozen=True)
class WheelCommand:
    left: float  # 양수: 전진, 음수: 후진, 0: 정지
    right: float
    state: str
    error_ratio: Optional[float]


class ArucoAlignController:
    """입력: 마커 중심 x좌표 + 영상 너비 / 출력: 좌·우 바퀴 속도.

    일반적인 차동구동 기준으로 '마커가 화면 오른쪽'이면
    왼쪽 바퀴 전진(+), 오른쪽 바퀴 후진(-)하여 오른쪽으로 회전합니다.
    실제 배선·모터 부호가 다를 수 있으니 바퀴를 띄워 먼저 확인하세요.
    """

    def __init__(self, config: AlignConfig = AlignConfig()):
        if not 0 < config.tolerance_ratio < 1:
            raise ValueError("tolerance_ratio는 0~1 사이여야 합니다.")
        if config.stable_frames < 1 or config.max_duration_sec <= 0:
            raise ValueError("stable_frames와 max_duration_sec는 양수여야 합니다.")
        if config.search_direction not in ("left", "right"):
            raise ValueError("search_direction은 left 또는 right입니다.")
        if not 0 <= config.min_turn_speed <= config.max_turn_speed <= 1:
            raise ValueError("회전 속도는 0~1 범위이며 최솟값 <= 최댓값이어야 합니다.")
        if not 0 <= config.search_turn_speed <= 1:
            raise ValueError("탐색 속도는 0~1이어야 합니다.")
        self.config = config
        self.started_at: Optional[float] = None
        self.stable_count = 0
        self.seen_target = False
        self.finished_state: Optional[str] = None

    @staticmethod
    def _turn(direction: str, speed: float, state: str, error: Optional[float]):
        # 차동구동(전진=양수) 가정. 오른쪽 회전: 왼쪽 전진 / 오른쪽 후진.
        if direction == "right":
            return WheelCommand(speed, -speed, state, error)
        return WheelCommand(-speed, speed, state, error)

    def update(
        self,
        marker_center_x: Optional[float],
        frame_width: int,
        now: Optional[float] = None,
    ) -> WheelCommand:
        if frame_width <= 0:
            raise ValueError("frame_width는 양수여야 합니다.")
        now = time.monotonic() if now is None else now
        if self.started_at is None:
            self.started_at = now

        if self.finished_state is not None:
            return WheelCommand(0.0, 0.0, self.finished_state, None)

        if now - self.started_at >= self.config.max_duration_sec:
            self.finished_state = "TIMEOUT"
            return WheelCommand(0.0, 0.0, "TIMEOUT", None)

        if marker_center_x is None:
            self.stable_count = 0
            if self.config.search_if_missing and not self.seen_target:
                return self._turn(
                    self.config.search_direction,
                    self.config.search_turn_speed,
                    "SEARCHING",
                    None,
                )
            # 한 번 본 타깃이 사라져도 무작정 회전하지 않고 멈춤.
            return WheelCommand(0.0, 0.0, "MARKER_MISSING", None)

        self.seen_target = True
        half_width = frame_width / 2.0
        error = (marker_center_x - half_width) / half_width

        if abs(error) <= self.config.tolerance_ratio:
            self.stable_count += 1
            if self.stable_count >= self.config.stable_frames:
                self.finished_state = "ALIGNED"
                return WheelCommand(0.0, 0.0, "ALIGNED", error)
            return WheelCommand(0.0, 0.0, "CENTERING_CHECK", error)

        self.stable_count = 0
        speed = min(
            self.config.max_turn_speed,
            max(
                self.config.min_turn_speed,
                self.config.proportional_gain * abs(error),
            ),
        )
        direction = "right" if error > 0 else "left"
        return self._turn(direction, speed, "TURNING_" + direction.upper(), error)


def target_center_from_detections(corners, ids, target_id: int) -> Optional[float]:
    """OpenCV detectMarkers의 (corners, ids)에서 목표 ID 중심 x만 추출."""
    if ids is None:
        return None
    for i, marker_id in enumerate(ids.flatten()):
        if int(marker_id) == target_id:
            points = corners[i].reshape(4, 2)
            return float(sum(float(point[0]) for point in points) / 4.0)
    return None


class PreviewMotor:
    """명령만 출력. 실제 바퀴는 절대 움직이지 않습니다."""

    def __init__(self):
        self.last_print_at = float("-inf")
        self.last_state = None

    def set_wheels(self, left: float, right: float, state: str):
        now = time.monotonic()
        if state != self.last_state or now - self.last_print_at >= 0.5:
            print("[미리보기] 상태=%s | 왼쪽=%+.2f, 오른쪽=%+.2f" % (state, left, right))
            self.last_print_at = now
            self.last_state = state

    def stop(self):
        self.set_wheels(0.0, 0.0, "STOP")


class QroverMotor:
    """실제 모터 드라이버 연결부. 배선/드라이버 API 확인 후 구현."""

    def __init__(self):
        raise NotImplementedError(
            "Q-Rover 모터 드라이버 모델, 모터 제어 함수, 배선 확인 후 연결해야 합니다."
        )

    def set_wheels(self, left: float, right: float, state: str):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError


def run(camera_index: int, target_id: int, show_window: bool, search: bool):
    import cv2  # 테스트만 할 때에는 OpenCV가 필요 없도록 지연 import

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary)
        detect = detector.detectMarkers
    else:
        detect = lambda frame: cv2.aruco.detectMarkers(frame, dictionary)

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError("카메라 %s를 열지 못했습니다." % camera_index)

    motor = PreviewMotor()  # 실제 구동할 때 하드웨어에 맞는 모터 클래스로 교체
    controller = ArucoAlignController(AlignConfig(search_if_missing=search))
    try:
        while True:
            success, frame = cap.read()
            if not success:
                print("카메라 프레임 수신 실패 → 정지")
                break

            corners, ids, _ = detect(frame)
            center_x = target_center_from_detections(corners, ids, target_id)
            command = controller.update(center_x, frame.shape[1])
            motor.set_wheels(command.left, command.right, command.state)

            if show_window:
                frame_center = frame.shape[1] // 2
                cv2.line(frame, (frame_center, 0), (frame_center, frame.shape[0]), (255, 0, 0), 1)
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                if center_x is not None:
                    cx = int(center_x)
                    cv2.circle(frame, (cx, frame.shape[0] // 2), 6, (0, 0, 255), -1)
                cv2.putText(frame, "TARGET ID: %s / %s" % (target_id, command.state),
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.imshow("Q-Rover ArUco Auto Align (Preview)", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("사용자가 q로 종료 → 정지")
                    break

            if command.state in ("ALIGNED", "TIMEOUT"):
                print("정렬 결과:", command.state)
                break
    except KeyboardInterrupt:
        print("Ctrl+C → 정지")
    finally:
        motor.stop()
        cap.release()
        if show_window:
            cv2.destroyAllWindows()


class ControllerTests(unittest.TestCase):
    def test_marker_right_turn_right(self):
        c = ArucoAlignController()
        cmd = c.update(500, 640, now=0.0)
        self.assertEqual(cmd.state, "TURNING_RIGHT")
        self.assertGreater(cmd.left, 0)
        self.assertLess(cmd.right, 0)

    def test_marker_left_turn_left(self):
        c = ArucoAlignController()
        cmd = c.update(120, 640, now=0.0)
        self.assertEqual(cmd.state, "TURNING_LEFT")
        self.assertLess(cmd.left, 0)
        self.assertGreater(cmd.right, 0)

    def test_five_stable_center_frames_stop_and_latch(self):
        c = ArucoAlignController()
        for i in range(4):
            cmd = c.update(325, 640, now=float(i))
            self.assertEqual(cmd.state, "CENTERING_CHECK")
            self.assertEqual((cmd.left, cmd.right), (0.0, 0.0))
        cmd = c.update(325, 640, now=4.0)
        self.assertEqual(cmd.state, "ALIGNED")
        self.assertEqual(c.update(600, 640, now=5.0).state, "ALIGNED")

    def test_lost_after_seen_stops(self):
        c = ArucoAlignController(AlignConfig(search_if_missing=True))
        self.assertEqual(c.update(None, 640, now=0.0).state, "SEARCHING")
        c.update(500, 640, now=1.0)
        cmd = c.update(None, 640, now=2.0)
        self.assertEqual((cmd.left, cmd.right), (0.0, 0.0))

    def test_timeout_stops(self):
        c = ArucoAlignController()
        c.update(500, 640, now=0.0)
        cmd = c.update(500, 640, now=15.0)
        self.assertEqual(cmd.state, "TIMEOUT")
        self.assertEqual((cmd.left, cmd.right), (0.0, 0.0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ArUco 마커를 보며 제자리 회전해 중앙 정렬")
    parser.add_argument("--camera", type=int, default=0, help="웹캠 0 / USB 카메라 1 등")
    parser.add_argument("--target-id", type=int, default=0, help="정렬할 목표 ArUco ID")
    parser.add_argument("--no-window", action="store_true", help="GUI 없는 로버 환경")
    parser.add_argument("--search", action="store_true", help="보이지 않을 때도 탐색 회전(실구동 시 주의)")
    parser.add_argument("--self-test", action="store_true", help="카메라 없이 제어 로직 자체 점검")
    args = parser.parse_args()
    if args.self_test:
        unittest.main(argv=["aruco_auto_align.py"], exit=False)
    else:
        run(args.camera, args.target_id, not args.no_window, args.search)



