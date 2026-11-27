"""전면 주차 상태 기계 단위 테스트 (state_machine/test_front_parking.py)"""

import unittest

from state_machine.front_parking import (
    DistanceObservation,
    FrontParkingConfig,
    FrontParkingController,
    MarkerObservation,
    ParkingState,
    RANGE_ERROR,
    RANGE_OK,
    RANGE_OUT_OF_RANGE,
)


class TestFrontParkingController(unittest.TestCase):
    def setUp(self) -> None:
        self.cfg = FrontParkingConfig()
        self.ctrl = FrontParkingController(self.cfg)

    def test_repeated_same_observation_counts_only_once(self) -> None:
        # 같은 프레임(t=1.0)과 같은 ToF(t=1.0)가 반복되면 1번만 카운트되어야 한다
        for i in range(5):
            out = self.ctrl.update(
                MarkerObservation(0, side_px=310.0, offset_px=5.0, t=1.0),
                DistanceObservation(dist_mm=120, status=RANGE_OK, t=1.0),
                now=1.0 + i * 0.02,
            )
        self.assertEqual(out.parking_samples, 1)
        self.assertEqual(out.state, ParkingState.APPROACH)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_three_fresh_samples_reach_parked(self) -> None:
        # 새 프레임 + 새 ToF 측정값이 3번 연속 들어오면 PARKED 로 전환
        for i in range(3):
            t = 1.0 + i * 0.1
            out = self.ctrl.update(
                MarkerObservation(0, side_px=310.0, offset_px=0.0, t=t),
                DistanceObservation(dist_mm=120, status=RANGE_OK, t=t),
                now=t,
            )
        self.assertEqual(out.parking_samples, 3)
        self.assertEqual(out.state, ParkingState.PARKED)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_out_of_range_continues_driving(self) -> None:
        # ToF 범위 밖(1)은 오류가 아니므로 마커 정렬/접근 주행을 계속한다
        out = self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=0.0, t=1.0),
            DistanceObservation(dist_mm=-1, status=RANGE_OUT_OF_RANGE, t=1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.APPROACH)
        self.assertEqual((out.left, out.right), (self.cfg.drive_speed, self.cfg.drive_speed))

    def test_sensor_error_stops_and_recovers(self) -> None:
        # ToF 오류(2) 발생 시 SENSOR_ERROR로 정지했다가 정상 값 수신 시 복구된다
        err_out = self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=0.0, t=1.0),
            DistanceObservation(dist_mm=-1, status=RANGE_ERROR, t=1.0),
            now=1.0,
        )
        self.assertEqual(err_out.state, ParkingState.SENSOR_ERROR)
        self.assertEqual((err_out.left, err_out.right), (0, 0))

        rec_out = self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=0.0, t=1.1),
            DistanceObservation(dist_mm=500, status=RANGE_OK, t=1.1),
            now=1.1,
        )
        self.assertEqual(rec_out.state, ParkingState.APPROACH)
        self.assertEqual((rec_out.left, rec_out.right), (self.cfg.drive_speed, self.cfg.drive_speed))

    def test_approach_corrects_direction_while_moving_forward(self) -> None:
        # 중간 크기 오차는 전진을 유지하면서 좌우 PWM 차이로 보정
        right_out = self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=60.0, t=1.0),
            DistanceObservation(dist_mm=500, status=RANGE_OK, t=1.0),
            now=1.0,
        )
        self.assertEqual(right_out.state, ParkingState.APPROACH)
        self.assertGreater(right_out.left, right_out.right)
        self.assertGreater(right_out.right, 0)

        left_out = self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=-60.0, t=1.1),
            DistanceObservation(dist_mm=500, status=RANGE_OK, t=1.1),
            now=1.1,
        )
        self.assertEqual(left_out.state, ParkingState.APPROACH)
        self.assertLess(left_out.left, left_out.right)
        self.assertGreater(left_out.left, 0)

    def test_large_offset_uses_in_place_alignment(self) -> None:
        out = self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=150.0, t=1.0),
            DistanceObservation(dist_mm=500, status=RANGE_OK, t=1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.ALIGN)
        self.assertEqual((out.left, out.right), (self.cfg.align_speed, -self.cfg.align_speed))

    def test_large_offset_near_wall_stops_instead_of_pivoting(self) -> None:
        out = self.ctrl.update(
            MarkerObservation(0, side_px=100.0, offset_px=150.0, t=1.0),
            DistanceObservation(dist_mm=250, status=RANGE_OK, t=1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.SAFETY_STOP)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_marker_timeout_triggers_marker_lost(self) -> None:
        # 마커 인식 후 0.6초 초과 미수신 시 MARKER_LOST 정지
        self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=0.0, t=1.0),
            DistanceObservation(dist_mm=500, status=RANGE_OK, t=1.0),
            now=1.0,
        )
        lost_out = self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=0.0, t=1.0),
            DistanceObservation(dist_mm=500, status=RANGE_OK, t=1.65),
            now=1.65,
        )
        self.assertEqual(lost_out.state, ParkingState.MARKER_LOST)
        self.assertEqual((lost_out.left, lost_out.right), (0, 0))

    def test_tof_close_with_small_marker_triggers_safety_stop(self) -> None:
        # ToF만 정지 거리 안인데 마커 크기 조건 미충족이면 SAFETY_STOP
        out = self.ctrl.update(
            MarkerObservation(0, side_px=90.0, offset_px=0.0, t=1.0),
            DistanceObservation(dist_mm=120, status=RANGE_OK, t=1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.SAFETY_STOP)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_tof_close_and_vision_close_with_offset_stops_instead_of_turning(self) -> None:
        # 벽 가까이에서는 제자리 회전 중 모서리가 닿을 수 있으므로 안전 정지
        out = self.ctrl.update(
            MarkerObservation(0, side_px=310.0, offset_px=55.0, t=1.0),
            DistanceObservation(dist_mm=120, status=RANGE_OK, t=1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.SAFETY_STOP)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_large_marker_with_invalid_tof_stops(self) -> None:
        # 영상은 이미 가깝다고 하는데 ToF가 유효하지 않으면 더 전진하지 않는다
        out = self.ctrl.update(
            MarkerObservation(0, side_px=310.0, offset_px=0.0, t=1.0),
            DistanceObservation(dist_mm=-1, status=RANGE_OUT_OF_RANGE, t=1.0),
            now=1.0,
        )
        self.assertEqual(out.state, ParkingState.SAFETY_STOP)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_parking_timeout_stops(self) -> None:
        self.cfg.parking_timeout_sec = 1.0
        self.ctrl.update(
            MarkerObservation(0, side_px=80.0, offset_px=0.0, t=1.0),
            DistanceObservation(dist_mm=500, status=RANGE_OK, t=1.0),
            now=1.0,
        )
        out = self.ctrl.update(
            MarkerObservation(0, side_px=90.0, offset_px=0.0, t=2.1),
            DistanceObservation(dist_mm=450, status=RANGE_OK, t=2.1),
            now=2.1,
        )
        self.assertEqual(out.state, ParkingState.SAFETY_STOP)
        self.assertEqual((out.left, out.right), (0, 0))


if __name__ == "__main__":
    unittest.main()
