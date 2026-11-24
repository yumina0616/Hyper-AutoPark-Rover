"""비콘 → ArUco 정렬 → 전면 주차 전체 상태 전환 테스트."""

import unittest
from dataclasses import dataclass

from beacon.peak_detector import PeakConfig, PeakDetector
from state_machine.autopark import AutoParkController, AutoParkPhase
from state_machine.beacon_search import BeaconSearch, SearchConfig
from state_machine.front_parking import (
    DistanceObservation,
    FrontParkingConfig,
    FrontParkingController,
    RANGE_ERROR,
    RANGE_OK,
)
from vision.align import AlignConfig
from vision.aruco_auto_align import ArucoAlignStage, MarkerFrame, StageConfig


@dataclass(frozen=True)
class FakeMarker:
    id: int
    err_x: float
    size_px: float
    t: float


def frame(
    seq: int,
    t: float,
    err_x=None,
    size_px: float = 80.0,
    marker_id: int = 0,
    width_px: int = 640,
):
    markers = [] if err_x is None else [FakeMarker(marker_id, err_x, size_px, t)]
    return MarkerFrame(seq=seq, t=t, markers=markers, width_px=width_px)


def distance(t: float, mm: int = 500) -> DistanceObservation:
    return DistanceObservation(dist_mm=mm, status=RANGE_OK, t=t)


def make_controller(search_timeout: float = 5.0) -> AutoParkController:
    detector = PeakDetector(
        PeakConfig(
            window_sec=0.01,
            min_samples=1,
            drop_db=3.0,
            confirm_sec=0.1,
            min_peak_rssi=-90.0,
            hold_sec=1.0,
        )
    )
    beacon = BeaconSearch(
        detector,
        SearchConfig(
            search_pwm=100,
            approach_rssi=None,
            search_timeout=search_timeout,
            return_timeout=2.0,
        ),
    )
    align = ArucoAlignStage(
        0,
        StageConfig(
            timeout_sec=3.0,
            search_if_missing=True,
            search_direction="left",
            search_pwm=60,
            search_sec=1.0,
        ),
        AlignConfig(align_frames=2, kp=100.0, max_turn=100, min_turn_pwm=60),
    )
    parking = FrontParkingController(
        FrontParkingConfig(target_marker_id=0, required_stop_samples=2)
    )
    return AutoParkController(
        target_slot=0,
        beacon_search=beacon,
        align_stage=align,
        parking_controller=parking,
    )


class AutoParkFlowTest(unittest.TestCase):
    def test_waits_for_target_beacon_and_start_permission(self) -> None:
        ctrl = make_controller()
        self.assertEqual(ctrl.step(0.0, target_rssi=[-70], allowed=False).left, 0)
        self.assertFalse(ctrl.beacon_started)
        self.assertEqual(ctrl.step(0.1, target_rssi=[]).detail, "목표 비콘 신호 대기")
        self.assertFalse(ctrl.beacon_started)

    def test_full_flow_reaches_parked(self) -> None:
        ctrl = make_controller()

        # 비콘에 접근하고 지나친 뒤 최댓값 지점으로 복귀한다.
        self.assertGreater(ctrl.step(0.00, target_rssi=[-80]).left, 0)
        ctrl.step(0.10, target_rssi=[-60])
        ctrl.step(0.20, target_rssi=[-64])
        returning = ctrl.step(0.31, target_rssi=[-64])
        self.assertLess(returning.left, 0)
        aligned_entry = ctrl.step(0.60)
        self.assertEqual(aligned_entry.phase, AutoParkPhase.ARUCO_ALIGN)
        self.assertEqual((aligned_entry.left, aligned_entry.right), (0, 0))

        # 마커가 없으면 왼쪽으로 천천히 탐색 회전한다.
        searching = ctrl.step(0.61, marker_frame=frame(1, 0.61))
        self.assertEqual(searching.detail, "SEARCHING")
        self.assertEqual((searching.left, searching.right), (-60, 60))

        # 마커를 발견해 두 개의 서로 다른 중앙 프레임으로 정렬을 확정한다.
        turning = ctrl.step(0.70, marker_frame=frame(2, 0.70, err_x=0.5))
        self.assertEqual(turning.detail, "TURNING")
        ctrl.step(0.80, marker_frame=frame(3, 0.80, err_x=0.01))
        parking_entry = ctrl.step(0.90, marker_frame=frame(4, 0.90, err_x=0.01))
        self.assertEqual(parking_entry.phase, AutoParkPhase.FRONT_PARK)
        self.assertEqual((parking_entry.left, parking_entry.right), (0, 0))

        # 마커 크기 + 중앙 + ToF 조건이 새 측정 2회 연속 맞으면 완료한다.
        first = ctrl.step(
            1.00,
            marker_frame=frame(5, 1.00, err_x=0.0, size_px=145.0),
            distance=distance(1.00, 140),
        )
        self.assertEqual((first.left, first.right), (0, 0))
        parked = ctrl.step(
            1.10,
            marker_frame=frame(6, 1.10, err_x=0.0, size_px=145.0),
            distance=distance(1.10, 140),
        )
        self.assertEqual(parked.phase, AutoParkPhase.PARKED)
        self.assertTrue(ctrl.terminal)
        self.assertEqual(ctrl.step(2.0).phase, AutoParkPhase.PARKED)

    def test_beacon_abort_never_starts_camera_alignment(self) -> None:
        ctrl = make_controller(search_timeout=0.1)
        ctrl.step(0.0, target_rssi=[-80])
        out = ctrl.step(0.2)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("비콘 탐색 실패", out.reason)
        self.assertFalse(ctrl.align_stage.finished)

    def test_distance_error_aborts_from_beacon_phase(self) -> None:
        ctrl = make_controller()
        bad_distance = DistanceObservation(-1, RANGE_ERROR, t=0.0)
        out = ctrl.step(0.0, target_rssi=[-80], distance=bad_distance)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("거리 센서", out.reason)

    def test_front_parking_uses_actual_frame_width(self) -> None:
        ctrl = make_controller()
        marker = ctrl._target_marker(
            frame(1, 1.0, err_x=0.1, size_px=80.0, width_px=800)
        )
        self.assertEqual(marker.offset_px, 40.0)

    def test_camera_error_aborts_alignment(self) -> None:
        ctrl = make_controller()
        ctrl.phase = AutoParkPhase.ARUCO_ALIGN
        ctrl.align_stage.reset(10.0)
        out = ctrl.step(10.1, camera_error="USB camera unavailable")
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_close_wall_aborts_alignment_rotation(self) -> None:
        ctrl = make_controller()
        ctrl.phase = AutoParkPhase.ARUCO_ALIGN
        ctrl.align_stage.reset(10.0)
        out = ctrl.step(
            10.1,
            marker_frame=frame(1, 10.1, err_x=0.7),
            distance=distance(10.1, 140),
        )
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("가까워", out.reason)
        self.assertEqual((out.left, out.right), (0, 0))


if __name__ == "__main__":
    unittest.main()
