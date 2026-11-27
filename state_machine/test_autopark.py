"""비콘 → ArUco 정렬 → 전면 주차 전체 상태 전환 테스트."""

import unittest
from dataclasses import dataclass

from beacon.peak_detector import PeakConfig, PeakDetector
from state_machine.autopark import (
    SAFETY_EMERGENCY,
    SAFETY_PASS,
    AutoParkController,
    AutoParkPhase,
    RssiObservation,
    SafetyObservation,
)
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


def safety(t: float, reason: int = SAFETY_PASS) -> SafetyObservation:
    return SafetyObservation(reason=reason, t=t)


def run_step(ctrl: AutoParkController, t: float, distance_obs=None, **kwargs):
    return ctrl.step(
        t,
        distance=distance_obs or distance(t),
        safety=safety(t),
        **kwargs,
    )


def ready(ctrl: AutoParkController, t: float = 0.0) -> None:
    out = run_step(ctrl, t)
    assert out.phase == AutoParkPhase.BEACON_SEARCH


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
        self.assertEqual(ctrl.step(0.1, allowed=True).detail, "거리 센서 정상 측정 대기")
        ready(ctrl, 0.2)
        self.assertEqual(run_step(ctrl, 0.3, target_rssi=[]).detail, "목표 비콘 신호 대기")
        self.assertFalse(ctrl.beacon_started)

    def test_sensor_ready_waits_for_late_feedback_instead_of_abort(self) -> None:
        # 앱 시작 직후 MCU 신호가 늦게 와도 바로 중단하지 않고 기다린다
        ctrl = make_controller()
        late = SafetyObservation(reason=SAFETY_PASS, t=0.0, stale=True)
        out = ctrl.step(0.5, distance=distance(0.5), safety=late)
        self.assertEqual(out.phase, AutoParkPhase.SENSOR_READY)
        self.assertEqual((out.left, out.right), (0, 0))
        bad = DistanceObservation(-1, RANGE_ERROR, t=1.0)
        self.assertEqual(ctrl.step(1.0, distance=bad, safety=safety(1.0)).phase,
                         AutoParkPhase.SENSOR_READY)
        ok = run_step(ctrl, 1.5)
        self.assertEqual(ok.phase, AutoParkPhase.BEACON_SEARCH)

    def test_sensor_ready_late_feedback_times_out(self) -> None:
        ctrl = make_controller()
        ctrl.cfg.sensor_ready_timeout_sec = 0.5
        late = SafetyObservation(reason=SAFETY_PASS, t=0.0, stale=True)
        ctrl.step(0.0, distance=distance(0.0), safety=late)
        out = ctrl.step(0.6, distance=distance(0.6), safety=late)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("준비 시간 초과", out.reason)

    def test_stale_feedback_still_aborts_after_start(self) -> None:
        ctrl = make_controller()
        ready(ctrl)
        late = SafetyObservation(reason=SAFETY_PASS, t=0.0, stale=True)
        out = ctrl.step(1.0, distance=distance(1.0), safety=late)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("오래됨", out.reason)

    def test_sensor_ready_timeout_aborts_without_first_reading(self) -> None:
        ctrl = make_controller()
        ctrl.cfg.sensor_ready_timeout_sec = 0.5
        ctrl.step(0.0, allowed=True)
        out = ctrl.step(0.6, allowed=True)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("준비 시간 초과", out.reason)

    def test_beacon_first_signal_timeout_aborts(self) -> None:
        ctrl = make_controller()
        ctrl.cfg.beacon_wait_timeout_sec = 0.5
        ready(ctrl)
        out = run_step(ctrl, 0.6)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("첫 신호", out.reason)

    def test_mcu_emergency_feedback_aborts(self) -> None:
        ctrl = make_controller()
        ready(ctrl)
        out = ctrl.step(
            0.1,
            distance=distance(0.1),
            safety=safety(0.1, SAFETY_EMERGENCY),
        )
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("비상정지", out.reason)

    def test_beacon_forward_motion_stops_for_obstacle(self) -> None:
        ctrl = make_controller()
        ready(ctrl)
        out = run_step(
            ctrl,
            0.1,
            target_rssi=[RssiObservation(0.1, -80)],
            distance_obs=distance(0.1, ctrl.cfg.beacon_obstacle_stop_mm),
        )
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("장애물", out.reason)

    def test_beacon_uses_original_rssi_measurement_time(self) -> None:
        ctrl = make_controller()
        ready(ctrl)
        run_step(ctrl, 1.0, target_rssi=[RssiObservation(0.9, -80)])
        self.assertEqual(ctrl.beacon_search.detector.last_t, 0.9)

    def test_future_rssi_timestamp_aborts(self) -> None:
        ctrl = make_controller()
        ready(ctrl)
        out = run_step(ctrl, 1.0, target_rssi=[RssiObservation(2.0, -80)])
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("미래", out.reason)

    def test_full_flow_reaches_parked(self) -> None:
        ctrl = make_controller()
        ready(ctrl)

        # 비콘에 접근하고 지나친 뒤 최댓값 지점으로 복귀한다.
        self.assertGreater(
            run_step(ctrl, 0.01, target_rssi=[RssiObservation(0.01, -80)]).left,
            0,
        )
        run_step(ctrl, 0.10, target_rssi=[RssiObservation(0.10, -60)])
        run_step(ctrl, 0.20, target_rssi=[RssiObservation(0.20, -64)])
        returning = run_step(ctrl, 0.31, target_rssi=[RssiObservation(0.31, -64)])
        self.assertLess(returning.left, 0)
        aligned_entry = run_step(ctrl, 0.60)
        self.assertEqual(aligned_entry.phase, AutoParkPhase.ARUCO_ALIGN)
        self.assertEqual((aligned_entry.left, aligned_entry.right), (0, 0))

        # 마커가 없으면 왼쪽으로 천천히 탐색 회전한다.
        searching = run_step(ctrl, 0.61, marker_frame=frame(1, 0.61))
        self.assertEqual(searching.detail, "SEARCHING")
        self.assertEqual((searching.left, searching.right), (-60, 60))

        # 마커를 발견해 두 개의 서로 다른 중앙 프레임으로 정렬을 확정한다.
        turning = run_step(ctrl, 0.70, marker_frame=frame(2, 0.70, err_x=0.5))
        self.assertEqual(turning.detail, "TURNING")
        run_step(ctrl, 0.80, marker_frame=frame(3, 0.80, err_x=0.01))
        parking_entry = run_step(ctrl, 0.90, marker_frame=frame(4, 0.90, err_x=0.01))
        self.assertEqual(parking_entry.phase, AutoParkPhase.FRONT_PARK)
        self.assertEqual((parking_entry.left, parking_entry.right), (0, 0))

        # 마커 크기 + 중앙 + ToF 조건이 새 측정 2회 연속 맞으면 완료한다.
        first = run_step(
            ctrl,
            1.00,
            marker_frame=frame(5, 1.00, err_x=0.0, size_px=310.0),
            distance_obs=distance(1.00, 120),
        )
        self.assertEqual((first.left, first.right), (0, 0))
        parked = run_step(
            ctrl,
            1.10,
            marker_frame=frame(6, 1.10, err_x=0.0, size_px=310.0),
            distance_obs=distance(1.10, 120),
        )
        self.assertEqual(parked.phase, AutoParkPhase.PARKED)
        self.assertTrue(ctrl.terminal)
        self.assertEqual(ctrl.step(2.0).phase, AutoParkPhase.PARKED)

    def test_beacon_abort_never_starts_camera_alignment(self) -> None:
        ctrl = make_controller(search_timeout=0.1)
        ready(ctrl)
        run_step(ctrl, 0.01, target_rssi=[-80])
        out = run_step(ctrl, 0.2)
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("비콘 탐색 실패", out.reason)
        self.assertFalse(ctrl.align_stage.finished)

    def test_distance_error_aborts_from_beacon_phase(self) -> None:
        ctrl = make_controller()
        ready(ctrl)
        bad_distance = DistanceObservation(-1, RANGE_ERROR, t=0.1)
        out = ctrl.step(0.1, target_rssi=[-80], distance=bad_distance, safety=safety(0.1))
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
        out = run_step(ctrl, 10.1, camera_error="USB camera unavailable")
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_close_wall_aborts_alignment_rotation(self) -> None:
        ctrl = make_controller()
        ctrl.phase = AutoParkPhase.ARUCO_ALIGN
        ctrl.align_stage.reset(10.0)
        out = run_step(
            ctrl,
            10.1,
            marker_frame=frame(1, 10.1, err_x=0.7),
            distance_obs=distance(10.1, 120),
        )
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("가까워", out.reason)
        self.assertEqual((out.left, out.right), (0, 0))

    def test_front_approach_reacquires_lost_marker_once_when_far(self) -> None:
        ctrl = make_controller()
        ctrl.phase = AutoParkPhase.FRONT_PARK
        ctrl._parking_started_at = 1.0
        hold = run_step(ctrl, 1.0, marker_frame=frame(1, 1.0))
        self.assertEqual(hold.detail, "접근 중 목표 마커 일시 유실")

        retry = run_step(ctrl, 1.31, marker_frame=frame(2, 1.31))
        self.assertEqual(retry.phase, AutoParkPhase.ARUCO_ALIGN)
        self.assertIn("재탐색", retry.detail)
        self.assertEqual(ctrl._parking_reacquire_attempts, 1)

    def test_front_approach_does_not_turn_to_reacquire_near_wall(self) -> None:
        ctrl = make_controller()
        ctrl.phase = AutoParkPhase.FRONT_PARK
        ctrl._parking_started_at = 1.0
        run_step(
            ctrl,
            1.0,
            marker_frame=frame(1, 1.0),
            distance_obs=distance(1.0, 250),
        )
        out = run_step(
            ctrl,
            1.31,
            marker_frame=frame(2, 1.31),
            distance_obs=distance(1.31, 250),
        )
        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertIn("재탐색 불가", out.reason)


if __name__ == "__main__":
    unittest.main()
