"""전체 자동주차 실행기와 Bridge/카메라 연결 테스트."""

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "apps" / "qrover_drive" / "python"))

from autopark_mode import AutoParkRunner  # noqa: E402
from state_machine.autopark import AutoParkOutput, AutoParkPhase  # noqa: E402


class FakeCamera:
    def __init__(self):
        self.started = 0
        self.closed = 0
        self.observation = object()

    def start(self):
        self.started += 1

    def snapshot(self):
        return self.observation, None

    def close(self):
        self.closed += 1


class FakeController:
    def __init__(self):
        self.terminal = False
        self.phase = AutoParkPhase.BEACON_SEARCH
        self.reason = ""
        self.kwargs = None
        self.beacon_started = False
        self.target_slot = 0
        self.reset_count = 0

    def step(self, **kwargs):
        self.kwargs = kwargs
        return AutoParkOutput(AutoParkPhase.BEACON_SEARCH, 70, 80, "SEARCH")

    def abort(self, reason):
        self.terminal = True
        self.phase = AutoParkPhase.ABORT
        self.reason = reason

    def reset(self):
        self.terminal = False
        self.phase = AutoParkPhase.SENSOR_READY
        self.reason = ""
        self.beacon_started = False
        self.reset_count += 1

    def set_target_slot(self, target_slot):
        self.target_slot = int(target_slot)


class FakeLink:
    def __init__(self):
        self.sent = []

    def latest_distance(self):
        return SimpleNamespace(dist_mm=420, status=0, t=12.0, stale=False)

    def latest_safety(self):
        return SimpleNamespace(reason=0, t=12.0, stale=False)

    def send_drive(self, left, right):
        self.sent.append((left, right))


class AutoParkRunnerTest(unittest.TestCase):
    def test_filters_target_rssi_and_sends_single_command(self):
        camera = FakeCamera()
        controller = FakeController()
        link = FakeLink()
        runner = AutoParkRunner(
            target_slot=2,
            poll_rssi=lambda: [(1, -40), (2, -65), (2, -63)],
            controller=controller,
            camera=camera,
            log=lambda _: None,
        )

        out = runner.step(link)

        self.assertEqual(out.phase, AutoParkPhase.BEACON_SEARCH)
        self.assertEqual(
            [sample.rssi for sample in controller.kwargs["target_rssi"]],
            [-65, -63],
        )
        self.assertIs(controller.kwargs["marker_frame"], camera.observation)
        self.assertEqual(controller.kwargs["distance"].dist_mm, 420)
        self.assertEqual(controller.kwargs["safety"].reason, 0)
        self.assertEqual(link.sent, [(70, 80)])
        self.assertEqual(camera.started, 1)

    def test_terminal_controller_does_not_reopen_camera(self):
        camera = FakeCamera()
        controller = FakeController()
        controller.terminal = True
        controller.phase = AutoParkPhase.PARKED
        link = FakeLink()
        runner = AutoParkRunner(
            target_slot=0,
            poll_rssi=lambda: [],
            controller=controller,
            camera=camera,
            log=lambda _: None,
        )

        runner.step(link)
        runner.step(link)

        self.assertEqual(camera.started, 0)
        self.assertEqual(camera.closed, 0)
        self.assertEqual(link.sent, [(0, 0), (0, 0)])

    def test_manual_start_gate_holds_stop(self):
        camera = FakeCamera()
        controller = FakeController()
        link = FakeLink()
        runner = AutoParkRunner(
            target_slot=0,
            poll_rssi=lambda: [],
            controller=controller,
            camera=camera,
            log=lambda _: None,
            auto_start=False,
        )

        waiting = runner.step(link)
        self.assertEqual(waiting.detail, "수동 시작 대기")
        self.assertEqual(link.sent[-1], (0, 0))
        self.assertEqual(camera.started, 0)

        runner.start()
        runner.step(link)
        self.assertEqual(camera.started, 1)

    def test_cancel_latches_abort_and_closes_camera(self):
        camera = FakeCamera()
        controller = FakeController()
        link = FakeLink()
        runner = AutoParkRunner(0, lambda: [], controller, camera, log=lambda _: None)
        runner.step(link)

        runner.cancel()
        out = runner.step(link)

        self.assertEqual(out.phase, AutoParkPhase.ABORT)
        self.assertEqual(out.reason, "사용자 취소")
        self.assertEqual(link.sent[-1], (0, 0))
        self.assertEqual(camera.closed, 1)

    def test_reset_waits_for_manual_restart(self):
        camera = FakeCamera()
        controller = FakeController()
        link = FakeLink()
        runner = AutoParkRunner(0, lambda: [], controller, camera, log=lambda _: None)
        runner.step(link)

        runner.reset()
        out = runner.step(link)

        self.assertEqual(controller.reset_count, 1)
        self.assertEqual(out.detail, "수동 시작 대기")
        self.assertEqual(camera.closed, 1)

    def test_target_change_and_status_reporting(self):
        camera = FakeCamera()
        controller = FakeController()
        link = FakeLink()
        published = []
        runner = AutoParkRunner(
            0,
            lambda: [],
            controller,
            camera,
            log=lambda _: None,
            on_status=published.append,
            auto_start=False,
        )

        runner.set_target_slot(4)
        waiting = runner.step(link)

        self.assertEqual((runner.target_slot, controller.target_slot), (4, 4))
        self.assertEqual(runner.status(), "BEACON_SEARCH:수동 시작 대기:")
        self.assertEqual(published, [waiting])

    def test_target_change_is_rejected_after_motion_started(self):
        controller = FakeController()
        controller.beacon_started = True
        runner = AutoParkRunner(
            0,
            lambda: [],
            controller,
            FakeCamera(),
            log=lambda _: None,
        )

        with self.assertRaises(RuntimeError):
            runner.set_target_slot(1)


if __name__ == "__main__":
    unittest.main()
