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
        self.kwargs = None

    def step(self, **kwargs):
        self.kwargs = kwargs
        return AutoParkOutput(AutoParkPhase.BEACON_SEARCH, 70, 80, "SEARCH")


class FakeLink:
    def __init__(self):
        self.sent = []

    def latest_distance(self):
        return SimpleNamespace(dist_mm=420, status=0, t=12.0, stale=False)

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
        self.assertEqual(controller.kwargs["target_rssi"], [-65, -63])
        self.assertIs(controller.kwargs["marker_frame"], camera.observation)
        self.assertEqual(controller.kwargs["distance"].dist_mm, 420)
        self.assertEqual(link.sent, [(70, 80)])
        self.assertEqual(camera.started, 1)

    def test_terminal_controller_does_not_reopen_camera(self):
        camera = FakeCamera()
        controller = FakeController()
        controller.terminal = True
        controller.step = lambda **_: AutoParkOutput(
            AutoParkPhase.PARKED, 0, 0, "주차 완료"
        )
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


if __name__ == "__main__":
    unittest.main()
