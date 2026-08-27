#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import time
import unittest


SCRIPTS = pathlib.Path(__file__).parent
sys.path.insert(0, str(SCRIPTS))


def load(name: str):
    path = SCRIPTS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


closed_loop = load("closed_loop_drive")
heading = load("drive_heading")
SteeringController = closed_loop.SteeringController


class Reading:
    stationary = False
    tilt_deg = 0.0


class FakeBase:
    """Stands in for both MotorStream and ImuClient.

    Yaw evolves as `omega = drift + k * (v_r - v_l)`, which is the
    differential-drive relation with the commanded values used directly, so a
    reverse command produces the opposite rotation for the same dropped tread.
    """

    latest = Reading()

    def __init__(self, drift_dps: float, one_tread_dps: float = 75.0) -> None:
        self.yaw = 0.0
        self.drift_dps = drift_dps
        self.k = one_tread_dps
        self.left_cmd = 0.0
        self.right_cmd = 0.0
        self.last = time.monotonic()
        self.sequence = 0
        self.errors: set[str] = set()
        self.commands: list[tuple[float, float]] = []

    def _advance(self) -> None:
        now = time.monotonic()
        dt = now - self.last
        self.last = now
        self.yaw += (self.drift_dps + self.k * (self.right_cmd - self.left_cmd)) * dt

    def send_now(self, left: float, right: float) -> int:
        self._advance()
        self.left_cmd, self.right_cmd = left, right
        self.commands.append((left, right))
        self.sequence += 1
        return self.sequence

    def refresh(self, left: float, right: float) -> None:
        self.send_now(left, right)

    def stop(self, confirm_seconds: float = 0.0) -> bool:
        self._advance()
        self.left_cmd = self.right_cmd = 0.0
        return True

    def pump(self) -> None:
        self._advance()

    def yaw_now(self) -> tuple[float, float]:
        self._advance()
        return self.yaw, 0.0


class SteeringLawTests(unittest.TestCase):
    def test_positive_error_slows_the_left_tread(self) -> None:
        left, right = SteeringController(gain=2.0).duties(math.radians(10.0))
        self.assertLess(left, right)
        self.assertEqual(right, 1.0)

    def test_negative_error_slows_the_right_tread(self) -> None:
        left, right = SteeringController(gain=2.0).duties(math.radians(-10.0))
        self.assertLess(right, left)
        self.assertEqual(left, 1.0)

    def test_inside_the_deadband_both_treads_run_full(self) -> None:
        controller = SteeringController(gain=2.0, deadband_rad=math.radians(2.0))
        self.assertEqual(controller.duties(math.radians(1.0)), (1.0, 1.0))

    def test_duty_never_drops_below_the_floor(self) -> None:
        left, _right = SteeringController(gain=2.0, min_duty=0.45).duties(math.radians(180.0))
        self.assertAlmostEqual(left, 0.45)

    def test_integral_is_off_unless_asked_for(self) -> None:
        controller = SteeringController(gain=2.0)
        for _ in range(50):
            controller.duties(math.radians(5.0), dt=0.05)
        self.assertEqual(controller.integral, 0.0)
        self.assertAlmostEqual(
            controller.duties(math.radians(5.0))[0],
            SteeringController(gain=2.0).duties(math.radians(5.0))[0],
        )

    def test_integral_accumulates_a_persistent_error(self) -> None:
        controller = SteeringController(gain=2.0, integral_gain=3.0)
        first = controller.duties(math.radians(5.0), dt=0.05)[0]
        for _ in range(20):
            controller.duties(math.radians(5.0), dt=0.05)
        later = controller.duties(math.radians(5.0), dt=0.05)[0]
        self.assertLess(later, first, "integral action did not increase the correction")

    def test_integral_cannot_wind_up_past_full_authority(self) -> None:
        controller = SteeringController(gain=2.0, integral_gain=3.0)
        for _ in range(10_000):
            controller.duties(math.radians(45.0), dt=0.05)
        self.assertLessEqual(abs(controller.integral), controller.integral_limit() + 1e-9)
        self.assertLessEqual(abs(controller.command(0.0)), 1.0 + 1e-9)

    def test_reset_clears_accumulated_integral(self) -> None:
        controller = SteeringController(gain=2.0, integral_gain=3.0)
        controller.duties(math.radians(5.0), dt=1.0)
        controller.reset()
        self.assertEqual(controller.integral, 0.0)


class ReverseSteeringTests(unittest.TestCase):
    """Driving backward reverses both tread velocities, so omega changes sign."""

    def test_forward_and_reverse_correct_with_opposite_treads(self) -> None:
        controller = SteeringController(gain=2.0)
        forward = heading.steering_duties(controller, 10.0, reverse=False)
        controller.reset()
        reverse = heading.steering_duties(controller, 10.0, reverse=True)
        self.assertEqual(forward, tuple(reversed(reverse)))

    def test_reverse_slows_the_right_tread_for_a_positive_error(self) -> None:
        left, right = heading.steering_duties(SteeringController(gain=2.0), 10.0, reverse=True)
        self.assertEqual(left, 1.0)
        self.assertLess(right, 1.0)


class DriveStatsTests(unittest.TestCase):
    def test_error_metrics_and_slot_accounting(self) -> None:
        stats = heading.DriveStats(target_yaw_deg=90.0, start_yaw_deg=87.0)
        stats.end_yaw_deg = 89.0
        stats.errors_deg = [3.0, -4.0, 1.0]
        stats.slots = 10
        stats.left_slots = 8
        stats.right_slots = 10
        self.assertAlmostEqual(stats.final_error_deg, 1.0)
        self.assertAlmostEqual(stats.max_abs_error_deg, 4.0)
        self.assertAlmostEqual(stats.rms_error_deg, math.sqrt((9 + 16 + 1) / 3))
        self.assertEqual(stats.dropped_slots, 2)

    def test_error_wraps_across_the_half_turn_boundary(self) -> None:
        """From -179 deg, the short way to a +179 deg target is -2 deg."""
        stats = heading.DriveStats(target_yaw_deg=179.0, start_yaw_deg=0.0)
        stats.end_yaw_deg = -179.0
        self.assertAlmostEqual(stats.final_error_deg, -2.0, places=6)


class ClosedLoopDriftTests(unittest.TestCase):
    DRIFT_DPS = 8.0
    SECONDS = 2.0

    def run_drive(self, reverse: bool, integral_gain: float) -> heading.DriveStats:
        base = FakeBase(drift_dps=self.DRIFT_DPS)
        controller = SteeringController(
            gain=heading.DEFAULT_GAIN,
            min_duty=0.45,
            deadband_rad=math.radians(1.0),
            integral_gain=integral_gain,
        )
        return heading.drive_heading(
            base,
            base,
            reverse=reverse,
            seconds=self.SECONDS,
            controller=controller,
            target_yaw_deg=0.0,
            rate_hz=50.0,
            settle_seconds=0.05,
        )

    def uncorrected_deg(self) -> float:
        return self.DRIFT_DPS * self.SECONDS

    def test_forward_drift_is_corrected(self) -> None:
        stats = self.run_drive(reverse=False, integral_gain=8.0)
        self.assertIsNone(stats.aborted)
        self.assertLess(
            abs(stats.final_error_deg),
            self.uncorrected_deg() / 3.0,
            f"drift not corrected: {stats.summary()}",
        )

    def test_reverse_drift_is_corrected_not_amplified(self) -> None:
        """The sign trap: without the duty swap this diverges instead."""
        stats = self.run_drive(reverse=True, integral_gain=8.0)
        self.assertIsNone(stats.aborted)
        self.assertLess(
            abs(stats.final_error_deg),
            self.uncorrected_deg() / 3.0,
            f"reverse drift not corrected: {stats.summary()}",
        )

    def test_integral_action_beats_proportional_alone(self) -> None:
        proportional = self.run_drive(reverse=False, integral_gain=0.0)
        with_integral = self.run_drive(reverse=False, integral_gain=8.0)
        self.assertLess(
            abs(with_integral.final_error_deg),
            abs(proportional.final_error_deg),
        )

    def test_reverse_commands_are_actually_negative(self) -> None:
        base = FakeBase(drift_dps=0.0)
        heading.drive_heading(
            base,
            base,
            reverse=True,
            seconds=0.4,
            controller=SteeringController(gain=2.0),
            target_yaw_deg=0.0,
            rate_hz=50.0,
            settle_seconds=0.05,
        )
        self.assertTrue(base.commands)
        self.assertTrue(all(left <= 0.0 and right <= 0.0 for left, right in base.commands))


if __name__ == "__main__":
    unittest.main()
