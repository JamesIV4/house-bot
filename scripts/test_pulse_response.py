#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import pathlib
import sys
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


turn = load("turn_by_imu")
pr = load("pulse_response")


class FakeRotator:
    """Stands in for both MotorStream and ImuClient during a pulse."""

    def __init__(self, degrees_per_second: float) -> None:
        self.degrees_per_second = degrees_per_second
        self.yaw = 0.0
        self.commands: list[tuple[float, float]] = []
        self.sequence = 0
        self.errors: set[str] = set()

    def send_now(self, left: float, right: float) -> int:
        self.commands.append((left, right))
        self.sequence += 1
        if (left, right) != (0.0, 0.0):
            # Attribute the whole pulse at once; the test does not model time.
            self.yaw += self.degrees_per_second * (1.0 if right > left else -1.0)
        return self.sequence

    def refresh(self, left: float, right: float) -> None:
        self.commands.append((left, right))

    def stop(self, confirm_seconds: float = 0.0) -> bool:
        return True

    def pump(self) -> None:
        pass

    def zero(self) -> dict[str, float]:
        self.yaw = 0.0
        return {"yaw": 0.0}

    def yaw_now(self) -> tuple[float, float]:
        return self.yaw, 0.0


def result_with(mode: str, seconds: float, left: float, right: float) -> "pr.DurationResult":
    entry = pr.DurationResult(mode, seconds)
    entry.left_deg = [left]
    entry.right_deg = [right]
    return entry


class LivenessTests(unittest.TestCase):
    """Regression cover: a sweep against a sleeping receiver reads as zeroes.

    That is indistinguishable from a base too coarse to nudge, so the sweep
    must prove the base answers before drawing any conclusion from a small
    pulse.
    """

    def test_a_responsive_base_passes(self) -> None:
        base = FakeRotator(degrees_per_second=90.0)
        moved = pr.confirm_responsive(base, base)
        self.assertGreaterEqual(abs(moved), pr.WAKE_MIN_DEG)

    def test_an_unresponsive_base_raises_instead_of_reporting_zeroes(self) -> None:
        base = FakeRotator(degrees_per_second=0.0)
        with self.assertRaises(turn.ImuError) as caught:
            pr.confirm_responsive(base, base, attempts=2)
        self.assertIn("not responding", str(caught.exception))

    def test_the_wake_check_uses_a_real_pivot_command(self) -> None:
        base = FakeRotator(degrees_per_second=90.0)
        pr.confirm_responsive(base, base)
        self.assertIn(turn.rotation_command("pivot", 1.0), base.commands)


class RepeatabilityGateTests(unittest.TestCase):
    def test_a_consistent_pulse_is_repeatable(self) -> None:
        self.assertTrue(result_with("pivot", 0.08, 23.54, -23.41).repeatable())

    def test_a_pulse_that_varies_wildly_is_not(self) -> None:
        """Measured 0.05 s pivot: 27.5 and 13.0 deg. Mean alone hid this."""
        entry = result_with("pivot", 0.05, 27.49, -13.02)
        self.assertGreater(entry.mean_deg(), pr.MOTION_FLOOR_DEG)
        self.assertFalse(entry.repeatable())

    def test_a_motionless_pulse_is_neither(self) -> None:
        entry = result_with("tread-reverse", 0.05, 0.0, 0.0)
        self.assertFalse(entry.moved())
        self.assertFalse(entry.repeatable())


class RecommendationTests(unittest.TestCase):
    def test_nothing_moving_is_reported_as_a_wiring_or_power_problem(self) -> None:
        message = pr.recommend([result_with("pivot", 0.08, 0.0, 0.0)], coast_deg=33.4)
        self.assertIn("No pulse length tested moved the base", message)
        self.assertIn("wake check", message)

    def test_unrepeatable_pulses_fall_back_to_driving(self) -> None:
        message = pr.recommend([result_with("pivot", 0.05, 27.49, -13.02)], coast_deg=33.4)
        self.assertIn("unpredictably", message)
        self.assertIn("drive_heading.py", message)

    def test_a_pulse_no_better_than_the_coast_is_rejected(self) -> None:
        # Measured 0.16 s pivot: 38.5 and 32.6 deg, so 35.6 +/-4.2. Allowing
        # for that spread it is no finer than the 33.4 deg coast.
        message = pr.recommend([result_with("pivot", 0.16, 38.52, -32.59)], coast_deg=33.4)
        self.assertIn("no better than", message)
        self.assertIn("drive_heading.py", message)

    def test_a_genuinely_finer_pulse_is_recommended_with_its_mode(self) -> None:
        entries = [
            result_with("pivot", 0.08, 23.54, -23.41),
            result_with("tread-reverse", 0.05, 4.10, -4.05),
        ]
        message = pr.recommend(entries, coast_deg=33.4)
        self.assertIn("--nudge-mode tread-reverse", message)
        self.assertIn("--nudge-pulse 0.05", message)

    def test_the_smallest_repeatable_option_wins_across_modes(self) -> None:
        entries = [
            result_with("tread-forward", 0.05, 9.0, -9.1),
            result_with("tread-reverse", 0.05, 4.10, -4.05),
            result_with("pivot", 0.05, 20.0, -20.1),
        ]
        self.assertIn("--nudge-mode tread-reverse", pr.recommend(entries, coast_deg=33.4))


if __name__ == "__main__":
    unittest.main()
