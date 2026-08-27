#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
import pathlib
import sys
import tempfile
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


turn = load("turn_by_imu")


class FakeImu:
    """Yaw that advances with real elapsed time at a fixed rate."""

    def __init__(self, rate_dps: float) -> None:
        self.rate_dps = rate_dps
        self.started = time.monotonic()
        self.pumps = 0

    def pump(self) -> None:
        self.pumps += 1

    def yaw_now(self) -> tuple[float, float]:
        return (time.monotonic() - self.started) * self.rate_dps, self.rate_dps


class FakeMotors:
    def __init__(self) -> None:
        self.commands: list[tuple[float, float]] = []

    def refresh(self, left: float, right: float) -> None:
        self.commands.append((left, right))


class ExtrapolationTests(unittest.TestCase):
    def test_a_fresh_sample_is_advanced_by_its_rate(self) -> None:
        reading = turn.YawReading(10.0, 180.0, received_at=100.0, stationary=False, tilt_deg=0.0)
        self.assertAlmostEqual(reading.extrapolated(100.02), 13.6, places=3)

    def test_a_stale_sample_is_not_extrapolated(self) -> None:
        reading = turn.YawReading(10.0, 180.0, received_at=100.0, stationary=False, tilt_deg=0.0)
        self.assertEqual(reading.extrapolated(100.9), 10.0)

    def test_clock_skew_backwards_is_ignored(self) -> None:
        reading = turn.YawReading(10.0, 180.0, received_at=100.0, stationary=False, tilt_deg=0.0)
        self.assertEqual(reading.extrapolated(99.5), 10.0)


class StopPointTests(unittest.TestCase):
    def test_stop_is_issued_a_lead_of_rate_before_the_target(self) -> None:
        imu = FakeImu(rate_dps=900.0)
        motors = FakeMotors()
        yaw_at_stop, rate, _duration, abort = turn.drive_until_angle(
            motors, imu, target_deg=90.0, lead_seconds=0.02, max_seconds=2.0, check_direction=False
        )
        self.assertIsNone(abort)
        # 900 dps for 0.02 s is 18 degrees of coast allowance.
        self.assertAlmostEqual(yaw_at_stop, 72.0, delta=4.0)
        self.assertAlmostEqual(rate, 900.0)

    def test_zero_lead_drives_all_the_way_to_the_target(self) -> None:
        imu = FakeImu(rate_dps=900.0)
        yaw_at_stop, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(), imu, target_deg=90.0, lead_seconds=0.0, max_seconds=2.0, check_direction=False
        )
        self.assertIsNone(abort)
        self.assertGreaterEqual(yaw_at_stop, 90.0)
        self.assertLess(yaw_at_stop, 96.0)

    def test_left_turn_commands_the_left_pivot_pair(self) -> None:
        motors = FakeMotors()
        turn.drive_until_angle(
            motors, FakeImu(900.0), 90.0, 0.0, max_seconds=2.0, check_direction=False
        )
        self.assertTrue(motors.commands)
        self.assertEqual(set(motors.commands), {turn.MOTIONS["left"]})

    def test_negative_degrees_command_the_right_pivot_pair(self) -> None:
        motors = FakeMotors()
        yaw_at_stop, _rate, _duration, abort = turn.drive_until_angle(
            motors, FakeImu(-900.0), -90.0, 0.0, max_seconds=2.0, check_direction=False
        )
        self.assertIsNone(abort)
        self.assertLessEqual(yaw_at_stop, -90.0)
        self.assertEqual(set(motors.commands), {turn.MOTIONS["right"]})


class AbortTests(unittest.TestCase):
    def test_a_base_that_is_not_moving_aborts_instead_of_spinning_out_the_clock(self) -> None:
        _yaw, _rate, duration, abort = turn.drive_until_angle(
            FakeMotors(), FakeImu(0.0), 90.0, 0.0, max_seconds=30.0, check_direction=True
        )
        self.assertIsNotNone(abort)
        self.assertIn("not turning", str(abort))
        self.assertLess(duration, 1.5)

    def test_turning_the_wrong_way_aborts_and_names_the_service_flags(self) -> None:
        _yaw, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(), FakeImu(-200.0), 90.0, 0.0, max_seconds=30.0, check_direction=True
        )
        self.assertIsNotNone(abort)
        self.assertIn("wrong way", str(abort))
        self.assertIn("invert-left", str(abort))

    def test_a_slow_base_hits_the_timeout(self) -> None:
        _yaw, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(), FakeImu(20.0), 180.0, 0.0, max_seconds=0.3, check_direction=False
        )
        self.assertIn("timeout", str(abort))

    def test_the_direction_check_is_skippable_for_corrections(self) -> None:
        """A correction nudge may finish before the check window even opens."""
        _yaw, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(), FakeImu(0.0), 90.0, 0.0, max_seconds=0.2, check_direction=False
        )
        self.assertIn("timeout", str(abort))


class LeadLearningTests(unittest.TestCase):
    def test_the_first_measurement_is_adopted_outright(self) -> None:
        self.assertAlmostEqual(turn.blend_lead(None, 0.14), 0.14)

    def test_later_measurements_move_the_stored_value_partway(self) -> None:
        self.assertAlmostEqual(turn.blend_lead(0.10, 0.20, weight=0.35), 0.135, places=4)

    def test_absurd_measurements_are_clamped_rather_than_trusted(self) -> None:
        self.assertAlmostEqual(turn.blend_lead(None, 9.0), 0.5)
        self.assertAlmostEqual(turn.blend_lead(None, -3.0), 0.0)


class CalibrationFileTests(unittest.TestCase):
    def test_values_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "nested" / "imu_turn_calibration.json"
            turn.save_calibration({"lead_seconds_left": 0.12}, path)
            self.assertEqual(turn.load_calibration(path), {"lead_seconds_left": 0.12})

    def test_a_missing_file_is_not_an_error(self) -> None:
        self.assertEqual(turn.load_calibration(pathlib.Path("/nonexistent/imu.json")), {})

    def test_corrupt_content_falls_back_to_defaults(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "imu.json"
            path.write_text("{not json")
            self.assertEqual(turn.load_calibration(path), {})

    def test_non_numeric_entries_are_dropped(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "imu.json"
            path.write_text(json.dumps({"lead_seconds_left": 0.1, "note": "hi", "flag": True}))
            self.assertEqual(turn.load_calibration(path), {"lead_seconds_left": 0.1})


class TurnResultTests(unittest.TestCase):
    def test_error_is_target_minus_achieved(self) -> None:
        result = turn.TurnResult(
            target_deg=90.0,
            achieved_deg=86.5,
            yaw_at_stop_deg=70.0,
            rate_at_stop_dps=180.0,
            coast_deg=16.5,
            implied_lead_s=0.0917,
            duration_s=0.6,
            packets=12,
        )
        self.assertAlmostEqual(result.error_deg, 3.5)
        self.assertIn("coast", result.summary())
        self.assertIn("implied lead", result.summary())


if __name__ == "__main__":
    unittest.main()
