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
    """Yaw that advances with real elapsed time at a fixed rate.

    `wake_delay_s` imitates the receiver swallowing the start of an unprimed
    command: the base reads as perfectly still until it fires.
    """

    def __init__(
        self,
        rate_dps: float,
        wake_delay_s: float = 0.0,
        initial_yaw: float = 0.0,
    ) -> None:
        self.rate_dps = rate_dps
        self.wake_delay_s = wake_delay_s
        self.initial_yaw = initial_yaw
        self.started = time.monotonic()
        self.pumps = 0

    def pump(self) -> None:
        self.pumps += 1

    def yaw_now(self) -> tuple[float, float]:
        moving_for = (time.monotonic() - self.started) - self.wake_delay_s
        if moving_for <= 0.0:
            return self.initial_yaw, 0.0
        return self.initial_yaw + moving_for * self.rate_dps, self.rate_dps


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
    def test_release_comes_a_whole_coast_angle_before_the_target(self) -> None:
        imu = FakeImu(rate_dps=900.0)
        motors = FakeMotors()
        yaw_at_stop, rate, _duration, abort = turn.drive_until_angle(
            motors, imu, target_deg=90.0, coast_deg=18.0, max_seconds=2.0, check_direction=False
        )
        self.assertIsNone(abort)
        self.assertAlmostEqual(yaw_at_stop, 72.0, delta=4.0)
        self.assertAlmostEqual(rate, 900.0)

    def test_the_release_point_does_not_move_with_yaw_rate(self) -> None:
        """The whole reason for an angle model rather than a time model."""
        stops = []
        for rate in (400.0, 900.0):
            yaw_at_stop, _rate, _duration, _abort = turn.drive_until_angle(
                FakeMotors(), FakeImu(rate), 90.0, 18.0, max_seconds=3.0, check_direction=False
            )
            stops.append(yaw_at_stop)
        self.assertAlmostEqual(stops[0], stops[1], delta=4.0)

    def test_zero_coast_drives_all_the_way_to_the_target(self) -> None:
        imu = FakeImu(rate_dps=900.0)
        yaw_at_stop, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(), imu, target_deg=90.0, coast_deg=0.0, max_seconds=2.0, check_direction=False
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


class RotationModeTests(unittest.TestCase):
    """Every mode must rotate the way its sign says, or a nudge runs away.

    For a differential base `omega = (v_r - v_l) / W`, so a left turn (positive
    yaw, counter-clockwise, matching the IMU) requires right minus left to be
    positive in every mode. That single invariant catches a swapped tread far
    more reliably than eyeballing four command tuples.
    """

    def omega_sign(self, mode: str, direction: float) -> float:
        left, right = turn.rotation_command(mode, direction)
        return right - left

    def test_every_mode_turns_left_for_a_positive_direction(self) -> None:
        for mode in turn.ROTATION_MODES:
            with self.subTest(mode=mode):
                self.assertGreater(self.omega_sign(mode, 1.0), 0.0)

    def test_every_mode_turns_right_for_a_negative_direction(self) -> None:
        for mode in turn.ROTATION_MODES:
            with self.subTest(mode=mode):
                self.assertLess(self.omega_sign(mode, -1.0), 0.0)

    def test_pivot_drives_both_treads_and_single_modes_drive_one(self) -> None:
        self.assertEqual(turn.rotation_command("pivot", 1.0), (-1.0, 1.0))
        self.assertEqual(turn.rotation_command("tread-forward", 1.0), (0.0, 1.0))
        self.assertEqual(turn.rotation_command("tread-reverse", 1.0), (-1.0, 0.0))
        self.assertEqual(turn.rotation_command("tread-forward", -1.0), (1.0, 0.0))
        self.assertEqual(turn.rotation_command("tread-reverse", -1.0), (0.0, -1.0))

    def test_single_tread_modes_command_exactly_one_tread(self) -> None:
        for mode in ("tread-forward", "tread-reverse"):
            for direction in (1.0, -1.0):
                with self.subTest(mode=mode, direction=direction):
                    command = turn.rotation_command(mode, direction)
                    self.assertEqual(sum(1 for value in command if value != 0.0), 1)

    def test_tread_reverse_only_ever_drives_backward(self) -> None:
        for direction in (1.0, -1.0):
            self.assertTrue(
                all(value <= 0.0 for value in turn.rotation_command("tread-reverse", direction))
            )

    def test_tread_forward_only_ever_drives_forward(self) -> None:
        for direction in (1.0, -1.0):
            self.assertTrue(
                all(value >= 0.0 for value in turn.rotation_command("tread-forward", direction))
            )

    def test_an_unknown_mode_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "unknown rotation mode"):
            turn.rotation_command("sideways", 1.0)

    def test_the_default_nudge_mode_is_the_steadiest_lever(self) -> None:
        """Equal to tread-reverse at 0.08 s, but tighter spread at longer pulses."""
        self.assertEqual(turn.DEFAULT_NUDGE_MODE, "tread-forward")


class CorrectionDirectionTests(unittest.TestCase):
    """Regression cover for an overshoot correction that drove nowhere.

    A turn that overshot to +109 deg has to come back to a +90 deg target by
    turning RIGHT. Deriving direction from the sign of the target instead of
    the sign of the remaining error made the stop condition true on entry, so
    the correction exited without ever commanding a motor.
    """

    def test_correcting_an_overshoot_turns_back_toward_the_target(self) -> None:
        motors = FakeMotors()
        imu = FakeImu(rate_dps=-900.0, initial_yaw=109.4)
        yaw_at_stop, _rate, _duration, abort = turn.drive_until_angle(
            motors, imu, target_deg=90.0, coast_deg=0.0, max_seconds=2.0, check_direction=False
        )
        self.assertIsNone(abort)
        self.assertTrue(motors.commands, "correction commanded no motion at all")
        self.assertEqual(set(motors.commands), {turn.MOTIONS["right"]})
        self.assertLessEqual(yaw_at_stop, 90.0)

    def test_correcting_an_undershoot_continues_in_the_same_direction(self) -> None:
        motors = FakeMotors()
        imu = FakeImu(rate_dps=900.0, initial_yaw=70.0)
        yaw_at_stop, _rate, _duration, abort = turn.drive_until_angle(
            motors, imu, target_deg=90.0, coast_deg=0.0, max_seconds=2.0, check_direction=False
        )
        self.assertIsNone(abort)
        self.assertEqual(set(motors.commands), {turn.MOTIONS["left"]})
        self.assertGreaterEqual(yaw_at_stop, 90.0)

    def test_a_negative_target_reached_from_further_negative_turns_left(self) -> None:
        motors = FakeMotors()
        imu = FakeImu(rate_dps=900.0, initial_yaw=-109.0)
        _yaw, _rate, _duration, abort = turn.drive_until_angle(
            motors, imu, target_deg=-90.0, coast_deg=0.0, max_seconds=2.0, check_direction=False
        )
        self.assertIsNone(abort)
        self.assertEqual(set(motors.commands), {turn.MOTIONS["left"]})


class AbortTests(unittest.TestCase):
    def test_a_base_that_never_moves_aborts_instead_of_spinning_out_the_clock(self) -> None:
        _yaw, _rate, duration, abort = turn.drive_until_angle(
            FakeMotors(),
            FakeImu(0.0),
            90.0,
            0.0,
            max_seconds=30.0,
            check_direction=True,
            motion_timeout_s=0.4,
        )
        self.assertIsNotNone(abort)
        self.assertIn("never started turning", str(abort))
        self.assertLess(duration, 1.0)

    def test_a_slow_receiver_wake_is_waited_out_rather_than_aborted(self) -> None:
        """Unprimed, the base can sit still for a good part of a second."""
        imu = FakeImu(rate_dps=900.0, wake_delay_s=0.35)
        yaw_at_stop, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(),
            imu,
            90.0,
            0.0,
            max_seconds=5.0,
            check_direction=True,
            motion_timeout_s=1.5,
        )
        self.assertIsNone(abort)
        self.assertGreaterEqual(yaw_at_stop, 90.0)

    def test_turning_the_wrong_way_aborts_and_names_the_service_flags(self) -> None:
        _yaw, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(), FakeImu(-200.0), 90.0, 0.0, max_seconds=30.0, check_direction=True
        )
        self.assertIsNotNone(abort)
        self.assertIn("wrong way", str(abort))
        self.assertIn("invert-left", str(abort))

    def test_a_wrong_way_turn_is_caught_even_after_a_slow_wake(self) -> None:
        _yaw, _rate, _duration, abort = turn.drive_until_angle(
            FakeMotors(),
            FakeImu(-200.0, wake_delay_s=0.3),
            90.0,
            0.0,
            max_seconds=30.0,
            check_direction=True,
            motion_timeout_s=2.0,
        )
        self.assertIn("wrong way", str(abort))

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


class CoastLearningTests(unittest.TestCase):
    def test_the_first_measurement_is_adopted_outright(self) -> None:
        self.assertAlmostEqual(turn.blend_coast(None, 33.8), 33.8)

    def test_later_measurements_move_the_stored_value_partway(self) -> None:
        self.assertAlmostEqual(turn.blend_coast(30.0, 40.0, weight=0.35), 33.5, places=4)

    def test_absurd_measurements_are_clamped_rather_than_trusted(self) -> None:
        self.assertAlmostEqual(turn.blend_coast(None, 400.0), 90.0)
        self.assertAlmostEqual(turn.blend_coast(None, -3.0), 0.0)


class CalibrationFileTests(unittest.TestCase):
    def test_values_round_trip(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "nested" / "imu_turn_calibration.json"
            turn.save_calibration({"coast_deg_left": 33.5}, path)
            self.assertEqual(turn.load_calibration(path), {"coast_deg_left": 33.5})

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
            path.write_text(json.dumps({"coast_deg_left": 33.5, "note": "hi", "flag": True}))
            self.assertEqual(turn.load_calibration(path), {"coast_deg_left": 33.5})


class TurnResultTests(unittest.TestCase):
    def test_error_is_target_minus_achieved(self) -> None:
        result = turn.TurnResult(
            target_deg=90.0,
            achieved_deg=86.5,
            yaw_at_stop_deg=70.0,
            rate_at_stop_dps=180.0,
            coast_deg=16.5,
            duration_s=0.6,
            packets=12,
        )
        self.assertAlmostEqual(result.error_deg, 3.5)
        self.assertIn("coast", result.summary())


if __name__ == "__main__":
    unittest.main()
