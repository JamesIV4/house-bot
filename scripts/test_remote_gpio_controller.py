#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("remote_gpio_controller.py")
SPEC = importlib.util.spec_from_file_location("remote_gpio_controller", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
remote = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(remote)


class FakeBackend:
    def __init__(self) -> None:
        self.events: list[tuple[str, tuple[str, ...]]] = []

    def press(self, names: tuple[str, ...]) -> None:
        self.events.append(("press", tuple(names)))

    def release_all(self) -> None:
        self.events.append(("release", ()))


class FakeGpio:
    BCM = "BCM"
    IN = "IN"
    OUT = "OUT"
    PUD_OFF = "PUD_OFF"
    LOW = 0
    HIGH = 1

    def __init__(self) -> None:
        self.setup_calls: list[tuple[int, str, dict[str, object]]] = []

    def setwarnings(self, _enabled: bool) -> None:
        pass

    def setmode(self, _mode: str) -> None:
        pass

    def setup(self, pin: int, mode: str, **kwargs: object) -> None:
        self.setup_calls.append((pin, mode, kwargs))

    def cleanup(self, _pins: object) -> None:
        pass


class RemoteGpioControllerTests(unittest.TestCase):
    def test_motion_mapping_is_differential_drive(self) -> None:
        self.assertEqual(
            remote.MOTIONS,
            {
                "forward": ("left-forward", "right-forward"),
                "reverse": ("left-reverse", "right-reverse"),
                "left": ("left-reverse", "right-forward"),
                "right": ("left-forward", "right-reverse"),
            },
        )

    def test_pulse_always_releases_after_sleep_failure(self) -> None:
        backend = FakeBackend()

        def fail_sleep(_seconds: float) -> None:
            raise RuntimeError("test failure")

        with self.assertRaisesRegex(RuntimeError, "test failure"):
            remote.pulse_buttons(
                backend,
                ("left-forward",),
                0.1,
                sleep_fn=fail_sleep,
            )

        self.assertEqual(
            backend.events,
            [("press", ("left-forward",)), ("release", ())],
        )

    def test_gpio_backend_only_uses_input_or_output_low(self) -> None:
        gpio = FakeGpio()
        backend = remote.RpiOpenDrainButtons(remote.DEFAULT_PINS, gpio_module=gpio)
        backend.press(("left-forward", "right-forward"))
        backend.close()

        output_calls = [call for call in gpio.setup_calls if call[1] == gpio.OUT]
        self.assertEqual(len(output_calls), 2)
        self.assertTrue(all(call[2].get("initial") == gpio.LOW for call in output_calls))
        self.assertFalse(
            any(call[2].get("initial") == gpio.HIGH for call in gpio.setup_calls)
        )
        self.assertTrue(
            all(
                call[2].get("pull_up_down") == gpio.PUD_OFF
                for call in gpio.setup_calls
                if call[1] == gpio.IN
            )
        )

    def test_duplicate_pins_are_rejected(self) -> None:
        pins = dict(remote.DEFAULT_PINS)
        pins["right-forward"] = pins["left-forward"]
        with self.assertRaisesRegex(ValueError, "different BCM GPIO"):
            remote.validate_pin_map(pins)

    def test_duration_is_bounded_to_short_pulses(self) -> None:
        self.assertEqual(remote.bounded_duration("0.2"), 0.2)
        with self.assertRaises(Exception):
            remote.bounded_duration("3.0")

    def test_matrix_gate_only_sinks_confirmed_column_during_row_low(self) -> None:
        gpio = FakeGpio()
        active = False
        active = remote.apply_matrix_row_level(gpio, 4, False, active)
        active = remote.apply_matrix_row_level(gpio, 4, True, active)
        active = remote.apply_matrix_row_level(gpio, 4, True, active)
        active = remote.apply_matrix_row_level(gpio, 4, False, active)

        self.assertFalse(active)
        self.assertEqual(
            gpio.setup_calls,
            [
                (4, gpio.OUT, {"initial": gpio.LOW}),
                (4, gpio.IN, {"pull_up_down": gpio.PUD_OFF}),
            ],
        )
        self.assertFalse(
            any(call[2].get("initial") == gpio.HIGH for call in gpio.setup_calls)
        )

    def test_matrix_gate_rejects_same_row_and_column_pin(self) -> None:
        with self.assertRaisesRegex(ValueError, "must be different"):
            remote.gated_matrix_pulse(4, 4, 0.02)

    def test_verified_matrix_motion_mapping_is_differential_drive(self) -> None:
        self.assertEqual(
            remote.MATRIX_PINS,
            {
                "left-forward": (5, 4),
                "left-reverse": (7, 6),
                "right-forward": (9, 8),
                "right-reverse": (11, 10),
            },
        )
        self.assertEqual(
            remote.MATRIX_MOTIONS,
            {
                "forward": ("left-forward", "right-forward"),
                "reverse": ("left-reverse", "right-reverse"),
                "left": ("left-reverse", "right-forward"),
                "right": ("left-forward", "right-reverse"),
            },
        )

    def test_matrix_actions_reject_opposed_wheel_commands(self) -> None:
        with self.assertRaisesRegex(ValueError, "left wheel"):
            remote.validate_matrix_actions(("left-forward", "left-reverse"))
        with self.assertRaisesRegex(ValueError, "right wheel"):
            remote.validate_matrix_actions(("right-forward", "right-reverse"))



def scan_trace(
    seconds: float,
    width_us: float = 215.0,
    period_us: float = 40_130.0,
) -> tuple[list[float], list[float]]:
    """A row's measured signature: a narrow low window once per scan period."""
    count = int(seconds / (period_us / 1_000_000.0))
    return [width_us] * count, [period_us] * (count - 1)


class ScanClassifierTests(unittest.TestCase):
    def test_a_measured_row_signature_is_called_a_row(self) -> None:
        low_widths, periods = scan_trace(2.0)
        verdict, detail = remote.classify_scan_pin(low_widths, periods, 2.0)
        self.assertEqual(verdict, "row")
        self.assertIn("median period 40.13ms", detail)

    def test_a_steady_pin_is_called_a_column(self) -> None:
        verdict, detail = remote.classify_scan_pin([], [], 2.0)
        self.assertEqual(verdict, "column")
        self.assertIn("steady", detail)

    def test_a_few_stray_edges_do_not_make_a_row(self) -> None:
        verdict, _detail = remote.classify_scan_pin([215.0] * 3, [40_130.0] * 2, 2.0)
        self.assertEqual(verdict, "column")

    def test_a_wrong_period_is_flagged_rather_than_guessed(self) -> None:
        low_widths, periods = scan_trace(2.0, period_us=5_000.0)
        verdict, _detail = remote.classify_scan_pin(low_widths, periods, 2.0)
        self.assertEqual(verdict, "unclear")

    def test_a_wide_low_window_is_flagged_rather_than_guessed(self) -> None:
        low_widths, periods = scan_trace(2.0, width_us=9_000.0)
        verdict, _detail = remote.classify_scan_pin(low_widths, periods, 2.0)
        self.assertEqual(verdict, "unclear")


class IdentifyRowsTests(unittest.TestCase):
    def fake_profiler(self, rows: set[int]):
        def profile(pin: int, seconds: float) -> tuple[list[float], list[float]]:
            return scan_trace(seconds) if pin in rows else ([], [])

        return profile

    def test_each_pair_is_ordered_row_then_column(self) -> None:
        pairs = {"left-forward": (17, 18), "right-reverse": (26, 20)}
        resolved, _report = remote.identify_matrix_rows(
            pairs, 2.0, self.fake_profiler({18, 26})
        )
        self.assertEqual(resolved["left-forward"], (18, 17))
        self.assertEqual(resolved["right-reverse"], (26, 20))

    def test_a_pair_with_no_scanning_wire_is_left_unresolved(self) -> None:
        resolved, report = remote.identify_matrix_rows(
            {"left-forward": (17, 18)}, 2.0, self.fake_profiler(set())
        )
        self.assertNotIn("left-forward", resolved)
        self.assertTrue(any("neither" in line for line in report))

    def test_two_wires_on_one_edge_are_reported_not_guessed(self) -> None:
        resolved, report = remote.identify_matrix_rows(
            {"left-forward": (17, 18)}, 2.0, self.fake_profiler({17, 18})
        )
        self.assertNotIn("left-forward", resolved)
        self.assertTrue(any("both scan" in line for line in report))

    def test_the_printed_map_marks_what_was_not_resolved(self) -> None:
        rendered = remote.format_matrix_pins({"left-forward": (18, 17)})
        self.assertIn('"left-forward": (18, 17),', rendered)
        self.assertIn("UNRESOLVED", rendered)

    def test_candidate_pairs_avoid_the_imu_and_reserved_buses(self) -> None:
        pins = sorted({pin for pair in remote.CANDIDATE_PAIRS.values() for pin in pair})
        self.assertEqual(len(pins), 8)
        # BCM2/3 carry the MPU-6050; 14/15 the serial console; 9/10/11 SPI0;
        # 12/13 both hardware PWM channels; 0/1 the HAT ID EEPROM.
        for reserved in (0, 1, 2, 3, 9, 10, 11, 12, 13, 14, 15):
            self.assertNotIn(reserved, pins)


if __name__ == "__main__":
    unittest.main()
