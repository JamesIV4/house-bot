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
                "left-forward": (18, 17),
                "left-reverse": (18, 20),
                "right-forward": (16, 20),
                "right-reverse": (16, 17),
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

    def test_the_matrix_uses_one_pin_per_electrical_net(self) -> None:
        """The harness wires every net twice; using both pins breaks pairs.

        Driving one pin of a net low and releasing the other does not release
        the net, so an intended single press becomes two buttons. Only pairs
        are affected, which is why every action verified correctly alone.
        """
        used = {pin for pair in remote.MATRIX_PINS.values() for pin in pair}
        for name, (first, second) in remote.MATRIX_NETS.items():
            with self.subTest(net=name):
                self.assertFalse(
                    first in used and second in used,
                    f"net {name} is driven through both BCM{first} and BCM{second}",
                )
                self.assertTrue(first in used or second in used, f"net {name} is unused")

    def test_the_matrix_is_two_rows_by_two_columns(self) -> None:
        rows = {row for row, _column in remote.MATRIX_PINS.values()}
        columns = {column for _row, column in remote.MATRIX_PINS.values()}
        self.assertEqual(len(rows), 2)
        self.assertEqual(len(columns), 2)
        # Every row/column intersection is a distinct button.
        self.assertEqual(len(set(remote.MATRIX_PINS.values())), 4)
        self.assertFalse(rows & columns, "a pin is used as both a row and a column")

    def test_each_wheel_direction_sits_at_its_own_intersection(self) -> None:
        pins = remote.MATRIX_PINS
        # A tread's two directions share its channel's own scan row.
        self.assertEqual(pins["left-forward"][0], pins["left-reverse"][0])
        self.assertEqual(pins["right-forward"][0], pins["right-reverse"][0])
        # The treads are on different channels of the remote.
        self.assertNotEqual(pins["left-forward"][0], pins["right-forward"][0])
        # Each tread uses both direction lines, one per direction. Which line
        # carries a given tread's "forward" depends on how that motor is geared
        # and is measured, never assumed: left-forward shared a line with
        # right-REVERSE until the right tread moved from channel C to channel D
        # on 2026-08-30, and shares one with right-FORWARD now.
        for side in ("left", "right"):
            self.assertNotEqual(pins[f"{side}-forward"][1], pins[f"{side}-reverse"][1])
        self.assertEqual(len(set(pins.values())), 4)

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


class GatePairsTests(unittest.TestCase):
    """Arbitrary intersections, for when the named action map is the suspect."""

    def test_a_spec_parses_into_intersections(self) -> None:
        self.assertEqual(
            remote.parse_intersections("18:17,19:22"), [(18, 17), (19, 22)]
        )

    def test_whitespace_and_trailing_commas_are_tolerated(self) -> None:
        self.assertEqual(remote.parse_intersections(" 18:17 , 19:22 ,"), [(18, 17), (19, 22)])

    def test_a_missing_column_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "expected row:column"):
            remote.parse_intersections("18")

    def test_a_row_equal_to_its_column_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "must differ"):
            remote.parse_intersections("18:18")

    def test_an_out_of_range_pin_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "between 0 and 27"):
            remote.parse_intersections("18:99")

    def test_an_empty_spec_is_refused(self) -> None:
        with self.assertRaisesRegex(ValueError, "no intersections"):
            remote.parse_intersections(" , ")

    def make_gpio(self, scanning: set[int] = frozenset()):
        """GPIO fake whose scanning rows dip low on a repeating read cycle."""

        class ScanningGpio:
            BCM = "BCM"
            IN = "IN"
            OUT = "OUT"
            PUD_OFF = "PUD_OFF"
            LOW = 0
            HIGH = 1

            def __init__(self) -> None:
                self.setup_calls: list[tuple[int, str, dict]] = []
                self.reads = 0

            def setwarnings(self, _enabled: bool) -> None:
                pass

            def setmode(self, _mode: str) -> None:
                pass

            def setup(self, pin: int, mode: str, **kwargs) -> None:
                self.setup_calls.append((pin, mode, kwargs))

            def input(self, pin: int) -> int:
                self.reads += 1
                if pin in scanning:
                    return 0 if (self.reads // 4) % 2 else 1
                return 1

            def cleanup(self, _pins) -> None:
                pass

        return ScanningGpio()

    def test_gating_never_drives_a_remote_wire_high(self) -> None:
        gpio = self.make_gpio(scanning={18, 19})
        remote.gate_intersections([(18, 17), (19, 22)], 0.02, gpio)
        self.assertTrue(gpio.setup_calls)
        self.assertFalse(
            any(call[2].get("initial") == gpio.HIGH for call in gpio.setup_calls)
        )

    def test_gating_counts_a_window_per_falling_edge(self) -> None:
        gpio = self.make_gpio(scanning={18})
        windows = remote.gate_intersections([(18, 17)], 0.05, gpio)
        self.assertGreater(windows[(18, 17)], 0)

    def test_a_steady_row_yields_no_windows(self) -> None:
        """A column mistaken for a row shows up as zero windows, not as motion."""
        gpio = self.make_gpio(scanning=set())
        windows = remote.gate_intersections([(18, 17)], 0.02, gpio)
        self.assertEqual(windows[(18, 17)], 0)

    def test_every_column_is_released_afterwards(self) -> None:
        gpio = self.make_gpio(scanning={18, 19})
        remote.gate_intersections([(18, 17), (19, 22)], 0.02, gpio)
        released = [
            call for call in gpio.setup_calls[-6:]
            if call[1] == gpio.IN and call[0] in (17, 22)
        ]
        self.assertTrue(released, "columns were not returned to inputs")


class SharedNetTests(unittest.TestCase):
    """The fault that cost a session: eight wires, four nets.

    `identify-rows` classified every wire correctly and each action worked
    alone, so nothing looked wrong until a command pressed two buttons at once
    and actuated a third.
    """

    # Measured on the GT004 harness, 2026-08-27.
    # Measured by identify-rows 2026-08-30, after the right tread moved from
    # channel C to channel D. Direction lines are 17+19 and 20+22; the scan
    # lines are 18+23 (channel A) and 16+26 (channel D).
    REAL_NETS = {17: [19], 19: [17], 20: [22], 22: [20], 18: [23], 23: [18], 16: [26], 26: [16]}

    def fake_driver(self, nets: dict[int, list[int]]):
        def drive(pin: int, others):
            return [other for other in others if other in nets.get(pin, [])]

        return drive

    def test_the_real_harness_collapses_to_four_nets(self) -> None:
        pins = sorted(self.REAL_NETS)
        nets = remote.detect_shared_nets(pins, self.fake_driver(self.REAL_NETS))
        groups = remote.net_groups(nets)
        self.assertEqual(len(groups), 4)
        self.assertIn((17, 19), groups)
        self.assertIn((20, 22), groups)
        self.assertIn((18, 23), groups)
        self.assertIn((16, 26), groups)

    def test_independent_wires_are_left_alone(self) -> None:
        pins = [4, 5, 6, 7]
        groups = remote.net_groups(remote.detect_shared_nets(pins, self.fake_driver({})))
        self.assertEqual(groups, [(4,), (5,), (6,), (7,)])

    def net_of(self, pin: int) -> frozenset[int]:
        return frozenset({pin, *self.REAL_NETS.get(pin, [])})

    def test_duplicates_are_rewritten_to_one_pin_per_net(self) -> None:
        """Which pin represents a net is arbitrary; the net it names is not."""
        # The four physical button pairs, each named by the wire that landed on
        # it, and each naming its net through the other of the two pins. The
        # direction labels are the ones verified on blocks, not the ones
        # identify-rows prints: identify-rows resolves the electrical roles but
        # cannot know which way a tread turns.
        resolved = {
            "left-forward": (18, 17),
            "left-reverse": (23, 22),
            "right-forward": (26, 20),
            "right-reverse": (16, 19),
        }
        groups = remote.net_groups(
            remote.detect_shared_nets(sorted(self.REAL_NETS), self.fake_driver(self.REAL_NETS))
        )
        rewritten = remote.matrix_pins_from_nets(resolved, groups)

        # Every action must still address the same electrical intersection as
        # the hand-verified map, even if it names it by the other pin.
        for name, (row, column) in remote.MATRIX_PINS.items():
            with self.subTest(action=name):
                self.assertEqual(self.net_of(rewritten[name][0]), self.net_of(row))
                self.assertEqual(self.net_of(rewritten[name][1]), self.net_of(column))

        used = {pin for pair in rewritten.values() for pin in pair}
        self.assertEqual(len(used), 4, "a net is still referenced by two pins")

    def test_the_shipped_map_uses_one_pin_per_net(self) -> None:
        used = {pin for pair in remote.MATRIX_PINS.values() for pin in pair}
        nets = {self.net_of(pin) for pin in used}
        self.assertEqual(len(nets), len(used), "two pins in MATRIX_PINS share a net")

    def test_rewriting_is_a_no_op_when_every_wire_is_its_own_net(self) -> None:
        resolved = {"left-forward": (5, 4)}
        groups = remote.net_groups(remote.detect_shared_nets([4, 5], self.fake_driver({})))
        self.assertEqual(remote.matrix_pins_from_nets(resolved, groups), resolved)


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
