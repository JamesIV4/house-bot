#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
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


service = load("pi_motor_service")
client = load("send_motor_command")


def paired_motion(*, shared: bool):
    """Find a paired motion whose two actions do or do not share a column.

    Which motion shares one is a property of the harness, not of the code. It
    was the pivots (both halves on BCM22 or BCM17) until the right tread moved
    from channel C to channel D on 2026-08-30; it is forward/reverse now. The
    cooperation rule below is the same either way, so the tests derive the
    motion rather than naming one.

    Returns (left_wheel, right_wheel, left_action, right_action, column).
    """
    candidates = (
        (1.0, 1.0, "left-forward", "right-forward"),
        (-1.0, -1.0, "left-reverse", "right-reverse"),
        (-1.0, 1.0, "left-reverse", "right-forward"),
        (1.0, -1.0, "left-forward", "right-reverse"),
    )
    for left, right, first, second in candidates:
        first_column = service.MATRIX_PINS[first][1]
        second_column = service.MATRIX_PINS[second][1]
        if (first_column == second_column) is shared:
            return left, right, first, second, first_column
    raise AssertionError(
        f"no paired motion with shared={shared}; MATRIX_PINS looks wrong"
    )


class PiMotorServiceTests(unittest.TestCase):
    def test_command_parser_accepts_normalized_wheels(self) -> None:
        command = service.parse_command(
            b'{"session":"abc","sequence":7,"left":1,"right":-0.5}'
        )
        self.assertEqual(command, service.WheelCommand("abc", 7, 1.0, -0.5))

    def test_command_parser_rejects_bad_or_out_of_range_values(self) -> None:
        bad_packets = (
            b"not-json",
            b"[]",
            b'{"session":"abc","sequence":-1,"left":0,"right":0}',
            b'{"session":"abc","sequence":1,"left":1.1,"right":0}',
            b'{"session":"abc","sequence":1,"left":true,"right":0}',
            b'{"session":"","sequence":1,"left":0,"right":0}',
        )
        for packet in bad_packets:
            with self.subTest(packet=packet), self.assertRaises(ValueError):
                service.parse_command(packet)

    def test_binary_mode_rejects_fractional_magnitudes(self) -> None:
        service.require_binary_wheels(
            service.WheelCommand("abc", 1, 1.0, -1.0)
        )
        service.require_binary_wheels(
            service.WheelCommand("abc", 2, 0.0, 0.0)
        )
        with self.assertRaisesRegex(ValueError, "fractional pulse-density"):
            service.require_binary_wheels(
                service.WheelCommand("abc", 3, 0.5, 1.0)
            )

    def test_wheel_signs_select_one_direction_per_side(self) -> None:
        self.assertEqual(
            service.actions_for_wheels(1.0, 1.0),
            ("left-forward", "right-forward"),
        )
        self.assertEqual(
            service.actions_for_wheels(-1.0, -1.0),
            ("left-reverse", "right-reverse"),
        )
        self.assertEqual(
            service.actions_for_wheels(-1.0, 1.0),
            ("left-reverse", "right-forward"),
        )
        self.assertEqual(service.actions_for_wheels(0.01, -0.01), ())

    def test_client_motion_mapping_matches_robot_directions(self) -> None:
        self.assertEqual(
            client.MOTIONS,
            {
                "forward": (1.0, 1.0),
                "reverse": (-1.0, -1.0),
                "left": (-1.0, 1.0),
                "right": (1.0, -1.0),
                "left-tread-forward": (1.0, 0.0),
                "left-tread-reverse": (-1.0, 0.0),
                "right-tread-forward": (0.0, 1.0),
                "right-tread-reverse": (0.0, -1.0),
                "stop": (0.0, 0.0),
            },
        )

    def test_client_packet_round_trips_through_service_parser(self) -> None:
        payload = client.encode_command("session-a", 12, -1.0, 1.0)
        self.assertEqual(
            json.loads(payload),
            {"session": "session-a", "sequence": 12, "left": -1.0, "right": 1.0},
        )
        self.assertEqual(
            service.parse_command(payload),
            service.WheelCommand("session-a", 12, -1.0, 1.0),
        )

    def make_gpio(self):
        class FakeGpio:
            BCM = "BCM"
            IN = "IN"
            OUT = "OUT"
            PUD_OFF = "PUD_OFF"
            LOW = 0

            def __init__(self) -> None:
                self.levels = {pin: 1 for pair in service.MATRIX_PINS.values() for pin in pair}
                self.setup_calls = []

            def setwarnings(self, _enabled):
                pass

            def setmode(self, _mode):
                pass

            def setup(self, pin, mode, **kwargs):
                self.setup_calls.append((pin, mode, kwargs))

            def input(self, pin):
                return self.levels[pin]

            def cleanup(self, _pins):
                pass

        return FakeGpio()

    def test_dropping_one_tread_leaves_the_other_treads_column_alone(self) -> None:
        """Regression: heading control drops slots on one tread at 20 Hz.

        Releasing every column on any action change tore down the tread that
        was meant to keep running, every 50 ms against a 40 ms scan period. The
        base then veered harder the more the controller corrected.
        """
        gpio = self.make_gpio()
        runtime = service.MatrixMotorRuntime(gpio)
        _right_row, right_column = service.MATRIX_PINS["right-forward"]

        runtime.set_wheels(1.0, 1.0)
        del gpio.setup_calls[:]
        # One dropped left slot, then back on, as the steering loop does.
        runtime.set_wheels(0.0, 1.0)
        runtime.set_wheels(1.0, 1.0)

        touched = [call for call in gpio.setup_calls if call[0] == right_column]
        self.assertEqual(
            touched,
            [],
            "the right tread's column was reconfigured while only the left tread changed",
        )

    def test_dropping_one_tread_preserves_the_other_treads_scan_state(self) -> None:
        gpio = self.make_gpio()
        runtime = service.MatrixMotorRuntime(gpio)
        runtime.set_wheels(1.0, 1.0)
        runtime.row_was_low["right-forward"] = True

        runtime.set_wheels(0.0, 1.0)

        self.assertTrue(runtime.row_was_low["right-forward"])

    def test_a_tread_that_stops_is_still_released_and_reset(self) -> None:
        # Uses a motion whose two actions are on separate columns, so stopping
        # one tread really must release its column. Where the two share a
        # column the correct behaviour is the opposite -- see
        # test_dropping_half_a_shared_column_motion_keeps_the_column.
        left, right, first, _second, column = paired_motion(shared=False)
        gpio = self.make_gpio()
        runtime = service.MatrixMotorRuntime(gpio)
        runtime.set_wheels(left, right)
        runtime.row_was_low[first] = True
        del gpio.setup_calls[:]

        runtime.set_wheels(0.0, right)

        self.assertIn(
            (column, gpio.IN, {"pull_up_down": gpio.PUD_OFF}),
            gpio.setup_calls,
        )
        self.assertFalse(runtime.row_was_low[first])
        self.assertFalse(runtime.sink_active[column])

    def test_two_actions_sharing_one_column_do_not_cancel(self) -> None:
        """Regression: a paired motion drove both actions through ONE pin.

        With per-action sink state the second action released the press the
        first had asserted microseconds earlier in the same poll pass, giving
        truncated presses. Those helped put the remote into a latched state
        recoverable only by power-cycling it, reproduced on the remote's own
        buttons on 2026-08-27.
        """
        left, right, first, second, column = paired_motion(shared=True)
        first_row = service.MATRIX_PINS[first][0]
        second_row = service.MATRIX_PINS[second][0]

        gpio = self.make_gpio()
        runtime = service.MatrixMotorRuntime(gpio)
        runtime.set_wheels(left, right)
        self.assertEqual(set(runtime.actions), {first, second})

        # First channel's scan window: the shared column must go low and stay
        # low for the whole pass, not be released again by the other action.
        gpio.levels[first_row] = 0
        gpio.levels[second_row] = 1
        del gpio.setup_calls[:]
        runtime.poll()
        self.assertTrue(runtime.sink_active[column])
        self.assertNotIn(
            (column, gpio.IN, {"pull_up_down": gpio.PUD_OFF}),
            gpio.setup_calls,
            "the other action released the shared column mid-press",
        )

        # Now the second channel's window: still low, still uncancelled.
        gpio.levels[first_row] = 1
        gpio.levels[second_row] = 0
        del gpio.setup_calls[:]
        runtime.poll()
        self.assertTrue(runtime.sink_active[column])
        self.assertNotIn(
            (column, gpio.IN, {"pull_up_down": gpio.PUD_OFF}),
            gpio.setup_calls,
        )

        # Neither row scanning: released exactly once.
        gpio.levels[first_row] = 1
        gpio.levels[second_row] = 1
        del gpio.setup_calls[:]
        runtime.poll()
        self.assertFalse(runtime.sink_active[column])

    def test_dropping_half_a_shared_column_motion_keeps_the_column(self) -> None:
        """Releasing one action must not release a column the other still owns."""
        left, right, first, _second, column = paired_motion(shared=True)
        gpio = self.make_gpio()
        runtime = service.MatrixMotorRuntime(gpio)
        runtime.set_wheels(left, right)
        del gpio.setup_calls[:]
        # Drop only the left action, keep the right one running.
        runtime.set_wheels(0.0, right)
        self.assertNotIn(first, runtime.actions)
        self.assertNotIn(
            (column, gpio.IN, {"pull_up_down": gpio.PUD_OFF}),
            gpio.setup_calls,
            "shared column released while the other action still needs it",
        )

    def test_no_frame_ever_has_two_columns_low_at_once(self) -> None:
        """Two buttons on one channel must never appear pressed together.

        Each channel's two directions share its scan row, so if both columns
        are ever low inside one scan window the remote sees forward and
        reverse pressed at the same instant -- an input no operator can
        produce. Applying the columns in pin order allowed exactly that for
        the length of one setup() call, on whichever motion happened to sort
        the wrong way. Releases must therefore precede asserts.
        """
        for left, right, first, second in (
            (1.0, 1.0, "left-forward", "right-forward"),
            (-1.0, -1.0, "left-reverse", "right-reverse"),
            (-1.0, 1.0, "left-reverse", "right-forward"),
            (1.0, -1.0, "left-forward", "right-reverse"),
        ):
            with self.subTest(motion=(left, right)):
                rows = sorted(
                    {service.MATRIX_PINS[first][0], service.MATRIX_PINS[second][0]}
                )
                gpio = self.make_gpio()
                runtime = service.MatrixMotorRuntime(gpio)
                runtime.set_wheels(left, right)

                # Walk a whole frame: each scan row low in turn, then neither.
                low = set()
                for scanning in (*rows, None):
                    for row in rows:
                        gpio.levels[row] = 0 if row == scanning else 1
                    del gpio.setup_calls[:]
                    runtime.poll()
                    for pin, mode, _kwargs in gpio.setup_calls:
                        if mode == gpio.OUT:
                            low.add(pin)
                        else:
                            low.discard(pin)
                        self.assertLessEqual(
                            len(low),
                            1,
                            f"columns {sorted(low)} both low while row "
                            f"{scanning} is scanned",
                        )

    def test_stopping_releases_every_column(self) -> None:
        gpio = self.make_gpio()
        runtime = service.MatrixMotorRuntime(gpio)
        runtime.set_wheels(1.0, 1.0)
        del gpio.setup_calls[:]
        runtime.set_wheels(0.0, 0.0)
        released = {call[0] for call in gpio.setup_calls if call[1] == gpio.IN}
        for name in ("left-forward", "right-forward"):
            self.assertIn(service.MATRIX_PINS[name][1], released)
        self.assertEqual(runtime.actions, ())

    def test_rebuilt_treads_can_be_inverted_without_changing_protocol(self) -> None:
        class NoopGpio:
            BCM = "BCM"
            IN = "IN"
            OUT = "OUT"
            PUD_OFF = "PUD_OFF"

            def setwarnings(self, _enabled):
                pass

            def setmode(self, _mode):
                pass

            def setup(self, _pin, _mode, **_kwargs):
                pass

            def cleanup(self, _pins):
                pass

        runtime = service.MatrixMotorRuntime(
            NoopGpio(), invert_left=True, invert_right=True
        )
        self.assertEqual(
            runtime.set_wheels(1.0, 1.0),
            ("left-reverse", "right-reverse"),
        )
        self.assertEqual(set(runtime.actions), {"left-reverse", "right-reverse"})

    def test_rebuilt_tread_channels_can_be_swapped(self) -> None:
        class NoopGpio:
            BCM = "BCM"
            IN = "IN"
            OUT = "OUT"
            PUD_OFF = "PUD_OFF"

            def setwarnings(self, _enabled):
                pass

            def setmode(self, _mode):
                pass

            def setup(self, _pin, _mode, **_kwargs):
                pass

            def cleanup(self, _pins):
                pass

        runtime = service.MatrixMotorRuntime(
            NoopGpio(),
            invert_left=True,
            invert_right=True,
            swap_sides=True,
        )
        self.assertEqual(
            runtime.set_wheels(-1.0, 1.0),
            ("left-reverse", "right-forward"),
        )


if __name__ == "__main__":
    unittest.main()
