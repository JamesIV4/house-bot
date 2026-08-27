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
        runtime.duty_accumulator["right-forward"] = 0.4
        runtime.row_was_low["right-forward"] = True

        runtime.set_wheels(0.0, 1.0)

        self.assertAlmostEqual(runtime.duty_accumulator["right-forward"], 0.4)
        self.assertTrue(runtime.row_was_low["right-forward"])

    def test_a_tread_that_stops_is_still_released_and_reset(self) -> None:
        gpio = self.make_gpio()
        runtime = service.MatrixMotorRuntime(gpio)
        runtime.set_wheels(1.0, 1.0)
        runtime.duty_accumulator["left-forward"] = 0.7
        _left_row, left_column = service.MATRIX_PINS["left-forward"]
        del gpio.setup_calls[:]

        runtime.set_wheels(0.0, 1.0)

        self.assertIn(
            (left_column, gpio.IN, {"pull_up_down": gpio.PUD_OFF}),
            gpio.setup_calls,
        )
        self.assertEqual(runtime.duty_accumulator["left-forward"], 0.0)
        self.assertFalse(runtime.sink_active["left-forward"])

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

    def test_scan_window_duty_cycle_preserves_magnitude(self) -> None:
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

        gpio = FakeGpio()
        runtime = service.MatrixMotorRuntime(gpio)
        runtime.set_wheels(0.5, 0.0)
        row_pin, column_pin = service.MATRIX_PINS["left-forward"]
        for _window in range(8):
            gpio.levels[row_pin] = 0
            runtime.poll()
            runtime.poll()
            gpio.levels[row_pin] = 1
            runtime.poll()

        sink_calls = [
            call for call in gpio.setup_calls
            if call[0] == column_pin and call[1] == gpio.OUT
        ]
        self.assertEqual(runtime.window_counts["left-forward"], 8)
        self.assertEqual(len(sink_calls), 4)

    def test_repeated_network_refresh_does_not_reset_duty_phase(self) -> None:
        class PhaseOnlyGpio:
            BCM = "BCM"
            IN = "IN"
            OUT = "OUT"
            PUD_OFF = "PUD_OFF"
            LOW = 0

            def setwarnings(self, _enabled):
                pass

            def setmode(self, _mode):
                pass

            def setup(self, _pin, _mode, **_kwargs):
                pass

            def input(self, _pin):
                return 1

            def cleanup(self, _pins):
                pass

        runtime = service.MatrixMotorRuntime(PhaseOnlyGpio())
        runtime.set_wheels(0.25, 0.0)
        runtime.duty_accumulator["left-forward"] = 0.25
        runtime.set_wheels(0.25, 0.0)
        self.assertEqual(runtime.duty_accumulator["left-forward"], 0.25)

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
        self.assertEqual(runtime.action_levels["left-reverse"], 1.0)
        self.assertEqual(runtime.action_levels["right-reverse"], 1.0)

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
