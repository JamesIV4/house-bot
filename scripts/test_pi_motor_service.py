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


if __name__ == "__main__":
    unittest.main()
