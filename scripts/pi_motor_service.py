#!/usr/bin/env python3
"""Low-latency UDP motor service for the GPIO-gated GT004 remote matrix."""

from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import time
from dataclasses import dataclass
from typing import Any

from remote_gpio_controller import MATRIX_PINS, apply_matrix_row_level


DEFAULT_PORT = 8765
DEFAULT_WATCHDOG_SECONDS = 0.35
NETWORK_POLL_NS = 500_000


@dataclass(frozen=True)
class WheelCommand:
    session: str
    sequence: int
    left: float
    right: float


def normalized_wheel(value: Any, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or not -1.0 <= result <= 1.0:
        raise ValueError(f"{name} must be finite and between -1 and 1")
    return result


def parse_command(payload: bytes) -> WheelCommand:
    try:
        document = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("packet must be a UTF-8 JSON object") from exc
    if not isinstance(document, dict):
        raise ValueError("packet must be a JSON object")
    session = document.get("session")
    if not isinstance(session, str) or not session or len(session) > 64:
        raise ValueError("session must be a non-empty string of at most 64 characters")
    sequence = document.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("sequence must be a non-negative integer")
    return WheelCommand(
        session=session,
        sequence=sequence,
        left=normalized_wheel(document.get("left"), "left"),
        right=normalized_wheel(document.get("right"), "right"),
    )


def require_binary_wheels(command: WheelCommand) -> None:
    """Reject magnitudes the toy receiver cannot reproduce reliably."""
    for name, value in (("left", command.left), ("right", command.right)):
        if math.isclose(value, 0.0, abs_tol=1e-9):
            continue
        if math.isclose(abs(value), 1.0, abs_tol=1e-9):
            continue
        raise ValueError(
            f"{name} magnitude must be 0 or 1; fractional pulse-density "
            "control is not physically verified"
        )


def actions_for_wheels(left: float, right: float, deadband: float = 0.05) -> tuple[str, ...]:
    actions: list[str] = []
    if left > deadband:
        actions.append("left-forward")
    elif left < -deadband:
        actions.append("left-reverse")
    if right > deadband:
        actions.append("right-forward")
    elif right < -deadband:
        actions.append("right-reverse")
    return tuple(actions)


class MatrixMotorRuntime:
    def __init__(
        self,
        gpio_module: Any | None = None,
        invert_left: bool = False,
        invert_right: bool = False,
        swap_sides: bool = False,
    ) -> None:
        if gpio_module is None:
            try:
                import RPi.GPIO as gpio_module  # type: ignore[import-not-found]
            except ImportError as exc:
                raise RuntimeError("RPi.GPIO is unavailable; run the service on the Pi") from exc

        self.gpio = gpio_module
        self.invert_left = invert_left
        self.invert_right = invert_right
        self.swap_sides = swap_sides
        self.gpio.setwarnings(False)
        self.gpio.setmode(self.gpio.BCM)
        self.all_pins = sorted({pin for pair in MATRIX_PINS.values() for pin in pair})
        self.column_pins = sorted({pair[1] for pair in MATRIX_PINS.values()})
        for pin in self.all_pins:
            self.gpio.setup(pin, self.gpio.IN, pull_up_down=self.gpio.PUD_OFF)
        self.actions: tuple[str, ...] = ()
        self.sink_active = {name: False for name in MATRIX_PINS}
        self.row_was_low = {name: False for name in MATRIX_PINS}
        self.window_enabled = {name: False for name in MATRIX_PINS}
        self.duty_accumulator = {name: 0.0 for name in MATRIX_PINS}
        self.action_levels = {name: 0.0 for name in MATRIX_PINS}
        self.window_counts = {name: 0 for name in MATRIX_PINS}

    def set_wheels(self, left: float, right: float) -> tuple[str, ...]:
        if self.swap_sides:
            left, right = right, left
        if self.invert_left:
            left = -left
        if self.invert_right:
            right = -right
        new_actions = actions_for_wheels(left, right)
        if new_actions != self.actions:
            # Release and reset ONLY the actions that stopped being commanded.
            #
            # This used to release every column and reset every action's scan
            # state on any change. That is fine for steady commands, but
            # heading control drops individual command slots on one tread at
            # 20 Hz, and each of those changes tore down the *other* tread's
            # column too, every 50 ms against a 40 ms scan period. The tread
            # that was supposed to keep running lost most of its press windows,
            # so steering one tread down slowed the other one more, and the
            # correction drove the base further off course instead of back.
            for name in set(self.actions) - set(new_actions):
                self.release_action(name)
                self.row_was_low[name] = False
                self.window_enabled[name] = False
                self.duty_accumulator[name] = 0.0
            self.actions = new_actions
        levels = {
            "left-forward": max(left, 0.0),
            "left-reverse": max(-left, 0.0),
            "right-forward": max(right, 0.0),
            "right-reverse": max(-right, 0.0),
        }
        for name, level in levels.items():
            previous = self.action_levels[name]
            self.action_levels[name] = level
            if name in new_actions and previous <= 0.05 < level:
                # Make the first requested scan window active. Subsequent
                # windows use an error accumulator, producing an even pulse
                # density without resetting phase on every network refresh.
                self.duty_accumulator[name] = 1.0 - level
        return self.actions

    def poll(self) -> None:
        for name in self.actions:
            row_pin, column_pin = MATRIX_PINS[name]
            row_low = int(self.gpio.input(row_pin)) == 0
            if row_low and not self.row_was_low[name]:
                self.window_counts[name] += 1
                accumulator = self.duty_accumulator[name] + self.action_levels[name]
                self.window_enabled[name] = accumulator >= 1.0 - 1e-9
                self.duty_accumulator[name] = (
                    accumulator - 1.0 if self.window_enabled[name] else accumulator
                )
            self.sink_active[name] = apply_matrix_row_level(
                self.gpio,
                column_pin,
                row_low and self.window_enabled[name],
                self.sink_active[name],
            )
            self.row_was_low[name] = row_low
            if not row_low:
                self.window_enabled[name] = False

    def release_action(self, name: str) -> None:
        """Return one action's column to a high-impedance input."""
        _row_pin, column_pin = MATRIX_PINS[name]
        self.gpio.setup(column_pin, self.gpio.IN, pull_up_down=self.gpio.PUD_OFF)
        self.sink_active[name] = False

    def release_columns(self) -> None:
        for pin in self.column_pins:
            self.gpio.setup(pin, self.gpio.IN, pull_up_down=self.gpio.PUD_OFF)
        for name in self.sink_active:
            self.sink_active[name] = False

    def close(self) -> None:
        self.actions = ()
        self.release_columns()
        for pin in self.all_pins:
            self.gpio.setup(pin, self.gpio.IN, pull_up_down=self.gpio.PUD_OFF)
        self.gpio.cleanup(self.all_pins)


def response_payload(
    command: WheelCommand | None,
    ok: bool,
    actions: tuple[str, ...] = (),
    error: str | None = None,
) -> bytes:
    document: dict[str, Any] = {
        "ok": ok,
        "session": command.session if command is not None else None,
        "sequence": command.sequence if command is not None else None,
        "actions": list(actions),
    }
    if error is not None:
        document["error"] = error
    return json.dumps(document, separators=(",", ":")).encode("utf-8")


def run_server(
    bind: str,
    port: int,
    watchdog_seconds: float,
    invert_left: bool = False,
    invert_right: bool = False,
    swap_sides: bool = False,
    allow_experimental_pulse_density: bool = False,
) -> int:
    runtime = MatrixMotorRuntime(
        invert_left=invert_left,
        invert_right=invert_right,
        swap_sides=swap_sides,
    )
    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((bind, port))
    server.setblocking(False)
    running = True
    last_command_ns = time.monotonic_ns()
    watchdog_ns = int(watchdog_seconds * 1_000_000_000)
    next_network_poll_ns = 0
    client_sequences: dict[tuple[str, int], tuple[str, int]] = {}

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(
        f"House Bot motor service listening on {bind}:{port}; "
        f"watchdog={watchdog_seconds:.3f}s "
        f"invert_left={invert_left} invert_right={invert_right} "
        f"swap_sides={swap_sides} "
        f"command_mode={'experimental-pulse-density' if allow_experimental_pulse_density else 'binary-only'}",
        flush=True,
    )

    try:
        while running:
            now_ns = time.monotonic_ns()
            if runtime.actions and now_ns - last_command_ns > watchdog_ns:
                runtime.set_wheels(0.0, 0.0)
                print("Command watchdog expired; motors released.", flush=True)

            if now_ns >= next_network_poll_ns:
                next_network_poll_ns = now_ns + NETWORK_POLL_NS
                while True:
                    try:
                        payload, address = server.recvfrom(4096)
                    except BlockingIOError:
                        break
                    command: WheelCommand | None = None
                    try:
                        command = parse_command(payload)
                        if not allow_experimental_pulse_density:
                            require_binary_wheels(command)
                        previous = client_sequences.get(address)
                        stale = (
                            previous is not None
                            and previous[0] == command.session
                            and command.sequence < previous[1]
                        )
                        if stale:
                            response = response_payload(
                                command,
                                False,
                                runtime.actions,
                                "stale sequence",
                            )
                        else:
                            client_sequences[address] = (
                                command.session,
                                command.sequence,
                            )
                            actions = runtime.set_wheels(command.left, command.right)
                            last_command_ns = now_ns
                            response = response_payload(command, True, actions)
                    except ValueError as exc:
                        response = response_payload(command, False, error=str(exc))
                    server.sendto(response, address)

            runtime.poll()
    finally:
        runtime.close()
        server.close()
        print("House Bot motor service stopped; motors released.", flush=True)

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve House Bot motor commands over UDP")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--watchdog", type=float, default=DEFAULT_WATCHDOG_SECONDS)
    parser.add_argument("--invert-left", action="store_true")
    parser.add_argument("--invert-right", action="store_true")
    parser.add_argument("--swap-sides", action="store_true")
    parser.add_argument(
        "--allow-experimental-pulse-density",
        action="store_true",
        help="accept fractional wheel magnitudes; unsafe until both treads are verified",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")
    if not 0.1 <= args.watchdog <= 2.0:
        raise SystemExit("--watchdog must be between 0.1 and 2.0 seconds")
    return run_server(
        args.bind,
        args.port,
        args.watchdog,
        args.invert_left,
        args.invert_right,
        args.swap_sides,
        args.allow_experimental_pulse_density,
    )


if __name__ == "__main__":
    raise SystemExit(main())
