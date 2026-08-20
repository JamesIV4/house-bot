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
    def __init__(self, gpio_module: Any | None = None) -> None:
        if gpio_module is None:
            try:
                import RPi.GPIO as gpio_module  # type: ignore[import-not-found]
            except ImportError as exc:
                raise RuntimeError("RPi.GPIO is unavailable; run the service on the Pi") from exc

        self.gpio = gpio_module
        self.gpio.setwarnings(False)
        self.gpio.setmode(self.gpio.BCM)
        self.all_pins = sorted({pin for pair in MATRIX_PINS.values() for pin in pair})
        self.column_pins = sorted({pair[1] for pair in MATRIX_PINS.values()})
        for pin in self.all_pins:
            self.gpio.setup(pin, self.gpio.IN, pull_up_down=self.gpio.PUD_OFF)
        self.actions: tuple[str, ...] = ()
        self.sink_active = {name: False for name in MATRIX_PINS}
        self.window_counts = {name: 0 for name in MATRIX_PINS}

    def set_wheels(self, left: float, right: float) -> tuple[str, ...]:
        new_actions = actions_for_wheels(left, right)
        if new_actions != self.actions:
            self.release_columns()
            self.actions = new_actions
        return self.actions

    def poll(self) -> None:
        for name in self.actions:
            row_pin, column_pin = MATRIX_PINS[name]
            row_low = int(self.gpio.input(row_pin)) == 0
            if row_low and not self.sink_active[name]:
                self.window_counts[name] += 1
            self.sink_active[name] = apply_matrix_row_level(
                self.gpio,
                column_pin,
                row_low,
                self.sink_active[name],
            )

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


def run_server(bind: str, port: int, watchdog_seconds: float) -> int:
    runtime = MatrixMotorRuntime()
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
        f"watchdog={watchdog_seconds:.3f}s",
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
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")
    if not 0.1 <= args.watchdog <= 2.0:
        raise SystemExit("--watchdog must be between 0.1 and 2.0 seconds")
    return run_server(args.bind, args.port, args.watchdog)


if __name__ == "__main__":
    raise SystemExit(main())
