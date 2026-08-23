#!/usr/bin/env python3
"""Drive a straight line by duty-cycling the faster tread at the command level.

The original remote is binary full-power/stop (D-020), so heading cannot be
trimmed by lowering a wheel magnitude. Instead every packet stays a verified
full-power command and the faster tread is dropped for a computed fraction of
command slots, which lowers its average speed to match the slower tread.

This is command-slot duty cycling at the packet rate (tens of milliseconds),
not the sub-scan-window GPIO pulse density that D-020 rejected.
"""

from __future__ import annotations

import argparse
import json
import secrets
import socket
import time
from pathlib import Path

from send_motor_command import encode_command


DEFAULT_SUMMARY = Path("config/local/base_calibration.summary.json")


def tread_speeds(summary_path: Path, reverse: bool) -> tuple[float, float]:
    document = json.loads(summary_path.read_text())
    suffix = "reverse" if reverse else "forward"
    left = float(document[f"left_{suffix}_mps"])
    right = float(document[f"right_{suffix}_mps"])
    if left <= 0.0 or right <= 0.0:
        raise ValueError("calibrated tread speeds must be positive")
    return left, right


def duty_from_speeds(left_mps: float, right_mps: float) -> tuple[float, float]:
    """Slow the faster tread to the slower tread's speed; slower runs at 1.0."""
    slower = min(left_mps, right_mps)
    return slower / left_mps, slower / right_mps


def run_route(
    host: str,
    port: int,
    duration: float,
    rate_hz: float,
    left_duty: float,
    right_duty: float,
    direction: float,
) -> tuple[int, int, int, int, bool]:
    interval = 1.0 / rate_hz
    deadline = time.monotonic() + duration
    sequence = 0
    left_slots = 0
    right_slots = 0
    acknowledged: set[int] = set()
    # Error accumulators spread the dropped slots evenly instead of bunching
    # them at one end of the run, which would curve the path and then correct.
    left_accumulator = 0.0
    right_accumulator = 0.0
    session = secrets.token_hex(8)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    def drain() -> None:
        while True:
            try:
                payload, _address = sock.recvfrom(4096)
            except BlockingIOError:
                return
            try:
                response = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if response.get("ok") and response.get("session") == session:
                response_sequence = response.get("sequence")
                if isinstance(response_sequence, int):
                    acknowledged.add(response_sequence)

    try:
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            left_accumulator += left_duty
            right_accumulator += right_duty
            left_on = left_accumulator >= 1.0 - 1e-9
            right_on = right_accumulator >= 1.0 - 1e-9
            if left_on:
                left_accumulator -= 1.0
                left_slots += 1
            if right_on:
                right_accumulator -= 1.0
                right_slots += 1
            left = direction if left_on else 0.0
            right = direction if right_on else 0.0
            sock.sendto(encode_command(session, sequence, left, right), (host, port))
            sequence += 1
            drain()
            sleep_seconds = interval - (time.monotonic() - now)
            if sleep_seconds > 0:
                time.sleep(sleep_seconds)
    finally:
        stop_sequence = sequence
        stop_payload = encode_command(session, stop_sequence, 0.0, 0.0)
        stop_deadline = time.monotonic() + 0.6
        next_stop_send = 0.0
        while time.monotonic() < stop_deadline and stop_sequence not in acknowledged:
            now = time.monotonic()
            if now >= next_stop_send:
                sock.sendto(stop_payload, (host, port))
                next_stop_send = now + 0.05
            drain()
            time.sleep(0.005)
        for _attempt in range(2):
            sock.sendto(stop_payload, (host, port))
            time.sleep(0.01)
        sock.close()

    drive_acknowledgements = sum(item < stop_sequence for item in acknowledged)
    return (
        sequence,
        drive_acknowledgements,
        left_slots,
        right_slots,
        stop_sequence in acknowledged,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=20.0, help="command slot rate in Hz")
    parser.add_argument("--reverse", action="store_true", help="compensate a reverse run")
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument(
        "--left-duty",
        type=float,
        help="override the calibrated left duty fraction (0-1)",
    )
    parser.add_argument(
        "--right-duty",
        type=float,
        help="override the calibrated right duty fraction (0-1)",
    )
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 0.5 <= args.duration <= 10.0:
        raise SystemExit("--duration must be between 0.5 and 10 seconds")
    if not 5.0 <= args.rate <= 50.0:
        raise SystemExit("--rate must be between 5 and 50 Hz")

    try:
        left_mps, right_mps = tread_speeds(args.summary, args.reverse)
    except (OSError, KeyError, ValueError) as exc:
        raise SystemExit(f"could not read calibrated tread speeds: {exc}")

    left_duty, right_duty = duty_from_speeds(left_mps, right_mps)
    if args.left_duty is not None:
        left_duty = args.left_duty
    if args.right_duty is not None:
        right_duty = args.right_duty
    for name, value in (("left", left_duty), ("right", right_duty)):
        if not 0.0 < value <= 1.0:
            raise SystemExit(f"{name} duty must be greater than 0 and at most 1")

    direction = -1.0 if args.reverse else 1.0
    label = "reverse" if args.reverse else "forward"
    effective_left = left_mps * left_duty
    effective_right = right_mps * right_duty
    print(
        f"{label}: calibrated left={left_mps:.4f} m/s right={right_mps:.4f} m/s\n"
        f"duty left={left_duty:.4f} right={right_duty:.4f}\n"
        f"effective left={effective_left:.4f} m/s right={effective_right:.4f} m/s\n"
        f"predicted travel={effective_left * args.duration:.3f} m over {args.duration:.1f}s "
        f"at {args.rate:.0f} Hz"
    )

    if not args.execute:
        print("Refusing to move the robot without --execute")
        return 0

    packets, acknowledgements, left_slots, right_slots, stopped = run_route(
        args.host,
        args.port,
        args.duration,
        args.rate,
        left_duty,
        right_duty,
        direction,
    )
    print(
        f"Sent {packets} packets ({acknowledgements} acknowledged); "
        f"left driven {left_slots}/{packets} slots, right driven {right_slots}/{packets}; "
        f"stop acknowledged={stopped}."
    )
    drive_succeeded = packets == 0 or acknowledgements > 0
    return 0 if drive_succeeded and stopped else 2


if __name__ == "__main__":
    raise SystemExit(main())
