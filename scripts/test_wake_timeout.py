#!/usr/bin/env python3
"""Measure how long the receiver stays awake between commands.

A command issued soon after another executes in full, while one issued after a
long idle loses its opening to the receiver waking up. This finds the boundary,
which decides when scripts/drive_primed.py needs its prime at all.

Runs a wake command, idles a measured interval, then issues an unprimed test
command. Compare the resulting turn against the primed reference for that
duration: matching it means the receiver was still awake.
"""

from __future__ import annotations

import argparse
import time

from send_motor_command import MOTIONS, send_motion


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--motion", default="left", choices=tuple(MOTIONS))
    parser.add_argument("--idle", type=float, required=True, help="idle seconds before the test")
    parser.add_argument(
        "--wake-duration",
        type=float,
        default=0.05,
        help="length of the initial wake command; 0.05s is a single packet, which "
             "wakes the receiver without moving the base and so cannot contaminate "
             "the measured test turn",
    )
    parser.add_argument(
        "--test-duration",
        type=float,
        default=0.61,
        help="test command; 0.61s is the primed 90 degree reference",
    )
    parser.add_argument("--rate", type=float, default=20.0)
    args = parser.parse_args()
    if not 0.0 <= args.idle <= 600.0:
        raise SystemExit("--idle must be between 0 and 600 seconds")

    left, right = MOTIONS[args.motion]
    print(f"wake: {args.motion} for {args.wake_duration:.2f}s (should not move the base)")
    send_motion(args.host, args.port, left, right, args.wake_duration, args.rate)
    wake_ended = time.monotonic()

    time.sleep(args.idle)

    test_started = time.monotonic()
    measured_idle = test_started - wake_ended
    print(f"measured idle: {measured_idle:.2f}s")
    print(f"test (unprimed): {args.motion} for {args.test_duration:.2f}s")
    packets, acks, stop_acked, errors = send_motion(
        args.host, args.port, left, right, args.test_duration, args.rate
    )
    print(f"test command: {packets} packets, {acks} acknowledged, stop={stop_acked}")
    if errors:
        print("rejected: " + "; ".join(errors))
    print()
    print(f"Compare this turn against the primed 90 degree reference.")
    print(f"Equal  -> receiver still awake after {measured_idle:.1f}s")
    print(f"Short  -> receiver had slept; prime is required at this interval")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
