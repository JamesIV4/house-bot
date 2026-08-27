#!/usr/bin/env python3
"""Wake the receiver with a short prime, then issue the real motion command.

Verified configuration (2026-08-23): a single prime packet followed by a 0.75 s
gap. A 3-packet prime was rejected because it moved the base whenever the
receiver happened to be awake already, adding its own variable rotation.

Calibrated with this configuration (2026-08-23):
    left  90 deg -> 0.53 s      right  90 deg -> 0.49 s
    left 180 deg -> 1.15 s      right 180 deg -> 1.14 s

Left and right agreeing to within 0.01 s is consistent with the symmetric
front-drive rebuild. The larger left/right gap measured before priming was a
wake artefact, not a drivetrain imbalance.

The receiver stays awake about 5 s and has slept by 10 s. Prime only after a
longer idle: priming while it is still awake makes the prime itself move the
base. Use --ensure-idle for reproducible single measurements.

Turn results vary far more than command duration can explain, and short commands
sometimes produce no motion at all. The suspected cause is the receiver ignoring
the first part of a command while it wakes, dropping a variable amount of each
run. A brief prime that is too short to move the base should absorb that wake-up
so the real command starts against an already-listening receiver.
"""

from __future__ import annotations

import argparse
import time

from send_motor_command import MOTIONS, send_motion


# Measured, not derived: turn angle is not proportional to duration, so these
# are the durations that were walked in physically for each angle.
TURN_DURATIONS = {
    ("left", 90): 0.53,
    ("left", 180): 1.15,
    ("right", 90): 0.49,
    ("right", 180): 1.14,
}


def primed_motion(
    host: str,
    port: int,
    motion: str,
    duration: float,
    prime_s: float,
    gap_s: float,
    rate_hz: float,
) -> tuple[int, int, bool, tuple[str, ...]]:
    left, right = MOTIONS[motion]
    if prime_s > 0:
        # Same direction as the real move, so any residual creep adds to the
        # intended motion rather than fighting it.
        prime_packets, _acks, _stop, _errors = send_motion(
            host, port, left, right, prime_s, rate_hz
        )
        print(f"prime: {prime_packets} packet(s) over {prime_s:.2f}s")
        time.sleep(gap_s)
    return send_motion(host, port, left, right, duration, rate_hz)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion", choices=tuple(MOTIONS))
    parser.add_argument(
        "duration",
        type=float,
        nargs="?",
        help="real command duration in seconds; omit when using --degrees",
    )
    parser.add_argument(
        "--degrees",
        type=int,
        choices=sorted({d for _, d in TURN_DURATIONS}),
        help="use the calibrated duration for this turn angle",
    )
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--prime",
        type=float,
        default=0.05,
        help="prime pulse length in seconds; 0.05 is a single packet, which wakes "
             "the receiver without moving the base",
    )
    parser.add_argument(
        "--gap",
        type=float,
        default=0.75,
        help="idle time after the prime, on top of the stop handshake",
    )
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--no-prime", action="store_true", help="control run, prime skipped")
    parser.add_argument(
        "--ensure-idle",
        type=float,
        default=0.0,
        help="wait this long before priming so the receiver is definitely asleep. "
             "It stays awake ~5s and has slept by 10s, so 12 makes primed runs "
             "reproducible: without it a prime issued while still awake moves the "
             "base and contaminates the result",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.degrees is not None:
        key = (args.motion, args.degrees)
        if key not in TURN_DURATIONS:
            raise SystemExit(
                f"no calibrated duration for {args.motion} {args.degrees} degrees; "
                f"calibrated: {sorted(TURN_DURATIONS)}"
            )
        args.duration = TURN_DURATIONS[key]
        print(f"calibrated {args.motion} {args.degrees} deg -> {args.duration:.2f}s")
    if args.duration is None:
        raise SystemExit("give a duration or --degrees")
    if args.duration <= 0:
        raise SystemExit("duration must be positive")
    if args.ensure_idle > 0:
        print(f"waiting {args.ensure_idle:.1f}s for the receiver to sleep...")
        time.sleep(args.ensure_idle)
    prime = 0.0 if args.no_prime else args.prime
    label = "unprimed control" if prime <= 0 else f"prime {prime:.2f}s + {args.gap:.2f}s gap"
    print(f"{args.motion}: {label}, then {args.duration:.2f}s")
    packets, acks, stop_acked, errors = primed_motion(
        args.host, args.port, args.motion, args.duration, prime, args.gap, args.rate
    )
    print(f"real command: {packets} packets, {acks} acknowledged, stop={stop_acked}")
    if errors:
        print("rejected: " + "; ".join(errors))
    return 0 if acks > 0 and stop_acked and not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
