#!/usr/bin/env python3
"""Drive a calibrated distance or turn angle at full power.

The base is binary full-power/stop (D-020, reconfirmed 2026-08-23), so distance
is controlled by segment duration rather than by command magnitude. Durations
come from the fitted tread speeds in the base calibration summary.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from send_motor_command import MOTIONS, send_motion


DEFAULT_SUMMARY = Path("config/local/base_calibration.summary.json")
# Below roughly a quarter second the base spends the whole segment overcoming
# stiction, so the fitted steady-state speeds do not apply.
MIN_DURATION_S = 0.25


def load_calibration(path: Path) -> dict:
    document = json.loads(path.read_text())
    for key in (
        "left_forward_mps",
        "left_reverse_mps",
        "right_forward_mps",
        "right_reverse_mps",
        "wheel_separation_m",
    ):
        if key not in document:
            raise KeyError(f"{path} is missing {key}; re-run scripts/calibrate_base.py")
    return document


def duration_for(motion: str, amount: float, cal: dict) -> tuple[float, str]:
    """Return (seconds, human description) for a motion and its amount."""
    width = float(cal["wheel_separation_m"])
    if motion in ("forward", "reverse"):
        suffix = motion
        speed = (float(cal[f"left_{suffix}_mps"]) + float(cal[f"right_{suffix}_mps"])) / 2.0
        return amount / speed, f"{amount:.3f} m at {speed:.4f} m/s"
    # A pivot runs one tread forward and the other back, so the rates add.
    if motion == "left":
        rate = (float(cal["right_forward_mps"]) + float(cal["left_reverse_mps"])) / width
    else:
        rate = (float(cal["left_forward_mps"]) + float(cal["right_reverse_mps"])) / width
    degrees_per_s = math.degrees(rate)
    return amount / degrees_per_s, f"{amount:.1f} deg at {degrees_per_s:.1f} deg/s"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion", choices=("forward", "reverse", "left", "right"))
    parser.add_argument(
        "amount",
        type=float,
        help="metres for forward/reverse, degrees for left/right",
    )
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.amount <= 0:
        raise SystemExit("amount must be positive")
    try:
        cal = load_calibration(args.summary)
    except (OSError, KeyError, ValueError) as exc:
        raise SystemExit(f"could not load calibration: {exc}")

    seconds, description = duration_for(args.motion, args.amount, cal)
    print(f"{args.motion}: {description} -> {seconds:.3f} s")
    if seconds < MIN_DURATION_S:
        raise SystemExit(
            f"computed duration {seconds:.3f}s is below the {MIN_DURATION_S}s "
            "stiction floor; the calibrated speeds do not apply that short"
        )
    if not args.execute:
        print("Refusing to move the robot without --execute")
        return 0

    left, right = MOTIONS[args.motion]
    packets, acks, stop_acked, errors = send_motion(
        args.host, args.port, left, right, seconds, args.rate
    )
    print(
        f"Sent {packets} packets, {acks} acknowledged; stop acknowledged={stop_acked}."
    )
    if errors:
        print("Rejected by motor service: " + "; ".join(errors))
    return 0 if (packets == 0 or acks > 0) and stop_acked and not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
