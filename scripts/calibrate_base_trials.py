#!/usr/bin/env python3
"""Run base-calibration trials open-loop, measuring heading with the IMU.

`calibrate_base.py` fits per-tread speeds and an effective skid-steer track
width from four numbers per trial: how far the base travelled, and how much its
heading changed. Distance still needs a tape measure. Heading does not: the IMU
measures it to well under a degree, where the worksheet previously took it by
eye.

**Trials are deliberately open-loop.** The solver derives the per-tread speed
split from the heading change during a straight run -- that asymmetry is the
signal. Running these through `drive_heading.py` would steer out the very thing
being measured. Heading control belongs on the scale route, not here.

Each trial re-calibrates gyro bias while stationary, zeroes yaw, runs one
bounded full-power command, lets the base settle, and records the total heading
change. Results are appended to the worksheet, so a trial can be repeated and
`calibrate_base.py` will average them and flag inconsistency.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

from send_motor_command import MOTIONS, send_motion
from turn_by_imu import (
    DEFAULT_IMU_PORT,
    DEFAULT_MOTOR_PORT,
    ImuClient,
    ImuError,
    settle_and_read,
)


WORKSHEET = Path(__file__).resolve().parent.parent / "config" / "local" / "base_calibration_measurements.json"
EXAMPLE = Path(__file__).resolve().parent.parent / "config" / "base-calibration-measurements.example.json"
STRAIGHT = ("forward", "reverse")
PIVOT = ("left", "right")
SETTLE_SECONDS = 1.0


def load_worksheet(path: Path) -> dict[str, Any]:
    if path.exists():
        return json.loads(path.read_text())
    document = json.loads(EXAMPLE.read_text())
    # A fresh worksheet starts with no trials rather than the example's
    # null-valued placeholders, which would fail validation if left unfilled.
    document["trials"] = {name: [] for name in (*STRAIGHT, *PIVOT)}
    return document


def save_worksheet(document: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")


def fill_distance(document: dict[str, Any], motion: str, distance: float) -> dict[str, Any]:
    """Set the distance on the most recent trial that is still missing one."""
    trials = document.get("trials", {}).get(motion, [])
    for trial in reversed(trials):
        if trial.get("distance_m") is None:
            trial["distance_m"] = distance
            return trial
    raise SystemExit(f"no {motion} trial is awaiting a distance")


def prompt_distance(motion: str) -> float:
    while True:
        raw = input(f"measured {motion} distance in metres (blank to discard trial): ").strip()
        if not raw:
            raise KeyboardInterrupt
        try:
            value = float(raw)
        except ValueError:
            print("  enter a number, for example 0.86")
            continue
        if not 0.0 < value <= 5.0:
            print("  distance must be between 0 and 5 metres")
            continue
        return value


def run_trial(
    imu: ImuClient,
    motion: str,
    duration: float,
    power: float,
    host: str,
    motor_port: int,
    rate_hz: float,
    calibrate_seconds: float,
) -> tuple[float, int, int]:
    """One open-loop trial; returns heading change and packet accounting."""
    print(f"calibrating gyro bias for {calibrate_seconds:.1f}s; keep the base still...")
    reply = imu.calibrate(calibrate_seconds)
    if not reply.get("oriented", True):
        print("  WARNING: " + str(reply.get("orientation")))
    imu.zero()

    left, right = MOTIONS[motion]
    print(f"running {motion} at power {power:.2f} for {duration:.2f}s...")
    packets, acknowledged, stopped, errors = send_motion(
        host, motor_port, left * power, right * power, duration, rate_hz
    )
    if errors:
        raise ImuError("motor service rejected the command: " + "; ".join(errors))
    if not stopped:
        raise ImuError("stop was not acknowledged; not recording this trial")

    heading = settle_and_read(imu, SETTLE_SECONDS)
    return heading, packets, acknowledged


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("motion", choices=(*STRAIGHT, *PIVOT))
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--power", type=float, default=1.0)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--motor-port", type=int, default=DEFAULT_MOTOR_PORT)
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--calibrate-seconds", type=float, default=2.0)
    parser.add_argument("--worksheet", type=Path, default=WORKSHEET)
    parser.add_argument(
        "--distance",
        type=float,
        default=None,
        help="measured distance in metres for a straight trial, instead of prompting",
    )
    parser.add_argument(
        "--fill-distance",
        type=float,
        default=None,
        help="set the distance on the most recent trial awaiting one, and exit",
    )
    parser.add_argument(
        "--rest",
        type=float,
        default=2.0,
        help="pause between repeats, for re-marking the start position",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 0.5 <= args.duration <= 5.0:
        raise SystemExit("--duration must be between 0.5 and 5 seconds")
    if not 1 <= args.repeats <= 10:
        raise SystemExit("--repeats must be between 1 and 10")
    if args.power != 1.0:
        raise SystemExit(
            "calibration trials must run at full power; the base has no verified "
            "proportional control (D-020)"
        )

    document = load_worksheet(args.worksheet)
    document.setdefault("trials", {}).setdefault(args.motion, [])

    if args.fill_distance is not None:
        if args.motion not in STRAIGHT:
            raise SystemExit("--fill-distance applies only to forward and reverse trials")
        if not 0.0 < args.fill_distance <= 5.0:
            raise SystemExit("--fill-distance must be between 0 and 5 metres")
        trial = fill_distance(document, args.motion, args.fill_distance)
        save_worksheet(document, args.worksheet)
        print(
            f"{args.motion}: distance {args.fill_distance:.3f} m recorded against the "
            f"trial with heading change {trial.get('heading_change_deg'):+.2f} deg"
        )
        return 0

    imu = ImuClient(args.host, args.imu_port)
    recorded = 0
    try:
        imu.wait_for_stream()
        for repeat in range(args.repeats):
            print(f"\n--- {args.motion} trial {repeat + 1} of {args.repeats} ---")
            if args.motion in STRAIGHT:
                print("Mark the starting tread midpoint and heading before this runs.")
            heading, packets, acknowledged = run_trial(
                imu,
                args.motion,
                args.duration,
                args.power,
                args.host,
                args.motor_port,
                args.rate,
                args.calibrate_seconds,
            )
            print(f"  {packets} packets, {acknowledged} acknowledged")
            print(f"  IMU heading change: {heading:+.2f} deg")

            trial: dict[str, Any] = {"power": args.power, "duration_s": args.duration}
            if args.motion in STRAIGHT:
                if args.distance is not None:
                    trial["distance_m"] = args.distance
                elif sys.stdin.isatty():
                    try:
                        trial["distance_m"] = prompt_distance(args.motion)
                    except KeyboardInterrupt:
                        print("  discarded")
                        continue
                else:
                    # Driven from a non-interactive shell: record the heading now
                    # (it cannot be recovered later) and take the tape measurement
                    # afterwards.
                    trial["distance_m"] = None
                    print(
                        "  distance pending -- record it with:\n"
                        f"    calibrate_base_trials.py {args.motion} "
                        "--fill-distance <metres>"
                    )
                # Signed: left positive, right negative, matching the IMU and
                # REP-103. No sign is entered by hand any more.
                trial["heading_change_deg"] = round(heading, 2)
            else:
                magnitude = abs(heading)
                if magnitude < 1.0:
                    print("  base barely turned; discarding this trial")
                    continue
                expected = 1.0 if args.motion == "left" else -1.0
                if (heading > 0) != (expected > 0):
                    print(
                        f"  WARNING: a {args.motion} command turned {heading:+.2f} deg, "
                        "the wrong way. Check the motor service inversion flags."
                    )
                # The worksheet wants a positive magnitude for pivots.
                trial["angle_deg"] = round(magnitude, 2)

            document["trials"][args.motion].append(trial)
            save_worksheet(document, args.worksheet)
            recorded += 1
            print(f"  recorded -> {args.worksheet}")

            if repeat + 1 < args.repeats and args.rest > 0:
                print(f"  resting {args.rest:.1f}s; re-mark the start position now")
                deadline = time.monotonic() + args.rest
                while time.monotonic() < deadline:
                    imu.pump()
                    time.sleep(0.005)
    except ImuError as exc:
        print(f"IMU error: {exc}")
        return 2
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130
    finally:
        imu.close()

    trials = document["trials"][args.motion]
    print(f"\n{recorded} trial(s) recorded; {len(trials)} total for {args.motion}.")
    if args.motion in STRAIGHT:
        headings = [t["heading_change_deg"] for t in trials if "heading_change_deg" in t]
        if headings:
            print("  heading changes: " + ", ".join(f"{h:+.2f}" for h in headings))
    else:
        angles = [t["angle_deg"] for t in trials if "angle_deg" in t]
        if angles:
            print("  pivot angles: " + ", ".join(f"{a:.2f}" for a in angles))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
