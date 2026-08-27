#!/usr/bin/env python3
"""Run and timestamp a bounded skid-steer route for visual scale alignment."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from drive_distance import DEFAULT_SUMMARY, load_calibration
from drive_primed import TURN_DURATIONS
from send_motor_command import MOTIONS, send_motion


# Every straight leg runs for the same 3.0 s used to fit the tread speeds. The
# base has a fixed startup cost (measured 2026-08-23), so distance is not
# proportional to duration; evaluating a leg at the duration the speeds were
# measured at keeps the expected-distance estimate honest. Legs alternate so the
# base returns near its start.
# Legs are (motion, amount, unit). Straight legs use seconds, held at the same
# 3.0 s the tread speeds were fitted at, because the base has a fixed startup
# cost and distance is not proportional to duration. Turns are specified in
# DEGREES and converted with the calibrated pivot rates: hardcoding turn seconds
# silently produced 225 degree spins after the drivetrain got faster.
ROUTE = (
    ("forward", 3.0, "s"),
    ("reverse", 3.0, "s"),
    ("left", 90.0, "deg"),
    ("forward", 3.0, "s"),
    ("reverse", 3.0, "s"),
    ("left", 90.0, "deg"),
    ("forward", 3.0, "s"),
    ("reverse", 3.0, "s"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--pause", type=float, default=1.0)
    parser.add_argument("--power", type=float, default=1.0)
    parser.add_argument("--experimental-pulse-density", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.execute:
        print("Refusing to move the robot without --execute")
        return 2
    if not 0.5 <= args.pause <= 5.0:
        raise ValueError("--pause must be between 0.5 and 5 seconds")
    if not 0.05 <= args.power <= 1.0:
        raise ValueError("--power must be between 0.05 and 1.0")
    if args.power != 1.0 and not args.experimental_pulse_density:
        raise ValueError("fractional power requires --experimental-pulse-density")

    route_started = time.monotonic()
    record = {
        "schema_version": 2,
        "actuation_command_model": (
            "binary_full_power" if args.power == 1.0 else "experimental_pulse_density"
        ),
        "physical_motion_observation": "unconfirmed",
        "host": args.host,
        "port": args.port,
        "route_started_monotonic_s": route_started,
        "route_started_unix_s": time.time(),
        "pause_s": args.pause,
        "segments": [],
    }
    result = 0
    try:
        load_calibration(args.summary)  # fail early if the calibration is missing
        resolved = []
        for motion, amount, unit in ROUTE:
            if unit == "deg":
                key = (motion, int(amount))
                if key not in TURN_DURATIONS:
                    raise ValueError(
                        f"no calibrated duration for {motion} {amount} deg; "
                        f"calibrated: {sorted(TURN_DURATIONS)}"
                    )
                seconds = TURN_DURATIONS[key]
            else:
                seconds = float(amount)
            resolved.append((motion, seconds, amount, unit))
        record["route_plan"] = [
            {"motion": m, "amount": a, "unit": u, "duration_s": round(d, 4)}
            for m, d, a, u in resolved
        ]
        first_motion = resolved[0][0]
        first_left, first_right = MOTIONS[first_motion]
        print("priming receiver (one packet, then 0.75s)", flush=True)
        send_motion(args.host, args.port, first_left, first_right, 0.05, 20.0)
        time.sleep(0.75)
        record["primed"] = True

        for index, (motion, duration, amount, unit) in enumerate(resolved):
            power = args.power
            left, right = MOTIONS[motion]
            started = time.monotonic()
            print(
                f"segment={index + 1}/{len(ROUTE)} motion={motion} "
                f"amount={amount}{unit} duration={duration:.2f}s power={power:.2f}",
                flush=True,
            )
            packets, acknowledgements, stop_acknowledged, errors = send_motion(
                args.host,
                args.port,
                left * power,
                right * power,
                duration,
                20.0,
            )
            ended = time.monotonic()
            record["segments"].append(
                {
                    "index": index,
                    "motion": motion,
                    "duration_s": duration,
                    "power": power,
                    "command_started_monotonic_s": started,
                    "command_ended_monotonic_s": started + duration,
                    "client_returned_monotonic_s": ended,
                    "packets": packets,
                    "acknowledgements": acknowledgements,
                    "stop_acknowledged": stop_acknowledged,
                    "errors": list(errors),
                }
            )
            if acknowledgements == 0 or not stop_acknowledged or errors:
                result = 3
                break
            if index + 1 < len(ROUTE):
                time.sleep(args.pause)
    finally:
        record["route_ended_monotonic_s"] = time.monotonic()
        record["result"] = result
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        print(f"Route log: {args.output}", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
