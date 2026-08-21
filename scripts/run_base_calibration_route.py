#!/usr/bin/env python3
"""Run and timestamp a bounded skid-steer route for visual scale alignment."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from send_motor_command import MOTIONS, send_motion


ROUTE = (
    ("forward", 3.0),
    ("reverse", 2.2),
    ("left", 1.5),
    ("forward", 3.0),
    ("reverse", 2.2),
    ("right", 1.5),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--pause", type=float, default=1.0)
    parser.add_argument("--power", type=float, default=1.0)
    parser.add_argument("--experimental-pulse-density", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
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
        for index, (motion, duration) in enumerate(ROUTE):
            power = args.power
            left, right = MOTIONS[motion]
            started = time.monotonic()
            print(
                f"segment={index + 1}/{len(ROUTE)} motion={motion} "
                f"duration={duration:.2f}s power={power:.2f}",
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
