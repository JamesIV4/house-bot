#!/usr/bin/env python3
"""Run and timestamp a bounded skid-steer route for visual scale alignment.

Closed-loop throughout, and logging IMU yaw continuously.

Turns used to be timed from a measured duration table. That is the failure this
script already carries a scar from -- hardcoded turn seconds silently produced
225 degree spins once the drivetrain got faster -- and the table went stale
again at the 2026-08-27 rewire. Turns are now driven until the IMU says the
angle has arrived, so no table can go stale.

Straight legs hold heading rather than running open loop. Uncorrected, this
base curves about 4.4 deg/s when driving forward, which would put a systematic
rotation into every leg of a route whose whole purpose is measuring visual
translation scale.

The continuous yaw log is the other reason for the rewrite: it gives
`align_dpvo_scale.py` a dense rotation ground truth to cross-correlate against
the visual trajectory, instead of estimating the time offset from command
timestamps alone.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

from closed_loop_drive import SteeringController
from drive_distance import DEFAULT_SUMMARY, load_calibration
from drive_heading import DEFAULT_GAIN, DEFAULT_INTEGRAL_GAIN, DEFAULT_MIN_DUTY, drive_heading
from turn_by_imu import (
    DEFAULT_COAST_DEG,
    DEFAULT_IMU_PORT,
    ImuClient,
    ImuError,
    MotorStream,
    execute_turn,
    load_calibration as load_turn_calibration,
)


# Legs are (motion, amount, unit). Straight legs use seconds, held at the same
# 3.0 s the tread speeds were fitted at, because the base has a fixed startup
# cost and distance is not proportional to duration; evaluating a leg at the
# duration the speeds were measured at keeps the expected-distance estimate
# honest. Turns are specified in DEGREES and driven closed-loop against the IMU.
# Legs alternate so the base returns near its start.
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
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT)
    parser.add_argument("--pause", type=float, default=1.0)
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN)
    parser.add_argument("--integral-gain", type=float, default=DEFAULT_INTEGRAL_GAIN)
    parser.add_argument("--min-duty", type=float, default=DEFAULT_MIN_DUTY)
    parser.add_argument("--turn-tolerance", type=float, default=2.0)
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
        "schema_version": 3,
        "actuation_command_model": (
            "binary_full_power" if args.power == 1.0 else "experimental_pulse_density"
        ),
        "control_model": "closed_loop_imu",
        "physical_motion_observation": "unconfirmed",
        "host": args.host,
        "port": args.port,
        "route_started_monotonic_s": route_started,
        "route_started_unix_s": time.time(),
        "pause_s": args.pause,
        "segments": [],
        "yaw_log": [],
    }
    result = 0
    imu = ImuClient(args.host, args.imu_port, record_history=True)
    motors = MotorStream(args.host, args.port, 20.0)

    def rest(seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            imu.pump()
            time.sleep(0.005)

    try:
        load_calibration(args.summary)  # fail early if the calibration is missing
        imu.wait_for_stream()
        print("calibrating gyro bias; keep the base still...", flush=True)
        calibrated = imu.calibrate(2.0)
        imu.zero()
        record["gyro_bias_dps"] = calibrated.get("bias")
        record["mount_tilt_deg"] = calibrated.get("tilt")

        turn_calibration = load_turn_calibration()
        controller = SteeringController(
            gain=args.gain,
            min_duty=args.min_duty,
            deadband_rad=math.radians(1.0),
            integral_gain=args.integral_gain,
        )
        record["route_plan"] = [
            {"motion": m, "amount": a, "unit": u} for m, a, u in ROUTE
        ]

        # Heading each straight leg is meant to hold. Turns advance it by their
        # commanded angle, so a turn that lands short is corrected during the
        # following leg rather than being carried through the whole route.
        target_heading = 0.0

        for index, (motion, amount, unit) in enumerate(ROUTE):
            started = time.monotonic()
            print(
                f"segment={index + 1}/{len(ROUTE)} motion={motion} amount={amount}{unit}",
                flush=True,
            )
            entry: dict = {
                "index": index,
                "motion": motion,
                "amount": amount,
                "unit": unit,
                "power": args.power,
                "command_started_monotonic_s": started,
            }

            if unit == "deg":
                signed = float(amount) if motion == "left" else -float(amount)
                key = "coast_deg_left" if signed >= 0 else "coast_deg_right"
                coast = turn_calibration.get(key) or DEFAULT_COAST_DEG
                turn = execute_turn(
                    motors,
                    imu,
                    target_heading + signed,
                    coast,
                    args.turn_tolerance,
                    max_seconds=6.0,
                    max_corrections=1,
                    min_correction_deg=4.0,
                    prime_seconds=0.0,
                    prime_gap_seconds=0.0,
                )
                target_heading += signed
                entry.update(
                    {
                        "target_heading_deg": round(target_heading, 3),
                        "achieved_heading_deg": round(turn.achieved_deg, 3),
                        "heading_error_deg": round(turn.error_deg, 3),
                        "coast_deg": round(turn.coast_deg, 3),
                        "duration_s": round(turn.duration_s, 4),
                        "aborted": turn.aborted,
                    }
                )
                if turn.aborted is not None:
                    print(f"  turn aborted: {turn.aborted}", flush=True)
                    record["segments"].append(entry)
                    result = 3
                    break
            else:
                stats = drive_heading(
                    motors,
                    imu,
                    reverse=motion == "reverse",
                    seconds=float(amount),
                    controller=controller,
                    target_yaw_deg=target_heading,
                    rate_hz=20.0,
                )
                controller.reset()
                entry.update(
                    {
                        "duration_s": round(stats.duration_s, 4),
                        "target_heading_deg": round(stats.target_yaw_deg, 3),
                        "start_heading_deg": round(stats.start_yaw_deg, 3),
                        "end_heading_deg": round(stats.end_yaw_deg, 3),
                        "heading_error_deg": round(stats.final_error_deg, 3),
                        "rms_heading_error_deg": round(stats.rms_error_deg, 3),
                        "slots": stats.slots,
                        "dropped_slots": stats.dropped_slots,
                        "aborted": stats.aborted,
                    }
                )
                if stats.aborted is not None:
                    print(f"  leg aborted: {stats.aborted}", flush=True)
                    record["segments"].append(entry)
                    result = 3
                    break

            entry["client_returned_monotonic_s"] = time.monotonic()
            # Kept for align_dpvo_scale's turn-marker estimator, which is the
            # fallback when a route has no yaw log.
            entry["command_ended_monotonic_s"] = started + float(entry["duration_s"])
            record["segments"].append(entry)
            print(
                f"  heading error {entry['heading_error_deg']:+.2f} deg",
                flush=True,
            )
            if index + 1 < len(ROUTE):
                rest(args.pause)
    except ImuError as exc:
        print(f"IMU error: {exc}")
        result = 2
    except KeyboardInterrupt:
        print("interrupted")
        result = 130
    finally:
        motors.stop()
        motors.close()
        imu.close()
        record["route_ended_monotonic_s"] = time.monotonic()
        record["result"] = result
        # Every packet the client absorbed, including through every segment,
        # on the same monotonic clock as the segment timestamps.
        record["yaw_log"] = [
            [round(stamp, 6), round(yaw, 4), round(rate, 3)] for stamp, yaw, rate in imu.history
        ]
        record["yaw_samples"] = len(record["yaw_log"])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        print(f"Route log: {args.output}", flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
