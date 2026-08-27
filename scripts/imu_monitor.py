#!/usr/bin/env python3
"""Watch live yaw from the Pi IMU service, or measure its stationary drift.

Two things worth doing before any motor is involved:

  imu_monitor.py --drift 60    quantifies the residual bias, which is the
                               floor on how accurately any turn can stop;
  imu_monitor.py               shows live yaw so the base can be rotated by
                               hand against a known angle to confirm the sign
                               convention and the scale factor.
"""

from __future__ import annotations

import argparse
import time

import json
from pathlib import Path

from mpu6050 import base_attitude, mounting_offset
from turn_by_imu import DEFAULT_IMU_PORT, ImuClient, ImuError


MOUNT_PATH = Path(__file__).resolve().parent.parent / "config" / "local" / "imu_mount.json"


def record_level(client: ImuClient, seconds: float, path: Path) -> int:
    """Capture gravity with the base on a known-level surface.

    Without this the board's own mounting angle is indistinguishable from the
    base being on a slope, so the IMU cannot report base pitch or roll at all.
    Yaw is unaffected either way.
    """
    print("The base must be on a surface you have confirmed level, and still.")
    reply = client.calibrate(seconds)
    accel = reply.get("level_accel")
    if not accel:
        raise ImuError("calibration returned no accelerometer reference")
    pitch, roll = mounting_offset(accel)
    document = {
        "level_accel": accel,
        "board_pitch_deg": round(pitch, 3),
        "board_roll_deg": round(roll, 3),
        "tilt_deg": reply.get("tilt"),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n")
    print(f"  gravity reference: {[round(a, 4) for a in accel]}")
    print(f"  board mounting offset: pitch {pitch:+.2f} deg, roll {roll:+.2f} deg")
    print(f"  total tilt from vertical: {reply.get('tilt')} deg")
    print(f"  saved -> {path}")
    print("\nBase pitch and roll are now measurable; yaw was never affected by this.")
    return 0


def show_attitude(client: ImuClient, path: Path) -> int:
    try:
        reference = json.loads(path.read_text())["level_accel"]
    except (OSError, json.JSONDecodeError, KeyError):
        print(f"No mounting reference at {path}; run --record-level first.")
        return 2
    reply = client.request({"cmd": "state"})
    accel = reply.get("accel")
    if not accel:
        print("service did not report an accelerometer vector")
        return 2
    pitch, roll = base_attitude(accel, reference)
    print(f"base pitch {pitch:+.2f} deg, roll {roll:+.2f} deg (mounting offset removed)")
    return 0


def live(client: ImuClient, seconds: float, interval: float) -> int:
    print(f"{'time':>7} {'yaw deg':>10} {'rate dps':>10} {'tilt':>7} {'still':>6} {'temp C':>7}")
    started = time.monotonic()
    next_print = started
    while time.monotonic() - started < seconds:
        client.pump()
        now = time.monotonic()
        if now >= next_print:
            next_print += interval
            reading = client.latest
            if reading is None:
                continue
            yaw, rate = client.yaw_now()
            print(
                f"{now - started:7.1f} {yaw:10.2f} {rate:10.2f} "
                f"{reading.tilt_deg:7.1f} {str(reading.stationary):>6}",
                flush=True,
            )
        time.sleep(0.005)
    return 0


def drift(client: ImuClient, seconds: float, calibrate_seconds: float) -> int:
    print(f"calibrating for {calibrate_seconds:.1f}s; do not touch the base...")
    reply = client.calibrate(calibrate_seconds)
    print("  bias dps:  " + ", ".join(f"{axis:+.5f}" for axis in reply.get("bias", [])))
    print("  noise dps: " + ", ".join(f"{axis:+.5f}" for axis in reply.get("noise", [])))
    print("  " + str(reply.get("orientation")))
    client.zero()

    print(f"measuring drift for {seconds:.0f}s; leave the base completely still...")
    started = time.monotonic()
    next_mark = started + 10.0
    peak = 0.0
    while time.monotonic() - started < seconds:
        client.pump()
        try:
            yaw, _rate = client.yaw_now()
        except ImuError as exc:
            print(f"stream lost: {exc}")
            return 2
        peak = max(peak, abs(yaw))
        now = time.monotonic()
        if now >= next_mark:
            next_mark += 10.0
            print(f"  {now - started:5.0f}s   yaw {yaw:+8.3f} deg", flush=True)
        time.sleep(0.005)

    yaw, _rate = client.yaw_now()
    elapsed = time.monotonic() - started
    rate = yaw / elapsed
    print()
    print(f"total drift    {yaw:+8.3f} deg over {elapsed:.0f}s")
    print(f"drift rate     {rate * 3600.0:+8.1f} deg/hour ({rate:+.4f} deg/s)")
    print(f"peak excursion {peak:8.3f} deg")
    print()
    # A pivot on this base lasts about a second, so the drift accumulated in
    # one second is the error floor for any closed-loop turn.
    print(f"error contributed to a 1 s turn: {abs(rate):.3f} deg")
    if abs(rate) < 0.05:
        print("verdict: well inside the 2 deg turn tolerance.")
    elif abs(rate) < 0.5:
        print("verdict: usable; calibrate immediately before each turn, as turn_by_imu does.")
    else:
        print(
            "verdict: too high. Re-run with the base on a solid surface, and check "
            "the module is not being warmed or vibrated by anything nearby."
        )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=DEFAULT_IMU_PORT)
    parser.add_argument("--seconds", type=float, default=60.0, help="live display duration")
    parser.add_argument("--interval", type=float, default=0.25, help="live display period")
    parser.add_argument(
        "--drift",
        type=float,
        metavar="SECONDS",
        help="run a stationary drift measurement instead of the live display",
    )
    parser.add_argument("--calibrate-seconds", type=float, default=2.0)
    parser.add_argument("--zero", action="store_true", help="zero yaw before the live display")
    parser.add_argument(
        "--record-level",
        action="store_true",
        help="capture the board-to-base mounting offset with the base on a "
             "known-level surface, so base pitch and roll become measurable",
    )
    parser.add_argument(
        "--attitude",
        action="store_true",
        help="report base pitch and roll using the recorded mounting offset",
    )
    parser.add_argument("--mount-file", type=Path, default=MOUNT_PATH)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    client = ImuClient(args.host, args.port)
    try:
        client.wait_for_stream()
        if args.record_level:
            return record_level(client, max(args.calibrate_seconds, 2.0), args.mount_file)
        if args.attitude:
            return show_attitude(client, args.mount_file)
        if args.drift is not None:
            if not 5.0 <= args.drift <= 3600.0:
                raise SystemExit("--drift must be between 5 and 3600 seconds")
            return drift(client, args.drift, args.calibrate_seconds)
        if args.zero:
            client.zero()
        return live(client, args.seconds, args.interval)
    except ImuError as exc:
        print(f"IMU error: {exc}")
        return 2
    except KeyboardInterrupt:
        print()
        return 130
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
