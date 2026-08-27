#!/usr/bin/env python3
"""Measure how far a short pivot pulse actually turns the base.

A full-power pivot on this base coasts about 33 degrees after the stop, so it
cannot correct anything smaller than that. Two levers make a rotation smaller:
a pulse short enough that the base never reaches full speed, and driving one
tread instead of two. Observed on this base, largest to smallest: pivot, then
one tread forward, then one tread backward.

Whether that works at all is a physical question, not a design choice, and this
is the measurement that answers it. Trials alternate direction so the base
stays near its starting heading instead of walking around the room.

The output is the number `turn_by_imu.py --nudge-pulse` wants.
"""

from __future__ import annotations

import argparse
import statistics
import time
from dataclasses import dataclass, field

from turn_by_imu import (
    DEFAULT_IMU_PORT,
    DEFAULT_MOTOR_PORT,
    ROTATION_MODES,
    ImuClient,
    ImuError,
    MotorStream,
    pulse_rotate,
)


DEFAULT_DURATIONS = (0.05, 0.08, 0.12, 0.16, 0.20, 0.30)
# Below this the base did not respond to the pulse at all.
MOTION_FLOOR_DEG = 0.3
# A pulse this long unambiguously moves a responsive base, so it separates
# "the base cannot turn that little" from "the base is not listening".
WAKE_PULSE_SECONDS = 0.5
WAKE_MIN_DEG = 5.0
# A nudge whose size varies by more than this cannot reliably reduce an error.
MAX_SPREAD_FRACTION = 0.15
MIN_SPREAD_DEG = 1.0


@dataclass
class DurationResult:
    mode: str
    seconds: float
    left_deg: list[float] = field(default_factory=list)
    right_deg: list[float] = field(default_factory=list)

    def all_magnitudes(self) -> list[float]:
        return [abs(value) for value in self.left_deg + self.right_deg]

    def moved(self) -> bool:
        magnitudes = self.all_magnitudes()
        return bool(magnitudes) and statistics.mean(magnitudes) >= MOTION_FLOOR_DEG

    def mean_deg(self) -> float:
        return statistics.mean(self.all_magnitudes())

    def spread_deg(self) -> float:
        magnitudes = self.all_magnitudes()
        return statistics.stdev(magnitudes) if len(magnitudes) > 1 else 0.0

    def repeatable(self) -> bool:
        """A nudge only helps if its size is predictable.

        A pulse averaging 20 deg that ranges 13-27 cannot close a 20 deg
        residual: it leaves 7 deg on either side, at a random sign. Mean alone
        makes such a pulse look usable, so the spread has to gate it too.
        """
        return self.moved() and self.spread_deg() <= max(
            MIN_SPREAD_DEG, self.mean_deg() * MAX_SPREAD_FRACTION
        )

    def summary_row(self) -> str:
        magnitudes = self.all_magnitudes()
        if not magnitudes:
            return f"{self.mode:>14} {self.seconds:6.2f}   no trials"
        left = statistics.mean([abs(v) for v in self.left_deg]) if self.left_deg else float("nan")
        right = statistics.mean([abs(v) for v in self.right_deg]) if self.right_deg else float("nan")
        return (
            f"{self.mode:>14} {self.seconds:6.2f} {self.mean_deg():9.2f} "
            f"{self.spread_deg():8.2f} {left:9.2f} {right:9.2f}   "
            f"{len(magnitudes):3d}  {'yes' if self.repeatable() else 'no'}"
        )


def confirm_responsive(motors: MotorStream, imu: ImuClient, attempts: int = 2) -> float:
    """Prove the base answers at all before reading anything into small pulses.

    Without this, a receiver that has gone to sleep produces a clean sweep of
    zeroes that looks exactly like a base too coarse to nudge, and the sweep
    would conclude stationary correction is impossible when nothing was even
    listening.
    """
    direction = 1.0
    for attempt in range(attempts):
        imu.zero()
        outcome = pulse_rotate(motors, imu, direction, WAKE_PULSE_SECONDS, "pivot")
        print(
            f"  wake check {attempt + 1}: {WAKE_PULSE_SECONDS:.2f}s pulse moved "
            f"{outcome.moved_deg:+.2f} deg",
            flush=True,
        )
        if abs(outcome.moved_deg) >= WAKE_MIN_DEG:
            return outcome.moved_deg
        direction = -direction
        time.sleep(0.5)
    raise ImuError(
        f"a {WAKE_PULSE_SECONDS:.2f}s full-power pulse moved the base less than "
        f"{WAKE_MIN_DEG:.0f} deg on {attempts} attempts. The base is not "
        "responding: receiver asleep or out of range, remote batteries flat, or "
        "motor battery flat. Nothing can be concluded about short pulses until "
        "this passes."
    )


def run_sweep(
    motors: MotorStream,
    imu: ImuClient,
    modes: tuple[str, ...],
    durations: tuple[float, ...],
    repeats: int,
    rest_seconds: float,
) -> list[DurationResult]:
    results = [DurationResult(mode, seconds) for mode in modes for seconds in durations]
    direction = 1.0
    for result in results:
        for repeat in range(repeats):
            imu.zero()
            outcome = pulse_rotate(motors, imu, direction, result.seconds, result.mode)
            if direction > 0:
                result.left_deg.append(outcome.moved_deg)
            else:
                result.right_deg.append(outcome.moved_deg)
            print(
                f"  {result.mode:>14} {result.seconds:.2f}s "
                f"{'left ' if direction > 0 else 'right'} "
                f"trial {repeat + 1}: {outcome.moved_deg:+7.2f} deg",
                flush=True,
            )
            # Alternating keeps the base near its starting heading. The
            # one-tread modes also translate slightly, so it will wander a
            # little over a long sweep even so.
            direction = -direction
            rest_until = time.monotonic() + rest_seconds
            while time.monotonic() < rest_until:
                imu.pump()
                time.sleep(0.005)
    return results


def recommend(results: list[DurationResult], coast_deg: float) -> str:
    """Say whether stationary nudging is worth using, judging spread as well as size."""
    responsive = [result for result in results if result.moved()]
    if not responsive:
        return (
            "No pulse length tested moved the base at all. Check the receiver is "
            "awake and the batteries are charged, then re-run; the wake check "
            "should have caught this."
        )

    fallback = (
        "Stationary correction is not usable on this base. Leave --nudge-pulse "
        "at 0 and correct residual heading while driving instead: "
        "`drive_heading.py forward --turn 90` holds the heading the turn was "
        "aiming for, steering the residual out over the first part of the run."
    )

    repeatable = [result for result in results if result.repeatable()]
    if not repeatable:
        worst = min(responsive, key=lambda result: result.mean_deg())
        return (
            f"Every pulse length that moves the base does so unpredictably. The "
            f"smallest, {worst.seconds:.2f}s, averages {worst.mean_deg():.1f} deg "
            f"with a spread of {worst.spread_deg():.1f} deg, so a nudge aimed at "
            f"its own average would miss by more than the error it was correcting.\n"
            + fallback
        )

    smallest = min(repeatable, key=lambda result: result.mean_deg())
    achievable = smallest.mean_deg() + 2.0 * smallest.spread_deg()
    if achievable >= coast_deg:
        return (
            f"The smallest repeatable pulse is {smallest.seconds:.2f}s "
            f"{smallest.mode}, still turning {smallest.mean_deg():.1f} deg "
            f"(+/-{smallest.spread_deg():.1f}), no better than the "
            f"{coast_deg:.1f} deg full-power coast it is meant to improve on.\n"
            + fallback
        )
    return (
        f"Use --nudge-mode {smallest.mode} --nudge-pulse {smallest.seconds:.2f} "
        f"with turn_by_imu.py. It turns {smallest.mean_deg():.1f} deg "
        f"(+/-{smallest.spread_deg():.1f}) per pulse against a {coast_deg:.1f} deg "
        f"full-power coast, so residuals down to about {achievable:.0f} deg can be "
        "closed while stationary. Anything smaller still needs correcting while "
        "driving."
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--motor-port", type=int, default=DEFAULT_MOTOR_PORT)
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT)
    parser.add_argument(
        "--modes",
        nargs="+",
        choices=sorted(ROTATION_MODES),
        default=sorted(ROTATION_MODES),
        help="rotation modes to test; tread-reverse is expected to move least",
    )
    parser.add_argument(
        "--durations",
        type=float,
        nargs="+",
        default=list(DEFAULT_DURATIONS),
        help="pulse lengths to test, in seconds",
    )
    parser.add_argument("--repeats", type=int, default=2, help="trials per duration")
    parser.add_argument("--rest", type=float, default=1.0, help="pause between trials")
    parser.add_argument("--calibrate-seconds", type=float, default=2.0)
    parser.add_argument(
        "--coast",
        type=float,
        default=33.4,
        help="measured full-power coast, for comparison in the recommendation",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    durations = tuple(sorted(args.durations))
    if not durations or any(not 0.02 <= value <= 1.0 for value in durations):
        raise SystemExit("--durations must each be between 0.02 and 1.0 seconds")
    if not 1 <= args.repeats <= 10:
        raise SystemExit("--repeats must be between 1 and 10")

    imu = ImuClient(args.host, args.imu_port)
    motors = MotorStream(args.host, args.motor_port)
    try:
        imu.wait_for_stream()
        print(f"calibrating gyro bias for {args.calibrate_seconds:.1f}s; keep the base still...")
        imu.calibrate(args.calibrate_seconds)
        print("confirming the base responds before measuring short pulses:")
        confirm_responsive(motors, imu)
        modes = tuple(args.modes)
        total = len(modes) * len(durations) * args.repeats
        print(f"running {total} pulses over {len(modes)} modes, alternating direction:\n")
        results = run_sweep(motors, imu, modes, durations, args.repeats, args.rest)
    except ImuError as exc:
        motors.stop()
        print(f"IMU error: {exc}")
        return 2
    except KeyboardInterrupt:
        motors.stop()
        print("interrupted; motors released")
        return 130
    finally:
        motors.stop()
        motors.close()
        imu.close()

    print()
    print(
        f"{'mode':>14} {'pulse':>6} {'mean deg':>9} {'sd':>8} {'left':>9} "
        f"{'right':>9}   {'n':>3}  repeatable"
    )
    for result in results:
        print(result.summary_row())
    print()
    print(recommend(results, args.coast))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
