#!/usr/bin/env python3
"""Drive forward or backward on a held heading, steering by IMU yaw.

The base is binary full-power/stop (D-020), so heading cannot be trimmed by
lowering a wheel magnitude. Instead every packet stays a verified full-power
command and the tread that is ahead is dropped for a fraction of command slots,
lowering its average speed. This is command-slot duty cycling at the packet
rate, not the sub-scan-window GPIO pulse density that D-020 rejected.

The control law is `closed_loop_drive.SteeringController`, unchanged. What
changes is the feedback: that path was fed SLAM poses at about 1 Hz, and this
one is fed gyro yaw at 100 Hz, which is why the default gain here is higher.

Two uses:

  drive_heading.py forward --seconds 3
      Hold the heading the base already has. This is the drift fix: without it
      the base curves, and nothing downstream can tell that from a real turn.

  drive_heading.py forward --seconds 3 --target 90
      Hold an absolute heading in the IMU's current frame. Run straight after
      `turn_by_imu.py 90` and the residual the turn could not correct while
      stationary is steered out over the first part of the drive instead.

  drive_heading.py forward --seconds 3 --turn 90
      Both in one command: pivot 90 degrees, then drive holding the heading the
      turn was *aiming* for rather than the one it reached. A turn that lands
      3 degrees short is corrected over the first metre of the drive, which
      costs nothing -- unlike a stationary correction, which this base cannot
      make below its own 33 degree coast.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass, field

from closed_loop_drive import SteeringController
from mpu6050 import wrap_degrees
from turn_by_imu import (
    DEFAULT_COAST_DEG,
    DEFAULT_IMU_PORT,
    DEFAULT_MOTION_TIMEOUT_S,
    DEFAULT_MOTOR_PORT,
    ImuClient,
    ImuError,
    MotorStream,
    execute_turn,
    load_calibration,
)


CONTROL_TICK_S = 0.002
SETTLE_SECONDS = 0.5
# Tuned on the base 2026-08-27 against its measured 4.4 dps forward drift.
# Proportional action of 2.5 settles around 6.6 deg of standing error against
# that drift; 6.0 settles near 2.8 deg and the integral term closes the rest
# inside a second. Measured over a 2 s forward run: 8.74 deg open loop, 4.77
# deg at the old gains, 0.14 deg at these.
DEFAULT_GAIN = 6.0
DEFAULT_DEADBAND_DEG = 1.0
# Authority measured at roughly 27 dps for a full one-tread drop, so nulling a
# 4.4 dps drift needs about a 16% duty reduction. A 0.45 floor leaves little
# headroom above that; 0.3 leaves plenty.
DEFAULT_MIN_DUTY = 0.3
# Proportional action alone balances a systematic drift at a standing error
# rather than removing it. This nulls that error over a second or two.
DEFAULT_INTEGRAL_GAIN = 10.0
# A heading hold that reaches this is not correcting, it is running away, and
# the only useful thing left to do is stop. A real drift is a few degrees per
# second; a base past this in a straight run has a fault, not an error.
MAX_ERROR_DEG = 45.0


def steering_duties(
    controller: SteeringController,
    error_deg: float,
    reverse: bool,
    dt: float = 0.0,
) -> tuple[float, float]:
    """Per-tread duty for one heading error, in the frame the base is driving.

    `SteeringController` assumes forward motion, where slowing the left tread
    increases yaw. Driving backward reverses both tread velocities, so
    `omega = (v_r - v_l)/W` changes sign and the correction must be applied to
    the other tread. Swapping the returned duties is exactly that.
    """
    left, right = controller.duties(math.radians(error_deg), dt)
    return (right, left) if reverse else (left, right)


@dataclass
class DriveStats:
    target_yaw_deg: float
    start_yaw_deg: float
    end_yaw_deg: float = 0.0
    slots: int = 0
    left_slots: int = 0
    right_slots: int = 0
    corrections: int = 0
    duration_s: float = 0.0
    aborted: str | None = None
    errors_deg: list[float] = field(default_factory=list)

    @property
    def final_error_deg(self) -> float:
        return wrap_degrees(self.target_yaw_deg - self.end_yaw_deg)

    @property
    def max_abs_error_deg(self) -> float:
        return max((abs(error) for error in self.errors_deg), default=0.0)

    @property
    def rms_error_deg(self) -> float:
        if not self.errors_deg:
            return 0.0
        return math.sqrt(sum(error * error for error in self.errors_deg) / len(self.errors_deg))

    @property
    def dropped_slots(self) -> int:
        return (2 * self.slots) - self.left_slots - self.right_slots

    def summary(self) -> str:
        lines = [
            f"target heading {self.target_yaw_deg:+8.2f} deg",
            f"start heading  {self.start_yaw_deg:+8.2f} deg",
            f"end heading    {self.end_yaw_deg:+8.2f} deg",
            f"final error    {self.final_error_deg:+8.2f} deg",
            f"peak error     {self.max_abs_error_deg:8.2f} deg",
            f"rms error      {self.rms_error_deg:8.2f} deg",
            f"slots          {self.slots:8d} ({self.left_slots} left, "
            f"{self.right_slots} right, {self.dropped_slots} dropped)",
            f"duration       {self.duration_s:8.2f} s",
        ]
        if self.aborted:
            lines.append(f"ABORTED: {self.aborted}")
        return "\n".join(lines)


def drive_heading(
    motors: MotorStream,
    imu: ImuClient,
    reverse: bool,
    seconds: float,
    controller: SteeringController,
    target_yaw_deg: float | None = None,
    rate_hz: float = 20.0,
    motion_timeout_s: float = DEFAULT_MOTION_TIMEOUT_S,
    settle_seconds: float = SETTLE_SECONDS,
    max_error_deg: float = MAX_ERROR_DEG,
) -> DriveStats:
    """Stream a straight run, correcting heading every command slot."""
    imu.pump()
    start_yaw, _rate = imu.yaw_now()
    target = start_yaw if target_yaw_deg is None else target_yaw_deg
    direction = -1.0 if reverse else 1.0

    stats = DriveStats(target_yaw_deg=target, start_yaw_deg=start_yaw)
    interval = 1.0 / rate_hz
    started = time.monotonic()
    next_slot = started
    left_accumulator = right_accumulator = 0.0
    previous_duties: tuple[float, float] | None = None
    moving = False

    while True:
        now = time.monotonic()
        elapsed = now - started
        if elapsed >= seconds:
            break

        imu.pump()
        yaw, _rate = imu.yaw_now()

        # A straight run holds yaw near zero, so rotation cannot report motion.
        # The service's stillness flag can: it needs a steady accelerometer as
        # well as a quiet gyro, and a driving tread base is never steady.
        if not moving:
            reading = imu.latest
            if reading is not None and not reading.stationary:
                moving = True
            elif elapsed >= motion_timeout_s:
                stats.aborted = (
                    f"base never started moving within {motion_timeout_s:.1f}s; "
                    "receiver asleep or out of range, battery flat, or motor "
                    "service not running"
                )
                break

        if now >= next_slot:
            next_slot = max(now, next_slot + interval)
            error = wrap_degrees(target - yaw)
            stats.errors_deg.append(error)
            if abs(error) > max_error_deg:
                stats.aborted = (
                    f"heading ran away to {error:+.1f} deg, past the "
                    f"{max_error_deg:.0f} deg limit; stopping rather than "
                    "steering harder into it"
                )
                break
            duties = steering_duties(controller, error, reverse, dt=interval)
            if previous_duties is not None and duties != previous_duties:
                stats.corrections += 1
            previous_duties = duties

            left_accumulator += duties[0]
            right_accumulator += duties[1]
            left_on = left_accumulator >= 1.0 - 1e-9
            right_on = right_accumulator >= 1.0 - 1e-9
            if left_on:
                left_accumulator -= 1.0
                stats.left_slots += 1
            if right_on:
                right_accumulator -= 1.0
                stats.right_slots += 1
            stats.slots += 1
            motors.send_now(
                direction if left_on else 0.0,
                direction if right_on else 0.0,
            )

        time.sleep(CONTROL_TICK_S)

    motors.stop()
    stats.duration_s = time.monotonic() - started

    settle_until = time.monotonic() + settle_seconds
    while time.monotonic() < settle_until:
        imu.pump()
        time.sleep(0.005)
    stats.end_yaw_deg, _rate = imu.yaw_now()
    return stats


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("direction", choices=("forward", "reverse"))
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--motor-port", type=int, default=DEFAULT_MOTOR_PORT)
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT)
    parser.add_argument("--rate", type=float, default=20.0, help="command slot rate in Hz")
    parser.add_argument(
        "--turn",
        type=float,
        default=None,
        help="pivot this many degrees first, then drive holding that heading; "
             "positive is left. Residual turn error is steered out while moving",
    )
    parser.add_argument("--turn-tolerance", type=float, default=2.0)
    parser.add_argument("--turn-max-seconds", type=float, default=6.0)
    parser.add_argument(
        "--target",
        type=float,
        default=None,
        help="absolute heading to hold, in the IMU's current frame; "
             "default is whatever heading the base already has",
    )
    parser.add_argument("--gain", type=float, default=DEFAULT_GAIN, help="duty correction per radian")
    parser.add_argument(
        "--integral-gain",
        type=float,
        default=DEFAULT_INTEGRAL_GAIN,
        help="integral action, which removes the standing error a systematic "
             "drift leaves under proportional control; 0 disables",
    )
    parser.add_argument("--min-duty", type=float, default=DEFAULT_MIN_DUTY)
    parser.add_argument("--deadband", type=float, default=DEFAULT_DEADBAND_DEG, help="degrees")
    parser.add_argument("--motion-timeout", type=float, default=DEFAULT_MOTION_TIMEOUT_S)
    parser.add_argument(
        "--max-error",
        type=float,
        default=MAX_ERROR_DEG,
        help="abort if heading error exceeds this; a straight run past it has a "
             "fault rather than a correctable error",
    )
    parser.add_argument(
        "--calibrate-seconds",
        type=float,
        default=0.0,
        help="stationary gyro bias capture before the run; 0 keeps the current "
             "bias and the current yaw frame, which is what a run following a "
             "turn needs",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 0.1 <= args.seconds <= 30.0:
        raise SystemExit("--seconds must be between 0.1 and 30")
    if not 5.0 <= args.rate <= 50.0:
        raise SystemExit("--rate must be between 5 and 50 Hz")
    if not 0.0 <= args.min_duty <= 1.0:
        raise SystemExit("--min-duty must be between 0 and 1")
    if not 0.0 <= args.gain <= 20.0:
        raise SystemExit("--gain must be between 0 and 20")
    if not 0.0 <= args.integral_gain <= 50.0:
        raise SystemExit("--integral-gain must be between 0 and 50")

    controller = SteeringController(
        gain=args.gain,
        min_duty=args.min_duty,
        deadband_rad=math.radians(args.deadband),
        integral_gain=args.integral_gain,
    )
    imu = ImuClient(args.host, args.imu_port)
    motors = MotorStream(args.host, args.motor_port, args.rate)
    target = args.target
    try:
        imu.wait_for_stream()
        if args.turn is not None:
            print(f"calibrating gyro bias for 1.5s; keep the base still...")
            imu.calibrate(1.5)
            imu.zero()
            calibration = load_calibration()
            key = "coast_deg_left" if args.turn >= 0 else "coast_deg_right"
            coast = calibration.get(key) or DEFAULT_COAST_DEG
            print(f"turning {args.turn:+.1f} deg, releasing {coast:.1f} deg early...")
            turn_result = execute_turn(
                motors,
                imu,
                args.turn,
                coast,
                args.turn_tolerance,
                args.turn_max_seconds,
                max_corrections=1,
                min_correction_deg=4.0,
                prime_seconds=0.0,
                prime_gap_seconds=0.0,
                motion_timeout_s=args.motion_timeout,
            )
            for line in turn_result.history:
                print("  " + line)
            print(
                f"  turn landed {turn_result.achieved_deg:+.2f} deg, "
                f"{turn_result.error_deg:+.2f} deg short of {args.turn:+.1f}; "
                "steering the rest out while driving"
            )
            if turn_result.aborted is not None:
                print(f"turn aborted: {turn_result.aborted}")
                return 2
            # Hold the heading the turn was aiming for, not the one it reached.
            target = args.turn
        elif args.calibrate_seconds > 0:
            print(f"calibrating gyro bias for {args.calibrate_seconds:.1f}s; keep the base still...")
            imu.calibrate(args.calibrate_seconds)
            imu.zero()
        stats = drive_heading(
            motors,
            imu,
            reverse=args.direction == "reverse",
            seconds=args.seconds,
            controller=controller,
            target_yaw_deg=target,
            rate_hz=args.rate,
            motion_timeout_s=args.motion_timeout,
            max_error_deg=args.max_error,
        )
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

    print(stats.summary())
    if motors.errors:
        print("rejected by motor service: " + "; ".join(sorted(motors.errors)))
    if stats.aborted is not None:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
