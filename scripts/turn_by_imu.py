#!/usr/bin/env python3
"""Closed-loop pivot turns using integrated gyro yaw instead of a stopwatch.

Why this replaces timed turns: the calibrated open-loop durations in
`drive_primed.py` are measured, not derived, because turn angle is not
proportional to command duration on this base. Battery state, floor surface,
and the receiver's variable wake-up all move the result. Yaw feedback removes
the need for that table -- the base turns until it has actually turned.

The base has no proportional speed (D-020), so this is a bang-bang controller:
full power until the projected stop point, then release. Because the base
coasts, the stop is issued early by `lead_seconds` worth of the current yaw
rate. The coast measured after each turn is written back to the calibration
file, so the lead converges on its own.

Sign convention follows REP-103 and the IMU mounting: positive degrees are a
left turn, counter-clockwise seen from above.
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import socket
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from send_motor_command import MOTIONS, encode_command


DEFAULT_MOTOR_PORT = 8765
DEFAULT_IMU_PORT = 8766
CALIBRATION_PATH = Path(__file__).resolve().parent.parent / "config" / "local" / "imu_turn_calibration.json"

# Conservative until the base measures its own coast: a 180 dps pivot travels
# about 18 degrees in 0.10 s, so starting lower undershoots rather than
# overshooting, and undershoot is correctable while overshoot costs a reversal.
DEFAULT_LEAD_SECONDS = 0.10
CONTROL_TICK_S = 0.002
SETTLE_SECONDS = 0.7
SUBSCRIBE_REFRESH_S = 1.0
DIRECTION_CHECK_AFTER_S = 0.45
DIRECTION_CHECK_MIN_DPS = 8.0


class ImuError(RuntimeError):
    pass


@dataclass
class YawReading:
    yaw_deg: float
    rate_dps: float
    received_at: float
    stationary: bool
    tilt_deg: float

    def extrapolated(self, now: float, max_age_s: float = 0.25) -> float:
        """Advance the sampled angle to `now` using the reported rate.

        Transport latency is small but a turn at 180 dps covers 1.8 degrees in
        10 ms, so steering on the raw sample would bias every stop late.
        """
        age = now - self.received_at
        if age <= 0.0 or age > max_age_s:
            return self.yaw_deg
        return self.yaw_deg + self.rate_dps * age


class ImuClient:
    """Subscriber for `imu_service.py`."""

    def __init__(self, host: str, port: int = DEFAULT_IMU_PORT, timeout: float = 2.0) -> None:
        self.address = (host, port)
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.latest: YawReading | None = None
        self.packets = 0
        self._next_refresh = 0.0

    def close(self) -> None:
        try:
            self._send({"cmd": "unsubscribe"})
        except OSError:
            pass
        self.socket.close()

    def _send(self, document: dict[str, Any]) -> None:
        self.socket.sendto(
            json.dumps(document, separators=(",", ":")).encode("utf-8"), self.address
        )

    def _absorb(self, document: dict[str, Any]) -> None:
        if "yaw" not in document:
            return
        self.packets += 1
        self.latest = YawReading(
            yaw_deg=float(document["yaw"]),
            rate_dps=float(document.get("rate", 0.0)),
            received_at=time.monotonic(),
            stationary=bool(document.get("still", False)),
            tilt_deg=float(document.get("tilt", 0.0)),
        )

    def pump(self) -> None:
        """Drain queued packets without blocking, keeping the subscription alive."""
        now = time.monotonic()
        if now >= self._next_refresh:
            self._next_refresh = now + SUBSCRIBE_REFRESH_S
            self._send({"cmd": "subscribe"})
        while True:
            try:
                payload, _address = self.socket.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError:
                return
            try:
                document = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(document, dict):
                self._absorb(document)

    def request(self, document: dict[str, Any], timeout: float | None = None) -> dict[str, Any]:
        """Send a command and wait for the reply carrying the same id."""
        request_id = secrets.token_hex(4)
        self._send({**document, "id": request_id})
        deadline = time.monotonic() + (timeout if timeout is not None else self.timeout)
        while time.monotonic() < deadline:
            try:
                payload, _address = self.socket.recvfrom(4096)
            except BlockingIOError:
                time.sleep(0.002)
                continue
            try:
                reply = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not isinstance(reply, dict):
                continue
            self._absorb(reply)
            if reply.get("id") == request_id:
                return reply
        raise ImuError(
            f"no reply from the IMU service at {self.address[0]}:{self.address[1]} "
            f"for {document.get('cmd')!r}"
        )

    def calibrate(self, seconds: float) -> dict[str, Any]:
        reply = self.request({"cmd": "calibrate", "seconds": seconds}, timeout=seconds + 2.0)
        if not reply.get("ok"):
            raise ImuError(reply.get("error", "gyro calibration was rejected"))
        return reply

    def zero(self) -> dict[str, Any]:
        return self.request({"cmd": "zero"})

    def wait_for_stream(self, seconds: float = 2.0) -> YawReading:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.pump()
            if self.latest is not None and time.monotonic() - self.latest.received_at < 0.2:
                return self.latest
            time.sleep(0.005)
        raise ImuError("IMU stream did not start; is imu_service.py running on the Pi?")

    def yaw_now(self) -> tuple[float, float]:
        if self.latest is None:
            raise ImuError("no IMU sample received yet")
        now = time.monotonic()
        if now - self.latest.received_at > 0.5:
            raise ImuError(
                f"IMU stream stalled for {now - self.latest.received_at:.2f}s"
            )
        return self.latest.extrapolated(now), self.latest.rate_dps


class MotorStream:
    """Refreshes one wheel command fast enough to hold off the 350 ms watchdog."""

    def __init__(self, host: str, port: int = DEFAULT_MOTOR_PORT, rate_hz: float = 20.0) -> None:
        self.address = (host, port)
        self.interval = 1.0 / rate_hz
        self.session = secrets.token_hex(8)
        self.sequence = 0
        self.acknowledged: set[int] = set()
        self.errors: set[str] = set()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self._next_send = 0.0

    def close(self) -> None:
        self.socket.close()

    def _drain(self) -> None:
        while True:
            try:
                payload, _address = self.socket.recvfrom(4096)
            except (BlockingIOError, OSError):
                return
            try:
                reply = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if reply.get("session") != self.session:
                continue
            if reply.get("ok") and isinstance(reply.get("sequence"), int):
                self.acknowledged.add(reply["sequence"])
            elif isinstance(reply.get("error"), str):
                self.errors.add(reply["error"])

    def send_now(self, left: float, right: float) -> int:
        sequence = self.sequence
        self.socket.sendto(
            encode_command(self.session, sequence, left, right), self.address
        )
        self.sequence += 1
        self._next_send = time.monotonic() + self.interval
        self._drain()
        return sequence

    def refresh(self, left: float, right: float) -> None:
        """Send only when the refresh interval is due; never blocks."""
        if time.monotonic() >= self._next_send:
            self.send_now(left, right)
        else:
            self._drain()

    def stop(self, confirm_seconds: float = 0.6) -> bool:
        """Release the treads and keep resending until the stop is acknowledged."""
        stop_sequence = self.send_now(0.0, 0.0)
        deadline = time.monotonic() + confirm_seconds
        next_retry = time.monotonic() + 0.05
        while time.monotonic() < deadline and stop_sequence not in self.acknowledged:
            if time.monotonic() >= next_retry:
                self.socket.sendto(
                    encode_command(self.session, self.sequence, 0.0, 0.0), self.address
                )
                self.sequence += 1
                next_retry = time.monotonic() + 0.05
            self._drain()
            time.sleep(0.005)
        for _attempt in range(2):
            self.socket.sendto(
                encode_command(self.session, self.sequence, 0.0, 0.0), self.address
            )
            self.sequence += 1
            time.sleep(0.01)
        self._drain()
        return stop_sequence in self.acknowledged


def load_calibration(path: Path = CALIBRATION_PATH) -> dict[str, float]:
    try:
        document = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(document, dict):
        return {}
    return {
        key: float(value)
        for key, value in document.items()
        if isinstance(value, (int, float)) and not isinstance(value, bool)
    }


def save_calibration(values: dict[str, float], path: Path = CALIBRATION_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(values, indent=2, sort_keys=True) + "\n")


def blend_lead(previous: float | None, measured: float, weight: float = 0.35) -> float:
    """Move the stored lead toward one measurement without chasing noise."""
    clamped = max(0.0, min(0.5, measured))
    if previous is None:
        return clamped
    return previous + weight * (clamped - previous)


@dataclass
class TurnResult:
    target_deg: float
    achieved_deg: float
    yaw_at_stop_deg: float
    rate_at_stop_dps: float
    coast_deg: float
    implied_lead_s: float | None
    duration_s: float
    packets: int
    corrections: int = 0
    stopped_cleanly: bool = True
    aborted: str | None = None
    history: list[str] = field(default_factory=list)

    @property
    def error_deg(self) -> float:
        return self.target_deg - self.achieved_deg

    def summary(self) -> str:
        lines = [
            f"target      {self.target_deg:+8.2f} deg",
            f"achieved    {self.achieved_deg:+8.2f} deg",
            f"error       {self.error_deg:+8.2f} deg",
            f"coast       {self.coast_deg:+8.2f} deg after stop "
            f"(at {self.rate_at_stop_dps:+.1f} dps)",
            f"duration    {self.duration_s:8.2f} s over {self.packets} packets",
        ]
        if self.implied_lead_s is not None:
            lines.append(f"implied lead{self.implied_lead_s:8.3f} s")
        if self.corrections:
            lines.append(f"corrections {self.corrections:8d}")
        if self.aborted:
            lines.append(f"ABORTED: {self.aborted}")
        return "\n".join(lines)


def drive_until_angle(
    motors: MotorStream,
    imu: ImuClient,
    target_deg: float,
    lead_seconds: float,
    max_seconds: float,
    check_direction: bool,
) -> tuple[float, float, float, str | None]:
    """Stream a pivot until the projected yaw reaches target_deg.

    Returns the yaw and rate observed as the stop was issued, the elapsed
    drive time, and an abort reason if one applies. The command refresh runs at
    the stream's own rate while the stop condition is evaluated every tick, so
    the release is not quantised to the 50 ms packet interval.
    """
    direction = 1.0 if target_deg >= 0.0 else -1.0
    left, right = MOTIONS["left" if direction > 0 else "right"]
    started = time.monotonic()
    abort: str | None = None

    while True:
        now = time.monotonic()
        elapsed = now - started
        imu.pump()
        yaw, rate = imu.yaw_now()

        if elapsed >= max_seconds:
            abort = f"timeout after {max_seconds:.1f}s at {yaw:+.1f} deg"
            break
        if check_direction and elapsed >= DIRECTION_CHECK_AFTER_S:
            if abs(rate) < DIRECTION_CHECK_MIN_DPS:
                abort = (
                    f"base is not turning ({rate:+.1f} dps after {elapsed:.2f}s); "
                    "receiver asleep, battery flat, or motor service not running"
                )
                break
            if math.copysign(1.0, rate) != direction:
                abort = (
                    f"base is turning the wrong way ({rate:+.1f} dps for a "
                    f"{'left' if direction > 0 else 'right'} command); check "
                    "--invert-left/--swap-sides on the motor service"
                )
                break
            check_direction = False

        projected = yaw + rate * lead_seconds
        if direction * projected >= direction * target_deg:
            break

        motors.refresh(left, right)
        time.sleep(CONTROL_TICK_S)

    imu.pump()
    yaw_at_stop, rate_at_stop = imu.yaw_now()
    return yaw_at_stop, rate_at_stop, time.monotonic() - started, abort


def settle_and_read(imu: ImuClient, seconds: float = SETTLE_SECONDS) -> float:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        imu.pump()
        time.sleep(0.005)
    yaw, _rate = imu.yaw_now()
    return yaw


def execute_turn(
    motors: MotorStream,
    imu: ImuClient,
    target_deg: float,
    lead_seconds: float,
    tolerance_deg: float,
    max_seconds: float,
    max_corrections: int,
    min_correction_deg: float,
    prime_seconds: float,
    prime_gap_seconds: float,
) -> TurnResult:
    direction = 1.0 if target_deg >= 0.0 else -1.0

    if prime_seconds > 0.0:
        # Same direction as the real move, so any creep the prime causes adds
        # to the intended turn. Yaw is already zeroed, so that creep is counted
        # rather than silently added to the final heading.
        left, right = MOTIONS["left" if direction > 0 else "right"]
        motors.send_now(left, right)
        time.sleep(prime_seconds)
        motors.stop()
        deadline = time.monotonic() + prime_gap_seconds
        while time.monotonic() < deadline:
            imu.pump()
            time.sleep(0.005)

    yaw_at_stop, rate_at_stop, duration, abort = drive_until_angle(
        motors, imu, target_deg, lead_seconds, max_seconds, check_direction=True
    )
    stopped = motors.stop()
    achieved = settle_and_read(imu)
    coast = achieved - yaw_at_stop
    implied_lead = coast / rate_at_stop if abs(rate_at_stop) > 5.0 else None

    result = TurnResult(
        target_deg=target_deg,
        achieved_deg=achieved,
        yaw_at_stop_deg=yaw_at_stop,
        rate_at_stop_dps=rate_at_stop,
        coast_deg=coast,
        implied_lead_s=implied_lead,
        duration_s=duration,
        packets=motors.sequence,
        stopped_cleanly=stopped,
        aborted=abort,
    )
    result.history.append(
        f"main: stopped at {yaw_at_stop:+.2f} deg ({rate_at_stop:+.1f} dps), "
        f"coasted {coast:+.2f} to {achieved:+.2f}"
    )
    if abort is not None:
        return result

    for attempt in range(max_corrections):
        error = target_deg - result.achieved_deg
        if abs(error) <= tolerance_deg:
            break
        if abs(error) < min_correction_deg:
            result.history.append(
                f"correction {attempt + 1}: residual {error:+.2f} deg is below the "
                f"{min_correction_deg:.1f} deg the base can reliably move; accepted"
            )
            break
        # The receiver is awake from the main turn, so corrections skip the
        # prime. Coast dominates a short nudge, so aim at the remaining error
        # with the same predictive stop rather than a fixed pulse.
        correction_target = result.achieved_deg + error
        yaw_at_stop, rate_at_stop, duration, abort = drive_until_angle(
            motors,
            imu,
            correction_target,
            lead_seconds,
            max_seconds=min(max_seconds, 2.0),
            check_direction=False,
        )
        motors.stop()
        achieved = settle_and_read(imu, 0.5)
        result.history.append(
            f"correction {attempt + 1}: {error:+.2f} deg wanted, stopped at "
            f"{yaw_at_stop:+.2f}, settled {achieved:+.2f}"
        )
        result.achieved_deg = achieved
        result.corrections += 1
        result.duration_s += duration
        result.packets = motors.sequence
        if abort is not None:
            result.aborted = abort
            break

    return result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "degrees",
        type=float,
        help="signed turn: positive is left/counter-clockwise, negative is right",
    )
    parser.add_argument("--host", default="192.168.0.241", help="Pi address for both services")
    parser.add_argument("--motor-port", type=int, default=DEFAULT_MOTOR_PORT)
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT)
    parser.add_argument("--rate", type=float, default=20.0, help="command refresh rate in Hz")
    parser.add_argument("--tolerance", type=float, default=2.0, help="accepted error in degrees")
    parser.add_argument(
        "--lead",
        type=float,
        default=None,
        help="stop this many seconds of yaw rate early; default is the learned value",
    )
    parser.add_argument("--max-seconds", type=float, default=6.0)
    parser.add_argument("--corrections", type=int, default=2)
    parser.add_argument(
        "--min-correction",
        type=float,
        default=4.0,
        help="residuals smaller than this are accepted; the base cannot reliably move less",
    )
    parser.add_argument("--prime", type=float, default=0.05, help="receiver wake pulse in seconds")
    parser.add_argument("--prime-gap", type=float, default=0.75)
    parser.add_argument("--no-prime", action="store_true")
    parser.add_argument(
        "--calibrate-seconds",
        type=float,
        default=1.5,
        help="stationary gyro bias capture taken immediately before the turn",
    )
    parser.add_argument(
        "--no-learn",
        action="store_true",
        help="do not write the measured coast back to the calibration file",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 1.0 <= abs(args.degrees) <= 360.0:
        raise SystemExit("degrees must be between 1 and 360 in magnitude")
    if not 5.0 <= args.rate <= 50.0:
        raise SystemExit("--rate must be between 5 and 50 Hz")
    if not 0.5 <= args.tolerance <= 20.0:
        raise SystemExit("--tolerance must be between 0.5 and 20 degrees")
    if not 0 <= args.corrections <= 5:
        raise SystemExit("--corrections must be between 0 and 5")

    calibration = load_calibration()
    key = "lead_seconds_left" if args.degrees >= 0 else "lead_seconds_right"
    stored_lead = calibration.get(key)
    lead = args.lead if args.lead is not None else (stored_lead or DEFAULT_LEAD_SECONDS)
    if not 0.0 <= lead <= 0.5:
        raise SystemExit("--lead must be between 0 and 0.5 seconds")

    imu = ImuClient(args.host, args.imu_port)
    motors = MotorStream(args.host, args.motor_port, args.rate)
    try:
        imu.wait_for_stream()
        print(f"IMU stream up; stopping lead {lead:.3f}s"
              + ("" if stored_lead is None or args.lead is not None else " (learned)"))

        print(f"calibrating gyro bias for {args.calibrate_seconds:.1f}s; keep the base still...")
        calibrated = imu.calibrate(args.calibrate_seconds)
        print(
            "  bias dps: "
            + ", ".join(f"{axis:+.4f}" for axis in calibrated.get("bias", []))
            + f"   tilt {calibrated.get('tilt')} deg"
        )
        if not calibrated.get("oriented", True):
            print("  WARNING: " + str(calibrated.get("orientation")))

        imu.zero()
        result = execute_turn(
            motors,
            imu,
            args.degrees,
            lead,
            args.tolerance,
            args.max_seconds,
            args.corrections,
            args.min_correction,
            0.0 if args.no_prime else args.prime,
            args.prime_gap,
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

    print()
    for line in result.history:
        print("  " + line)
    print()
    print(result.summary())
    if motors.errors:
        print("rejected by motor service: " + "; ".join(sorted(motors.errors)))

    if not args.no_learn and result.aborted is None and result.implied_lead_s is not None:
        calibration[key] = round(blend_lead(stored_lead, result.implied_lead_s), 4)
        save_calibration(calibration)
        print(f"learned {key} -> {calibration[key]:.4f}s ({CALIBRATION_PATH})")

    if result.aborted is not None:
        return 2
    return 0 if abs(result.error_deg) <= max(args.tolerance, args.min_correction) else 3


if __name__ == "__main__":
    raise SystemExit(main())
