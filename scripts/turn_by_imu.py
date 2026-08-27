#!/usr/bin/env python3
"""Closed-loop pivot turns using integrated gyro yaw instead of a stopwatch.

Why this replaces timed turns: the calibrated open-loop durations in
`drive_primed.py` are measured, not derived, because turn angle is not
proportional to command duration on this base. Battery state, floor surface,
and the receiver's variable wake-up all move the result. Yaw feedback removes
the need for that table -- the base turns until it has actually turned.

The base has no proportional speed (D-020), so this is a bang-bang controller:
full power until the stop point, then release. Because the base coasts, the
release comes a whole coast angle short of the target, and the coast measured
after each turn is written back to the calibration file so it converges on its
own.

Coast is a constant angle here, not a constant time: stops at 150.4 and
124.8 dps both coasted about 33.5 degrees, so scaling a time-based lead by the
yaw rate mispredicts as the rate varies.

No priming. The wake pulse in `drive_primed.py` existed because the receiver
swallows a variable amount of the first command after it sleeps, which ruined
a turn measured by stopwatch. A closed-loop turn simply keeps driving until the
angle arrives, so a slow wake costs time rather than accuracy. What the wake
does still affect is *when* motion starts, so the controller waits for real
rotation before judging anything about it.

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

# Coast is modelled as a constant ANGLE, not a constant time. Measured on this
# base 2026-08-27: 33.76 deg after a stop at 150.4 dps, and 33.27 deg after a
# stop at 124.8 dps. A 20% difference in rate moved the coast by 1.5%, so a
# time-based lead multiplied by the rate mispredicts as the rate varies.
# Conservative until a base measures its own: undershoot is recoverable,
# overshoot costs a reversal.
DEFAULT_COAST_DEG = 15.0
CONTROL_TICK_S = 0.002
SETTLE_SECONDS = 0.7
SUBSCRIBE_REFRESH_S = 1.0
# Rotation this fast is unambiguously the base moving rather than gyro noise.
MOTION_DETECT_DPS = 8.0
# How long to wait for that motion before calling the base unresponsive. It has
# to clear the receiver's unprimed wake, which swallows a variable and
# unmeasured part of the first command after it sleeps.
DEFAULT_MOTION_TIMEOUT_S = 2.5


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

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_IMU_PORT,
        timeout: float = 2.0,
        record_history: bool = False,
    ) -> None:
        self.address = (host, port)
        self.timeout = timeout
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setblocking(False)
        self.latest: YawReading | None = None
        self.packets = 0
        # Recorded at absorb time so the log stays dense through motion, not
        # only when a caller happens to be idle enough to sample it.
        self.record_history = record_history
        self.history: list[tuple[float, float, float]] = []
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
        if self.record_history:
            self.history.append(
                (self.latest.received_at, self.latest.yaw_deg, self.latest.rate_dps)
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

    def request(
        self,
        document: dict[str, Any],
        timeout: float | None = None,
        retransmit_interval: float = 0.25,
    ) -> dict[str, Any]:
        """Send a command and wait for the reply carrying the same id.

        Retransmits while waiting. This is UDP over Wi-Fi alongside a motor
        command stream, and a single dropped packet used to abort a whole run.
        Every command is safe to repeat: the service treats a duplicate
        calibrate from the same address as the same capture rather than
        restarting it.
        """
        request_id = secrets.token_hex(4)
        # Deliberately not named `payload`: the receive loop below binds that
        # to incoming bytes, and reusing it here re-sent those bytes.
        request_payload = {**document, "id": request_id}
        self._send(request_payload)
        deadline = time.monotonic() + (timeout if timeout is not None else self.timeout)
        next_retransmit = time.monotonic() + retransmit_interval
        while time.monotonic() < deadline:
            if retransmit_interval > 0.0 and time.monotonic() >= next_retransmit:
                next_retransmit = time.monotonic() + retransmit_interval
                self._send(request_payload)
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
            f"for {document.get('cmd')!r} after retransmitting for "
            f"{timeout if timeout is not None else self.timeout:.1f}s"
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


def blend_coast(previous: float | None, measured: float, weight: float = 0.35) -> float:
    """Move the stored coast angle toward one measurement without chasing noise."""
    clamped = max(0.0, min(90.0, measured))
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
    duration_s: float
    packets: int
    correction_floor_deg: float = 0.0
    corrections: int = 0
    nudges: int = 0
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
            f"floor       {self.correction_floor_deg:8.2f} deg, the smallest "
            "correction this base can make",
            f"duration    {self.duration_s:8.2f} s over {self.packets} packets",
        ]
        if self.corrections:
            lines.append(f"corrections {self.corrections:8d}")
        if self.nudges:
            lines.append(f"nudges      {self.nudges:8d}")
        if self.aborted:
            lines.append(f"ABORTED: {self.aborted}")
        return "\n".join(lines)


def drive_until_angle(
    motors: MotorStream,
    imu: ImuClient,
    target_deg: float,
    coast_deg: float,
    max_seconds: float,
    check_direction: bool,
    motion_timeout_s: float = DEFAULT_MOTION_TIMEOUT_S,
) -> tuple[float, float, float, str | None]:
    """Stream a pivot until the projected yaw reaches target_deg.

    Returns the yaw and rate observed as the stop was issued, the elapsed
    drive time, and an abort reason if one applies. The command refresh runs at
    the stream's own rate while the stop condition is evaluated every tick, so
    the release is not quantised to the 50 ms packet interval.

    Direction is judged the moment real rotation first appears rather than at a
    fixed offset from the command. Without a wake pulse the base can sit still
    for a good fraction of a second before the receiver acts, and a fixed check
    would read that silence as a fault.
    """
    # Direction comes from the remaining error, not from the sign of the
    # target. A correction can start anywhere: turning from +109 deg back to a
    # +90 deg target is a RIGHT turn even though the target is positive.
    imu.pump()
    start_yaw, _start_rate = imu.yaw_now()
    direction = 1.0 if target_deg >= start_yaw else -1.0
    left, right = MOTIONS["left" if direction > 0 else "right"]
    started = time.monotonic()
    abort: str | None = None
    moving = False

    while True:
        now = time.monotonic()
        elapsed = now - started
        imu.pump()
        yaw, rate = imu.yaw_now()

        if elapsed >= max_seconds:
            abort = f"timeout after {max_seconds:.1f}s at {yaw:+.1f} deg"
            break

        if not moving:
            if abs(rate) >= MOTION_DETECT_DPS:
                moving = True
                if check_direction and math.copysign(1.0, rate) != direction:
                    abort = (
                        f"base is turning the wrong way ({rate:+.1f} dps for a "
                        f"{'left' if direction > 0 else 'right'} command); check "
                        "--invert-left/--invert-right/--swap-sides on the motor service"
                    )
                    break
            elif elapsed >= motion_timeout_s:
                abort = (
                    f"base never started turning within {motion_timeout_s:.1f}s "
                    f"({rate:+.1f} dps); receiver asleep or out of range, battery "
                    "flat, or motor service not running"
                )
                break

        # Release a whole coast angle short of the target. `yaw` is already
        # extrapolated across transport latency by the client, so this is the
        # only correction the stop needs.
        if direction * yaw >= direction * target_deg - coast_deg:
            break

        motors.refresh(left, right)
        time.sleep(CONTROL_TICK_S)

    imu.pump()
    yaw_at_stop, rate_at_stop = imu.yaw_now()
    return yaw_at_stop, rate_at_stop, time.monotonic() - started, abort


# Three ways to rotate, in decreasing order of how far one pulse moves the
# base. Measured, not assumed -- kinematically one tread forward and one tread
# backward should sweep the same angle, but on this drivetrain they do not.
#
# Measured 2026-08-27 after the duplicate-net wiring fix, two independent
# sweeps of four trials per cell. Only pulses of 0.08 s and longer reproduce:
#
#   pulse   tread-forward      tread-reverse      pivot
#   0.05     8.7 -> 3.0         8.3 -> 7.8        15.7  (all unrepeatable)
#   0.08    11.2 -> 12.0       11.1 -> 11.6       22.7
#   0.12    14.4 -> 13.9       13.9 -> 13.5       28.8
#   0.20    20.8 -> 20.5       20.6 -> 19.9       40.7
#
# 0.05 s is a single command packet, so whether it lands inside a scan window
# is chance; both tread modes swing wildly there across sweeps. A first sweep
# suggested tread-reverse managed 8.3 deg +/-1.0 at 0.05 s, which the repeat
# did not reproduce.
#
# At 0.08 s, the shortest reliable length, the two tread modes are equal within
# noise. tread-forward is the default because its spread stays tighter at
# longer pulses (+/-1.0 and +/-2.0 at 0.12 and 0.20 s, against tread-reverse's
# +/-2.3 and +/-4.3).
#
# The practical floor for a stationary nudge is therefore about 12 deg, so
# residuals down to roughly 14 deg can be closed standing still. A typical
# post-turn residual is 1-6 deg, well below that, so --nudge-pulse stays 0 by
# default and heading is corrected while driving; see drive_heading.py.
#
ROTATION_MODES: dict[str, tuple[str, str]] = {
    "pivot": ("left", "right"),
    "tread-forward": ("right-tread-forward", "left-tread-forward"),
    "tread-reverse": ("left-tread-reverse", "right-tread-reverse"),
}
DEFAULT_NUDGE_MODE = "tread-forward"


def rotation_command(mode: str, direction: float) -> tuple[float, float]:
    """Wheel command that rotates the base the given way in the given mode.

    Left is positive, counter-clockwise seen from above, matching the IMU.
    """
    if mode not in ROTATION_MODES:
        raise ValueError(f"unknown rotation mode {mode!r}; choose from {sorted(ROTATION_MODES)}")
    left_motion, right_motion = ROTATION_MODES[mode]
    return MOTIONS[left_motion if direction > 0 else right_motion]


@dataclass
class NudgeOutcome:
    """One short stationary pulse and what it actually achieved."""

    pulse_seconds: float
    before_deg: float
    after_deg: float
    direction: float
    mode: str = "pivot"

    @property
    def moved_deg(self) -> float:
        return self.after_deg - self.before_deg


def pulse_rotate(
    motors: MotorStream,
    imu: ImuClient,
    direction: float,
    pulse_seconds: float,
    mode: str = "pivot",
    settle_seconds: float = SETTLE_SECONDS,
) -> NudgeOutcome:
    """Rotate briefly in one mode, then settle and measure what it achieved.

    Two levers make a pulse smaller than a full-power turn: a short pulse never
    reaches full speed, so it carries less momentum into the stop and coasts
    less; and driving one tread instead of two sweeps a smaller angle for the
    same command. Together they are the only way this base rotates less than
    its full-power coast, since it has no proportional speed.
    """
    imu.pump()
    before, _rate = imu.yaw_now()
    left, right = rotation_command(mode, direction)

    deadline = time.monotonic() + pulse_seconds
    motors.send_now(left, right)
    while time.monotonic() < deadline:
        # Refresh anyway: a pulse longer than the Pi's 350 ms watchdog would
        # otherwise release the treads mid-pulse.
        motors.refresh(left, right)
        imu.pump()
        time.sleep(CONTROL_TICK_S)
    motors.stop()

    after = settle_and_read(imu, settle_seconds)
    return NudgeOutcome(pulse_seconds, before, after, direction, mode)


def nudge_to_heading(
    motors: MotorStream,
    imu: ImuClient,
    target_deg: float,
    tolerance_deg: float,
    pulse_seconds: float,
    max_pulses: int,
    history: list[str],
    mode: str = DEFAULT_NUDGE_MODE,
) -> float:
    """Close a residual heading error with short pulses while stationary.

    Returns the final heading. Stops early on three conditions, each of which
    means further pulses would make things worse rather than better: the error
    is inside tolerance, a pulse moved the base further from the target than it
    started, or a pulse produced no motion at all.
    """
    imu.pump()
    heading, _rate = imu.yaw_now()

    for attempt in range(max_pulses):
        error = target_deg - heading
        if abs(error) <= tolerance_deg:
            break
        direction = 1.0 if error > 0.0 else -1.0
        outcome = pulse_rotate(motors, imu, direction, pulse_seconds, mode)
        heading = outcome.after_deg
        new_error = target_deg - heading
        history.append(
            f"nudge {attempt + 1}: {pulse_seconds:.2f}s {mode} "
            f"{'left' if direction > 0 else 'right'} pulse moved "
            f"{outcome.moved_deg:+.2f} deg, error {error:+.2f} -> {new_error:+.2f}"
        )
        if abs(outcome.moved_deg) < 0.3:
            history.append(
                f"nudge {attempt + 1}: pulse produced no motion; "
                "the base cannot be corrected this finely while stationary"
            )
            break
        if abs(new_error) > abs(error):
            history.append(
                f"nudge {attempt + 1}: overshot past the target; stopping before "
                "it oscillates"
            )
            break

    return heading


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
    coast_deg: float,
    tolerance_deg: float,
    max_seconds: float,
    max_corrections: int,
    min_correction_deg: float,
    prime_seconds: float,
    prime_gap_seconds: float,
    motion_timeout_s: float = DEFAULT_MOTION_TIMEOUT_S,
    nudge_pulse_seconds: float = 0.0,
    max_nudges: int = 4,
    nudge_mode: str = DEFAULT_NUDGE_MODE,
) -> TurnResult:
    direction = 1.0 if target_deg >= 0.0 else -1.0

    if prime_seconds > 0.0:
        # Off by default: closed-loop turns do not need it. Kept as an escape
        # hatch only. Same direction as the real move, and yaw is already
        # zeroed, so any creep the prime causes is counted rather than silently
        # added to the final heading.
        left, right = MOTIONS["left" if direction > 0 else "right"]
        motors.send_now(left, right)
        time.sleep(prime_seconds)
        motors.stop()
        deadline = time.monotonic() + prime_gap_seconds
        while time.monotonic() < deadline:
            imu.pump()
            time.sleep(0.005)

    yaw_at_stop, rate_at_stop, duration, abort = drive_until_angle(
        motors,
        imu,
        target_deg,
        coast_deg,
        max_seconds,
        check_direction=True,
        motion_timeout_s=motion_timeout_s,
    )
    stopped = motors.stop()
    achieved = settle_and_read(imu)
    coast = achieved - yaw_at_stop

    result = TurnResult(
        target_deg=target_deg,
        achieved_deg=achieved,
        yaw_at_stop_deg=yaw_at_stop,
        rate_at_stop_dps=rate_at_stop,
        coast_deg=coast,
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

    # At full power the base cannot rotate less than it coasts, so the coast
    # measured on this very turn is the real floor on correction size, and it
    # is far larger than any fixed constant would guess.
    coast_floor = abs(coast)
    min_effective = max(min_correction_deg, coast_floor)
    result.correction_floor_deg = min_effective

    for attempt in range(max_corrections):
        error = target_deg - result.achieved_deg
        if abs(error) <= tolerance_deg:
            break
        if abs(error) < min_effective:
            result.history.append(
                f"correction {attempt + 1}: residual {error:+.2f} deg is below the "
                f"{min_effective:.1f} deg floor set by a {coast_floor:.1f} deg coast; "
                "a full-power turn would overshoot further than the error itself"
            )
            if nudge_pulse_seconds > 0.0:
                result.achieved_deg = nudge_to_heading(
                    motors,
                    imu,
                    target_deg,
                    tolerance_deg,
                    nudge_pulse_seconds,
                    max_nudges,
                    result.history,
                    nudge_mode,
                )
                result.nudges = sum(
                    1 for line in result.history if line.startswith("nudge ")
                )
                result.packets = motors.sequence
            break
        before_correction = result.achieved_deg
        # The receiver is awake from the main turn, so corrections skip the
        # prime. Coast dominates a short nudge, so aim at the remaining error
        # with the same predictive stop rather than a fixed pulse.
        correction_target = result.achieved_deg + error
        yaw_at_stop, rate_at_stop, duration, abort = drive_until_angle(
            motors,
            imu,
            correction_target,
            coast_deg,
            max_seconds=min(max_seconds, 2.0),
            check_direction=False,
            motion_timeout_s=motion_timeout_s,
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
        if abs(achieved - before_correction) < 1.0:
            result.history.append(
                f"correction {attempt + 1}: base did not move; abandoning corrections"
            )
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
        "--coast",
        type=float,
        default=None,
        help="release this many degrees before the target; default is the learned value",
    )
    parser.add_argument("--max-seconds", type=float, default=6.0)
    parser.add_argument("--corrections", type=int, default=2)
    parser.add_argument(
        "--min-correction",
        type=float,
        default=4.0,
        help="residuals smaller than this are accepted; the base cannot reliably move less",
    )
    parser.add_argument(
        "--motion-timeout",
        type=float,
        default=DEFAULT_MOTION_TIMEOUT_S,
        help="abort if the base has not started turning within this long",
    )
    parser.add_argument(
        "--prime",
        type=float,
        default=0.0,
        help="receiver wake pulse in seconds; not needed for a closed-loop turn "
             "and off by default, since a slow wake costs time rather than accuracy",
    )
    parser.add_argument("--prime-gap", type=float, default=0.75)
    parser.add_argument(
        "--nudge-pulse",
        type=float,
        default=0.0,
        help="length of the short stationary pulse used to close a residual too "
             "small for a full-power turn; 0 disables. Measure a workable value "
             "with pulse_response.py before enabling this",
    )
    parser.add_argument("--max-nudges", type=int, default=4)
    parser.add_argument(
        "--nudge-mode",
        choices=sorted(ROTATION_MODES),
        default=DEFAULT_NUDGE_MODE,
        help="how a stationary nudge rotates. About 12 deg per 0.08 s pulse on "
             "this base; shorter pulses are not repeatable. Re-measure with "
             "pulse_response.py",
    )
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
    key = "coast_deg_left" if args.degrees >= 0 else "coast_deg_right"
    stored_coast = calibration.get(key)
    coast = args.coast if args.coast is not None else (stored_coast or DEFAULT_COAST_DEG)
    if not 0.0 <= coast <= 90.0:
        raise SystemExit("--coast must be between 0 and 90 degrees")
    if abs(args.degrees) <= coast:
        raise SystemExit(
            f"a {abs(args.degrees):.0f} deg turn is smaller than the base's "
            f"{coast:.1f} deg coast; it cannot be made at full power"
        )

    imu = ImuClient(args.host, args.imu_port)
    motors = MotorStream(args.host, args.motor_port, args.rate)
    try:
        imu.wait_for_stream()
        print(f"IMU stream up; releasing {coast:.1f} deg before target"
              + ("" if stored_coast is None or args.coast is not None else " (learned)"))

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
            coast,
            args.tolerance,
            args.max_seconds,
            args.corrections,
            args.min_correction,
            args.prime,
            args.prime_gap,
            args.motion_timeout,
            args.nudge_pulse,
            args.max_nudges,
            args.nudge_mode,
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

    if not args.no_learn and result.aborted is None and abs(result.coast_deg) > 0.5:
        calibration[key] = round(blend_coast(stored_coast, abs(result.coast_deg)), 2)
        save_calibration(calibration)
        print(f"learned {key} -> {calibration[key]:.2f} deg ({CALIBRATION_PATH})")

    if result.aborted is not None:
        return 2
    # A residual smaller than the base's own coast is not a controller failure;
    # it is the floor imposed by bang-bang control with no proportional speed.
    acceptable = max(args.tolerance, result.correction_floor_deg)
    return 0 if abs(result.error_deg) <= acceptable else 3


if __name__ == "__main__":
    raise SystemExit(main())
