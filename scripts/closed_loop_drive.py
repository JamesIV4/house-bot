#!/usr/bin/env python3
"""Continuous closed-loop driving for the binary House Bot base.

The base cannot vary motor power (D-020), so heading is steered by dropping
whole command slots on one tread inside a *single continuous* command stream.
Corrections change what is being streamed; they never stop the base, so there is
no start/stop cycle when a new pose arrives.

Two rates run in one loop:
  - commands go out at --rate Hz and must never pause (350 ms Pi watchdog);
  - pose updates are consumed whenever available, without blocking sending.
"""

from __future__ import annotations

import argparse
import json
import math
import secrets
import socket
import time
from dataclasses import dataclass, field
from typing import Callable, Protocol

from send_motor_command import encode_command


@dataclass
class Pose:
    """Planar pose in metres/radians, with the time it was captured."""

    x: float
    y: float
    yaw: float
    stamp: float


class PoseSource(Protocol):
    def latest(self) -> Pose | None:
        """Return a new pose if one has arrived, else None. Must not block."""


class ReplayPoseSource:
    """Deterministic pose source for verifying the controller without SLAM.

    Integrates the commanded motion and injects a constant heading bias, which
    imitates the base drifting for a reason the controller cannot observe.
    """

    def __init__(self, drift_rad_s: float = 0.20, period_s: float = 1.0) -> None:
        self.drift_rad_s = drift_rad_s
        self.period_s = period_s
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.now = 0.0
        self._last_emit = -period_s

    def step(
        self,
        left_on: bool,
        right_on: bool,
        speed: float,
        width: float,
        dt: float,
    ) -> None:
        """Advance the simulated base by dt seconds of commanded motion."""
        self.now += dt
        v_l = speed if left_on else 0.0
        v_r = speed if right_on else 0.0
        v = (v_l + v_r) / 2.0
        omega = (v_r - v_l) / width + self.drift_rad_s
        self.yaw += omega * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

    def latest(self) -> Pose | None:
        if self.now - self._last_emit < self.period_s:
            return None
        self._last_emit = self.now
        return Pose(self.x, self.y, self.yaw, self.now)


def wrap_angle(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class SteeringController:
    """Map heading error to per-tread slot duties.

    Duty is the fraction of command slots a tread is driven. Both treads run at
    1.0 when on course; correcting drops slots on the tread that is ahead. The
    gain is deliberately soft because the duty/speed curve is compressed by
    tread inertia: half duty yields roughly three quarters speed.

    `integral_gain` is off by default, which leaves the proportional-only
    behaviour this class originally had. Turning it on matters against a
    *systematic* drift: proportional control alone settles wherever its output
    happens to balance the drift, leaving a standing heading error rather than
    removing it. The integral term keeps growing until the error is actually
    nulled. It is clamped so its contribution can never exceed full authority,
    which is what stops it winding up while the base is stalled or held.
    """

    gain: float = 1.2
    min_duty: float = 0.45
    deadband_rad: float = math.radians(2.0)
    integral_gain: float = 0.0
    integral: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def integral_limit(self) -> float:
        if self.integral_gain <= 0.0:
            return 0.0
        return 1.0 / self.integral_gain

    def command(self, heading_error: float, dt: float = 0.0) -> float:
        """Signed steering demand: positive slows the left tread."""
        if dt > 0.0 and self.integral_gain > 0.0:
            limit = self.integral_limit()
            self.integral = max(-limit, min(limit, self.integral + heading_error * dt))
        return self.gain * heading_error + self.integral_gain * self.integral

    def duties(self, heading_error: float, dt: float = 0.0) -> tuple[float, float]:
        demand = self.command(heading_error, dt)
        # With integral_gain at 0 this is exactly `|heading_error| <= deadband`.
        if abs(demand) <= self.deadband_rad * self.gain:
            return 1.0, 1.0
        # error = target - yaw. A positive error means yaw must increase, i.e.
        # turn left, which needs omega = (v_r - v_l)/W > 0: slow the LEFT tread.
        magnitude = min(1.0, abs(demand))
        reduced = 1.0 - magnitude * (1.0 - self.min_duty)
        return (reduced, 1.0) if demand > 0 else (1.0, reduced)


@dataclass
class DriveStats:
    packets: int = 0
    acknowledged: int = 0
    pose_updates: int = 0
    left_slots: int = 0
    right_slots: int = 0
    corrections: int = 0
    heading_errors: list[float] = field(default_factory=list)


def run_closed_loop(
    host: str,
    port: int,
    duration: float,
    rate_hz: float,
    target_yaw: float,
    pose_source: PoseSource,
    controller: SteeringController,
    direction: float = 1.0,
    on_slot: Callable[[bool, bool], None] | None = None,
) -> DriveStats:
    """Stream commands continuously, adjusting steering as poses arrive."""
    interval = 1.0 / rate_hz
    session = secrets.token_hex(8)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    stats = DriveStats()
    acknowledged: set[int] = set()

    left_duty = right_duty = 1.0
    left_accumulator = right_accumulator = 0.0
    sequence = 0
    deadline = time.monotonic() + duration

    def drain() -> None:
        while True:
            try:
                payload, _ = sock.recvfrom(4096)
            except BlockingIOError:
                return
            try:
                response = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if response.get("ok") and response.get("session") == session:
                if isinstance(response.get("sequence"), int):
                    acknowledged.add(response["sequence"])

    try:
        while True:
            now = time.monotonic()
            if now >= deadline:
                break

            # Consume feedback without ever pausing the command stream.
            pose = pose_source.latest()
            if pose is not None:
                stats.pose_updates += 1
                error = wrap_angle(target_yaw - pose.yaw)
                stats.heading_errors.append(error)
                new_left, new_right = controller.duties(error)
                if (new_left, new_right) != (left_duty, right_duty):
                    stats.corrections += 1
                left_duty, right_duty = new_left, new_right

            left_accumulator += left_duty
            right_accumulator += right_duty
            left_on = left_accumulator >= 1.0 - 1e-9
            right_on = right_accumulator >= 1.0 - 1e-9
            if left_on:
                left_accumulator -= 1.0
                stats.left_slots += 1
            if right_on:
                right_accumulator -= 1.0
                stats.right_slots += 1

            sock.sendto(
                encode_command(
                    session,
                    sequence,
                    direction if left_on else 0.0,
                    direction if right_on else 0.0,
                ),
                (host, port),
            )
            sequence += 1
            stats.packets += 1
            if on_slot is not None:
                on_slot(left_on, right_on)
            drain()

            sleep_for = interval - (time.monotonic() - now)
            if sleep_for > 0:
                time.sleep(sleep_for)
    finally:
        stop_payload = encode_command(session, sequence, 0.0, 0.0)
        stop_deadline = time.monotonic() + 0.6
        next_send = 0.0
        while time.monotonic() < stop_deadline and sequence not in acknowledged:
            now = time.monotonic()
            if now >= next_send:
                sock.sendto(stop_payload, (host, port))
                next_send = now + 0.05
            drain()
            time.sleep(0.005)
        for _ in range(2):
            sock.sendto(stop_payload, (host, port))
            time.sleep(0.01)
        sock.close()

    stats.acknowledged = len(acknowledged)
    return stats


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--pose-period", type=float, default=1.0)
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="run the controller against a simulated drifting base; sends no packets",
    )
    parser.add_argument(
        "--drift-deg-s",
        type=float,
        default=12.0,
        help="simulated uncorrected heading drift",
    )
    parser.add_argument("--execute", action="store_true", help="actually move the robot")
    return parser


def simulate(args: argparse.Namespace) -> int:
    """Verify the control law offline: does it null a constant drift?"""
    source = ReplayPoseSource(
        drift_rad_s=math.radians(args.drift_deg_s),
        period_s=args.pose_period,
    )
    controller = SteeringController()
    speed, width = 0.3016, 0.2159
    interval = 1.0 / args.rate
    left_duty = right_duty = 1.0
    la = ra = 0.0
    errors: list[float] = []
    steps = int(args.duration / interval)

    for _ in range(steps):
        pose = source.latest()
        if pose is not None:
            error = wrap_angle(0.0 - pose.yaw)
            errors.append(error)
            left_duty, right_duty = controller.duties(error)
        la += left_duty
        ra += right_duty
        left_on = la >= 1.0 - 1e-9
        right_on = ra >= 1.0 - 1e-9
        if left_on:
            la -= 1.0
        if right_on:
            ra -= 1.0
        source.step(left_on, right_on, speed, width, interval)

    print(f"simulated {args.duration:.1f}s at {args.drift_deg_s:.1f} deg/s drift")
    print(f"pose updates: {len(errors)}")
    if errors:
        print(f"first heading error: {math.degrees(errors[0]):+.2f} deg")
        print(f"final heading error: {math.degrees(errors[-1]):+.2f} deg")
        print(f"final yaw:           {math.degrees(source.yaw):+.2f} deg")
        print(f"uncorrected would be:{args.drift_deg_s * args.duration:+.2f} deg")
    return 0


def main() -> int:
    args = build_parser().parse_args()
    if args.simulate:
        return simulate(args)
    if not args.execute:
        print("Refusing to move the robot without --execute (or use --simulate)")
        return 0

    source = ReplayPoseSource(drift_rad_s=0.0, period_s=args.pose_period)
    stats = run_closed_loop(
        args.host,
        args.port,
        args.duration,
        args.rate,
        target_yaw=0.0,
        pose_source=source,
        controller=SteeringController(),
        on_slot=lambda l, r: source.step(l, r, 0.3016, 0.2159, 1.0 / args.rate),
    )
    print(
        f"packets={stats.packets} acknowledged={stats.acknowledged} "
        f"pose_updates={stats.pose_updates} corrections={stats.corrections} "
        f"left_slots={stats.left_slots} right_slots={stats.right_slots}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
