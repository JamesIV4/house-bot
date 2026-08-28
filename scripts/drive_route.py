#!/usr/bin/env python3
"""Drive a whole route from a single process, socket, and session.

Built after a session in which every route was driven by spawning one
`send_motor_command.py` per leg from a shell loop. That approach has four
problems this script removes:

- each leg paid ~0.5 s of interpreter start-up, so the real gaps between legs
  were neither what the shell asked for nor repeatable;
- each leg opened a new socket, so the motor service saw a brand-new client
  address and restarted its sequence tracking every time;
- each leg ended with a stop that waited a variable time for its
  acknowledgement, adding jitter no one could see;
- the shell loop itself was a source of bugs -- an unquoted split silently
  passed "forward 1.0" as a single argument and drove nothing at all.

Here the whole route is one uninterrupted 20 Hz stream on a monotonic clock.
Gaps are commanded zeros rather than silence, so the base is explicitly stopped
instead of relying on the service's 350 ms watchdog, and the service never sees
a gap in its command stream.

    drive_route.py --dry-run forward:3 reverse:3
    drive_route.py --execute --gap 2.5 forward:3 left:0.7 forward:3
"""

from __future__ import annotations

import argparse
import json
import secrets
import socket
import sys
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path

from send_motor_command import MOTIONS, encode_command
from turn_by_imu import DEFAULT_IMU_PORT, ImuClient, ImuError


MAX_SEGMENT_SECONDS = 10.0
MAX_ROUTE_SECONDS = 300.0
STOP_CONFIRM_SECONDS = 0.6


@dataclass(frozen=True)
class Leg:
    """One constant-command interval of the route."""

    label: str
    left: float
    right: float
    start_s: float
    end_s: float

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    @property
    def is_stop(self) -> bool:
        return self.left == 0.0 and self.right == 0.0


@dataclass
class LegResult:
    leg: Leg
    packets: int = 0
    acknowledged: int = 0
    sequences: list[int] = field(default_factory=list)


def parse_segment(text: str) -> tuple[str, float]:
    """Parse "motion:seconds" into a validated segment."""
    motion, _, duration_text = text.partition(":")
    motion = motion.strip()
    if not duration_text:
        raise argparse.ArgumentTypeError(f'expected "motion:seconds", got {text!r}')
    if motion not in MOTIONS:
        raise argparse.ArgumentTypeError(
            f"unknown motion {motion!r}; choose from {', '.join(sorted(MOTIONS))}"
        )
    try:
        duration = float(duration_text)
    except ValueError:
        raise argparse.ArgumentTypeError(f"duration must be a number, got {duration_text!r}")
    if not 0.05 <= duration <= MAX_SEGMENT_SECONDS:
        raise argparse.ArgumentTypeError(
            f"duration must be between 0.05 and {MAX_SEGMENT_SECONDS} s, got {duration}"
        )
    return motion, duration


def build_timeline(
    segments: Sequence[tuple[str, float]],
    gap_s: float,
    lead_in_s: float = 0.0,
    tail_s: float = 0.5,
) -> list[Leg]:
    """Lay the route out on one monotonic timeline, gaps included as stop legs.

    Gaps are explicit zero-command legs rather than silence. Commanding a stop
    is deterministic; falling silent and letting the watchdog expire is not, and
    it leaves the moment of release dependent on packet timing.
    """
    if not segments:
        raise ValueError("a route needs at least one segment")
    if gap_s < 0:
        raise ValueError("gap must not be negative")

    legs: list[Leg] = []
    cursor = 0.0
    if lead_in_s > 0:
        legs.append(Leg("lead-in", 0.0, 0.0, 0.0, lead_in_s))
        cursor = lead_in_s

    for index, (motion, duration) in enumerate(segments):
        left, right = MOTIONS[motion]
        legs.append(Leg(motion, left, right, cursor, cursor + duration))
        cursor += duration
        is_last = index + 1 == len(segments)
        if not is_last and gap_s > 0:
            legs.append(Leg("gap", 0.0, 0.0, cursor, cursor + gap_s))
            cursor += gap_s

    if tail_s > 0:
        legs.append(Leg("tail-stop", 0.0, 0.0, cursor, cursor + tail_s))
        cursor += tail_s

    if cursor > MAX_ROUTE_SECONDS:
        raise ValueError(f"route is {cursor:.1f} s; the limit is {MAX_ROUTE_SECONDS:.0f} s")
    return legs


def leg_at(legs: Sequence[Leg], elapsed_s: float) -> Leg | None:
    """The leg covering this instant, or None once the route is over."""
    for leg in legs:
        if leg.start_s <= elapsed_s < leg.end_s:
            return leg
    return None


def format_timeline(legs: Sequence[Leg]) -> str:
    lines = [f"{'leg':<12} {'start':>7} {'end':>7} {'left':>6} {'right':>6}"]
    for leg in legs:
        lines.append(
            f"{leg.label:<12} {leg.start_s:7.2f} {leg.end_s:7.2f} "
            f"{leg.left:6.1f} {leg.right:6.1f}"
        )
    lines.append(f"total {legs[-1].end_s:.2f} s")
    return "\n".join(lines)


def run_route(
    legs: Sequence[Leg],
    host: str,
    port: int,
    rate_hz: float,
    sock: socket.socket | None = None,
    now_fn=time.monotonic,
    sleep_fn=time.sleep,
    on_tick=None,
) -> list[LegResult]:
    """Stream the whole route from one socket and one session."""
    owns_socket = sock is None
    if sock is None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)

    session = secrets.token_hex(8)
    interval = 1.0 / rate_hz
    results = {id(leg): LegResult(leg) for leg in legs}
    pending: dict[int, LegResult] = {}
    sequence = 0
    tick = 0
    started = now_fn()
    total = legs[-1].end_s

    def drain() -> None:
        while True:
            try:
                payload, _address = sock.recvfrom(4096)
            except BlockingIOError:
                return
            except OSError:
                return
            try:
                reply = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not reply.get("ok") or reply.get("session") != session:
                continue
            result = pending.pop(reply.get("sequence"), None)
            if result is not None:
                result.acknowledged += 1

    try:
        while True:
            elapsed = now_fn() - started
            if elapsed >= total:
                break
            leg = leg_at(legs, elapsed)
            if leg is None:
                break
            result = results[id(leg)]
            sock.sendto(encode_command(session, sequence, leg.left, leg.right), (host, port))
            result.packets += 1
            result.sequences.append(sequence)
            pending[sequence] = result
            sequence += 1
            drain()
            if on_tick is not None:
                on_tick()
            # Schedule against the route clock, not against sleep duration, so
            # per-tick overhead cannot accumulate into drift across a long route.
            #
            # The tick is counted, never derived as `elapsed // interval`: that
            # floor division is wrong under binary floating point (0.25 // 0.05
            # is 4, not 5, because 0.25/0.05 evaluates just under 5). It puts the
            # next target on *now*, so the loop stops sleeping and floods the
            # service with packets as fast as it can build them.
            tick += 1
            remaining = started + tick * interval - now_fn()
            if remaining > 0:
                sleep_fn(remaining)

        # Confirmed stop: keep asking until the service acknowledges zero.
        stop_sequence = sequence
        stop_payload = encode_command(session, stop_sequence, 0.0, 0.0)
        stop_acked = False
        deadline = now_fn() + STOP_CONFIRM_SECONDS
        next_send = 0.0
        while now_fn() < deadline and not stop_acked:
            if now_fn() >= next_send:
                sock.sendto(stop_payload, (host, port))
                next_send = now_fn() + 0.05
            try:
                payload, _address = sock.recvfrom(4096)
                reply = json.loads(payload.decode("utf-8"))
                if reply.get("ok") and reply.get("session") == session \
                        and reply.get("sequence") == stop_sequence:
                    stop_acked = True
            except (BlockingIOError, OSError, UnicodeDecodeError, json.JSONDecodeError):
                pass
            sleep_fn(0.005)
        for _ in range(2):
            sock.sendto(stop_payload, (host, port))
            sleep_fn(0.01)
        if not stop_acked:
            print("WARNING: final stop was not acknowledged", file=sys.stderr)
    finally:
        if owns_socket:
            sock.close()

    return [results[id(leg)] for leg in legs]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "segments",
        nargs="*",
        type=parse_segment,
        metavar="MOTION:SECONDS",
        help='for example "forward:3" "left:0.7"',
    )
    parser.add_argument("--route-file", type=Path, help="JSON list of [motion, seconds]")
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--gap", type=float, default=2.5, help="stop time between segments")
    parser.add_argument("--lead-in", type=float, default=0.0)
    parser.add_argument("--dry-run", action="store_true", help="print the timeline and exit")
    parser.add_argument("--execute", action="store_true", help="required to actually move")
    parser.add_argument("--log", type=Path, help="write a route log for scale alignment")
    parser.add_argument("--imu-log", action="store_true",
                        help="record IMU yaw into the route log; scale alignment needs it")
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    segments = list(args.segments)
    if args.route_file:
        raw = json.loads(args.route_file.read_text())
        segments.extend(parse_segment(f"{m}:{d}") for m, d in raw)
    if not segments:
        parser.error("give at least one MOTION:SECONDS segment, or --route-file")
    if not 5.0 <= args.rate <= 50.0:
        parser.error("--rate must be between 5 and 50 Hz")

    try:
        legs = build_timeline(segments, args.gap, args.lead_in)
    except ValueError as exc:
        parser.error(str(exc))

    print(format_timeline(legs))
    if args.dry_run:
        return 0
    if not args.execute:
        print("\nRefusing to move without --execute", file=sys.stderr)
        return 2

    imu = None
    if args.imu_log:
        imu = ImuClient(args.host, args.imu_port, record_history=True)
        try:
            imu.wait_for_stream()
            print("calibrating gyro bias; keep the base still...")
            imu.calibrate(2.0)
            imu.zero()
        except ImuError as exc:
            print(f"IMU unavailable: {exc}", file=sys.stderr)
            imu.close()
            return 2

    print()
    started_unix = time.time()
    started_monotonic = time.monotonic()
    try:
        results = run_route(
            legs, args.host, args.port, args.rate,
            on_tick=(imu.pump if imu is not None else None),
        )
    finally:
        yaw_log = None
        if imu is not None:
            yaw_log = [
                [round(stamp, 6), round(yaw, 4), round(rate, 3)]
                for stamp, yaw, rate in imu.history
            ]
            imu.close()

    failures = 0
    for result in results:
        leg = result.leg
        rate = (100.0 * result.acknowledged / result.packets) if result.packets else 100.0
        flag = ""
        if result.packets and rate < 95.0:
            flag = "   <-- LOW ACK RATE"
            failures += 1
        print(
            f"{leg.label:<12} {leg.duration_s:5.2f}s  "
            f"{result.packets:3d} packets  {result.acknowledged:3d} acked "
            f"({rate:5.1f}%){flag}"
        )

    if args.log:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        # schema_version 2 is what align_dpvo_scale.py gates on; it is a
        # contract version, not a file revision, so it stays 2 here.
        document = {
            "schema_version": 2,
            "driver": "drive_route.py",
            "actuation_command_model": "binary_full_power",
            "physical_motion_observation": "unconfirmed",
            "result": 2 if failures else 0,
            "host": args.host,
            "port": args.port,
            "rate_hz": args.rate,
            "route_started_unix_s": started_unix,
            "route_started_monotonic_s": started_monotonic,
            "segments": [
                {
                    "motion": r.leg.label,
                    "power": 1.0,
                    "duration_s": round(r.leg.duration_s, 4),
                    "left": r.leg.left,
                    "right": r.leg.right,
                    "command_started_monotonic_s": started_monotonic + r.leg.start_s,
                    "command_ended_monotonic_s": started_monotonic + r.leg.end_s,
                    "packets": r.packets,
                    "acknowledgements": r.acknowledged,
                }
                for r in results
            ],
        }
        if yaw_log is not None:
            document["yaw_log"] = yaw_log
            document["yaw_samples"] = len(yaw_log)
        args.log.write_text(json.dumps(document, indent=2) + "\n")
        print(f"\nroute log -> {args.log}")

    return 2 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
