#!/usr/bin/env python3
"""Drive arbitrary matrix intersections on the Pi and measure the result by IMU.

Built while diagnosing a base that would not drive straight. The useful property
is that it bypasses everything: no motor service, no networking in the gating
loop, no action names, no `MATRIX_PINS`. It gates whichever row/column
intersections you name and reports what the base actually did, which is what
separates a mapping problem from a timing problem from a hardware problem.

    probe_matrix.py "left fwd=18:17" "right fwd=19:22" "FORWARD=18:17,19:22"

Each probe stops the motor service, runs the gating, restarts the service, and
reports total yaw. A healthy base shows the two single treads roughly equal and
opposite, and the pair near zero.
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

from turn_by_imu import DEFAULT_IMU_PORT, ImuClient, ImuError


DEFAULT_KEY = Path.home() / ".ssh" / "housebot_ed25519"
REMOTE_SCRIPT = "/home/james/remote_gpio_controller.py"
MOTOR_SERVICE = "house-bot-motors.service"


def parse_probe(text: str) -> tuple[str, str]:
    label, _, spec = text.partition("=")
    if not spec:
        raise argparse.ArgumentTypeError(
            f'expected "label=row:column[,row:column]", got {text!r}'
        )
    return label.strip(), spec.strip()


def run_on_pi(host: str, key: Path, spec: str, seconds: float, timeout: float) -> str:
    """Stop the motor service, gate the intersections, restart it."""
    command = (
        f"systemctl --user stop {MOTOR_SERVICE} >/dev/null 2>&1; sleep 0.4; "
        f"python3 {REMOTE_SCRIPT} gate-pairs {shlex.quote(spec)} --duration {seconds}; "
        f"status=$?; systemctl --user start {MOTOR_SERVICE} >/dev/null 2>&1; exit $status"
    )
    result = subprocess.run(
        ["ssh", "-i", str(key), "-o", "BatchMode=yes", f"james@{host}", command],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    return (result.stdout or result.stderr).strip().replace("\n", " | ")


def probe(
    imu: ImuClient,
    host: str,
    key: Path,
    label: str,
    spec: str,
    seconds: float,
    settle: float,
) -> float:
    imu.calibrate(1.5)
    imu.zero()
    stop = threading.Event()

    def pump() -> None:
        # The gating runs over SSH, so nothing else is draining the socket.
        while not stop.is_set():
            imu.pump()
            time.sleep(0.002)

    worker = threading.Thread(target=pump, daemon=True)
    worker.start()
    try:
        output = run_on_pi(host, key, spec, seconds, timeout=seconds + 60.0)
        time.sleep(settle)
    finally:
        stop.set()
        worker.join(timeout=1.0)

    yaw, _rate = imu.yaw_now()
    peak = max((abs(rate) for _stamp, _yaw, rate in imu.history), default=0.0)
    print(f"{label:32s} {spec:20s} yaw {yaw:+8.2f} deg  peak {peak:6.1f} dps   {output}")
    imu.history.clear()
    return yaw


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "probes",
        nargs="+",
        type=parse_probe,
        metavar="LABEL=ROW:COL[,ROW:COL]",
        help='for example "FORWARD=18:17,19:22"',
    )
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--imu-port", type=int, default=DEFAULT_IMU_PORT)
    parser.add_argument("--key", type=Path, default=DEFAULT_KEY)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--settle", type=float, default=0.8)
    parser.add_argument("--rest", type=float, default=1.0, help="pause between probes")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 0.1 <= args.duration <= 2.0:
        raise SystemExit("--duration must be between 0.1 and 2 seconds")
    if not args.key.exists():
        raise SystemExit(f"House Bot SSH key is missing: {args.key}")

    imu = ImuClient(args.host, args.imu_port, record_history=True)
    try:
        imu.wait_for_stream()
        for index, (label, spec) in enumerate(args.probes):
            probe(imu, args.host, args.key, label, spec, args.duration, args.settle)
            if index + 1 < len(args.probes):
                deadline = time.monotonic() + args.rest
                while time.monotonic() < deadline:
                    imu.pump()
                    time.sleep(0.005)
    except ImuError as exc:
        print(f"IMU error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("\ninterrupted", file=sys.stderr)
        return 130
    finally:
        imu.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
