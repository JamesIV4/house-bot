#!/usr/bin/env python3
"""Pi-side MPU-6050 sampling service: integrated yaw over UDP.

Runs as its own process, deliberately not inside `pi_motor_service.py`. That
service busy-polls the remote's matrix rows looking for a 215 us low window
every 40 ms; a 14-byte I2C burst takes longer than the window itself, so
sharing a thread with it would drop scan windows and quietly weaken every
motor command. On the Pi 3B's four cores these two loops run side by side.

Clients subscribe and receive pushed state at --publish-rate. Every packet
carries the yaw rate as well as the angle, so a controller can extrapolate
across transport latency instead of steering on a stale angle.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import signal
import socket
import time
from dataclasses import dataclass
from typing import Any

from mpu6050 import (
    ADDRESS_HIGH,
    ADDRESS_LOW,
    SAMPLE_RATE_HZ,
    BiasEstimate,
    Mpu6050,
    Sample,
    YawEstimator,
    describe_orientation,
    estimate_bias,
    open_bus,
)


DEFAULT_PORT = 8766
DEFAULT_PUBLISH_RATE_HZ = 100.0
SUBSCRIPTION_TTL_S = 2.0
MAX_CALIBRATION_S = 5.0
# A stationary MPU-6050 at 42 Hz bandwidth sits near 0.05 dps of noise. Ten
# times that means something was moving during the capture.
CALIBRATION_NOISE_LIMIT_DPS = 0.5


class SimulatedDevice:
    """Deterministic stand-in so the controller can be exercised off the Pi."""

    def __init__(self, bias_dps: float = 0.8, noise_dps: float = 0.05) -> None:
        self.bias_dps = bias_dps
        self.noise_dps = noise_dps
        self.rate_dps = 0.0
        self._random = random.Random(20260827)

    def read(self) -> Sample:
        jitter = self._random.gauss(0.0, self.noise_dps)
        return Sample(
            accel=(0.0, 0.0, 1.0),
            gyro=(0.0, 0.0, self.rate_dps + self.bias_dps + jitter),
            temperature_c=32.0,
            stamp=time.monotonic(),
        )


@dataclass
class PendingCalibration:
    address: tuple[str, int]
    deadline: float
    samples: list[Sample]
    request_id: Any


class ImuService:
    def __init__(
        self,
        device: Any,
        estimator: YawEstimator,
        publish_rate_hz: float = DEFAULT_PUBLISH_RATE_HZ,
    ) -> None:
        self.device = device
        self.estimator = estimator
        self.publish_interval = 1.0 / publish_rate_hz
        self.sequence = 0
        self.read_errors = 0
        self.last_error: str | None = None
        self.subscribers: dict[tuple[str, int], float] = {}
        self.pending: PendingCalibration | None = None
        self.latest: Sample | None = None
        self._rate_window: list[float] = []
        self.measured_hz = 0.0

    def state_document(self) -> dict[str, Any]:
        estimator = self.estimator
        return {
            "t": "imu",
            "seq": self.sequence,
            "stamp": time.monotonic(),
            "yaw": round(estimator.yaw_deg, 4),
            "rate": round(estimator.yaw_rate_dps, 4),
            "bias": [round(axis, 5) for axis in estimator.gyro_bias],
            "tilt": round(estimator.tilt_degrees(), 2),
            "still": estimator.stationary,
            "accel": (
                [round(axis, 5) for axis in self.estimator.gravity]
                if self.estimator.gravity is not None
                else None
            ),
            "temp": round(self.latest.temperature_c, 2) if self.latest else None,
            "hz": round(self.measured_hz, 1),
            "samples": estimator.samples,
            "errors": self.read_errors,
        }

    def _track_rate(self, stamp: float) -> None:
        self._rate_window.append(stamp)
        if len(self._rate_window) > 64:
            self._rate_window.pop(0)
        span = self._rate_window[-1] - self._rate_window[0]
        if span > 0:
            self.measured_hz = (len(self._rate_window) - 1) / span

    def sample_once(self) -> Sample | None:
        try:
            sample = self.device.read()
        except OSError as exc:
            self.read_errors += 1
            self.last_error = str(exc)
            return None
        self.latest = sample
        self.estimator.update(sample)
        self._track_rate(sample.stamp)
        if self.pending is not None:
            self.pending.samples.append(sample)
        return sample

    def handle_command(self, document: dict[str, Any], address: tuple[str, int]) -> dict[str, Any] | None:
        command = document.get("cmd")
        request_id = document.get("id")
        if command == "state":
            return {"ok": True, "id": request_id, **self.state_document()}
        if command == "subscribe":
            self.subscribers[address] = time.monotonic() + SUBSCRIPTION_TTL_S
            return {"ok": True, "id": request_id, "subscribed": True, **self.state_document()}
        if command == "unsubscribe":
            self.subscribers.pop(address, None)
            return {"ok": True, "id": request_id, "subscribed": False}
        if command == "zero":
            self.estimator.reset_yaw(float(document.get("yaw", 0.0)))
            return {"ok": True, "id": request_id, **self.state_document()}
        if command == "calibrate":
            seconds = float(document.get("seconds", 1.5))
            if not 0.2 <= seconds <= MAX_CALIBRATION_S:
                return {
                    "ok": False,
                    "id": request_id,
                    "error": f"seconds must be between 0.2 and {MAX_CALIBRATION_S}",
                }
            if self.pending is not None and self.pending.address == address:
                # A retransmitted request, not a new one. Keep the capture and
                # its deadline; only the id to reply to changes. Restarting
                # here would mean a client that retransmits never finishes.
                self.pending.request_id = request_id
                return None
            self.pending = PendingCalibration(
                address=address,
                deadline=time.monotonic() + seconds,
                samples=[],
                request_id=request_id,
            )
            return None
        return {"ok": False, "id": request_id, "error": f"unknown command: {command!r}"}

    def finish_calibration(self) -> tuple[tuple[str, int], dict[str, Any]] | None:
        pending = self.pending
        if pending is None or time.monotonic() < pending.deadline:
            return None
        self.pending = None
        try:
            estimate = estimate_bias(pending.samples)
        except ValueError as exc:
            return pending.address, {"ok": False, "id": pending.request_id, "error": str(exc)}
        oriented, orientation = describe_orientation(estimate.accel)
        accepted = estimate.worst_noise_dps <= CALIBRATION_NOISE_LIMIT_DPS
        if accepted:
            self.estimator.set_bias(estimate.bias)
            self.estimator.gravity = estimate.accel
            self.estimator.reset_yaw(0.0)
        document: dict[str, Any] = {
            "ok": accepted,
            "id": pending.request_id,
            "bias": [round(axis, 5) for axis in estimate.bias],
            "noise": [round(axis, 5) for axis in estimate.noise_dps],
            "level_accel": [round(axis, 5) for axis in estimate.accel],
            "calibration_samples": estimate.samples,
            "oriented": oriented,
            "orientation": orientation,
            **self.state_document(),
        }
        if not accepted:
            document["error"] = (
                f"gyro noise {estimate.worst_noise_dps:.3f} dps exceeds "
                f"{CALIBRATION_NOISE_LIMIT_DPS} dps; the base was not still"
            )
        return pending.address, document


def encode(document: dict[str, Any]) -> bytes:
    return json.dumps(document, separators=(",", ":")).encode("utf-8")


def run_service(
    bind: str,
    port: int,
    publish_rate_hz: float,
    address_7bit: int,
    bus_number: int,
    simulate: bool,
    calibrate_seconds: float,
) -> int:
    if simulate:
        device: Any = SimulatedDevice()
        print("IMU service running with a SIMULATED device; no I2C is touched.", flush=True)
    else:
        bus = open_bus(bus_number)
        device = Mpu6050(bus, address_7bit)
        device.check_identity()
        device.configure()
        print(
            f"MPU-6050 confirmed at 0x{address_7bit:02X} on i2c-{bus_number}; "
            f"+/-500 dps, 42 Hz bandwidth, {SAMPLE_RATE_HZ:.0f} Hz internal rate.",
            flush=True,
        )

    estimator = YawEstimator()
    service = ImuService(device, estimator, publish_rate_hz)

    print(f"Calibrating gyro bias for {calibrate_seconds:.1f}s; keep the base still.", flush=True)
    warmup: list[Sample] = []
    warmup_deadline = time.monotonic() + calibrate_seconds
    while time.monotonic() < warmup_deadline:
        try:
            warmup.append(device.read())
        except OSError as exc:
            print(f"I2C read failed during calibration: {exc}", flush=True)
        time.sleep(1.0 / SAMPLE_RATE_HZ)
    if len(warmup) < 2:
        raise SystemExit("no usable samples during startup calibration; check the I2C wiring")
    startup = estimate_bias(warmup)
    estimator.set_bias(startup.bias)
    estimator.gravity = startup.accel
    oriented, orientation = describe_orientation(startup.accel)
    print(
        "gyro bias dps: "
        + ", ".join(f"{axis:+.4f}" for axis in startup.bias)
        + f" (noise {startup.worst_noise_dps:.4f} dps over {startup.samples} samples)",
        flush=True,
    )
    print(("  " if oriented else "WARNING: ") + orientation, flush=True)
    if startup.worst_noise_dps > CALIBRATION_NOISE_LIMIT_DPS:
        print(
            "WARNING: the base was not still during startup calibration. "
            "Send a calibrate command before relying on yaw.",
            flush=True,
        )

    server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server.bind((bind, port))
    server.setblocking(False)
    running = True

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(f"House Bot IMU service listening on {bind}:{port}.", flush=True)

    sample_interval = 1.0 / SAMPLE_RATE_HZ
    next_sample = time.monotonic()
    next_publish = time.monotonic()

    try:
        while running:
            now = time.monotonic()
            if now >= next_sample:
                next_sample = max(now, next_sample + sample_interval)
                service.sample_once()

            while True:
                try:
                    payload, address = server.recvfrom(4096)
                except BlockingIOError:
                    break
                try:
                    document = json.loads(payload.decode("utf-8"))
                    if not isinstance(document, dict):
                        raise ValueError("packet must be a JSON object")
                    reply = service.handle_command(document, address)
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError, TypeError) as exc:
                    reply = {"ok": False, "error": str(exc)}
                if reply is not None:
                    server.sendto(encode(reply), address)

            finished = service.finish_calibration()
            if finished is not None:
                calibration_address, document = finished
                server.sendto(encode(document), calibration_address)
                print(
                    "calibrate: "
                    + ("accepted" if document.get("ok") else f"rejected ({document.get('error')})")
                    + " bias="
                    + ", ".join(f"{axis:+.4f}" for axis in document.get("bias", [])),
                    flush=True,
                )

            now = time.monotonic()
            if now >= next_publish and service.subscribers:
                next_publish = max(now, next_publish + service.publish_interval)
                service.sequence += 1
                packet = encode(service.state_document())
                for subscriber, expiry in list(service.subscribers.items()):
                    if expiry < now:
                        del service.subscribers[subscriber]
                        continue
                    server.sendto(packet, subscriber)
            elif now >= next_publish:
                next_publish = now + service.publish_interval

            remaining = min(next_sample, next_publish) - time.monotonic()
            if remaining > 0.0005:
                time.sleep(min(remaining, 0.002))
    finally:
        server.close()
        print(
            f"House Bot IMU service stopped after {service.estimator.samples} samples, "
            f"{service.read_errors} I2C errors.",
            flush=True,
        )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Serve MPU-6050 yaw over UDP")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--publish-rate", type=float, default=DEFAULT_PUBLISH_RATE_HZ)
    parser.add_argument(
        "--address",
        type=lambda value: int(value, 0),
        default=ADDRESS_LOW,
        help="I2C address; 0x68 with AD0 low (default) or 0x69 with AD0 high",
    )
    parser.add_argument("--bus", type=int, default=1)
    parser.add_argument("--calibrate-seconds", type=float, default=2.0)
    parser.add_argument(
        "--simulate",
        action="store_true",
        help="run without hardware so clients can be tested off the Pi",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if not 1 <= args.port <= 65535:
        raise SystemExit("--port must be between 1 and 65535")
    if not 10.0 <= args.publish_rate <= SAMPLE_RATE_HZ:
        raise SystemExit(f"--publish-rate must be between 10 and {SAMPLE_RATE_HZ:.0f} Hz")
    if args.address not in (ADDRESS_LOW, ADDRESS_HIGH):
        raise SystemExit("--address must be 0x68 or 0x69")
    if not 0.5 <= args.calibrate_seconds <= MAX_CALIBRATION_S:
        raise SystemExit(f"--calibrate-seconds must be between 0.5 and {MAX_CALIBRATION_S}")
    return run_service(
        args.bind,
        args.port,
        args.publish_rate,
        args.address,
        args.bus,
        args.simulate,
        args.calibrate_seconds,
    )


if __name__ == "__main__":
    raise SystemExit(main())
