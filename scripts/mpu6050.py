#!/usr/bin/env python3
"""MPU-6050 driver and gravity-projected yaw estimator for the House Bot base.

Mounting contract (2026-08-27): the GY-521 lies flat on the chassis with its
+Y axis pointing to the front of the base and +X to the right, so +Z is up and
gyro Z is yaw rate, positive counter-clockwise (a left turn) under REP-103.

Yaw rate is not read straight off gyro Z. It is the gyro vector projected onto
the measured gravity direction, so a mount that is a few degrees off level
still reports true rotation about the vertical rather than a cosine-shrunk
component plus a leak from roll and pitch. On a perfectly level mount the
projection reduces to gyro Z exactly.

The estimator holds no I/O and is unit-tested off the Pi; `Mpu6050` is the only
part that needs a real I2C bus.
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol


ADDRESS_LOW = 0x68
ADDRESS_HIGH = 0x69

REG_SMPLRT_DIV = 0x19
REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_INT_PIN_CFG = 0x37
REG_INT_ENABLE = 0x38
REG_ACCEL_XOUT_H = 0x3B
REG_PWR_MGMT_1 = 0x6B
REG_WHO_AM_I = 0x75

# Gyro full scale. +/-500 dps keeps headroom over the ~180 dps peak measured on
# a 90 degree pivot; +/-250 dps would clip a fast turn.
GYRO_FS_500_DPS = 0x08
GYRO_SCALE_LSB_PER_DPS = 65.5
ACCEL_FS_2G = 0x00
ACCEL_SCALE_LSB_PER_G = 16384.0

# DLPF 3 gives 44 Hz accel / 42 Hz gyro bandwidth at a 4.9 ms group delay,
# which rejects tread and gearbox vibration well above any real turn rate.
DLPF_44HZ = 0x03
# Internal rate is 1 kHz once the DLPF is enabled, so divisor 4 yields 200 Hz.
SAMPLE_RATE_DIV_200HZ = 0x04
SAMPLE_RATE_HZ = 200.0

GRAVITY_TIME_CONSTANT_S = 2.0
STATIONARY_GYRO_DPS = 1.5
STATIONARY_ACCEL_G = 0.06
STATIONARY_HOLD_S = 0.75
BIAS_TRACK_TIME_CONSTANT_S = 30.0


class I2CBus(Protocol):
    """The subset of smbus2/smbus used here."""

    def write_byte_data(self, address: int, register: int, value: int) -> None: ...

    def read_byte_data(self, address: int, register: int) -> int: ...

    def read_i2c_block_data(self, address: int, register: int, length: int) -> list[int]: ...


@dataclass(frozen=True)
class Sample:
    """One synchronous burst read, in g, degrees per second, and Celsius."""

    accel: tuple[float, float, float]
    gyro: tuple[float, float, float]
    temperature_c: float
    stamp: float


def to_signed(high: int, low: int) -> int:
    value = (high << 8) | low
    return value - 65536 if value & 0x8000 else value


def decode_burst(raw: Sequence[int], stamp: float) -> Sample:
    """Decode the 14-byte ACCEL_XOUT_H burst into engineering units."""
    if len(raw) != 14:
        raise ValueError(f"expected a 14-byte burst, got {len(raw)}")
    accel = tuple(
        to_signed(raw[index], raw[index + 1]) / ACCEL_SCALE_LSB_PER_G
        for index in (0, 2, 4)
    )
    temperature_c = to_signed(raw[6], raw[7]) / 340.0 + 36.53
    gyro = tuple(
        to_signed(raw[index], raw[index + 1]) / GYRO_SCALE_LSB_PER_DPS
        for index in (8, 10, 12)
    )
    return Sample(accel, gyro, temperature_c, stamp)  # type: ignore[arg-type]


def norm(vector: Sequence[float]) -> float:
    return math.sqrt(sum(component * component for component in vector))


def unit(vector: Sequence[float]) -> tuple[float, float, float]:
    magnitude = norm(vector)
    if magnitude < 1e-9:
        raise ValueError("cannot normalise a zero-length vector")
    return tuple(component / magnitude for component in vector)  # type: ignore[return-value]


def dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def wrap_degrees(value: float) -> float:
    """Fold an angle into (-180, 180]."""
    return -((-value + 180.0) % 360.0 - 180.0)


class Mpu6050:
    """Blocking driver for one MPU-6050 on an I2C bus."""

    def __init__(self, bus: I2CBus, address: int = ADDRESS_LOW) -> None:
        if address not in (ADDRESS_LOW, ADDRESS_HIGH):
            raise ValueError("MPU-6050 address must be 0x68 or 0x69")
        self.bus = bus
        self.address = address

    def configure(self, settle_fn: Any = time.sleep) -> None:
        """Reset, wake, and put the device into the House Bot sampling mode."""
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x80)
        settle_fn(0.10)
        # Clock source 1 is the X gyro PLL, which is markedly more stable than
        # the internal 8 MHz oscillator the reset default selects.
        self.bus.write_byte_data(self.address, REG_PWR_MGMT_1, 0x01)
        settle_fn(0.01)
        self.bus.write_byte_data(self.address, REG_CONFIG, DLPF_44HZ)
        self.bus.write_byte_data(self.address, REG_GYRO_CONFIG, GYRO_FS_500_DPS)
        self.bus.write_byte_data(self.address, REG_ACCEL_CONFIG, ACCEL_FS_2G)
        self.bus.write_byte_data(self.address, REG_SMPLRT_DIV, SAMPLE_RATE_DIV_200HZ)
        # INT stays latched-off: the service polls, and BCM4 is wired only so a
        # future data-ready path does not need a rewire.
        self.bus.write_byte_data(self.address, REG_INT_ENABLE, 0x00)
        self.bus.write_byte_data(self.address, REG_INT_PIN_CFG, 0x00)
        settle_fn(0.05)

    def identity(self) -> int:
        return self.bus.read_byte_data(self.address, REG_WHO_AM_I)

    def check_identity(self) -> None:
        found = self.identity()
        # Genuine parts report 0x68 regardless of the AD0 pin; several clone
        # dies report 0x72 or 0x98 and are otherwise register-compatible.
        if found not in (0x68, 0x72, 0x98):
            raise RuntimeError(
                f"WHO_AM_I returned 0x{found:02X}; no MPU-6050 at 0x{self.address:02X}"
            )

    def read(self) -> Sample:
        raw = self.bus.read_i2c_block_data(self.address, REG_ACCEL_XOUT_H, 14)
        return decode_burst(raw, time.monotonic())


@dataclass
class YawEstimator:
    """Integrate gravity-projected yaw rate with an auto-tracked gyro bias.

    Bias is the whole game for turn accuracy: an uncorrected 1 dps offset is a
    full degree of error per second of turning, and the MPU-6050's offset moves
    with die temperature. Bias is therefore seeded by an explicit stationary
    calibration and then tracked slowly whenever the base is confirmed still.
    """

    gyro_bias: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gravity: tuple[float, float, float] | None = None
    yaw_deg: float = 0.0
    yaw_rate_dps: float = 0.0
    stationary: bool = False
    track_bias: bool = True
    samples: int = 0
    _last_stamp: float | None = field(default=None, repr=False)
    _still_since: float | None = field(default=None, repr=False)
    _accel_reference: tuple[float, float, float] | None = field(default=None, repr=False)

    def reset_yaw(self, yaw_deg: float = 0.0) -> None:
        self.yaw_deg = yaw_deg

    def set_bias(self, bias: Sequence[float]) -> None:
        self.gyro_bias = (float(bias[0]), float(bias[1]), float(bias[2]))

    def corrected_gyro(self, sample: Sample) -> tuple[float, float, float]:
        return tuple(  # type: ignore[return-value]
            axis - bias for axis, bias in zip(sample.gyro, self.gyro_bias)
        )

    def up_vector(self) -> tuple[float, float, float]:
        """Unit vector pointing up, in IMU axes.

        At rest an accelerometer reads +1 g along whichever axis points up, so
        the low-passed acceleration is the up direction directly.
        """
        if self.gravity is None:
            return (0.0, 0.0, 1.0)
        try:
            return unit(self.gravity)
        except ValueError:
            return (0.0, 0.0, 1.0)

    def tilt_degrees(self) -> float:
        """Angle between the mount's +Z axis and true up."""
        return math.degrees(math.acos(max(-1.0, min(1.0, self.up_vector()[2]))))

    def _update_gravity(self, sample: Sample, dt: float) -> None:
        if self.gravity is None:
            self.gravity = sample.accel
            return
        alpha = 1.0 - math.exp(-dt / GRAVITY_TIME_CONSTANT_S)
        self.gravity = tuple(  # type: ignore[assignment]
            previous + alpha * (measured - previous)
            for previous, measured in zip(self.gravity, sample.accel)
        )

    def _update_stationary(self, sample: Sample, dt: float) -> None:
        """Require both a quiet gyro and a steady accelerometer.

        Gyro alone is not enough: a tread base driving straight has a near-zero
        yaw rate while very much in motion, and folding that into the bias
        would teach the estimator to ignore real drift.
        """
        rotating = max(abs(axis) for axis in self.corrected_gyro(sample)) > STATIONARY_GYRO_DPS
        shaking = False
        if self._accel_reference is None:
            self._accel_reference = sample.accel
        else:
            delta = [
                measured - reference
                for measured, reference in zip(sample.accel, self._accel_reference)
            ]
            shaking = norm(delta) > STATIONARY_ACCEL_G
            beta = 1.0 - math.exp(-dt / 0.25)
            self._accel_reference = tuple(  # type: ignore[assignment]
                reference + beta * (measured - reference)
                for reference, measured in zip(self._accel_reference, sample.accel)
            )

        if rotating or shaking:
            self._still_since = None
            self.stationary = False
            return
        if self._still_since is None:
            self._still_since = sample.stamp
        self.stationary = sample.stamp - self._still_since >= STATIONARY_HOLD_S

    def _track_bias(self, sample: Sample, dt: float) -> None:
        if not (self.track_bias and self.stationary):
            return
        gamma = 1.0 - math.exp(-dt / BIAS_TRACK_TIME_CONSTANT_S)
        self.gyro_bias = tuple(  # type: ignore[assignment]
            bias + gamma * (measured - bias)
            for bias, measured in zip(self.gyro_bias, sample.gyro)
        )

    def update(self, sample: Sample) -> float:
        """Fold one sample in and return the integrated yaw in degrees."""
        self.samples += 1
        if self._last_stamp is None:
            self._last_stamp = sample.stamp
            self._update_gravity(sample, 1.0 / SAMPLE_RATE_HZ)
            return self.yaw_deg
        dt = sample.stamp - self._last_stamp
        self._last_stamp = sample.stamp
        # A hung I2C read or a descheduled loop must not integrate a huge step.
        if dt <= 0.0 or dt > 0.25:
            return self.yaw_deg

        self._update_gravity(sample, dt)
        self._update_stationary(sample, dt)
        self._track_bias(sample, dt)

        self.yaw_rate_dps = dot(self.corrected_gyro(sample), self.up_vector())
        self.yaw_deg += self.yaw_rate_dps * dt
        return self.yaw_deg


@dataclass(frozen=True)
class BiasEstimate:
    bias: tuple[float, float, float]
    noise_dps: tuple[float, float, float]
    samples: int
    accel: tuple[float, float, float]

    @property
    def worst_noise_dps(self) -> float:
        return max(self.noise_dps)


def estimate_bias(samples: Sequence[Sample]) -> BiasEstimate:
    """Mean and spread of a stationary capture, used to seed the estimator."""
    if len(samples) < 2:
        raise ValueError("bias estimation needs at least two samples")
    count = len(samples)
    bias = tuple(
        sum(sample.gyro[axis] for sample in samples) / count for axis in range(3)
    )
    noise = tuple(
        math.sqrt(
            sum((sample.gyro[axis] - bias[axis]) ** 2 for sample in samples) / (count - 1)
        )
        for axis in range(3)
    )
    accel = tuple(
        sum(sample.accel[axis] for sample in samples) / count for axis in range(3)
    )
    return BiasEstimate(bias, noise, count, accel)  # type: ignore[arg-type]


def describe_orientation(accel: Sequence[float]) -> tuple[bool, str]:
    """Check a stationary accel reading against the documented mounting.

    Expected: flat board, +Y to the front, +X to the right, so gravity shows up
    as roughly +1 g on Z and near zero on X and Y.
    """
    magnitude = norm(accel)
    if not 0.85 <= magnitude <= 1.15:
        return False, (
            f"|accel| = {magnitude:.3f} g, expected about 1.000 g; the base is "
            "either moving or the accelerometer scale is misconfigured"
        )
    up = unit(accel)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))
    if up[2] < 0.0:
        return False, (
            f"gravity reads {up[2]:+.3f} on Z: the board is upside down "
            "relative to the documented mounting"
        )
    if tilt > 30.0:
        dominant = max(range(3), key=lambda axis: abs(up[axis]))
        return False, (
            f"up vector is {tilt:.1f} deg off the board's +Z axis and lies "
            f"mostly along {'XYZ'[dominant]}; the board is not flat"
        )
    verdict = "level" if tilt <= 5.0 else "usable but tilted"
    return True, f"mount {verdict}: {tilt:.1f} deg from vertical, |accel| = {magnitude:.3f} g"


def open_bus(bus_number: int = 1) -> I2CBus:
    """Open /dev/i2c-N through smbus2, falling back to the older smbus."""
    try:
        from smbus2 import SMBus  # type: ignore[import-not-found]
    except ImportError:
        try:
            from smbus import SMBus  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError(
                "no I2C library found; install python3-smbus2 (or python3-smbus) on the Pi"
            ) from exc
    return SMBus(bus_number)  # type: ignore[return-value]


def collect(device: Mpu6050, seconds: float, rate_hz: float = SAMPLE_RATE_HZ) -> list[Sample]:
    """Read for a fixed wall-clock window at approximately rate_hz."""
    interval = 1.0 / rate_hz
    deadline = time.monotonic() + seconds
    samples: list[Sample] = []
    while time.monotonic() < deadline:
        started = time.monotonic()
        samples.append(device.read())
        remaining = interval - (time.monotonic() - started)
        if remaining > 0:
            time.sleep(remaining)
    return samples
