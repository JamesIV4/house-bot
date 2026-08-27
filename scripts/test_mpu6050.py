#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import unittest


SCRIPTS = pathlib.Path(__file__).parent
sys.path.insert(0, str(SCRIPTS))


def load(name: str):
    path = SCRIPTS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


mpu = load("mpu6050")


def burst(accel_counts: tuple[int, int, int], temp: int, gyro_counts: tuple[int, int, int]) -> list[int]:
    raw: list[int] = []
    for value in (*accel_counts, temp, *gyro_counts):
        unsigned = value + 65536 if value < 0 else value
        raw.extend(((unsigned >> 8) & 0xFF, unsigned & 0xFF))
    return raw


def stationary_sample(stamp: float, gyro_z: float, accel=(0.0, 0.0, 1.0)) -> "mpu.Sample":
    return mpu.Sample(accel=accel, gyro=(0.0, 0.0, gyro_z), temperature_c=30.0, stamp=stamp)


class FakeBus:
    def __init__(self, identity: int = 0x68) -> None:
        self.identity = identity
        self.writes: list[tuple[int, int]] = []

    def write_byte_data(self, _address: int, register: int, value: int) -> None:
        self.writes.append((register, value))

    def read_byte_data(self, _address: int, _register: int) -> int:
        return self.identity

    def read_i2c_block_data(self, _address: int, _register: int, length: int) -> list[int]:
        return burst((0, 0, 16384), 0, (0, 0, 655))[:length]


class DecodeTests(unittest.TestCase):
    def test_burst_decodes_scales_and_sign(self) -> None:
        raw = burst((0, 0, 16384), 0, (-6550, 655, 0))
        sample = mpu.decode_burst(raw, stamp=1.0)
        self.assertAlmostEqual(sample.accel[2], 1.0, places=4)
        self.assertAlmostEqual(sample.gyro[0], -100.0, places=1)
        self.assertAlmostEqual(sample.gyro[1], 10.0, places=1)
        self.assertAlmostEqual(sample.temperature_c, 36.53, places=2)

    def test_burst_length_is_enforced(self) -> None:
        with self.assertRaisesRegex(ValueError, "14-byte"):
            mpu.decode_burst([0] * 13, stamp=0.0)

    def test_wrap_degrees_folds_into_half_turn(self) -> None:
        self.assertAlmostEqual(mpu.wrap_degrees(190.0), -170.0)
        self.assertAlmostEqual(mpu.wrap_degrees(-190.0), 170.0)
        self.assertAlmostEqual(mpu.wrap_degrees(180.0), 180.0)
        self.assertAlmostEqual(mpu.wrap_degrees(45.0), 45.0)


class DeviceTests(unittest.TestCase):
    def test_configure_selects_gyro_pll_500dps_and_dlpf(self) -> None:
        bus = FakeBus()
        device = mpu.Mpu6050(bus)
        device.configure(settle_fn=lambda _seconds: None)
        written = dict(bus.writes)
        self.assertEqual(written[mpu.REG_PWR_MGMT_1], 0x01)
        self.assertEqual(written[mpu.REG_GYRO_CONFIG], mpu.GYRO_FS_500_DPS)
        self.assertEqual(written[mpu.REG_CONFIG], mpu.DLPF_44HZ)
        self.assertEqual(written[mpu.REG_SMPLRT_DIV], mpu.SAMPLE_RATE_DIV_200HZ)
        self.assertEqual(bus.writes[0], (mpu.REG_PWR_MGMT_1, 0x80))

    def test_unknown_identity_is_rejected(self) -> None:
        device = mpu.Mpu6050(FakeBus(identity=0xFF))
        with self.assertRaisesRegex(RuntimeError, "WHO_AM_I"):
            device.check_identity()

    def test_clone_identities_are_accepted(self) -> None:
        for identity in (0x68, 0x72, 0x98):
            mpu.Mpu6050(FakeBus(identity=identity)).check_identity()

    def test_address_must_be_a_real_ad0_option(self) -> None:
        with self.assertRaisesRegex(ValueError, "0x68 or 0x69"):
            mpu.Mpu6050(FakeBus(), address=0x70)


class BiasTests(unittest.TestCase):
    def test_bias_is_the_stationary_mean_with_its_spread(self) -> None:
        samples = [stationary_sample(index / 200.0, 1.0 + (index % 2) * 0.2) for index in range(200)]
        estimate = mpu.estimate_bias(samples)
        self.assertAlmostEqual(estimate.bias[2], 1.1, places=3)
        self.assertGreater(estimate.noise_dps[2], 0.0)
        self.assertEqual(estimate.samples, 200)

    def test_bias_needs_more_than_one_sample(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least two"):
            mpu.estimate_bias([stationary_sample(0.0, 0.0)])


class YawEstimatorTests(unittest.TestCase):
    def integrate(self, estimator: "mpu.YawEstimator", rate_dps: float, bias_dps: float, seconds: float) -> float:
        step = 1.0 / mpu.SAMPLE_RATE_HZ
        steps = int(seconds / step)
        for index in range(steps + 1):
            estimator.update(stationary_sample(index * step, rate_dps + bias_dps))
        return estimator.yaw_deg

    def test_constant_rate_integrates_to_the_swept_angle(self) -> None:
        estimator = mpu.YawEstimator(gravity=(0.0, 0.0, 1.0))
        yaw = self.integrate(estimator, rate_dps=90.0, bias_dps=0.0, seconds=1.0)
        self.assertAlmostEqual(yaw, 90.0, delta=0.6)

    def test_seeded_bias_is_removed_from_the_integral(self) -> None:
        estimator = mpu.YawEstimator(gravity=(0.0, 0.0, 1.0))
        estimator.set_bias((0.0, 0.0, 2.5))
        yaw = self.integrate(estimator, rate_dps=90.0, bias_dps=2.5, seconds=1.0)
        self.assertAlmostEqual(yaw, 90.0, delta=0.6)

    def test_uncorrected_bias_is_the_error_it_looks_like(self) -> None:
        """A 2.5 dps offset must cost 2.5 degrees per second, not be absorbed."""
        estimator = mpu.YawEstimator(gravity=(0.0, 0.0, 1.0))
        estimator.track_bias = False
        yaw = self.integrate(estimator, rate_dps=0.0, bias_dps=2.5, seconds=1.0)
        self.assertAlmostEqual(yaw, 2.5, delta=0.1)

    def test_tilted_mount_still_measures_rotation_about_true_vertical(self) -> None:
        tilt = math.radians(30.0)
        up = (0.0, math.sin(tilt), math.cos(tilt))
        estimator = mpu.YawEstimator(gravity=up)
        step = 1.0 / mpu.SAMPLE_RATE_HZ
        rate = 90.0
        for index in range(int(1.0 / step) + 1):
            estimator.update(
                mpu.Sample(
                    accel=up,
                    gyro=(0.0, rate * up[1], rate * up[2]),
                    temperature_c=30.0,
                    stamp=index * step,
                )
            )
        # Reading gyro Z alone would report cos(30 deg) * 90 = 77.9 degrees.
        self.assertAlmostEqual(estimator.yaw_deg, 90.0, delta=0.8)
        self.assertAlmostEqual(estimator.tilt_degrees(), 30.0, places=3)

    def test_a_long_gap_is_not_integrated(self) -> None:
        estimator = mpu.YawEstimator(gravity=(0.0, 0.0, 1.0))
        estimator.update(stationary_sample(0.0, 100.0))
        estimator.update(stationary_sample(0.005, 100.0))
        before = estimator.yaw_deg
        estimator.update(stationary_sample(9.0, 100.0))
        self.assertEqual(estimator.yaw_deg, before)

    def test_rotation_blocks_stationary_bias_tracking(self) -> None:
        estimator = mpu.YawEstimator(gravity=(0.0, 0.0, 1.0))
        self.integrate(estimator, rate_dps=90.0, bias_dps=0.0, seconds=1.5)
        self.assertFalse(estimator.stationary)
        self.assertAlmostEqual(estimator.gyro_bias[2], 0.0, places=6)

    def test_stillness_must_persist_before_it_counts(self) -> None:
        estimator = mpu.YawEstimator(gravity=(0.0, 0.0, 1.0))
        step = 1.0 / mpu.SAMPLE_RATE_HZ
        for index in range(int(0.5 / step)):
            estimator.update(stationary_sample(index * step, 0.05))
        self.assertFalse(estimator.stationary)
        for index in range(int(0.5 / step), int(1.5 / step)):
            estimator.update(stationary_sample(index * step, 0.05))
        self.assertTrue(estimator.stationary)


class OrientationTests(unittest.TestCase):
    def test_flat_mount_is_accepted(self) -> None:
        ok, message = mpu.describe_orientation((0.01, -0.02, 1.0))
        self.assertTrue(ok)
        self.assertIn("level", message)

    def test_moderate_tilt_is_usable_but_reported(self) -> None:
        ok, message = mpu.describe_orientation((0.0, 0.34, 0.94))
        self.assertTrue(ok)
        self.assertIn("tilted", message)

    def test_board_on_its_side_is_rejected(self) -> None:
        ok, message = mpu.describe_orientation((0.0, 1.0, 0.0))
        self.assertFalse(ok)
        self.assertIn("not flat", message)

    def test_upside_down_is_named_as_such(self) -> None:
        ok, message = mpu.describe_orientation((0.0, 0.0, -1.0))
        self.assertFalse(ok)
        self.assertIn("upside down", message)

    def test_moving_base_is_rejected_rather_than_measured(self) -> None:
        ok, message = mpu.describe_orientation((0.0, 0.0, 0.4))
        self.assertFalse(ok)
        self.assertIn("expected about 1.000 g", message)


if __name__ == "__main__":
    unittest.main()
