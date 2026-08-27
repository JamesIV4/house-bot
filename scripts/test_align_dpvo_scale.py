#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys
import unittest

SCRIPTS = pathlib.Path(__file__).parent
sys.path.insert(0, str(SCRIPTS))

try:
    import numpy as np
except ImportError:  # pragma: no cover - depends on which interpreter runs this
    np = None


def load(name: str):
    path = SCRIPTS / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# align_dpvo_scale needs numpy and so runs under envs/dpvo, not the system
# interpreter that runs the rest of the suite.
align = load("align_dpvo_scale") if np is not None else None


def yaw_profile(t: float) -> float:
    """Two pivot bursts separated by straight legs, in deg/s."""
    if 3.0 <= t < 4.0:
        return 140.0
    if 8.0 <= t < 9.0:
        return -140.0
    return 0.0


def make_trajectory(lag_s: float, duration: float = 14.0, hz: float = 20.0) -> np.ndarray:
    """TUM trajectory whose rotation lags the commands by lag_s."""
    rows = []
    yaw = 0.0
    steps = int(duration * hz)
    for index in range(steps):
        t = index / hz
        yaw += math.radians(yaw_profile(t - lag_s)) / hz
        half = yaw / 2.0
        rows.append([t, 0.0, 0.0, 0.0, 0.0, 0.0, math.sin(half), math.cos(half)])
    return np.asarray(rows, dtype=np.float64)


def make_route(duration: float = 14.0, hz: float = 100.0) -> dict:
    log = []
    yaw = 0.0
    for index in range(int(duration * hz)):
        t = index / hz
        rate = yaw_profile(t)
        yaw += rate / hz
        log.append([t, yaw, rate])
    return {"yaw_log": log, "segments": []}


@unittest.skipIf(np is None, "numpy is unavailable; run under envs/dpvo/bin/python")
class YawTimeOffsetTests(unittest.TestCase):
    """Cross-correlating a dense IMU yaw log against the visual rotation."""

    def recover(self, lag_s: float) -> tuple[float, float]:
        result = align.estimate_time_offset_from_yaw(
            make_trajectory(lag_s), make_route(), first_source_time=0.0, first_arrival=0.0
        )
        self.assertIsNotNone(result, "no offset recovered")
        return result

    def test_a_known_lag_is_recovered(self) -> None:
        offset, correlation = self.recover(0.40)
        self.assertAlmostEqual(offset, 0.40, delta=0.06)
        self.assertGreater(correlation, align.MIN_YAW_CORRELATION)

    def test_zero_lag_is_recovered(self) -> None:
        offset, _correlation = self.recover(0.0)
        self.assertAlmostEqual(offset, 0.0, delta=0.06)

    def test_a_negative_lag_is_recovered(self) -> None:
        offset, _correlation = self.recover(-0.30)
        self.assertAlmostEqual(offset, -0.30, delta=0.06)

    def test_uncorrelated_rotation_is_rejected_not_guessed(self) -> None:
        """A weak match must be reported, not returned as a confident lag."""
        trajectory = make_trajectory(0.0)
        # Replace the visual rotation with noise unrelated to the commands.
        rng = np.random.default_rng(20260827)
        yaw = np.cumsum(rng.normal(0.0, 0.02, trajectory.shape[0]))
        trajectory[:, 6] = np.sin(yaw / 2.0)
        trajectory[:, 7] = np.cos(yaw / 2.0)
        result = align.estimate_time_offset_from_yaw(
            trajectory, make_route(), first_source_time=0.0, first_arrival=0.0
        )
        if result is not None:
            _offset, correlation = result
            self.assertLess(correlation, align.MIN_YAW_CORRELATION)

    def test_a_route_without_a_yaw_log_returns_nothing(self) -> None:
        self.assertIsNone(
            align.estimate_time_offset_from_yaw(
                make_trajectory(0.0), {"segments": []}, 0.0, 0.0
            )
        )

    def test_a_short_yaw_log_is_refused(self) -> None:
        route = {"yaw_log": [[0.0, 0.0, 0.0]] * 10, "segments": []}
        self.assertIsNone(
            align.estimate_time_offset_from_yaw(make_trajectory(0.0), route, 0.0, 0.0)
        )

    def test_clock_mapping_is_applied(self) -> None:
        """IMU stamps are monotonic; the trajectory has its own epoch."""
        result = align.estimate_time_offset_from_yaw(
            make_trajectory(0.20),
            make_route(),
            first_source_time=1000.0,
            first_arrival=0.0,
        )
        self.assertIsNone(result, "overlap should be empty once the clocks differ by 1000s")


if __name__ == "__main__":
    unittest.main()
