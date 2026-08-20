#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import math
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).with_name("calibrate_base.py")
SPEC = importlib.util.spec_from_file_location("calibrate_base", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
calibrate = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibrate)


def measurement_document():
    return {
        "geometry": {
            "wheel_separation_m": 0.2,
            "footprint_length_m": 0.3,
            "footprint_width_m": 0.2,
            "footprint_safety_margin_m": 0.02,
        },
        "camera_transform": {
            "x_m": 0.1,
            "y_m": 0.0,
            "z_m": 0.25,
            "roll_deg": 0.0,
            "pitch_deg": -5.0,
            "yaw_deg": 0.0,
        },
        "trials": {
            "forward": [
                {"duration_s": 2.0, "distance_m": 0.4},
                {"duration_s": 2.0, "distance_m": 0.4},
            ],
            "reverse": [
                {"duration_s": 2.0, "distance_m": 0.36},
                {"duration_s": 2.0, "distance_m": 0.36},
            ],
            "left": [
                {"duration_s": 2.0, "angle_deg": math.degrees(4.0)},
                {"duration_s": 2.0, "angle_deg": math.degrees(4.0)},
            ],
            "right": [
                {"duration_s": 2.0, "angle_deg": math.degrees(3.6)},
                {"duration_s": 2.0, "angle_deg": math.degrees(3.6)},
            ],
        },
    }


class BaseCalibrationTests(unittest.TestCase):
    def test_solver_combines_straight_and_pivot_trials(self) -> None:
        solution = calibrate.solve_calibration(measurement_document())
        self.assertAlmostEqual(solution["left_forward_mps"], 0.19)
        self.assertAlmostEqual(solution["left_reverse_mps"], 0.19)
        self.assertAlmostEqual(solution["right_forward_mps"], 0.20)
        self.assertAlmostEqual(solution["right_reverse_mps"], 0.18)
        self.assertAlmostEqual(solution["camera"]["pitch_deg"], -5.0)

    def test_rendered_parameters_are_calibration_gated(self) -> None:
        rendered = calibrate.render_ros_parameters(
            calibrate.solve_calibration(measurement_document())
        )
        self.assertIn("calibrated: true", rendered)
        self.assertIn("footprint:", rendered)
        self.assertIn("camera_pitch_rad: -0.087266", rendered)

    def test_missing_repeated_trials_is_rejected(self) -> None:
        document = measurement_document()
        document["trials"]["left"] = document["trials"]["left"][:1]
        with self.assertRaisesRegex(ValueError, "at least two"):
            calibrate.solve_calibration(document)


if __name__ == "__main__":
    unittest.main()
