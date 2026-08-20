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
        self.assertAlmostEqual(solution["wheel_separation_m"], 0.2005555556)
        self.assertAlmostEqual(solution["left_forward_mps"], 0.19025)
        self.assertAlmostEqual(solution["left_reverse_mps"], 0.1902777778)
        self.assertAlmostEqual(solution["right_forward_mps"], 0.2002777778)
        self.assertAlmostEqual(solution["right_reverse_mps"], 0.18025)
        self.assertAlmostEqual(solution["camera"]["pitch_deg"], -5.0)

    def test_rendered_parameters_are_calibration_gated(self) -> None:
        rendered = calibrate.render_ros_parameters(
            calibrate.solve_calibration(measurement_document())
        )
        self.assertIn("calibrated: true", rendered)
        self.assertIn("footprint:", rendered)
        self.assertIn("camera_pitch_rad: -0.087266", rendered)

    def test_single_coarse_trial_per_motion_is_accepted(self) -> None:
        document = measurement_document()
        for motion in document["trials"]:
            document["trials"][motion] = document["trials"][motion][:1]
        solution = calibrate.solve_calibration(document)
        self.assertGreater(solution["wheel_separation_m"], 0.0)

    def test_missing_motion_trial_is_rejected(self) -> None:
        document = measurement_document()
        document["trials"]["left"] = []
        with self.assertRaisesRegex(ValueError, "at least one"):
            calibrate.solve_calibration(document)

    def test_forward_heading_change_captures_tread_asymmetry(self) -> None:
        document = measurement_document()
        for motion in document["trials"]:
            document["trials"][motion] = document["trials"][motion][:1]
        document["trials"]["forward"][0]["heading_change_deg"] = -15.0
        solution = calibrate.solve_calibration(document)
        self.assertGreater(
            solution["left_forward_mps"], solution["right_forward_mps"]
        )


if __name__ == "__main__":
    unittest.main()
