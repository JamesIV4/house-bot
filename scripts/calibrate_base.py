#!/usr/bin/env python3
"""Fit the real-base ROS parameters from measured timed motion trials."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path
from typing import Any


MOTIONS = ("forward", "reverse", "left", "right")


def positive_number(document: dict[str, Any], name: str) -> float:
    value = document.get(name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be finite and greater than zero")
    return result


def trial_speed(trial: dict[str, Any], measurement_name: str) -> float:
    duration = positive_number(trial, "duration_s")
    measurement = positive_number(trial, measurement_name)
    power_value = trial.get("power", 1.0)
    if isinstance(power_value, bool) or not isinstance(power_value, (int, float)):
        raise ValueError("power must be a number")
    power = float(power_value)
    if not 0.05 <= power <= 1.0:
        raise ValueError("power must be between 0.05 and 1.0")
    return measurement / duration / power


def required_trials(document: dict[str, Any], motion: str) -> list[dict[str, Any]]:
    trials = document.get("trials")
    if not isinstance(trials, dict):
        raise ValueError("trials must be an object")
    values = trials.get(motion)
    if not isinstance(values, list) or len(values) < 2:
        raise ValueError(f"trials.{motion} must contain at least two trials")
    if not all(isinstance(value, dict) for value in values):
        raise ValueError(f"every trials.{motion} entry must be an object")
    return values


def solve_calibration(document: dict[str, Any]) -> dict[str, Any]:
    geometry = document.get("geometry")
    camera = document.get("camera_transform")
    if not isinstance(geometry, dict) or not isinstance(camera, dict):
        raise ValueError("geometry and camera_transform must be objects")
    track = positive_number(geometry, "wheel_separation_m")
    length = positive_number(geometry, "footprint_length_m")
    width = positive_number(geometry, "footprint_width_m")
    margin_value = geometry.get("footprint_safety_margin_m", 0.02)
    if isinstance(margin_value, bool) or not isinstance(margin_value, (int, float)):
        raise ValueError("footprint_safety_margin_m must be a number")
    margin = float(margin_value)
    if not 0.0 <= margin <= 0.10:
        raise ValueError("footprint_safety_margin_m must be between 0 and 0.10")

    forward = [trial_speed(trial, "distance_m") for trial in required_trials(document, "forward")]
    reverse = [trial_speed(trial, "distance_m") for trial in required_trials(document, "reverse")]
    left_angular = [
        math.radians(trial_speed(trial, "angle_deg"))
        for trial in required_trials(document, "left")
    ]
    right_angular = [
        math.radians(trial_speed(trial, "angle_deg"))
        for trial in required_trials(document, "right")
    ]
    left_pivot_wheel = [value * track * 0.5 for value in left_angular]
    right_pivot_wheel = [value * track * 0.5 for value in right_angular]
    observations = {
        "left_forward_mps": forward + right_pivot_wheel,
        "left_reverse_mps": reverse + left_pivot_wheel,
        "right_forward_mps": forward + left_pivot_wheel,
        "right_reverse_mps": reverse + right_pivot_wheel,
    }
    wheel_speeds = {
        name: float(statistics.median(values)) for name, values in observations.items()
    }

    camera_values = {}
    for name in ("x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg"):
        value = camera.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"camera_transform.{name} must be a number")
        camera_values[name] = float(value)
    if camera_values["z_m"] <= 0.0:
        raise ValueError("camera_transform.z_m must be greater than zero")

    half_length = length * 0.5 + margin
    half_width = width * 0.5 + margin
    footprint = [
        [half_length, half_width],
        [half_length, -half_width],
        [-half_length, -half_width],
        [-half_length, half_width],
    ]
    variation = {}
    for name, values in observations.items():
        mean = statistics.mean(values)
        variation[name] = statistics.pstdev(values) / mean if len(values) > 1 else 0.0

    return {
        "wheel_separation_m": track,
        **wheel_speeds,
        "camera": camera_values,
        "footprint": footprint,
        "robot_radius_m": math.hypot(half_length, half_width),
        "observation_coefficient_of_variation": variation,
    }


def yaml_number(value: float) -> str:
    return f"{value:.6f}"


def render_ros_parameters(solution: dict[str, Any]) -> str:
    camera = solution["camera"]
    footprint_text = "[" + ", ".join(
        f"[{yaml_number(point[0])}, {yaml_number(point[1])}]"
        for point in solution["footprint"]
    ) + "]"
    lines = [
        "# Generated by scripts/calibrate_base.py; do not guess these values.",
        "house_bot_base:",
        "  ros__parameters:",
        "    calibrated: true",
        f"    wheel_separation_m: {yaml_number(solution['wheel_separation_m'])}",
        f"    left_forward_mps: {yaml_number(solution['left_forward_mps'])}",
        f"    left_reverse_mps: {yaml_number(solution['left_reverse_mps'])}",
        f"    right_forward_mps: {yaml_number(solution['right_forward_mps'])}",
        f"    right_reverse_mps: {yaml_number(solution['right_reverse_mps'])}",
        f"    camera_x_m: {yaml_number(camera['x_m'])}",
        f"    camera_y_m: {yaml_number(camera['y_m'])}",
        f"    camera_z_m: {yaml_number(camera['z_m'])}",
        f"    camera_roll_rad: {yaml_number(math.radians(camera['roll_deg']))}",
        f"    camera_pitch_rad: {yaml_number(math.radians(camera['pitch_deg']))}",
        f"    camera_yaw_rad: {yaml_number(math.radians(camera['yaw_deg']))}",
        "local_costmap:",
        "  local_costmap:",
        "    ros__parameters:",
        f"      footprint: \"{footprint_text}\"",
        "global_costmap:",
        "  global_costmap:",
        "    ros__parameters:",
        f"      footprint: \"{footprint_text}\"",
        "",
    ]
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate calibrated ROS base and footprint parameters"
    )
    parser.add_argument("measurements", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("config/local/base_calibration.yaml"),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    document = json.loads(args.measurements.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("measurement file must contain a JSON object")
    solution = solve_calibration(document)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(render_ros_parameters(solution), encoding="utf-8")
    summary_path = args.output.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(solution, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote ROS parameters: {args.output}")
    print(f"Wrote calibration summary: {summary_path}")
    print(f"Conservative Nav2 radius: {solution['robot_radius_m']:.3f} m")
    high_variation = {
        name: value
        for name, value in solution["observation_coefficient_of_variation"].items()
        if value > 0.20
    }
    if high_variation:
        print(f"CALIBRATION_QUALITY=REVIEW high variation: {high_variation}")
        return 3
    print("CALIBRATION_QUALITY=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
