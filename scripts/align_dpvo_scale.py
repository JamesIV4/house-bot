#!/usr/bin/env python3
"""Estimate DPV-SLAM translation scale from timestamped base motion legs."""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trajectory", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--route", type=Path, required=True)
    parser.add_argument("--base-summary", type=Path, required=True)
    parser.add_argument("--measurements", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--accept-observed-motion",
        action="store_true",
        help="confirm an observer saw both treads engage throughout the route",
    )
    return parser.parse_args()


def load_tum(path: Path) -> np.ndarray:
    values = np.loadtxt(path, dtype=np.float64)
    if values.ndim == 1:
        values = values[None, :]
    if values.shape[1] != 8:
        raise ValueError("TUM trajectory must contain timestamp plus seven pose values")
    return values


def nearest_index(timestamps: np.ndarray, target: float) -> int:
    return int(np.argmin(np.abs(timestamps - target)))


def validation_speed(measurements: dict, motion: str, power: float) -> float | None:
    for sample in measurements.get("validation", []):
        if sample.get("motion") == motion and abs(float(sample.get("power", -1)) - power) < 1e-6:
            return float(sample["displacement_m"]) / float(sample["duration_s"])
    return None


def main() -> int:
    args = parse_args()
    trajectory = load_tum(args.trajectory)
    metrics = json.loads(args.metrics.read_text(encoding="utf-8"))
    route = json.loads(args.route.read_text(encoding="utf-8"))
    base = json.loads(args.base_summary.read_text(encoding="utf-8"))
    measurements = json.loads(args.measurements.read_text(encoding="utf-8"))
    if not args.accept_observed_motion:
        raise RuntimeError(
            "Refusing scale alignment without --accept-observed-motion; "
            "packet acknowledgements do not prove that both treads moved"
        )
    if route.get("schema_version") != 2:
        raise RuntimeError("Route predates the physical actuation validity gate")
    if route.get("actuation_command_model") != "binary_full_power":
        raise RuntimeError("Only the verified binary full-power command model is accepted")
    if route.get("result") != 0:
        raise RuntimeError("Route did not complete successfully")
    transport = metrics["transport"]
    first_arrival = float(transport["first_processed_arrival_monotonic_s"])
    first_source_time = float(transport["first_source_sequence"]) / float(
        transport["source_fps_assumption"]
    )

    samples = []
    trajectory_start = float(trajectory[0, 0])
    trajectory_end = float(trajectory[-1, 0])
    for segment in route["segments"]:
        motion = segment["motion"]
        if motion not in ("forward", "reverse"):
            continue
        start_time = first_source_time + (
            float(segment["command_started_monotonic_s"]) - first_arrival
        )
        end_time = first_source_time + (
            float(segment["command_ended_monotonic_s"]) - first_arrival
        )
        if float(segment["power"]) != 1.0:
            raise RuntimeError("Scale route contains an unsupported fractional command")
        if start_time < trajectory_start or end_time > trajectory_end:
            raise RuntimeError(
                f"Segment {segment['index']} lies outside the saved trajectory time range"
            )
        start_index = nearest_index(trajectory[:, 0], start_time)
        end_index = nearest_index(trajectory[:, 0], end_time)
        if abs(float(trajectory[start_index, 0]) - start_time) > 0.1:
            raise RuntimeError(f"Segment {segment['index']} start has no nearby pose")
        if abs(float(trajectory[end_index, 0]) - end_time) > 0.1:
            raise RuntimeError(f"Segment {segment['index']} end has no nearby pose")
        visual_distance = float(
            np.linalg.norm(trajectory[end_index, 1:4] - trajectory[start_index, 1:4])
        )
        power = float(segment["power"])
        speed = validation_speed(measurements, motion, power)
        if speed is None:
            if motion == "forward":
                speed = 0.5 * (
                    float(base["left_forward_mps"]) + float(base["right_forward_mps"])
                ) * power
            else:
                speed = 0.5 * (
                    float(base["left_reverse_mps"]) + float(base["right_reverse_mps"])
                ) * power
        expected_distance = speed * float(segment["duration_s"])
        if visual_distance <= 1e-6:
            continue
        samples.append(
            {
                "segment": int(segment["index"]),
                "motion": motion,
                "source_start_s": start_time,
                "source_end_s": end_time,
                "pose_start_index": start_index,
                "pose_end_index": end_index,
                "visual_distance_unscaled": visual_distance,
                "expected_distance_m": expected_distance,
                "scale_m_per_visual_unit": expected_distance / visual_distance,
            }
        )
    if len(samples) < 4:
        raise RuntimeError("Four usable straight route segments are required")
    scale_values = [sample["scale_m_per_visual_unit"] for sample in samples]
    scale = float(statistics.median(scale_values))
    max_relative_deviation = max(abs(value - scale) / scale for value in scale_values)
    if max_relative_deviation > 0.35:
        raise RuntimeError(
            "Straight-leg scale estimates disagree by more than 35%; reject this route"
        )
    scaled = trajectory.copy()
    scaled[:, 1:4] *= scale
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(args.output, scaled, fmt="%.9f")
    report = {
        "scale_m_per_visual_unit": scale,
        "max_relative_deviation": max_relative_deviation,
        "physical_motion_confirmed_by_operator": True,
        "samples": samples,
        "scaled_trajectory": str(args.output),
    }
    report_path = args.output.with_suffix(".scale.json")
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
