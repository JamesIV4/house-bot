"""Pure differential-drive calibration and integration helpers."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DriveCalibration:
    wheel_separation_m: float
    left_forward_mps: float
    left_reverse_mps: float
    right_forward_mps: float
    right_reverse_mps: float

    def validate(self) -> None:
        values = {
            "wheel_separation_m": self.wheel_separation_m,
            "left_forward_mps": self.left_forward_mps,
            "left_reverse_mps": self.left_reverse_mps,
            "right_forward_mps": self.right_forward_mps,
            "right_reverse_mps": self.right_reverse_mps,
        }
        for name, value in values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")

    def capacity(self, side: str, speed_mps: float) -> float:
        if side == "left":
            return self.left_forward_mps if speed_mps >= 0.0 else self.left_reverse_mps
        if side == "right":
            return self.right_forward_mps if speed_mps >= 0.0 else self.right_reverse_mps
        raise ValueError(f"unknown wheel side: {side}")


def twist_to_wheels(
    linear_mps: float,
    angular_radps: float,
    wheel_separation_m: float,
) -> tuple[float, float]:
    half_track = wheel_separation_m * 0.5
    return linear_mps - angular_radps * half_track, linear_mps + angular_radps * half_track


def wheels_to_twist(
    left_mps: float,
    right_mps: float,
    wheel_separation_m: float,
) -> tuple[float, float]:
    return (left_mps + right_mps) * 0.5, (right_mps - left_mps) / wheel_separation_m


def calibrated_wheel_command(
    linear_mps: float,
    angular_radps: float,
    calibration: DriveCalibration,
    duty_deadband: float = 0.05,
) -> tuple[float, float, float, float]:
    """Return left/right duty and achievable wheel speeds.

    Saturation scales both physical wheel targets together, preserving the
    requested path curvature even when direction-specific capacities differ.
    """
    calibration.validate()
    if not math.isfinite(linear_mps) or not math.isfinite(angular_radps):
        raise ValueError("requested Twist must be finite")
    requested_left, requested_right = twist_to_wheels(
        linear_mps, angular_radps, calibration.wheel_separation_m
    )
    left_capacity = calibration.capacity("left", requested_left)
    right_capacity = calibration.capacity("right", requested_right)
    saturation = max(
        1.0,
        abs(requested_left) / left_capacity,
        abs(requested_right) / right_capacity,
    )
    actual_left = requested_left / saturation
    actual_right = requested_right / saturation
    left_duty = actual_left / calibration.capacity("left", actual_left)
    right_duty = actual_right / calibration.capacity("right", actual_right)
    if abs(left_duty) <= duty_deadband:
        left_duty = 0.0
        actual_left = 0.0
    if abs(right_duty) <= duty_deadband:
        right_duty = 0.0
        actual_right = 0.0
    return left_duty, right_duty, actual_left, actual_right


def integrate_pose(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    left_mps: float,
    right_mps: float,
    wheel_separation_m: float,
    dt_s: float,
) -> tuple[float, float, float]:
    linear_mps, angular_radps = wheels_to_twist(
        left_mps, right_mps, wheel_separation_m
    )
    if abs(angular_radps) < 1e-9:
        x_m += linear_mps * math.cos(yaw_rad) * dt_s
        y_m += linear_mps * math.sin(yaw_rad) * dt_s
    else:
        next_yaw = yaw_rad + angular_radps * dt_s
        radius = linear_mps / angular_radps
        x_m += radius * (math.sin(next_yaw) - math.sin(yaw_rad))
        y_m -= radius * (math.cos(next_yaw) - math.cos(yaw_rad))
        yaw_rad = next_yaw
    yaw_rad = math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))
    return x_m, y_m, yaw_rad
