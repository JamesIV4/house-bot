import math

import pytest

from house_bot_navigation.base_kinematics import (
    DriveCalibration,
    calibrated_wheel_command,
    integrate_pose,
    twist_to_wheels,
    wheels_to_twist,
)


def calibration() -> DriveCalibration:
    return DriveCalibration(
        wheel_separation_m=0.20,
        left_forward_mps=0.30,
        left_reverse_mps=0.25,
        right_forward_mps=0.20,
        right_reverse_mps=0.25,
    )


def test_differential_drive_round_trip() -> None:
    wheels = twist_to_wheels(0.12, 0.6, 0.20)
    assert wheels_to_twist(*wheels, 0.20) == pytest.approx((0.12, 0.6))


def test_calibrated_command_accounts_for_asymmetric_wheels() -> None:
    left_duty, right_duty, left_mps, right_mps = calibrated_wheel_command(
        0.15, 0.0, calibration()
    )
    assert (left_mps, right_mps) == pytest.approx((0.15, 0.15))
    assert (left_duty, right_duty) == pytest.approx((0.5, 0.75))


def test_saturation_preserves_requested_curvature() -> None:
    requested = twist_to_wheels(0.2, 1.0, 0.20)
    left_duty, right_duty, actual_left, actual_right = calibrated_wheel_command(
        0.2, 1.0, calibration()
    )
    assert max(abs(left_duty), abs(right_duty)) == pytest.approx(1.0)
    assert actual_left / actual_right == pytest.approx(requested[0] / requested[1])


def test_exact_arc_integration() -> None:
    x, y, yaw = integrate_pose(0.0, 0.0, 0.0, 0.1, 0.3, 0.2, 1.0)
    assert yaw == pytest.approx(1.0)
    assert x == pytest.approx(0.2 * math.sin(1.0))
    assert y == pytest.approx(0.2 * (1.0 - math.cos(1.0)))


def test_invalid_zero_calibration_is_rejected() -> None:
    invalid = DriveCalibration(0.0, 0.1, 0.1, 0.1, 0.1)
    with pytest.raises(ValueError, match="wheel_separation_m"):
        invalid.validate()


def test_service_deadband_is_reflected_in_open_loop_motion() -> None:
    left_duty, right_duty, left_mps, right_mps = calibrated_wheel_command(
        0.05, 0.49, calibration()
    )
    assert left_duty == 0.0
    assert left_mps == 0.0
    assert right_duty > 0.0
    assert right_mps > 0.0


def test_nonfinite_twist_is_rejected() -> None:
    with pytest.raises(ValueError, match="finite"):
        calibrated_wheel_command(math.nan, 0.0, calibration())
