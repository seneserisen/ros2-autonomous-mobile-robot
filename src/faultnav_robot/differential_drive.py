"""Deterministic planar kinematics for a differential-drive robot."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin


@dataclass(frozen=True)
class RobotState:
    """Planar robot pose in an odometry frame."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to the half-open interval [-pi, pi)."""

    return (angle_rad + pi) % (2.0 * pi) - pi


def integrate_twist(
    state: RobotState,
    linear_velocity_m_s: float,
    angular_velocity_rad_s: float,
    dt_s: float,
    *,
    straight_line_epsilon: float = 1e-9,
) -> RobotState:
    """Integrate a constant body-frame twist using the exact unicycle solution.

    Args:
        state: Current planar pose.
        linear_velocity_m_s: Forward velocity in the robot body frame.
        angular_velocity_rad_s: Yaw rate.
        dt_s: Integration interval. Must be non-negative.
        straight_line_epsilon: Threshold below which angular motion is treated as straight.

    Returns:
        The pose after ``dt_s`` seconds.

    Raises:
        ValueError: If ``dt_s`` or ``straight_line_epsilon`` is invalid.
    """

    if dt_s < 0.0:
        raise ValueError("dt_s must be non-negative")
    if straight_line_epsilon <= 0.0:
        raise ValueError("straight_line_epsilon must be positive")
    if dt_s == 0.0:
        return state

    theta_0 = state.yaw_rad
    delta_theta = angular_velocity_rad_s * dt_s

    if abs(angular_velocity_rad_s) < straight_line_epsilon:
        x_m = state.x_m + linear_velocity_m_s * cos(theta_0) * dt_s
        y_m = state.y_m + linear_velocity_m_s * sin(theta_0) * dt_s
    else:
        radius_m = linear_velocity_m_s / angular_velocity_rad_s
        theta_1 = theta_0 + delta_theta
        x_m = state.x_m + radius_m * (sin(theta_1) - sin(theta_0))
        y_m = state.y_m - radius_m * (cos(theta_1) - cos(theta_0))

    return RobotState(x_m=x_m, y_m=y_m, yaw_rad=wrap_angle(theta_0 + delta_theta))
