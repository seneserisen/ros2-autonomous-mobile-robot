"""ROS-independent Extended Kalman Filter prediction for planar odometry.

The five-state ordering is fixed as ``[p_x_m, p_y_m, yaw_rad,
linear_velocity_m_s, yaw_rate_rad_s]``.  This module intentionally contains no
measurement updates, innovation monitoring, gating, or ROS 2 integration.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, sin

import numpy as np

from faultnav_robot.differential_drive import wrap_angle

STATE_SIZE = 5
POSITION_X = 0
POSITION_Y = 1
YAW = 2
LINEAR_VELOCITY = 3
YAW_RATE = 4
STATE_ORDER = (
    "p_x_m",
    "p_y_m",
    "yaw_rad",
    "linear_velocity_m_s",
    "yaw_rate_rad_s",
)


@dataclass(frozen=True)
class EkfPredictionConfig:
    """Numerical and continuous process-noise settings for EKF prediction.

    ``q_linear_accel`` is the longitudinal acceleration white-noise density in
    m^2/s^3. ``q_yaw_accel`` is the yaw-acceleration white-noise density in
    rad^2/s^3. Zero defaults keep process noise explicit rather than embedding
    scenario-specific tuning in the prediction core.
    """

    q_linear_accel: float = 0.0
    q_yaw_accel: float = 0.0
    straight_line_epsilon: float = 1e-9
    covariance_symmetry_tolerance: float = 1e-12
    covariance_psd_tolerance: float = 1e-12

    def __post_init__(self) -> None:
        densities = (self.q_linear_accel, self.q_yaw_accel)
        if not all(isfinite(value) and value >= 0.0 for value in densities):
            raise ValueError("process-noise densities must be finite and non-negative")
        if not isfinite(self.straight_line_epsilon) or self.straight_line_epsilon <= 0.0:
            raise ValueError("straight_line_epsilon must be finite and positive")
        tolerances = (
            self.covariance_symmetry_tolerance,
            self.covariance_psd_tolerance,
        )
        if not all(isfinite(value) and value >= 0.0 for value in tolerances):
            raise ValueError("covariance tolerances must be finite and non-negative")


@dataclass(frozen=True)
class EkfPredictionResult:
    """Predicted state, covariance, and the matrices used to calculate them."""

    state: np.ndarray
    covariance: np.ndarray
    state_transition_jacobian: np.ndarray
    process_covariance: np.ndarray


def _validated_state(state: np.ndarray) -> np.ndarray:
    array = np.asarray(state, dtype=float)
    if array.shape != (STATE_SIZE,):
        raise ValueError(f"state must have shape ({STATE_SIZE},) in ordering {STATE_ORDER}")
    if not np.all(np.isfinite(array)):
        raise ValueError("state must contain only finite values")
    return array.copy()


def _validation_scale(matrix: np.ndarray) -> float:
    return max(1.0, float(np.max(np.abs(matrix), initial=0.0)))


def _validated_covariance(
    covariance: np.ndarray,
    *,
    symmetry_tolerance: float,
    psd_tolerance: float,
) -> np.ndarray:
    matrix = np.asarray(covariance, dtype=float)
    if matrix.shape != (STATE_SIZE, STATE_SIZE):
        raise ValueError(f"covariance must have shape ({STATE_SIZE}, {STATE_SIZE})")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("covariance must contain only finite values")

    scale = _validation_scale(matrix)
    maximum_asymmetry = float(np.max(np.abs(matrix - matrix.T), initial=0.0))
    if maximum_asymmetry > symmetry_tolerance * scale:
        raise ValueError("covariance must be symmetric within tolerance")

    symmetric_matrix = 0.5 * (matrix + matrix.T)
    minimum_eigenvalue = float(np.min(np.linalg.eigvalsh(symmetric_matrix)))
    if minimum_eigenvalue < -psd_tolerance * scale:
        raise ValueError("covariance must be positive semi-definite within tolerance")
    return symmetric_matrix.copy()


def validate_covariance(
    covariance: np.ndarray,
    *,
    symmetry_tolerance: float = 1e-12,
    psd_tolerance: float = 1e-12,
) -> None:
    """Validate five-state covariance shape, finiteness, symmetry, and PSD.

    Symmetry and positive-semi-definite tolerances are relative to the larger
    of one and the largest absolute covariance entry.
    """

    tolerances = (symmetry_tolerance, psd_tolerance)
    if not all(isfinite(value) and value >= 0.0 for value in tolerances):
        raise ValueError("covariance tolerances must be finite and non-negative")
    _validated_covariance(
        covariance,
        symmetry_tolerance=symmetry_tolerance,
        psd_tolerance=psd_tolerance,
    )


def _sinc_and_derivative(value: float) -> tuple[float, float]:
    """Return sin(value)/value and its derivative without near-zero cancellation."""

    if abs(value) < 1e-4:
        value_squared = value * value
        sinc = 1.0 - value_squared / 6.0 + value_squared * value_squared / 120.0
        derivative = -value / 3.0 + value * value_squared / 30.0
        return sinc, derivative
    return sin(value) / value, (value * cos(value) - sin(value)) / (value * value)


def _state_and_jacobian(
    state: np.ndarray,
    dt_s: float,
    straight_line_epsilon: float,
) -> tuple[np.ndarray, np.ndarray]:
    if dt_s == 0.0:
        return state.copy(), np.eye(STATE_SIZE)

    p_x_m, p_y_m, yaw_rad, linear_velocity_m_s, yaw_rate_rad_s = state
    jacobian = np.eye(STATE_SIZE)
    jacobian[YAW, YAW_RATE] = dt_s

    if abs(yaw_rate_rad_s) < straight_line_epsilon:
        cosine_yaw = cos(yaw_rad)
        sine_yaw = sin(yaw_rad)
        delta_x_m = linear_velocity_m_s * cosine_yaw * dt_s
        delta_y_m = linear_velocity_m_s * sine_yaw * dt_s

        jacobian[POSITION_X, YAW] = -linear_velocity_m_s * dt_s * sine_yaw
        jacobian[POSITION_Y, YAW] = linear_velocity_m_s * dt_s * cosine_yaw
        jacobian[POSITION_X, LINEAR_VELOCITY] = dt_s * cosine_yaw
        jacobian[POSITION_Y, LINEAR_VELOCITY] = dt_s * sine_yaw
        jacobian[POSITION_X, YAW_RATE] = (
            -0.5 * linear_velocity_m_s * dt_s * dt_s * sine_yaw
        )
        jacobian[POSITION_Y, YAW_RATE] = (
            0.5 * linear_velocity_m_s * dt_s * dt_s * cosine_yaw
        )
    else:
        half_delta_yaw = 0.5 * yaw_rate_rad_s * dt_s
        midpoint_yaw = yaw_rad + half_delta_yaw
        sinc, sinc_derivative = _sinc_and_derivative(half_delta_yaw)
        distance_scale = linear_velocity_m_s * dt_s * sinc
        delta_x_m = distance_scale * cos(midpoint_yaw)
        delta_y_m = distance_scale * sin(midpoint_yaw)

        jacobian[POSITION_X, YAW] = -delta_y_m
        jacobian[POSITION_Y, YAW] = delta_x_m
        jacobian[POSITION_X, LINEAR_VELOCITY] = dt_s * sinc * cos(midpoint_yaw)
        jacobian[POSITION_Y, LINEAR_VELOCITY] = dt_s * sinc * sin(midpoint_yaw)
        derivative_scale = 0.5 * linear_velocity_m_s * dt_s * dt_s
        jacobian[POSITION_X, YAW_RATE] = derivative_scale * (
            -sin(midpoint_yaw) * sinc + cos(midpoint_yaw) * sinc_derivative
        )
        jacobian[POSITION_Y, YAW_RATE] = derivative_scale * (
            cos(midpoint_yaw) * sinc + sin(midpoint_yaw) * sinc_derivative
        )

    predicted_state = np.array(
        [
            p_x_m + delta_x_m,
            p_y_m + delta_y_m,
            wrap_angle(yaw_rad + yaw_rate_rad_s * dt_s),
            linear_velocity_m_s,
            yaw_rate_rad_s,
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(predicted_state)) or not np.all(np.isfinite(jacobian)):
        raise FloatingPointError("prediction produced non-finite state or Jacobian values")
    return predicted_state, jacobian


def predict_state(
    state: np.ndarray,
    dt_s: float,
    *,
    straight_line_epsilon: float = 1e-9,
) -> np.ndarray:
    """Propagate one five-state estimate with exact constant-twist motion."""

    validated_state = _validated_state(state)
    _validate_interval(dt_s)
    _validate_straight_line_epsilon(straight_line_epsilon)
    predicted_state, _ = _state_and_jacobian(
        validated_state,
        float(dt_s),
        float(straight_line_epsilon),
    )
    return predicted_state


def prediction_jacobian(
    state: np.ndarray,
    dt_s: float,
    *,
    straight_line_epsilon: float = 1e-9,
) -> np.ndarray:
    """Return the documented five-state constant-twist transition Jacobian."""

    validated_state = _validated_state(state)
    _validate_interval(dt_s)
    _validate_straight_line_epsilon(straight_line_epsilon)
    _, jacobian = _state_and_jacobian(
        validated_state,
        float(dt_s),
        float(straight_line_epsilon),
    )
    return jacobian


def _validate_interval(dt_s: float) -> None:
    if not isfinite(dt_s) or dt_s < 0.0:
        raise ValueError("dt_s must be finite and non-negative")


def _validate_straight_line_epsilon(straight_line_epsilon: float) -> None:
    if not isfinite(straight_line_epsilon) or straight_line_epsilon <= 0.0:
        raise ValueError("straight_line_epsilon must be finite and positive")


def process_covariance(
    state: np.ndarray,
    dt_s: float,
    *,
    q_linear_accel: float,
    q_yaw_accel: float,
) -> np.ndarray:
    """Discretize continuous acceleration white-noise densities over ``dt_s``.

    The longitudinal acceleration block uses a frozen midpoint body heading to
    project the standard continuous white-noise acceleration covariance into
    odometry x/y. The yaw/yaw-rate block uses the corresponding angular
    double-integrator covariance.
    """

    validated_state = _validated_state(state)
    _validate_interval(dt_s)
    densities = (q_linear_accel, q_yaw_accel)
    if not all(isfinite(value) and value >= 0.0 for value in densities):
        raise ValueError("process-noise densities must be finite and non-negative")

    dt_s = float(dt_s)
    if dt_s == 0.0:
        return np.zeros((STATE_SIZE, STATE_SIZE))

    dt_squared = dt_s * dt_s
    dt_cubed = dt_squared * dt_s
    midpoint_yaw = validated_state[YAW] + 0.5 * validated_state[YAW_RATE] * dt_s
    direction = np.array([cos(midpoint_yaw), sin(midpoint_yaw)])

    covariance = np.zeros((STATE_SIZE, STATE_SIZE))
    position_variance_scale = q_linear_accel * dt_cubed / 3.0
    position_velocity_scale = q_linear_accel * dt_squared / 2.0
    covariance[:2, :2] = position_variance_scale * np.outer(direction, direction)
    covariance[:2, LINEAR_VELOCITY] = position_velocity_scale * direction
    covariance[LINEAR_VELOCITY, :2] = position_velocity_scale * direction
    covariance[LINEAR_VELOCITY, LINEAR_VELOCITY] = q_linear_accel * dt_s

    covariance[YAW, YAW] = q_yaw_accel * dt_cubed / 3.0
    covariance[YAW, YAW_RATE] = q_yaw_accel * dt_squared / 2.0
    covariance[YAW_RATE, YAW] = covariance[YAW, YAW_RATE]
    covariance[YAW_RATE, YAW_RATE] = q_yaw_accel * dt_s

    if not np.all(np.isfinite(covariance)):
        raise FloatingPointError("process covariance contains non-finite values")
    return covariance


def predict(
    state: np.ndarray,
    covariance: np.ndarray,
    dt_s: float,
    config: EkfPredictionConfig | None = None,
) -> EkfPredictionResult:
    """Propagate EKF mean and covariance without using any measurements.

    ``state`` and ``covariance`` are posterior inputs at the start of the
    interval. They are never mutated. A zero interval is an explicit no-op.
    """

    resolved_config = config if config is not None else EkfPredictionConfig()
    validated_state = _validated_state(state)
    _validate_interval(dt_s)
    validated_covariance = _validated_covariance(
        covariance,
        symmetry_tolerance=resolved_config.covariance_symmetry_tolerance,
        psd_tolerance=resolved_config.covariance_psd_tolerance,
    )

    predicted_state, jacobian = _state_and_jacobian(
        validated_state,
        float(dt_s),
        resolved_config.straight_line_epsilon,
    )
    process_noise = process_covariance(
        validated_state,
        float(dt_s),
        q_linear_accel=resolved_config.q_linear_accel,
        q_yaw_accel=resolved_config.q_yaw_accel,
    )
    with np.errstate(over="ignore", invalid="ignore"):
        predicted_covariance = (
            jacobian @ validated_covariance @ jacobian.T + process_noise
        )
    if not np.all(np.isfinite(predicted_covariance)):
        raise FloatingPointError("prediction produced non-finite covariance values")

    validated_prediction_covariance = _validated_covariance(
        predicted_covariance,
        symmetry_tolerance=resolved_config.covariance_symmetry_tolerance,
        psd_tolerance=resolved_config.covariance_psd_tolerance,
    )
    return EkfPredictionResult(
        state=predicted_state,
        covariance=validated_prediction_covariance,
        state_transition_jacobian=jacobian,
        process_covariance=process_noise,
    )
