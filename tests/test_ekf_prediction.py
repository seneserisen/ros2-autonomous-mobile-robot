from __future__ import annotations

from math import cos, pi, sin

import numpy as np
import pytest

from faultnav_robot.differential_drive import wrap_angle
from faultnav_robot.ekf import (
    LINEAR_VELOCITY,
    POSITION_X,
    POSITION_Y,
    STATE_SIZE,
    YAW,
    YAW_RATE,
    EkfPredictionConfig,
    predict,
    predict_state,
    prediction_jacobian,
    process_covariance,
    validate_covariance,
)


def _finite_difference_jacobian(state: np.ndarray, dt_s: float) -> np.ndarray:
    step = 1e-6
    reference = np.zeros((STATE_SIZE, STATE_SIZE))
    for column in range(STATE_SIZE):
        plus = state.copy()
        minus = state.copy()
        plus[column] += step
        minus[column] -= step
        difference = predict_state(plus, dt_s) - predict_state(minus, dt_s)
        difference[YAW] = wrap_angle(difference[YAW])
        reference[:, column] = difference / (2.0 * step)
    return reference


def test_straight_prediction_and_jacobian_match_analytical_reference() -> None:
    state = np.array([0.0, 0.0, 0.0, 2.0, 0.0])

    result = predict(state, np.zeros((STATE_SIZE, STATE_SIZE)), 1.5)

    np.testing.assert_allclose(result.state, [3.0, 0.0, 0.0, 2.0, 0.0], atol=1e-12)
    expected_jacobian = np.eye(STATE_SIZE)
    expected_jacobian[POSITION_Y, YAW] = 3.0
    expected_jacobian[POSITION_X, LINEAR_VELOCITY] = 1.5
    expected_jacobian[POSITION_Y, YAW_RATE] = 2.25
    expected_jacobian[YAW, YAW_RATE] = 1.5
    np.testing.assert_allclose(result.state_transition_jacobian, expected_jacobian, atol=1e-12)


def test_turning_prediction_and_jacobian_match_constant_radius_reference() -> None:
    state = np.array([0.0, 0.0, 0.0, 1.0, 1.0])

    result = predict(state, np.zeros((STATE_SIZE, STATE_SIZE)), pi / 2.0)

    np.testing.assert_allclose(result.state, [1.0, 1.0, pi / 2.0, 1.0, 1.0], atol=1e-12)
    expected_jacobian = np.eye(STATE_SIZE)
    expected_jacobian[POSITION_X, YAW] = -1.0
    expected_jacobian[POSITION_Y, YAW] = 1.0
    expected_jacobian[POSITION_X, LINEAR_VELOCITY] = 1.0
    expected_jacobian[POSITION_Y, LINEAR_VELOCITY] = 1.0
    expected_jacobian[POSITION_X, YAW_RATE] = -1.0
    expected_jacobian[POSITION_Y, YAW_RATE] = pi / 2.0 - 1.0
    expected_jacobian[YAW, YAW_RATE] = pi / 2.0
    np.testing.assert_allclose(result.state_transition_jacobian, expected_jacobian, atol=1e-12)


@pytest.mark.parametrize(
    ("state", "dt_s", "expected_pose"),
    [
        (np.array([1.0, -2.0, 0.4, 0.0, 0.0]), 3.0, [1.0, -2.0, 0.4]),
        (np.array([0.0, 0.0, 0.0, 0.0, pi / 2.0]), 1.0, [0.0, 0.0, pi / 2.0]),
        (np.array([0.0, 0.0, 0.0, 0.0, pi]), 1.0, [0.0, 0.0, -pi]),
    ],
    ids=["stationary", "rotation-in-place", "yaw-wrap"],
)
def test_prediction_pose_reference_cases(
    state: np.ndarray,
    dt_s: float,
    expected_pose: list[float],
) -> None:
    result = predict_state(state, dt_s)

    np.testing.assert_allclose(result[:3], expected_pose, atol=1e-12)
    np.testing.assert_array_equal(result[3:], state[3:])


@pytest.mark.parametrize(
    "yaw_rate_rad_s",
    [0.0, 0.8, -0.6, 0.5e-9],
    ids=["straight", "turning", "negative-turning", "near-branch"],
)
def test_prediction_jacobian_matches_central_finite_differences(
    yaw_rate_rad_s: float,
) -> None:
    state = np.array([0.4, -0.7, 0.35, 1.3, yaw_rate_rad_s])
    dt_s = 0.4

    analytical = prediction_jacobian(state, dt_s)
    reference = _finite_difference_jacobian(state, dt_s)

    np.testing.assert_allclose(analytical, reference, rtol=2e-7, atol=2e-8)


def test_zero_interval_preserves_state_and_covariance() -> None:
    state = np.array([1.0, -2.0, 0.4, 3.0, -0.2])
    covariance = np.diag([0.2, 0.3, 0.04, 0.5, 0.06])
    config = EkfPredictionConfig(q_linear_accel=2.0, q_yaw_accel=3.0)

    result = predict(state, covariance, 0.0, config)

    np.testing.assert_array_equal(result.state, state)
    np.testing.assert_array_equal(result.covariance, covariance)
    np.testing.assert_array_equal(result.state_transition_jacobian, np.eye(STATE_SIZE))
    np.testing.assert_array_equal(result.process_covariance, np.zeros_like(covariance))


def test_covariance_propagation_matches_independent_straight_motion_reference() -> None:
    state = np.array([1.0, 2.0, 0.0, 2.0, 0.0])
    covariance = np.diag([0.1, 0.2, 0.03, 0.4, 0.05])
    dt_s = 0.5
    q_linear_accel = 2.0
    q_yaw_accel = 3.0
    config = EkfPredictionConfig(
        q_linear_accel=q_linear_accel,
        q_yaw_accel=q_yaw_accel,
    )

    result = predict(state, covariance, dt_s, config)

    expected_jacobian = np.eye(STATE_SIZE)
    expected_jacobian[POSITION_Y, YAW] = 1.0
    expected_jacobian[POSITION_X, LINEAR_VELOCITY] = dt_s
    expected_jacobian[POSITION_Y, YAW_RATE] = 0.25
    expected_jacobian[YAW, YAW_RATE] = dt_s
    expected_process_covariance = np.zeros((STATE_SIZE, STATE_SIZE))
    expected_process_covariance[POSITION_X, POSITION_X] = q_linear_accel * dt_s**3 / 3.0
    expected_process_covariance[POSITION_X, LINEAR_VELOCITY] = (
        q_linear_accel * dt_s**2 / 2.0
    )
    expected_process_covariance[LINEAR_VELOCITY, POSITION_X] = (
        q_linear_accel * dt_s**2 / 2.0
    )
    expected_process_covariance[LINEAR_VELOCITY, LINEAR_VELOCITY] = (
        q_linear_accel * dt_s
    )
    expected_process_covariance[YAW, YAW] = q_yaw_accel * dt_s**3 / 3.0
    expected_process_covariance[YAW, YAW_RATE] = q_yaw_accel * dt_s**2 / 2.0
    expected_process_covariance[YAW_RATE, YAW] = q_yaw_accel * dt_s**2 / 2.0
    expected_process_covariance[YAW_RATE, YAW_RATE] = q_yaw_accel * dt_s
    expected_covariance = (
        expected_jacobian @ covariance @ expected_jacobian.T
        + expected_process_covariance
    )

    np.testing.assert_allclose(result.process_covariance, expected_process_covariance, atol=1e-12)
    np.testing.assert_allclose(result.covariance, expected_covariance, atol=1e-12)
    np.testing.assert_allclose(result.covariance, result.covariance.T, atol=1e-14)
    assert np.linalg.eigvalsh(result.covariance).min() >= -1e-12


def test_process_covariance_projects_linear_noise_at_midpoint_heading() -> None:
    state = np.array([0.0, 0.0, pi / 4.0, 1.0, 0.0])

    covariance = process_covariance(
        state,
        0.3,
        q_linear_accel=0.7,
        q_yaw_accel=0.0,
    )

    direction = np.array([cos(pi / 4.0), sin(pi / 4.0)])
    expected_position_block = 0.7 * 0.3**3 / 3.0 * np.outer(direction, direction)
    np.testing.assert_allclose(covariance[:2, :2], expected_position_block, atol=1e-12)
    validate_covariance(covariance)


def test_turning_covariance_is_finite_symmetric_and_positive_semi_definite() -> None:
    factor = np.array(
        [
            [0.3, 0.0, 0.0, 0.0, 0.0],
            [0.1, 0.2, 0.0, 0.0, 0.0],
            [0.0, -0.1, 0.15, 0.0, 0.0],
            [0.05, 0.0, 0.02, 0.25, 0.0],
            [0.0, 0.04, 0.0, -0.03, 0.1],
        ]
    )
    covariance = factor @ factor.T
    config = EkfPredictionConfig(q_linear_accel=0.2, q_yaw_accel=0.1)

    result = predict(np.array([0.2, -0.4, 0.7, 1.1, -0.6]), covariance, 0.3, config)

    assert np.all(np.isfinite(result.covariance))
    np.testing.assert_allclose(result.covariance, result.covariance.T, atol=1e-14)
    assert np.linalg.eigvalsh(result.covariance).min() >= -1e-12
    validate_covariance(result.covariance)


@pytest.mark.parametrize(
    ("state", "covariance", "dt_s", "message"),
    [
        (np.zeros(4), np.eye(STATE_SIZE), 0.1, "state must have shape"),
        (np.array([0.0, 0.0, np.nan, 0.0, 0.0]), np.eye(STATE_SIZE), 0.1, "finite"),
        (np.zeros(STATE_SIZE), np.eye(4), 0.1, "covariance must have shape"),
        (
            np.zeros(STATE_SIZE),
            np.diag([1.0, 1.0, np.inf, 1.0, 1.0]),
            0.1,
            "finite",
        ),
        (np.zeros(STATE_SIZE), np.eye(STATE_SIZE), -0.1, "non-negative"),
        (np.zeros(STATE_SIZE), np.eye(STATE_SIZE), np.inf, "finite"),
    ],
)
def test_prediction_rejects_invalid_dimensions_values_and_intervals(
    state: np.ndarray,
    covariance: np.ndarray,
    dt_s: float,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        predict(state, covariance, dt_s)


def test_prediction_rejects_asymmetric_and_indefinite_covariance() -> None:
    asymmetric = np.eye(STATE_SIZE)
    asymmetric[0, 1] = 0.1
    indefinite = np.eye(STATE_SIZE)
    indefinite[0, 0] = -1.0

    with pytest.raises(ValueError, match="symmetric"):
        predict(np.zeros(STATE_SIZE), asymmetric, 0.1)
    with pytest.raises(ValueError, match="positive semi-definite"):
        predict(np.zeros(STATE_SIZE), indefinite, 0.1)


@pytest.mark.parametrize(
    "config",
    [
        EkfPredictionConfig(q_linear_accel=0.1),
        EkfPredictionConfig(q_yaw_accel=0.1),
    ],
)
def test_prediction_does_not_mutate_posterior_inputs(config: EkfPredictionConfig) -> None:
    state = np.array([1.0, 2.0, 0.3, 0.4, 0.5])
    covariance = np.eye(STATE_SIZE)
    original_state = state.copy()
    original_covariance = covariance.copy()

    predict(state, covariance, 0.2, config)

    np.testing.assert_array_equal(state, original_state)
    np.testing.assert_array_equal(covariance, original_covariance)


def test_invalid_prediction_configuration_is_rejected() -> None:
    with pytest.raises(ValueError, match="process-noise"):
        EkfPredictionConfig(q_linear_accel=-1.0)
    with pytest.raises(ValueError, match="straight_line_epsilon"):
        EkfPredictionConfig(straight_line_epsilon=0.0)
    with pytest.raises(ValueError, match="covariance tolerances"):
        EkfPredictionConfig(covariance_psd_tolerance=np.nan)
