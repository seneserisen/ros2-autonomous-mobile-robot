from __future__ import annotations

from math import pi

import numpy as np
import pytest

from faultnav_robot.ekf import (
    LINEAR_VELOCITY,
    STATE_SIZE,
    YAW,
    YAW_RATE,
    update_encoder_velocity,
    update_gyro_yaw_rate,
    validate_covariance,
)
from faultnav_robot.experiments import simulate_scenario
from faultnav_robot.scenarios import straight_scenario
from faultnav_robot.sensors import (
    RobotGeometry,
    SensorFaultConfig,
    SensorSimulationConfig,
    TimeWindow,
    encoder_counts_to_body_twist,
    simulate_sensors,
)


def _encoder_velocity_between_samples(
    previous_count_left: int,
    previous_count_right: int,
    current_count_left: int,
    current_count_right: int,
    dt_s: float,
    geometry: RobotGeometry,
) -> float:
    velocity_m_s, _ = encoder_counts_to_body_twist(
        previous_count_left,
        previous_count_right,
        current_count_left,
        current_count_right,
        dt_s,
        geometry,
    )
    return velocity_m_s


def test_encoder_velocity_update_matches_independent_scalar_reference() -> None:
    state = np.array([0.0, 0.0, 0.1, 1.0, 0.0])
    covariance = np.diag([0.2, 0.3, 0.04, 0.4, 0.05])

    result = update_encoder_velocity(state, covariance, 1.2, 0.1)

    expected_innovation = 0.2
    expected_innovation_covariance = 0.5
    expected_gain = 0.4 / 0.5
    expected_velocity = 1.0 + expected_gain * expected_innovation
    expected_velocity_variance = (1.0 - expected_gain) ** 2 * 0.4 + expected_gain**2 * 0.1

    assert result.predicted_measurement == pytest.approx(1.0)
    assert result.actual_measurement == pytest.approx(1.2)
    assert result.innovation == pytest.approx(expected_innovation)
    assert result.innovation_covariance == pytest.approx(expected_innovation_covariance)
    assert result.kalman_gain[LINEAR_VELOCITY, 0] == pytest.approx(expected_gain)
    assert result.state[LINEAR_VELOCITY] == pytest.approx(expected_velocity)
    assert result.covariance[LINEAR_VELOCITY, LINEAR_VELOCITY] == pytest.approx(
        expected_velocity_variance
    )
    np.testing.assert_array_equal(
        result.measurement_jacobian,
        np.array([[0.0, 0.0, 0.0, 1.0, 0.0]]),
    )


def test_gyro_update_matches_independent_scalar_reference() -> None:
    state = np.array([0.0, 0.0, -0.2, 0.8, 0.3])
    covariance = np.diag([0.2, 0.3, 0.04, 0.5, 0.2])

    result = update_gyro_yaw_rate(state, covariance, 0.5, 0.05)

    expected_innovation = 0.2
    expected_innovation_covariance = 0.25
    expected_gain = 0.2 / 0.25
    expected_yaw_rate = 0.3 + expected_gain * expected_innovation
    expected_yaw_rate_variance = (1.0 - expected_gain) ** 2 * 0.2 + expected_gain**2 * 0.05

    assert result.innovation == pytest.approx(expected_innovation)
    assert result.innovation_covariance == pytest.approx(expected_innovation_covariance)
    assert result.kalman_gain[YAW_RATE, 0] == pytest.approx(expected_gain)
    assert result.state[YAW_RATE] == pytest.approx(expected_yaw_rate)
    assert result.covariance[YAW_RATE, YAW_RATE] == pytest.approx(expected_yaw_rate_variance)
    np.testing.assert_array_equal(
        result.measurement_jacobian,
        np.array([[0.0, 0.0, 0.0, 0.0, 1.0]]),
    )


def test_correlated_covariance_updates_related_states_and_matches_joseph_form() -> None:
    factor = np.array(
        [
            [0.6, 0.0, 0.0, 0.0, 0.0],
            [0.1, 0.5, 0.0, 0.0, 0.0],
            [0.2, -0.1, 0.4, 0.0, 0.0],
            [0.3, 0.1, 0.2, 0.5, 0.0],
            [0.0, 0.1, -0.1, 0.2, 0.3],
        ]
    )
    covariance = factor @ factor.T
    state = np.array([1.0, -0.5, 0.2, 0.8, -0.1])
    measurement = 1.1
    variance = 0.07
    jacobian = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
    innovation = measurement - state[LINEAR_VELOCITY]
    innovation_covariance = (jacobian @ covariance @ jacobian.T).item() + variance
    expected_gain = covariance @ jacobian.T / innovation_covariance
    expected_state = state + expected_gain[:, 0] * innovation
    identity_minus_gain_jacobian = np.eye(STATE_SIZE) - expected_gain @ jacobian
    expected_covariance = (
        identity_minus_gain_jacobian @ covariance @ identity_minus_gain_jacobian.T
        + variance * (expected_gain @ expected_gain.T)
    )

    result = update_encoder_velocity(state, covariance, measurement, variance)

    np.testing.assert_allclose(result.kalman_gain, expected_gain, atol=1e-14)
    np.testing.assert_allclose(result.state, expected_state, atol=1e-14)
    np.testing.assert_allclose(result.covariance, expected_covariance, atol=1e-14)
    assert result.state[0] != pytest.approx(state[0])
    assert result.state[YAW] != pytest.approx(state[YAW])
    validate_covariance(result.covariance)


def test_perfect_agreement_preserves_mean_but_reduces_observed_variance() -> None:
    state = np.array([0.1, 0.2, 0.3, 1.4, -0.2])
    covariance = np.diag([0.2, 0.2, 0.1, 0.6, 0.3])

    result = update_encoder_velocity(state, covariance, 1.4, 0.2)

    assert result.innovation == pytest.approx(0.0)
    np.testing.assert_allclose(result.state, state, atol=0.0)
    assert (
        result.covariance[LINEAR_VELOCITY, LINEAR_VELOCITY]
        < covariance[LINEAR_VELOCITY, LINEAR_VELOCITY]
    )


def test_measurement_uncertainty_controls_weight_without_zero_variance() -> None:
    state = np.zeros(STATE_SIZE)
    covariance = np.eye(STATE_SIZE)

    low_uncertainty = update_gyro_yaw_rate(state, covariance, 1.0, 1e-4)
    high_uncertainty = update_gyro_yaw_rate(state, covariance, 1.0, 1e6)

    assert low_uncertainty.kalman_gain[YAW_RATE, 0] > high_uncertainty.kalman_gain[YAW_RATE, 0]
    assert low_uncertainty.state[YAW_RATE] > high_uncertainty.state[YAW_RATE]
    assert (
        low_uncertainty.covariance[YAW_RATE, YAW_RATE]
        < high_uncertainty.covariance[YAW_RATE, YAW_RATE]
    )


def test_measurement_updates_do_not_mutate_inputs_and_remain_stable_at_large_scale() -> None:
    state = np.array([2.0, -1.0, 0.4, 3.0, 0.2])
    covariance = np.diag([1e8, 2e8, 3e6, 4e7, 5e5])
    original_state = state.copy()
    original_covariance = covariance.copy()

    result = update_encoder_velocity(state, covariance, 2.5, 1e-3)

    np.testing.assert_array_equal(state, original_state)
    np.testing.assert_array_equal(covariance, original_covariance)
    assert np.all(np.isfinite(result.state))
    assert np.all(np.isfinite(result.covariance))
    np.testing.assert_allclose(result.covariance, result.covariance.T, atol=1e-12)
    validate_covariance(result.covariance)


@pytest.mark.parametrize(
    ("measurement", "variance", "message"),
    [
        (None, 0.1, "finite numeric"),
        (np.nan, 0.1, "finite numeric"),
        (np.inf, 0.1, "finite numeric"),
        (0.0, 0.0, "strictly positive"),
        (0.0, -0.1, "strictly positive"),
        (0.0, np.inf, "finite numeric"),
    ],
)
def test_gyro_update_rejects_absent_nonfinite_or_invalid_uncertainty(
    measurement: float | None,
    variance: float,
    message: str,
) -> None:
    with pytest.raises(ValueError, match=message):
        update_gyro_yaw_rate(np.zeros(STATE_SIZE), np.eye(STATE_SIZE), measurement, variance)


def test_measurement_updates_reuse_existing_covariance_validation() -> None:
    asymmetric = np.eye(STATE_SIZE)
    asymmetric[0, 1] = 0.1
    indefinite = np.eye(STATE_SIZE)
    indefinite[0, 0] = -1.0

    with pytest.raises(ValueError, match="state must have shape"):
        update_encoder_velocity(np.zeros(4), np.eye(STATE_SIZE), 0.0, 0.1)
    with pytest.raises(ValueError, match="covariance must have shape"):
        update_encoder_velocity(np.zeros(STATE_SIZE), np.eye(4), 0.0, 0.1)
    with pytest.raises(ValueError, match="symmetric"):
        update_encoder_velocity(np.zeros(STATE_SIZE), asymmetric, 0.0, 0.1)
    with pytest.raises(ValueError, match="positive semi-definite"):
        update_encoder_velocity(np.zeros(STATE_SIZE), indefinite, 0.0, 0.1)
    with pytest.raises(ValueError, match="tolerances must be finite and non-negative"):
        update_encoder_velocity(
            np.zeros(STATE_SIZE),
            np.eye(STATE_SIZE),
            0.0,
            0.1,
            covariance_psd_tolerance=np.nan,
        )


@pytest.mark.parametrize(
    ("left_count", "right_count", "expected_velocity_sign"),
    [
        (100, 100, 1),
        (0, 0, 0),
        (-100, 100, 0),
    ],
    ids=["straight", "zero", "pure-rotation"],
)
def test_encoder_count_geometry_reference_cases(
    left_count: int,
    right_count: int,
    expected_velocity_sign: int,
) -> None:
    geometry = RobotGeometry(
        wheel_radius_m=0.1,
        wheel_separation_m=0.4,
        encoder_counts_per_revolution=1000,
    )

    velocity_m_s, _ = encoder_counts_to_body_twist(
        0,
        0,
        left_count,
        right_count,
        0.5,
        geometry,
    )

    expected_velocity_m_s = 0.1 * 2.0 * pi * (left_count + right_count) / (2.0 * 1000 * 0.5)
    assert velocity_m_s == pytest.approx(expected_velocity_m_s)
    assert np.sign(velocity_m_s) == expected_velocity_sign


def test_encoder_count_geometry_scales_with_wheel_radius() -> None:
    small = RobotGeometry(wheel_radius_m=0.05, encoder_counts_per_revolution=1000)
    large = RobotGeometry(wheel_radius_m=0.10, encoder_counts_per_revolution=1000)

    small_velocity, _ = encoder_counts_to_body_twist(0, 0, 100, 100, 0.5, small)
    large_velocity, _ = encoder_counts_to_body_twist(0, 0, 100, 100, 0.5, large)

    assert large_velocity == pytest.approx(2.0 * small_velocity)


def test_encoder_count_helper_rejects_invalid_counts_and_interval() -> None:
    geometry = RobotGeometry()

    with pytest.raises(ValueError, match="integers"):
        encoder_counts_to_body_twist(0.0, 0, 1, 1, 0.1, geometry)
    with pytest.raises(ValueError, match="finite and positive"):
        encoder_counts_to_body_twist(0, 0, 1, 1, 0.0, geometry)


def test_simulated_encoder_and_gyro_measurements_form_a_valid_update_sequence() -> None:
    geometry = RobotGeometry(encoder_counts_per_revolution=100_000)
    truth = simulate_scenario(straight_scenario(), integration_step_s=0.1)
    samples = simulate_sensors(truth, SensorSimulationConfig(geometry=geometry))
    previous, current = samples[0], samples[1]
    encoder_velocity_m_s = _encoder_velocity_between_samples(
        previous.left_encoder_count,
        previous.right_encoder_count,
        current.left_encoder_count,
        current.right_encoder_count,
        current.dt_s,
        geometry,
    )
    assert current.imu_yaw_rate_rad_s is not None

    prior_state = np.array([0.0, 0.0, 0.0, 0.5, 0.1])
    prior_covariance = np.diag([0.2, 0.2, 0.1, 0.4, 0.3])
    encoder_result = update_encoder_velocity(
        prior_state,
        prior_covariance,
        encoder_velocity_m_s,
        0.02,
    )
    gyro_result = update_gyro_yaw_rate(
        encoder_result.state,
        encoder_result.covariance,
        current.imu_yaw_rate_rad_s,
        0.01,
    )

    assert np.all(np.isfinite(gyro_result.state))
    validate_covariance(gyro_result.covariance)


@pytest.mark.parametrize(
    ("faults", "minimum_innovation"),
    [
        (SensorFaultConfig(gyro_bias_rad_s=0.25), 0.2),
        (
            SensorFaultConfig(
                gyro_outlier_window=TimeWindow(0.0, 1.0),
                gyro_outlier_rad_s=0.8,
            ),
            0.7,
        ),
    ],
    ids=["gyro-bias", "gyro-outlier"],
)
def test_gyro_faults_remain_visible_as_ungated_innovations(
    faults: SensorFaultConfig,
    minimum_innovation: float,
) -> None:
    truth = simulate_scenario(straight_scenario(), integration_step_s=0.1)
    samples = simulate_sensors(truth, SensorSimulationConfig(faults=faults))
    measurement = samples[1].imu_yaw_rate_rad_s
    assert measurement is not None

    result = update_gyro_yaw_rate(
        np.zeros(STATE_SIZE),
        np.eye(STATE_SIZE),
        measurement,
        0.1,
    )

    assert abs(result.innovation) > minimum_innovation
    assert result.state[YAW_RATE] != pytest.approx(0.0)
    assert not hasattr(result, "accepted")


def test_imu_dropout_is_not_converted_to_zero_measurement() -> None:
    truth = simulate_scenario(straight_scenario(), integration_step_s=0.1)
    samples = simulate_sensors(
        truth,
        SensorSimulationConfig(
            faults=SensorFaultConfig(imu_dropout_window=TimeWindow(0.0, 1.0)),
        ),
    )
    assert samples[1].imu_yaw_rate_rad_s is None

    with pytest.raises(ValueError, match="finite numeric"):
        update_gyro_yaw_rate(
            np.zeros(STATE_SIZE),
            np.eye(STATE_SIZE),
            samples[1].imu_yaw_rate_rad_s,
            0.1,
        )


def test_corrupted_encoder_counts_influence_update_without_truth_correction() -> None:
    geometry = RobotGeometry(encoder_counts_per_revolution=100_000)
    truth = simulate_scenario(straight_scenario(), integration_step_s=0.1)
    nominal = simulate_sensors(truth, SensorSimulationConfig(geometry=geometry))
    corrupted = simulate_sensors(
        truth,
        SensorSimulationConfig(
            geometry=geometry,
            faults=SensorFaultConfig(
                left_encoder_scale_error=0.2,
                right_encoder_scale_error=0.2,
            ),
        ),
    )
    nominal_velocity = _encoder_velocity_between_samples(
        nominal[0].left_encoder_count,
        nominal[0].right_encoder_count,
        nominal[1].left_encoder_count,
        nominal[1].right_encoder_count,
        nominal[1].dt_s,
        geometry,
    )
    corrupted_velocity = _encoder_velocity_between_samples(
        corrupted[0].left_encoder_count,
        corrupted[0].right_encoder_count,
        corrupted[1].left_encoder_count,
        corrupted[1].right_encoder_count,
        corrupted[1].dt_s,
        geometry,
    )
    prior_state = np.array([0.0, 0.0, 0.0, nominal_velocity, 0.0])

    result = update_encoder_velocity(
        prior_state,
        np.eye(STATE_SIZE),
        corrupted_velocity,
        0.1,
    )

    assert corrupted_velocity > nominal_velocity
    assert result.innovation == pytest.approx(corrupted_velocity - nominal_velocity)
    assert result.state[LINEAR_VELOCITY] > nominal_velocity
