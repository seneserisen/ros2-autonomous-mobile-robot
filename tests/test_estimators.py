from __future__ import annotations

import json
from math import isclose

import numpy as np
import pytest

from faultnav_robot.estimators import (
    STATE_SIZE,
    YAW_RATE,
    PlanarEkf,
    calculate_estimator_metrics,
    encoder_twist,
    fault_aware_ekf_config,
    run_ekf,
    standard_ekf_config,
)
from faultnav_robot.experiments import simulate_scenario
from faultnav_robot.scenarios import figure_eight_scenario, straight_scenario
from faultnav_robot.sensors import (
    RobotGeometry,
    SensorSimulationConfig,
    sensor_profile,
    simulate_sensors,
)


def test_prediction_preserves_covariance_symmetry_and_psd() -> None:
    ekf = PlanarEkf(0.0, 0.0, 0.0)

    for _ in range(50):
        ekf.predict(0.1, 0.2)

    assert ekf.covariance.shape == (STATE_SIZE, STATE_SIZE)
    assert np.allclose(ekf.covariance, ekf.covariance.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(ekf.covariance)) >= -1e-12


def test_scalar_update_rejects_large_outlier_when_gated() -> None:
    ekf = PlanarEkf(0.0, 0.0, 0.0)
    ekf.predict(0.1)

    result = ekf.update_scalar(
        measurement=5.0,
        state_index=YAW_RATE,
        variance=0.01**2,
        nis_threshold=9.0,
    )

    assert not result.accepted
    assert result.nis > 9.0
    assert isclose(ekf.state[YAW_RATE], 0.0)


def test_encoder_twist_reconstructs_straight_velocity() -> None:
    truth = simulate_scenario(straight_scenario(), integration_step_s=0.1)
    geometry = RobotGeometry(encoder_counts_per_revolution=100_000)
    sensor_samples = simulate_sensors(
        truth,
        SensorSimulationConfig(geometry=geometry),
    )

    velocity, yaw_rate = encoder_twist(sensor_samples[0], sensor_samples[1], geometry)

    assert velocity == pytest.approx(0.5, abs=1e-4)
    assert yaw_rate == pytest.approx(0.0, abs=1e-4)


def test_standard_ekf_handles_imu_dropout_without_fabricated_measurement() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    config = sensor_profile("combined-faults", seed=7)
    sensor_samples = simulate_sensors(truth, config)

    estimates = run_ekf(sensor_samples, config.geometry, standard_ekf_config())

    dropout_samples = [sample for sample in estimates if sample.imu_dropout_active]
    assert dropout_samples
    assert all(sample.imu_yaw_rate_rad_s is None for sample in dropout_samples)
    assert all(not sample.gyro_accepted for sample in dropout_samples)
    assert all(np.isfinite(sample.covariance_trace) for sample in estimates)


def test_fault_aware_ekf_rejects_injected_gyro_outlier() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    config = sensor_profile("combined-faults", seed=7)
    sensor_samples = simulate_sensors(truth, config)

    estimates = run_ekf(sensor_samples, config.geometry, fault_aware_ekf_config())

    outlier_samples = [sample for sample in estimates if sample.gyro_outlier_active]
    assert outlier_samples
    assert any(not sample.gyro_accepted for sample in outlier_samples)
    assert max(sample.gyro_nis or 0.0 for sample in outlier_samples) > 9.0


def test_nominal_ekf_is_deterministic() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    config = sensor_profile("nominal", seed=21)
    sensor_samples = simulate_sensors(truth, config)

    first = run_ekf(sensor_samples, config.geometry, fault_aware_ekf_config())
    second = run_ekf(sensor_samples, config.geometry, fault_aware_ekf_config())

    assert first == second


def test_fault_aware_ekf_improves_combined_fault_position_rmse() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    config = sensor_profile("combined-faults", seed=7)
    sensor_samples = simulate_sensors(truth, config)

    standard_metrics = calculate_estimator_metrics(
        run_ekf(sensor_samples, config.geometry, standard_ekf_config())
    )
    gated_metrics = calculate_estimator_metrics(
        run_ekf(sensor_samples, config.geometry, fault_aware_ekf_config())
    )

    assert gated_metrics.position_rmse_m < standard_metrics.position_rmse_m
    assert gated_metrics.gyro_rejections >= 1
    assert gated_metrics.transient_fault_rejections >= 1


def test_prediction_wraps_yaw_to_principal_interval() -> None:
    ekf = PlanarEkf(0.0, 0.0, 3.13)
    ekf.state[YAW_RATE] = 1.0

    ekf.predict(0.2)

    assert -3.141592653589793 <= ekf.state[2] < 3.141592653589793


def test_gyro_bias_state_converges_for_constant_bias() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    config = sensor_profile("gyro-bias", seed=7)
    sensor_samples = simulate_sensors(truth, config)

    estimates = run_ekf(sensor_samples, config.geometry, fault_aware_ekf_config())
    metrics = calculate_estimator_metrics(estimates)

    assert estimates[-1].estimated_gyro_bias_rad_s == pytest.approx(0.05, abs=0.005)
    assert metrics.heading_rmse_rad < 0.01


def test_estimator_comparison_writes_complete_artifacts(tmp_path) -> None:
    from faultnav_robot.estimator_reports import run_estimator_comparison

    scenario = figure_eight_scenario()
    metrics, artifacts = run_estimator_comparison(
        scenario,
        tmp_path,
        "combined-faults",
        sensor_profile("combined-faults", seed=7),
        integration_step_s=0.1,
    )

    assert set(artifacts) == {
        "estimator_csv",
        "estimator_metrics_json",
        "estimator_report_svg",
    }
    assert all(path.exists() for path in artifacts.values())
    assert metrics.fault_aware_ekf.position_rmse_m < metrics.standard_ekf.position_rmse_m
    assert metrics.fault_aware_ekf.nominal_false_rejections == 0

    stored = json.loads(artifacts["estimator_metrics_json"].read_text(encoding="utf-8"))
    assert stored["sensor_profile"] == "combined-faults"
    assert stored["fault_aware_ekf"]["gyro_rejections"] >= 1

    svg = artifacts["estimator_report_svg"].read_text(encoding="utf-8")
    assert "Fault-aware EKF" in svg
    assert "NIS gate = 9" in svg
    assert "Raw wheel odometry" in svg
