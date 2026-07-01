from __future__ import annotations

import json
from math import isclose
from pathlib import Path

import pytest

from faultnav_robot.experiments import simulate_scenario
from faultnav_robot.scenarios import figure_eight_scenario, straight_scenario
from faultnav_robot.sensor_reports import run_sensor_experiment
from faultnav_robot.sensors import (
    RobotGeometry,
    SensorFaultConfig,
    SensorNoiseConfig,
    SensorSimulationConfig,
    TimeWindow,
    body_twist_to_wheel_rates,
    calculate_sensor_metrics,
    sensor_profile,
    simulate_sensors,
)


def test_body_twist_to_wheel_rates_matches_differential_drive_equations() -> None:
    geometry = RobotGeometry(wheel_radius_m=0.1, wheel_separation_m=0.4)

    left_rad_s, right_rad_s = body_twist_to_wheel_rates(1.0, 0.5, geometry)

    assert isclose(left_rad_s, 9.0)
    assert isclose(right_rad_s, 11.0)


def test_ideal_measurements_match_truth_without_noise_or_faults() -> None:
    truth = simulate_scenario(straight_scenario(), integration_step_s=0.1)
    samples = simulate_sensors(
        truth,
        SensorSimulationConfig(
            geometry=RobotGeometry(encoder_counts_per_revolution=100_000),
        ),
    )

    assert all(
        isclose(sample.measured_left_wheel_rad_s, sample.ideal_left_wheel_rad_s)
        for sample in samples
    )
    assert all(
        isclose(sample.measured_right_wheel_rad_s, sample.ideal_right_wheel_rad_s)
        for sample in samples
    )
    assert all(
        sample.imu_yaw_rate_rad_s is not None
        and isclose(sample.imu_yaw_rate_rad_s, sample.truth_angular_velocity_rad_s)
        for sample in samples
    )
    assert samples[-1].wheel_odom_x_m == pytest.approx(2.0, abs=1e-4)


def test_seeded_noise_is_repeatable() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    config = SensorSimulationConfig(
        noise=SensorNoiseConfig(
            wheel_velocity_std_rad_s=0.02,
            gyro_std_rad_s=0.01,
            acceleration_std_m_s2=0.03,
            seed=42,
        ),
    )

    first = simulate_sensors(truth, config)
    second = simulate_sensors(truth, config)

    assert first == second


def test_wheel_slip_increases_odometry_error() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    nominal = simulate_sensors(
        truth,
        SensorSimulationConfig(
            geometry=RobotGeometry(encoder_counts_per_revolution=100_000),
        ),
    )
    faulty = simulate_sensors(
        truth,
        SensorSimulationConfig(
            geometry=RobotGeometry(encoder_counts_per_revolution=100_000),
            faults=SensorFaultConfig(
                left_wheel_slip_fraction=0.25,
                right_wheel_slip_fraction=-0.05,
                wheel_slip_window=TimeWindow(6.0, 12.0),
            ),
        ),
    )

    nominal_metrics = calculate_sensor_metrics(nominal)
    faulty_metrics = calculate_sensor_metrics(faulty)

    assert faulty_metrics.wheel_position_rmse_m > nominal_metrics.wheel_position_rmse_m
    assert faulty_metrics.wheel_slip_duration_s == pytest.approx(6.0, abs=0.11)


def test_gyro_bias_dropout_and_outlier_are_reported() -> None:
    truth = simulate_scenario(figure_eight_scenario(), integration_step_s=0.1)
    samples = simulate_sensors(truth, sensor_profile("combined-faults", seed=7))
    metrics = calculate_sensor_metrics(samples)

    assert metrics.imu_dropout_duration_s == pytest.approx(2.0, abs=0.11)
    assert metrics.gyro_outlier_duration_s == pytest.approx(0.2, abs=0.11)
    assert metrics.max_abs_gyro_error_rad_s > 0.7
    assert any(sample.imu_yaw_rate_rad_s is None for sample in samples)


def test_invalid_time_window_is_rejected() -> None:
    with pytest.raises(ValueError, match="greater"):
        TimeWindow(2.0, 2.0)


def test_run_sensor_experiment_writes_artifacts(tmp_path: Path) -> None:
    scenario = figure_eight_scenario()
    metrics, artifacts = run_sensor_experiment(
        scenario,
        tmp_path,
        "combined-faults",
        sensor_profile("combined-faults", seed=7),
        integration_step_s=0.2,
    )

    assert set(artifacts) == {
        "sensor_csv",
        "sensor_metrics_json",
        "sensor_report_svg",
    }
    assert all(path.exists() for path in artifacts.values())

    stored = json.loads(artifacts["sensor_metrics_json"].read_text(encoding="utf-8"))
    assert stored["sample_count"] == metrics.sample_count
    assert stored["imu_dropout_duration_s"] == pytest.approx(2.0, abs=0.21)

    svg = artifacts["sensor_report_svg"].read_text(encoding="utf-8")
    assert "Trajectory comparison" in svg
    assert "IMU yaw-rate comparison" in svg
    assert "Wheel odometry" in svg
