"""Reporting and end-to-end comparison workflows for FaultNav estimators."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from html import escape
from math import hypot
from pathlib import Path
from typing import Callable

from faultnav_robot.estimators import (
    EstimatorMetrics,
    EstimatorSample,
    calculate_estimator_metrics,
    fault_aware_ekf_config,
    run_ekf,
    standard_ekf_config,
)
from faultnav_robot.experiments import simulate_scenario
from faultnav_robot.scenarios import MotionScenario
from faultnav_robot.sensors import (
    SensorMetrics,
    SensorSample,
    SensorSimulationConfig,
    calculate_sensor_metrics,
    simulate_sensors,
)


@dataclass(frozen=True)
class EstimatorComparisonMetrics:
    """Raw odometry, standard EKF, and fault-aware EKF metrics."""

    scenario: str
    sensor_profile: str
    integration_step_s: float
    raw_wheel_odometry: SensorMetrics
    standard_ekf: EstimatorMetrics
    fault_aware_ekf: EstimatorMetrics


def write_estimator_csv(
    sensor_samples: tuple[SensorSample, ...],
    standard_samples: tuple[EstimatorSample, ...],
    fault_aware_samples: tuple[EstimatorSample, ...],
    output_path: str | Path,
) -> Path:
    """Write aligned truth, raw odometry, and EKF results to one CSV file."""

    if not sensor_samples:
        raise ValueError("sensor_samples must not be empty")
    if not (
        len(sensor_samples) == len(standard_samples) == len(fault_aware_samples)
    ):
        raise ValueError("sensor and estimator sample counts must match")

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "time_s",
        "truth_x_m",
        "truth_y_m",
        "truth_yaw_rad",
        "raw_x_m",
        "raw_y_m",
        "raw_yaw_rad",
        "standard_x_m",
        "standard_y_m",
        "standard_yaw_rad",
        "standard_covariance_trace",
        "fault_aware_x_m",
        "fault_aware_y_m",
        "fault_aware_yaw_rad",
        "fault_aware_gyro_bias_rad_s",
        "fault_aware_covariance_trace",
        "wheel_velocity_nis",
        "wheel_yaw_rate_nis",
        "gyro_nis",
        "wheel_velocity_accepted",
        "wheel_yaw_rate_accepted",
        "gyro_accepted",
        "wheel_slip_active",
        "imu_dropout_active",
        "gyro_outlier_active",
    ]
    with destination.open("w", encoding="utf-8", newline="") as file_handle:
        writer = csv.DictWriter(file_handle, fieldnames=fieldnames)
        writer.writeheader()
        for sensor, standard, gated in zip(
            sensor_samples,
            standard_samples,
            fault_aware_samples,
            strict=True,
        ):
            writer.writerow(
                {
                    "time_s": sensor.time_s,
                    "truth_x_m": sensor.truth_x_m,
                    "truth_y_m": sensor.truth_y_m,
                    "truth_yaw_rad": sensor.truth_yaw_rad,
                    "raw_x_m": sensor.wheel_odom_x_m,
                    "raw_y_m": sensor.wheel_odom_y_m,
                    "raw_yaw_rad": sensor.wheel_odom_yaw_rad,
                    "standard_x_m": standard.estimated_x_m,
                    "standard_y_m": standard.estimated_y_m,
                    "standard_yaw_rad": standard.estimated_yaw_rad,
                    "standard_covariance_trace": standard.covariance_trace,
                    "fault_aware_x_m": gated.estimated_x_m,
                    "fault_aware_y_m": gated.estimated_y_m,
                    "fault_aware_yaw_rad": gated.estimated_yaw_rad,
                    "fault_aware_gyro_bias_rad_s": gated.estimated_gyro_bias_rad_s,
                    "fault_aware_covariance_trace": gated.covariance_trace,
                    "wheel_velocity_nis": gated.wheel_velocity_nis,
                    "wheel_yaw_rate_nis": gated.wheel_yaw_rate_nis,
                    "gyro_nis": gated.gyro_nis,
                    "wheel_velocity_accepted": gated.wheel_velocity_accepted,
                    "wheel_yaw_rate_accepted": gated.wheel_yaw_rate_accepted,
                    "gyro_accepted": gated.gyro_accepted,
                    "wheel_slip_active": sensor.wheel_slip_active,
                    "imu_dropout_active": sensor.imu_dropout_active,
                    "gyro_outlier_active": sensor.gyro_outlier_active,
                }
            )
    return destination


def write_estimator_metrics_json(
    metrics: EstimatorComparisonMetrics,
    output_path: str | Path,
) -> Path:
    """Write nested estimator comparison metrics to JSON."""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as file_handle:
        json.dump(asdict(metrics), file_handle, indent=2, sort_keys=True)
        file_handle.write("\n")
    return destination


def _polyline(
    x_values: list[float],
    y_values: list[float],
    project_x: Callable[[float], float],
    project_y: Callable[[float], float],
) -> str:
    return " ".join(
        f"{project_x(x_value):.2f},{project_y(y_value):.2f}"
        for x_value, y_value in zip(x_values, y_values, strict=True)
    )


def _fault_rectangles(
    sensor_samples: tuple[SensorSample, ...],
    project_x: Callable[[float], float],
    top: float,
    height: float,
) -> str:
    rectangles: list[str] = []
    for sample in sensor_samples[1:]:
        transient_fault = (
            sample.wheel_slip_active
            or sample.imu_dropout_active
            or sample.gyro_outlier_active
        )
        if not transient_fault:
            continue
        start_x = project_x(sample.time_s - sample.dt_s)
        width = max(project_x(sample.time_s) - start_x, 0.5)
        rectangles.append(
            f'<rect x="{start_x:.2f}" y="{top:.2f}" width="{width:.2f}" '
            f'height="{height:.2f}" fill="#c0392b" opacity="0.10"/>'
        )
    return "\n".join(rectangles)


def write_estimator_report_svg(
    scenario: MotionScenario,
    profile_name: str,
    sensor_samples: tuple[SensorSample, ...],
    standard_samples: tuple[EstimatorSample, ...],
    fault_aware_samples: tuple[EstimatorSample, ...],
    metrics: EstimatorComparisonMetrics,
    output_path: str | Path,
) -> Path:
    """Generate trajectory, error, and NIS comparison panels as SVG."""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    width = 1280
    height = 820

    path_left = 65
    path_top = 120
    path_width = 650
    path_height = 560
    right_left = 770
    right_width = 450
    error_top = 120
    error_height = 245
    nis_top = 435
    nis_height = 245

    truth_x = [sample.truth_x_m for sample in sensor_samples]
    truth_y = [sample.truth_y_m for sample in sensor_samples]
    raw_x = [sample.wheel_odom_x_m for sample in sensor_samples]
    raw_y = [sample.wheel_odom_y_m for sample in sensor_samples]
    standard_x = [sample.estimated_x_m for sample in standard_samples]
    standard_y = [sample.estimated_y_m for sample in standard_samples]
    gated_x = [sample.estimated_x_m for sample in fault_aware_samples]
    gated_y = [sample.estimated_y_m for sample in fault_aware_samples]
    all_x = truth_x + raw_x + standard_x + gated_x
    all_y = truth_y + raw_y + standard_y + gated_y
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    span = max(x_max - x_min, y_max - y_min, 1e-6)
    padding = 0.10 * span
    x_min -= padding
    x_max += padding
    y_min -= padding
    y_max += padding

    def path_project_x(value: float) -> float:
        return path_left + (value - x_min) / (x_max - x_min) * path_width

    def path_project_y(value: float) -> float:
        return path_top + (y_max - value) / (y_max - y_min) * path_height

    times = [sample.time_s for sample in sensor_samples]
    time_max = max(times[-1], 1e-6)

    def time_project_x(value: float) -> float:
        return right_left + value / time_max * right_width

    raw_errors = [
        hypot(
            sample.wheel_odom_x_m - sample.truth_x_m,
            sample.wheel_odom_y_m - sample.truth_y_m,
        )
        for sample in sensor_samples
    ]
    standard_errors = [
        hypot(
            sample.estimated_x_m - sample.truth_x_m,
            sample.estimated_y_m - sample.truth_y_m,
        )
        for sample in standard_samples
    ]
    gated_errors = [
        hypot(
            sample.estimated_x_m - sample.truth_x_m,
            sample.estimated_y_m - sample.truth_y_m,
        )
        for sample in fault_aware_samples
    ]
    error_max = max(raw_errors + standard_errors + gated_errors + [1e-6])

    def error_project_y(value: float) -> float:
        return error_top + error_height - value / error_max * error_height

    nis_values = [
        max(
            sample.wheel_velocity_nis,
            sample.wheel_yaw_rate_nis,
            sample.gyro_nis or 0.0,
        )
        for sample in fault_aware_samples
    ]
    displayed_nis_max = max(12.0, min(max(nis_values + [12.0]), 80.0))

    def nis_project_y(value: float) -> float:
        clipped = min(max(value, 0.0), displayed_nis_max)
        return nis_top + nis_height - clipped / displayed_nis_max * nis_height

    fault_error_rectangles = _fault_rectangles(
        sensor_samples,
        time_project_x,
        error_top,
        error_height,
    )
    fault_nis_rectangles = _fault_rectangles(
        sensor_samples,
        time_project_x,
        nis_top,
        nis_height,
    )
    rejected_points = "\n".join(
        f'<circle cx="{time_project_x(sample.time_s):.2f}" '
        f'cy="{nis_project_y(max(sample.wheel_velocity_nis, sample.wheel_yaw_rate_nis, sample.gyro_nis or 0.0)):.2f}" '
        f'r="3.2" fill="#c0392b"/>'
        for sample in fault_aware_samples[1:]
        if (
            not sample.wheel_velocity_accepted
            or not sample.wheel_yaw_rate_accepted
            or (
                sample.imu_yaw_rate_rad_s is not None
                and not sample.gyro_accepted
            )
        )
    )

    title = escape(f"FaultNav estimator comparison — {scenario.name} / {profile_name}")
    description = escape(
        "Ground truth, raw wheel odometry, standard EKF, and innovation-gated EKF."
    )
    threshold_y = nis_project_y(9.0)
    gated_metrics = metrics.fault_aware_ekf

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">
  <title id="title">{title}</title>
  <desc id="desc">{description}</desc>
  <rect width="100%" height="100%" fill="#ffffff"/>
  <text x="65" y="42" font-family="Arial, sans-serif" font-size="27" font-weight="700" fill="#17202a">{title}</text>
  <text x="65" y="74" font-family="Arial, sans-serif" font-size="15" fill="#566573">{description}</text>

  <text x="{path_left}" y="103" font-family="Arial, sans-serif" font-size="17" font-weight="700" fill="#17202a">Trajectory</text>
  <rect x="{path_left}" y="{path_top}" width="{path_width}" height="{path_height}" fill="#f8f9f9" stroke="#d5d8dc"/>
  <polyline points="{_polyline(truth_x, truth_y, path_project_x, path_project_y)}" fill="none" stroke="#21618c" stroke-width="4"/>
  <polyline points="{_polyline(raw_x, raw_y, path_project_x, path_project_y)}" fill="none" stroke="#d68910" stroke-width="3"/>
  <polyline points="{_polyline(standard_x, standard_y, path_project_x, path_project_y)}" fill="none" stroke="#7f8c8d" stroke-width="2.5"/>
  <polyline points="{_polyline(gated_x, gated_y, path_project_x, path_project_y)}" fill="none" stroke="#1e8449" stroke-width="3"/>

  <text x="{right_left}" y="103" font-family="Arial, sans-serif" font-size="17" font-weight="700" fill="#17202a">Position error</text>
  <rect x="{right_left}" y="{error_top}" width="{right_width}" height="{error_height}" fill="#f8f9f9" stroke="#d5d8dc"/>
  {fault_error_rectangles}
  <polyline points="{_polyline(times, raw_errors, time_project_x, error_project_y)}" fill="none" stroke="#d68910" stroke-width="2.5"/>
  <polyline points="{_polyline(times, standard_errors, time_project_x, error_project_y)}" fill="none" stroke="#7f8c8d" stroke-width="2.2"/>
  <polyline points="{_polyline(times, gated_errors, time_project_x, error_project_y)}" fill="none" stroke="#1e8449" stroke-width="2.8"/>
  <text x="{right_left + 8}" y="{error_top + 20}" font-family="Arial, sans-serif" font-size="12" fill="#566573">max {error_max:.2f} m</text>

  <text x="{right_left}" y="418" font-family="Arial, sans-serif" font-size="17" font-weight="700" fill="#17202a">Maximum scalar NIS and rejected updates</text>
  <rect x="{right_left}" y="{nis_top}" width="{right_width}" height="{nis_height}" fill="#f8f9f9" stroke="#d5d8dc"/>
  {fault_nis_rectangles}
  <line x1="{right_left}" y1="{threshold_y:.2f}" x2="{right_left + right_width}" y2="{threshold_y:.2f}" stroke="#922b21" stroke-width="1.5" stroke-dasharray="7,5"/>
  <polyline points="{_polyline(times, nis_values, time_project_x, nis_project_y)}" fill="none" stroke="#5b2c6f" stroke-width="2"/>
  {rejected_points}
  <text x="{right_left + 8}" y="{threshold_y - 7:.2f}" font-family="Arial, sans-serif" font-size="12" fill="#922b21">NIS gate = 9</text>

  <line x1="65" y1="724" x2="95" y2="724" stroke="#21618c" stroke-width="4"/><text x="105" y="729" font-family="Arial, sans-serif" font-size="14">Ground truth</text>
  <line x1="225" y1="724" x2="255" y2="724" stroke="#d68910" stroke-width="4"/><text x="265" y="729" font-family="Arial, sans-serif" font-size="14">Raw wheel odometry</text>
  <line x1="455" y1="724" x2="485" y2="724" stroke="#7f8c8d" stroke-width="4"/><text x="495" y="729" font-family="Arial, sans-serif" font-size="14">Standard EKF</text>
  <line x1="650" y1="724" x2="680" y2="724" stroke="#1e8449" stroke-width="4"/><text x="690" y="729" font-family="Arial, sans-serif" font-size="14">Fault-aware EKF</text>
  <rect x="890" y="713" width="24" height="14" fill="#c0392b" opacity="0.24"/><text x="924" y="727" font-family="Arial, sans-serif" font-size="14">Injected transient fault</text>

  <text x="65" y="770" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Fault-aware position RMSE: {gated_metrics.position_rmse_m:.3f} m</text>
  <text x="345" y="770" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Heading RMSE: {gated_metrics.heading_rmse_rad:.3f} rad</text>
  <text x="585" y="770" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Transient rejection rate: {100.0 * gated_metrics.transient_fault_rejection_rate:.1f}%</text>
  <text x="900" y="770" font-family="Arial, sans-serif" font-size="14" fill="#17202a">False rejections: {gated_metrics.nominal_false_rejections}</text>
  <text x="65" y="797" font-family="Arial, sans-serif" font-size="13" fill="#566573">Red circles mark rejected measurement updates. NIS values above the display range are clipped visually but retained in CSV output.</text>
</svg>
"""
    destination.write_text(svg, encoding="utf-8")
    return destination


def run_estimator_comparison(
    scenario: MotionScenario,
    output_dir: str | Path,
    profile_name: str,
    sensor_config: SensorSimulationConfig,
    integration_step_s: float = 0.1,
) -> tuple[EstimatorComparisonMetrics, dict[str, Path]]:
    """Run raw odometry, standard EKF, and fault-aware EKF comparisons."""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    truth_samples = simulate_scenario(scenario, integration_step_s=integration_step_s)
    sensor_samples = simulate_sensors(truth_samples, sensor_config)
    standard_samples = run_ekf(
        sensor_samples,
        sensor_config.geometry,
        standard_ekf_config(),
    )
    fault_aware_samples = run_ekf(
        sensor_samples,
        sensor_config.geometry,
        fault_aware_ekf_config(),
    )
    metrics = EstimatorComparisonMetrics(
        scenario=scenario.name,
        sensor_profile=profile_name,
        integration_step_s=integration_step_s,
        raw_wheel_odometry=calculate_sensor_metrics(sensor_samples),
        standard_ekf=calculate_estimator_metrics(standard_samples),
        fault_aware_ekf=calculate_estimator_metrics(fault_aware_samples),
    )
    stem = f"{scenario.name}_{profile_name}_estimator".replace("-", "_")
    artifacts = {
        "estimator_csv": write_estimator_csv(
            sensor_samples,
            standard_samples,
            fault_aware_samples,
            destination / f"{stem}.csv",
        ),
        "estimator_metrics_json": write_estimator_metrics_json(
            metrics,
            destination / f"{stem}_metrics.json",
        ),
        "estimator_report_svg": write_estimator_report_svg(
            scenario,
            profile_name,
            sensor_samples,
            standard_samples,
            fault_aware_samples,
            metrics,
            destination / f"{stem}_report.svg",
        ),
    }
    return metrics, artifacts
