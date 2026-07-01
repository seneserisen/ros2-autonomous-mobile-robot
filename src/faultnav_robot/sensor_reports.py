"""CSV, JSON, and SVG reporting for FaultNav sensor experiments."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict
from html import escape
from pathlib import Path

from faultnav_robot.experiments import simulate_scenario
from faultnav_robot.scenarios import MotionScenario
from faultnav_robot.sensors import (
    SensorMetrics,
    SensorSample,
    SensorSimulationConfig,
    calculate_sensor_metrics,
    simulate_sensors,
)


def write_sensor_csv(samples: tuple[SensorSample, ...], output_path: str | Path) -> Path:
    """Write ground truth, sensor measurements, flags, and wheel odometry to CSV."""

    if not samples:
        raise ValueError("samples must not be empty")

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(asdict(samples[0]).keys())
    with destination.open("w", encoding="utf-8", newline="") as file_handle:
        writer = csv.DictWriter(file_handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in samples)
    return destination


def write_sensor_metrics_json(metrics: SensorMetrics, output_path: str | Path) -> Path:
    """Write sensor metrics to deterministic, human-readable JSON."""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as file_handle:
        json.dump(asdict(metrics), file_handle, indent=2, sort_keys=True)
        file_handle.write("\n")
    return destination


def _polyline_segments(
    x_values: list[float],
    y_values: list[float | None],
    project_x,
    project_y,
) -> list[str]:
    segments: list[str] = []
    current: list[str] = []
    for x_value, y_value in zip(x_values, y_values, strict=True):
        if y_value is None:
            if len(current) >= 2:
                segments.append(" ".join(current))
            current = []
            continue
        current.append(f"{project_x(x_value):.2f},{project_y(y_value):.2f}")
    if len(current) >= 2:
        segments.append(" ".join(current))
    return segments


def write_sensor_report_svg(
    scenario: MotionScenario,
    profile_name: str,
    samples: tuple[SensorSample, ...],
    metrics: SensorMetrics,
    output_path: str | Path,
) -> Path:
    """Generate a two-panel trajectory and gyroscope comparison report."""

    if not samples:
        raise ValueError("samples must not be empty")

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)

    width = 1180
    height = 660
    path_left = 70
    path_top = 125
    path_width = 620
    plot_height = 430
    gyro_left = 750
    gyro_width = 370

    truth_x = [sample.truth_x_m for sample in samples]
    truth_y = [sample.truth_y_m for sample in samples]
    odom_x = [sample.wheel_odom_x_m for sample in samples]
    odom_y = [sample.wheel_odom_y_m for sample in samples]
    all_x = truth_x + odom_x
    all_y = truth_y + odom_y
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    span = max(x_max - x_min, y_max - y_min, 1e-6)
    padding = 0.12 * span
    x_min -= padding
    x_max += padding
    y_min -= padding
    y_max += padding

    def path_project_x(value: float) -> float:
        return path_left + (value - x_min) / (x_max - x_min) * path_width

    def path_project_y(value: float) -> float:
        return path_top + (y_max - value) / (y_max - y_min) * plot_height

    truth_path = " ".join(
        f"{path_project_x(x_value):.2f},{path_project_y(y_value):.2f}"
        for x_value, y_value in zip(truth_x, truth_y, strict=True)
    )
    odom_path = " ".join(
        f"{path_project_x(x_value):.2f},{path_project_y(y_value):.2f}"
        for x_value, y_value in zip(odom_x, odom_y, strict=True)
    )

    times = [sample.time_s for sample in samples]
    truth_gyro = [sample.truth_angular_velocity_rad_s for sample in samples]
    measured_gyro = [sample.imu_yaw_rate_rad_s for sample in samples]
    finite_gyro = truth_gyro + [value for value in measured_gyro if value is not None]
    gyro_min = min(finite_gyro)
    gyro_max = max(finite_gyro)
    gyro_padding = 0.12 * max(gyro_max - gyro_min, 0.1)
    gyro_min -= gyro_padding
    gyro_max += gyro_padding
    time_max = max(times[-1], 1e-6)

    def gyro_project_x(value: float) -> float:
        return gyro_left + value / time_max * gyro_width

    def gyro_project_y(value: float) -> float:
        return path_top + (gyro_max - value) / (gyro_max - gyro_min) * plot_height

    truth_gyro_path = " ".join(
        f"{gyro_project_x(time_s):.2f},{gyro_project_y(value):.2f}"
        for time_s, value in zip(times, truth_gyro, strict=True)
    )
    measured_segments = _polyline_segments(
        times,
        measured_gyro,
        gyro_project_x,
        gyro_project_y,
    )

    slip_rectangles: list[str] = []
    dropout_rectangles: list[str] = []
    for sample in samples[1:]:
        rect_x = gyro_project_x(sample.time_s - sample.dt_s)
        rect_width = max(gyro_project_x(sample.time_s) - rect_x, 0.5)
        if sample.wheel_slip_active:
            slip_rectangles.append(
                f'<rect x="{rect_x:.2f}" y="{path_top}" width="{rect_width:.2f}" '
                f'height="{plot_height}" fill="#f5b041" opacity="0.13"/>'
            )
        if sample.imu_dropout_active:
            dropout_rectangles.append(
                f'<rect x="{rect_x:.2f}" y="{path_top}" width="{rect_width:.2f}" '
                f'height="{plot_height}" fill="#c0392b" opacity="0.12"/>'
            )

    measured_svg = "\n".join(
        f'  <polyline points="{points}" fill="none" stroke="#c0392b" '
        f'stroke-width="2.5" stroke-linejoin="round" stroke-linecap="round"/>'
        for points in measured_segments
    )
    title = escape(f"FaultNav sensor simulation — {scenario.name} / {profile_name}")
    description = escape(
        "Ground truth, encoder-derived wheel odometry, gyroscope faults, and dropout intervals."
    )

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">
  <title id="title">{title}</title>
  <desc id="desc">{description}</desc>
  <rect width="100%" height="100%" fill="#ffffff"/>
  <text x="70" y="42" font-family="Arial, sans-serif" font-size="26" font-weight="700" fill="#17202a">{title}</text>
  <text x="70" y="72" font-family="Arial, sans-serif" font-size="15" fill="#566573">{description}</text>

  <text x="{path_left}" y="108" font-family="Arial, sans-serif" font-size="17" font-weight="700" fill="#17202a">Trajectory comparison</text>
  <rect x="{path_left}" y="{path_top}" width="{path_width}" height="{plot_height}" fill="#f8f9f9" stroke="#d5d8dc"/>
  <polyline points="{truth_path}" fill="none" stroke="#21618c" stroke-width="4" stroke-linejoin="round" stroke-linecap="round"/>
  <polyline points="{odom_path}" fill="none" stroke="#d68910" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"/>
  <circle cx="{path_project_x(truth_x[0]):.2f}" cy="{path_project_y(truth_y[0]):.2f}" r="7" fill="#1e8449"/>
  <circle cx="{path_project_x(odom_x[-1]):.2f}" cy="{path_project_y(odom_y[-1]):.2f}" r="7" fill="#d68910"/>

  <text x="{gyro_left}" y="108" font-family="Arial, sans-serif" font-size="17" font-weight="700" fill="#17202a">IMU yaw-rate comparison</text>
  <rect x="{gyro_left}" y="{path_top}" width="{gyro_width}" height="{plot_height}" fill="#f8f9f9" stroke="#d5d8dc"/>
  {"".join(slip_rectangles)}
  {"".join(dropout_rectangles)}
  <polyline points="{truth_gyro_path}" fill="none" stroke="#566573" stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>
{measured_svg}

  <line x1="70" y1="590" x2="100" y2="590" stroke="#21618c" stroke-width="4"/>
  <text x="110" y="595" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Ground truth</text>
  <line x1="240" y1="590" x2="270" y2="590" stroke="#d68910" stroke-width="4"/>
  <text x="280" y="595" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Wheel odometry</text>
  <line x1="440" y1="590" x2="470" y2="590" stroke="#c0392b" stroke-width="4"/>
  <text x="480" y="595" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Measured gyro</text>
  <rect x="650" y="580" width="24" height="14" fill="#f5b041" opacity="0.35"/>
  <text x="684" y="593" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Wheel slip</text>
  <rect x="800" y="580" width="24" height="14" fill="#c0392b" opacity="0.28"/>
  <text x="834" y="593" font-family="Arial, sans-serif" font-size="14" fill="#17202a">IMU dropout</text>

  <text x="70" y="628" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Position RMSE: {metrics.wheel_position_rmse_m:.3f} m</text>
  <text x="300" y="628" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Heading RMSE: {metrics.wheel_heading_rmse_rad:.3f} rad</text>
  <text x="550" y="628" font-family="Arial, sans-serif" font-size="14" fill="#17202a">Final position error: {metrics.final_wheel_position_error_m:.3f} m</text>
  <text x="840" y="628" font-family="Arial, sans-serif" font-size="14" fill="#17202a">IMU dropout: {metrics.imu_dropout_duration_s:.2f} s</text>
</svg>
"""
    destination.write_text(svg, encoding="utf-8")
    return destination


def run_sensor_experiment(
    scenario: MotionScenario,
    output_dir: str | Path,
    profile_name: str,
    config: SensorSimulationConfig,
    integration_step_s: float = 0.05,
) -> tuple[SensorMetrics, dict[str, Path]]:
    """Run one ground-truth and sensor-fault experiment and export all artifacts."""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    truth_samples = simulate_scenario(scenario, integration_step_s=integration_step_s)
    sensor_samples = simulate_sensors(truth_samples, config)
    metrics = calculate_sensor_metrics(sensor_samples)
    stem = f"{scenario.name}_{profile_name}".replace("-", "_")
    artifacts = {
        "sensor_csv": write_sensor_csv(sensor_samples, destination / f"{stem}_sensor.csv"),
        "sensor_metrics_json": write_sensor_metrics_json(
            metrics,
            destination / f"{stem}_sensor_metrics.json",
        ),
        "sensor_report_svg": write_sensor_report_svg(
            scenario,
            profile_name,
            sensor_samples,
            metrics,
            destination / f"{stem}_sensor_report.svg",
        ),
    }
    return metrics, artifacts
