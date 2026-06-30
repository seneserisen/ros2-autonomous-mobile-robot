"""Deterministic motion experiments, metrics, and lightweight report generation."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass
from html import escape
from math import hypot, isfinite
from pathlib import Path

from faultnav_robot.differential_drive import RobotState, integrate_twist
from faultnav_robot.scenarios import MotionScenario


@dataclass(frozen=True)
class ExperimentSample:
    """One timestamped command and resulting robot state."""

    time_s: float
    segment: str
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    travelled_distance_m: float


@dataclass(frozen=True)
class ExperimentMetrics:
    """Repeatable engineering metrics derived from a simulated scenario."""

    scenario: str
    duration_s: float
    integration_step_s: float
    sample_count: int
    path_length_m: float
    net_displacement_m: float
    final_x_m: float
    final_y_m: float
    final_yaw_rad: float
    closure_error_m: float


def simulate_scenario(
    scenario: MotionScenario,
    integration_step_s: float = 0.05,
    initial_state: RobotState = RobotState(),
) -> tuple[ExperimentSample, ...]:
    """Execute a scenario with exact segment-boundary handling.

    The output is deterministic because no wall-clock timing or random source is used. A shortened
    final integration step is applied whenever a fixed step would cross a segment boundary.
    """

    if not isfinite(integration_step_s) or integration_step_s <= 0.0:
        raise ValueError("integration_step_s must be finite and positive")

    state = initial_state
    elapsed_s = 0.0
    travelled_distance_m = 0.0
    samples = [
        ExperimentSample(
            time_s=0.0,
            segment="initial_state",
            linear_velocity_m_s=0.0,
            angular_velocity_rad_s=0.0,
            x_m=state.x_m,
            y_m=state.y_m,
            yaw_rad=state.yaw_rad,
            travelled_distance_m=0.0,
        )
    ]

    for segment in scenario.segments:
        remaining_s = segment.duration_s
        while remaining_s > 1e-12:
            step_s = min(integration_step_s, remaining_s)
            state = integrate_twist(
                state,
                segment.linear_velocity_m_s,
                segment.angular_velocity_rad_s,
                step_s,
            )
            elapsed_s += step_s
            travelled_distance_m += abs(segment.linear_velocity_m_s) * step_s
            remaining_s -= step_s
            samples.append(
                ExperimentSample(
                    time_s=elapsed_s,
                    segment=segment.label,
                    linear_velocity_m_s=segment.linear_velocity_m_s,
                    angular_velocity_rad_s=segment.angular_velocity_rad_s,
                    x_m=state.x_m,
                    y_m=state.y_m,
                    yaw_rad=state.yaw_rad,
                    travelled_distance_m=travelled_distance_m,
                )
            )

    return tuple(samples)


def calculate_metrics(
    scenario: MotionScenario,
    samples: tuple[ExperimentSample, ...],
    integration_step_s: float,
) -> ExperimentMetrics:
    """Calculate summary metrics from a completed experiment."""

    if not samples:
        raise ValueError("samples must not be empty")

    first = samples[0]
    final = samples[-1]
    net_displacement_m = hypot(final.x_m - first.x_m, final.y_m - first.y_m)
    return ExperimentMetrics(
        scenario=scenario.name,
        duration_s=final.time_s,
        integration_step_s=integration_step_s,
        sample_count=len(samples),
        path_length_m=final.travelled_distance_m,
        net_displacement_m=net_displacement_m,
        final_x_m=final.x_m,
        final_y_m=final.y_m,
        final_yaw_rad=final.yaw_rad,
        closure_error_m=net_displacement_m,
    )


def write_samples_csv(samples: tuple[ExperimentSample, ...], output_path: str | Path) -> Path:
    """Write all timestamped experiment samples to CSV."""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(asdict(samples[0]).keys())
    with destination.open("w", encoding="utf-8", newline="") as file_handle:
        writer = csv.DictWriter(file_handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(asdict(sample) for sample in samples)
    return destination


def write_metrics_json(metrics: ExperimentMetrics, output_path: str | Path) -> Path:
    """Write metrics in a stable, human-readable JSON representation."""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("w", encoding="utf-8") as file_handle:
        json.dump(asdict(metrics), file_handle, indent=2, sort_keys=True)
        file_handle.write("\n")
    return destination


def write_trajectory_svg(
    scenario: MotionScenario,
    samples: tuple[ExperimentSample, ...],
    metrics: ExperimentMetrics,
    output_path: str | Path,
) -> Path:
    """Generate a dependency-free SVG trajectory report suitable for GitHub rendering."""

    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)

    width = 960
    height = 600
    margin_left = 90
    margin_right = 50
    margin_top = 105
    margin_bottom = 85
    plot_width = width - margin_left - margin_right
    plot_height = height - margin_top - margin_bottom

    x_values = [sample.x_m for sample in samples]
    y_values = [sample.y_m for sample in samples]
    x_min, x_max = min(x_values), max(x_values)
    y_min, y_max = min(y_values), max(y_values)
    x_span = max(x_max - x_min, 1e-6)
    y_span = max(y_max - y_min, 1e-6)
    padding = 0.12 * max(x_span, y_span)
    x_min -= padding
    x_max += padding
    y_min -= padding
    y_max += padding
    x_span = x_max - x_min
    y_span = y_max - y_min

    def project_x(value: float) -> float:
        return margin_left + (value - x_min) / x_span * plot_width

    def project_y(value: float) -> float:
        return margin_top + (y_max - value) / y_span * plot_height

    points = " ".join(f"{project_x(x_value):.2f},{project_y(y_value):.2f}" for x_value, y_value in zip(x_values, y_values, strict=True))
    start_x, start_y = project_x(x_values[0]), project_y(y_values[0])
    end_x, end_y = project_x(x_values[-1]), project_y(y_values[-1])

    x_axis_y = project_y(0.0) if y_min <= 0.0 <= y_max else margin_top + plot_height
    y_axis_x = project_x(0.0) if x_min <= 0.0 <= x_max else margin_left
    title = escape(f"FaultNav deterministic trajectory — {scenario.name}")
    description = escape(scenario.description)

    svg = f"""<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-labelledby="title desc">
  <title id="title">{title}</title>
  <desc id="desc">{description}</desc>
  <rect width="100%" height="100%" fill="#ffffff"/>
  <text x="{margin_left}" y="42" font-family="Arial, sans-serif" font-size="26" font-weight="700" fill="#17202a">{title}</text>
  <text x="{margin_left}" y="70" font-family="Arial, sans-serif" font-size="15" fill="#4d5656">{description}</text>
  <rect x="{margin_left}" y="{margin_top}" width="{plot_width}" height="{plot_height}" fill="#f8f9f9" stroke="#d5d8dc"/>
  <line x1="{margin_left}" y1="{x_axis_y:.2f}" x2="{margin_left + plot_width}" y2="{x_axis_y:.2f}" stroke="#aab7b8" stroke-width="1"/>
  <line x1="{y_axis_x:.2f}" y1="{margin_top}" x2="{y_axis_x:.2f}" y2="{margin_top + plot_height}" stroke="#aab7b8" stroke-width="1"/>
  <polyline points="{points}" fill="none" stroke="#21618c" stroke-width="4" stroke-linejoin="round" stroke-linecap="round"/>
  <circle cx="{start_x:.2f}" cy="{start_y:.2f}" r="8" fill="#1e8449"/>
  <circle cx="{end_x:.2f}" cy="{end_y:.2f}" r="8" fill="#c0392b"/>
  <text x="{margin_left}" y="{height - 48}" font-family="Arial, sans-serif" font-size="15" fill="#17202a">Path length: {metrics.path_length_m:.3f} m</text>
  <text x="{margin_left + 225}" y="{height - 48}" font-family="Arial, sans-serif" font-size="15" fill="#17202a">Duration: {metrics.duration_s:.3f} s</text>
  <text x="{margin_left + 420}" y="{height - 48}" font-family="Arial, sans-serif" font-size="15" fill="#17202a">Closure error: {metrics.closure_error_m:.3e} m</text>
  <text x="{margin_left}" y="{height - 20}" font-family="Arial, sans-serif" font-size="13" fill="#566573">Green: start · Red: end · Exact constant-twist integration</text>
</svg>
"""
    destination.write_text(svg, encoding="utf-8")
    return destination


def run_experiment(
    scenario: MotionScenario,
    output_dir: str | Path,
    integration_step_s: float = 0.05,
) -> tuple[ExperimentMetrics, dict[str, Path]]:
    """Simulate one scenario and generate CSV, JSON, and SVG artifacts."""

    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    samples = simulate_scenario(scenario, integration_step_s=integration_step_s)
    metrics = calculate_metrics(scenario, samples, integration_step_s)
    stem = scenario.name.replace("-", "_")
    artifacts = {
        "trajectory_csv": write_samples_csv(samples, destination / f"{stem}_trajectory.csv"),
        "metrics_json": write_metrics_json(metrics, destination / f"{stem}_metrics.json"),
        "trajectory_svg": write_trajectory_svg(
            scenario,
            samples,
            metrics,
            destination / f"{stem}_trajectory.svg",
        ),
    }
    return metrics, artifacts
