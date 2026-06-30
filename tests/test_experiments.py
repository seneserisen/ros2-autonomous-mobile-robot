from __future__ import annotations

import json
from math import isclose, tau
from pathlib import Path

import pytest

from faultnav_robot.experiments import calculate_metrics, run_experiment, simulate_scenario
from faultnav_robot.scenarios import circle_scenario, figure_eight_scenario, straight_scenario


def test_straight_scenario_reaches_expected_pose() -> None:
    scenario = straight_scenario()
    samples = simulate_scenario(scenario, integration_step_s=0.3)
    metrics = calculate_metrics(scenario, samples, integration_step_s=0.3)

    assert isclose(metrics.duration_s, 4.0, abs_tol=1e-12)
    assert isclose(metrics.path_length_m, 2.0, abs_tol=1e-12)
    assert isclose(metrics.final_x_m, 2.0, abs_tol=1e-12)
    assert isclose(metrics.final_y_m, 0.0, abs_tol=1e-12)


def test_circle_closes_with_exact_integration() -> None:
    scenario = circle_scenario()
    samples = simulate_scenario(scenario, integration_step_s=0.07)
    metrics = calculate_metrics(scenario, samples, integration_step_s=0.07)

    assert isclose(metrics.duration_s, 2.0 * tau, abs_tol=1e-11)
    assert isclose(metrics.path_length_m, tau, abs_tol=1e-11)
    assert metrics.closure_error_m < 1e-11
    assert abs(metrics.final_yaw_rad) < 1e-11


def test_figure_eight_is_repeatable() -> None:
    scenario = figure_eight_scenario()

    first = simulate_scenario(scenario, integration_step_s=0.1)
    second = simulate_scenario(scenario, integration_step_s=0.1)

    assert first == second
    assert first[-1].travelled_distance_m == second[-1].travelled_distance_m


def test_rejects_invalid_integration_step() -> None:
    with pytest.raises(ValueError, match="positive"):
        simulate_scenario(straight_scenario(), integration_step_s=0.0)


def test_run_experiment_writes_complete_artifact_set(tmp_path: Path) -> None:
    scenario = figure_eight_scenario()
    metrics, artifacts = run_experiment(scenario, tmp_path, integration_step_s=0.2)

    assert set(artifacts) == {"metrics_json", "trajectory_csv", "trajectory_svg"}
    assert all(path.exists() for path in artifacts.values())

    stored_metrics = json.loads(artifacts["metrics_json"].read_text(encoding="utf-8"))
    assert stored_metrics["scenario"] == "figure-eight"
    assert isclose(stored_metrics["path_length_m"], metrics.path_length_m)

    csv_lines = artifacts["trajectory_csv"].read_text(encoding="utf-8").splitlines()
    assert csv_lines[0].startswith("time_s,segment,linear_velocity_m_s")
    assert len(csv_lines) == metrics.sample_count + 1

    svg = artifacts["trajectory_svg"].read_text(encoding="utf-8")
    assert "FaultNav deterministic trajectory" in svg
    assert "<polyline" in svg
    assert "Closure error" in svg
