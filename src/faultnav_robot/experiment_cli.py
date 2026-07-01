"""Command-line interface for deterministic FaultNav experiments."""

from __future__ import annotations

import argparse
from pathlib import Path

from faultnav_robot.estimator_reports import run_estimator_comparison
from faultnav_robot.experiments import run_experiment
from faultnav_robot.scenarios import available_scenarios, get_scenario
from faultnav_robot.sensor_reports import run_sensor_experiment
from faultnav_robot.sensors import sensor_profile

SENSOR_PROFILES = ("none", "nominal", "wheel-slip", "gyro-bias", "combined-faults")
ESTIMATOR_MODES = ("none", "compare")


def build_parser() -> argparse.ArgumentParser:
    """Build the experiment CLI argument parser."""

    parser = argparse.ArgumentParser(
        description="Run deterministic FaultNav motion, sensor, or estimator experiments."
    )
    parser.add_argument(
        "--scenario",
        choices=sorted(available_scenarios()),
        default="figure-eight",
        help="Built-in motion scenario to execute.",
    )
    parser.add_argument(
        "--step",
        type=float,
        default=0.05,
        help="Integration step in seconds. Segment boundaries remain exact.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("artifacts") / "motion-experiments",
        help="Directory for generated CSV, JSON, and SVG artifacts.",
    )
    parser.add_argument(
        "--sensor-profile",
        choices=SENSOR_PROFILES,
        default="none",
        help="Optional deterministic encoder and IMU profile.",
    )
    parser.add_argument(
        "--estimator",
        choices=ESTIMATOR_MODES,
        default="none",
        help="Run raw odometry, standard EKF, and fault-aware EKF comparison.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=7,
        help="Random seed used by sensor-noise profiles.",
    )
    return parser


def main(args: list[str] | None = None) -> int:
    """Run one experiment and print its reproducible result summary."""

    parsed = build_parser().parse_args(args)
    scenario = get_scenario(parsed.scenario)

    if parsed.estimator == "compare":
        if parsed.sensor_profile == "none":
            raise SystemExit("--estimator compare requires a non-'none' --sensor-profile")
        metrics, artifacts = run_estimator_comparison(
            scenario,
            output_dir=parsed.output_dir,
            profile_name=parsed.sensor_profile,
            sensor_config=sensor_profile(parsed.sensor_profile, seed=parsed.seed),
            integration_step_s=parsed.step,
        )
        print(f"Scenario: {scenario.name}")
        print(f"Sensor profile: {parsed.sensor_profile}")
        print(f"Seed: {parsed.seed}")
        print(
            "Raw wheel position RMSE: "
            f"{metrics.raw_wheel_odometry.wheel_position_rmse_m:.6f} m"
        )
        print(f"Standard EKF position RMSE: {metrics.standard_ekf.position_rmse_m:.6f} m")
        print(
            "Fault-aware EKF position RMSE: "
            f"{metrics.fault_aware_ekf.position_rmse_m:.6f} m"
        )
        print(
            "Fault-aware EKF heading RMSE: "
            f"{metrics.fault_aware_ekf.heading_rmse_rad:.6f} rad"
        )
        print(
            "Transient-fault rejection rate: "
            f"{100.0 * metrics.fault_aware_ekf.transient_fault_rejection_rate:.1f}%"
        )
        print(
            "Healthy-period false rejections: "
            f"{metrics.fault_aware_ekf.nominal_false_rejections}"
        )
    elif parsed.sensor_profile == "none":
        metrics, artifacts = run_experiment(
            scenario,
            output_dir=parsed.output_dir,
            integration_step_s=parsed.step,
        )
        print(f"Scenario: {metrics.scenario}")
        print(f"Duration: {metrics.duration_s:.6f} s")
        print(f"Path length: {metrics.path_length_m:.6f} m")
        print(f"Closure error: {metrics.closure_error_m:.6e} m")
    else:
        metrics, artifacts = run_sensor_experiment(
            scenario,
            output_dir=parsed.output_dir,
            profile_name=parsed.sensor_profile,
            config=sensor_profile(parsed.sensor_profile, seed=parsed.seed),
            integration_step_s=parsed.step,
        )
        print(f"Scenario: {scenario.name}")
        print(f"Sensor profile: {parsed.sensor_profile}")
        print(f"Seed: {parsed.seed}")
        print(f"Wheel position RMSE: {metrics.wheel_position_rmse_m:.6f} m")
        print(f"Wheel heading RMSE: {metrics.wheel_heading_rmse_rad:.6f} rad")
        print(f"Final wheel position error: {metrics.final_wheel_position_error_m:.6f} m")
        print(f"IMU dropout duration: {metrics.imu_dropout_duration_s:.3f} s")

    for artifact_name, artifact_path in artifacts.items():
        print(f"{artifact_name}: {artifact_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
