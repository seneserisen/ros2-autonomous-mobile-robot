"""Command-line interface for deterministic FaultNav motion experiments."""

from __future__ import annotations

import argparse
from pathlib import Path

from faultnav_robot.experiments import run_experiment
from faultnav_robot.scenarios import available_scenarios, get_scenario


def build_parser() -> argparse.ArgumentParser:
    """Build the experiment CLI argument parser."""

    parser = argparse.ArgumentParser(
        description="Run a deterministic FaultNav trajectory experiment and export reports."
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
        help="Directory for CSV, JSON, and SVG artifacts.",
    )
    return parser


def main(args: list[str] | None = None) -> int:
    """Run one experiment and print its reproducible result summary."""

    parsed = build_parser().parse_args(args)
    scenario = get_scenario(parsed.scenario)
    metrics, artifacts = run_experiment(
        scenario,
        output_dir=parsed.output_dir,
        integration_step_s=parsed.step,
    )

    print(f"Scenario: {metrics.scenario}")
    print(f"Duration: {metrics.duration_s:.6f} s")
    print(f"Path length: {metrics.path_length_m:.6f} m")
    print(f"Closure error: {metrics.closure_error_m:.6e} m")
    for artifact_name, artifact_path in artifacts.items():
        print(f"{artifact_name}: {artifact_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
