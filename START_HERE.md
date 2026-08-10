# Start Here — FaultNav

FaultNav is a deterministic mobile-robot simulation. The demonstration drives a virtual robot
through a figure-eight while simulated wheel-encoder and IMU faults are active, then creates a
visual comparison report. It does not command hardware and does not require ROS 2 for this demo.

## Windows: run the demonstration

Prerequisite: install Python 3.10, 3.11, or 3.12 from
[python.org](https://www.python.org/downloads/). During installation, enable **Add Python to PATH**.

1. Double-click `SETUP.bat` once. It creates a private `.venv` environment inside this project and
   installs the documented Python dependencies.
2. Double-click `RUN.bat`.
3. The generated SVG report opens in your default web browser. If it does not, open
   `artifacts\demo\figure_eight_combined_faults_sensor_report.svg` manually.

`RUN.bat` also performs setup automatically when the environment is missing or outdated. Running
`SETUP.bat` repeatedly is safe: an unchanged, working environment is reused.

Windows launchers pause at the end so double-click users can read the result. Developers and
automation can set `FAULTNAV_NO_PAUSE=1`; `FAULTNAV_NO_OPEN=1` also suppresses automatic report
opening.

## What you should see

The terminal reports deterministic wheel-odometry error metrics and creates:

```text
artifacts/demo/
├── figure_eight_combined_faults_sensor.csv
├── figure_eight_combined_faults_sensor_metrics.json
└── figure_eight_combined_faults_sensor_report.svg
```

The SVG compares ground truth with encoder-derived wheel odometry and shows the simulated IMU yaw
rate. These are controlled simulation results, not physical-robot measurements or localization
accuracy claims.

## Something went wrong?

Double-click `DOCTOR.bat`. It checks Python, the local environment, package imports, dependencies,
the installed CLI, output permissions, and—when available—Git, the current branch, `origin`, and
GitHub reachability.

Common fixes:

- **Python was not found:** install Python and enable **Add Python to PATH**, then rerun `SETUP.bat`.
- **Virtual environment or package failed:** rerun `SETUP.bat` while connected to the internet.
- **GitHub connection warning:** the Python demo can still run offline after setup.
- **Local changes warning:** review changes in GitHub Desktop before committing or pushing.

The launchers do not request administrator privileges, install system software, or publish Git
changes.

## Linux/macOS

From a terminal in the repository root:

```bash
sh setup.sh
sh run.sh
sh doctor.sh
sh test.sh
```

Python 3.10–3.12 is recommended. The first setup requires internet access to download declared
Python packages. ROS 2 remains a separate advanced workflow.

## Run the tests

- Windows: double-click `TEST.bat`.
- Linux/macOS: run `sh test.sh`.

The test launcher runs Ruff and pytest. It does not run ROS 2, simulator, or hardware checks.

## Advanced and developer usage

The original interfaces remain available. After setup, activate `.venv` and use:

```bash
faultnav-experiment --scenario figure-eight --step 0.1 --sensor-profile nominal --seed 7 --output-dir artifacts/nominal
ruff check src tests scripts setup.py launch
pytest
```

For ROS 2 workspace instructions, model assumptions, and complete verification commands, see
[`README.md`](README.md), [`docs/architecture.md`](docs/architecture.md), and
[`docs/testing.md`](docs/testing.md).

## Local and GitHub workflow

GitHub Desktop sees saved local changes immediately, but GitHub.com changes only after you review,
commit, and explicitly push them. The recommended workflow is:

```text
Fetch/Pull → work locally → run TEST → inspect changes → commit → Push origin
```

No FaultNav launcher commits, pushes, merges, releases, or deploys anything.
