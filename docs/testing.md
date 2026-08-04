# FaultNav Testing and Verification

Use these commands as the repository source of truth. Do not report a check as passed unless it actually ran successfully.

## Supported environment

- Python: 3.10, 3.11, and 3.12 in CI
- Package type: Python package and ROS 2 `ament_python` package
- Core validation: can run without ROS 2
- ROS runtime validation: requires a compatible ROS 2 environment

## Local Python setup

From the repository root:

```bash
python -m venv .venv
```

Activate the environment:

```bash
# Linux/macOS
source .venv/bin/activate

# Windows PowerShell
.venv\Scripts\Activate.ps1
```

Install the package and development dependencies:

```bash
python -m pip install --upgrade pip
python -m pip install -e . -r dev-requirements.txt
```

## Fast validation

Run after a focused local change:

```bash
ruff check src tests setup.py launch
pytest
```

## Installed CLI validation

Validate the ideal-motion workflow:

```bash
faultnav-experiment \
  --scenario figure-eight \
  --step 0.2 \
  --output-dir artifacts/motion-check
```

Expected non-empty outputs:

```text
artifacts/motion-check/figure_eight_trajectory.csv
artifacts/motion-check/figure_eight_metrics.json
artifacts/motion-check/figure_eight_trajectory.svg
```

Validate the combined sensor-fault workflow:

```bash
faultnav-experiment \
  --scenario figure-eight \
  --step 0.2 \
  --sensor-profile combined-faults \
  --seed 7 \
  --output-dir artifacts/sensor-check
```

Expected non-empty outputs:

```text
artifacts/sensor-check/figure_eight_combined_faults_sensor.csv
artifacts/sensor-check/figure_eight_combined_faults_sensor_metrics.json
artifacts/sensor-check/figure_eight_combined_faults_sensor_report.svg
```

## Reproducibility check

Run the same seeded sensor experiment twice into different directories and compare the generated CSV and JSON files.

```bash
faultnav-experiment --scenario figure-eight --step 0.1 --sensor-profile nominal --seed 7 --output-dir artifacts/repro-a
faultnav-experiment --scenario figure-eight --step 0.1 --sensor-profile nominal --seed 7 --output-dir artifacts/repro-b
```

On Linux/macOS:

```bash
cmp artifacts/repro-a/figure_eight_nominal_sensor.csv artifacts/repro-b/figure_eight_nominal_sensor.csv
cmp artifacts/repro-a/figure_eight_nominal_sensor_metrics.json artifacts/repro-b/figure_eight_nominal_sensor_metrics.json
```

On Windows PowerShell:

```powershell
Compare-Object (Get-Content artifacts/repro-a/figure_eight_nominal_sensor.csv) (Get-Content artifacts/repro-b/figure_eight_nominal_sensor.csv)
Compare-Object (Get-Content artifacts/repro-a/figure_eight_nominal_sensor_metrics.json) (Get-Content artifacts/repro-b/figure_eight_nominal_sensor_metrics.json)
```

No differences are expected for identical configuration and seed.

## Manual engineering checks

| Scenario | Check | Expected result |
|---|---|---|
| Ideal figure-eight | Run the motion CLI with `step=0.2` | Path returns to the initial position within documented floating-point tolerance |
| Seeded repeatability | Repeat identical sensor experiment | CSV and JSON outputs match |
| Combined faults | Generate stress-test report | Errors and fault durations are non-zero and artifacts are readable |
| Invalid input | Use an invalid scenario, non-positive step, or invalid geometry | Command fails clearly rather than silently using a default |
| Stale ROS command | Stop publishing `cmd_vel` beyond timeout | Published velocity falls to zero and integration stops |

## ROS 2 workspace validation

These commands require a compatible ROS 2 installation:

```bash
mkdir -p ~/faultnav_ws/src
cd ~/faultnav_ws/src
git clone https://github.com/seneserisen/ros2-autonomous-mobile-robot.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch faultnav_robot faultnav.launch.py
```

In a separate sourced terminal:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
ros2 topic echo /odom
```

ROS 2 runtime checks must not be marked complete when only the Python core was tested.

## Test strategy

- Unit tests: kinematics, scenarios, sensors, metrics, validation, and failure behavior.
- Integration tests: installed CLI and generated artifacts.
- Regression tests: every corrected defect where practical.
- Numerical reference tests: analytical constant-twist cases and independently calculated wheel geometry.
- ROS tests: topics, frames, parameters, timeout behavior, and launch configuration when a ROS environment is available.
- Future estimator tests: covariance properties, innovation statistics, measurement rejection, and deterministic comparisons.

## CI

Workflow: `.github/workflows/python-core.yml`

CI currently performs:

- Python 3.10–3.12 matrix installation;
- Ruff linting;
- pytest;
- ideal-motion CLI artifact checks;
- combined-fault CLI artifact checks.

CI does not currently prove:

- ROS 2 runtime compatibility;
- physics-simulator behavior;
- hardware behavior;
- real-world accuracy;
- security certification.

## Baseline failures

Record any existing failures in `docs/project_status.md` before attributing them to a new change.
