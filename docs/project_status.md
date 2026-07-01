# FaultNav Project Status

- Last updated: 2 July 2026
- Current maturity: Portfolio MVP in active development
- Current documented release: `v0.3.0`
- Default branch: `main`
- Latest validated implementation commit before governance setup: `c6ce39571ff4315dc20bcd1c13a43f07c079ffa1`

## Implemented and documented

- exact planar differential-drive/unicycle integration;
- typed straight, circle, square, and figure-eight motion scenarios;
- deterministic CSV, JSON, and SVG experiment artifacts;
- wheel-rate conversion and encoder quantisation;
- encoder-derived wheel odometry;
- seeded encoder, gyroscope, and accelerometer noise;
- encoder scale error and asymmetric wheel-slip simulation;
- gyro bias, IMU dropout, and gyro outlier simulation;
- error and fault-duration metrics;
- ROS 2 command subscription, odometry publication, TF, parameters, launch file, and command timeout;
- Python unit and integration tests;
- Ruff linting and Python 3.10–3.12 GitHub Actions validation.

## Current limitations

- ROS odometry still integrates commanded motion rather than simulated encoder measurements.
- ROS covariance values remain placeholders.
- No Extended Kalman Filter or innovation monitoring is implemented yet.
- No URDF/Xacro model, physics simulator, SLAM, localisation, or Nav2 integration is validated yet.
- Sensor and fault parameters are controlled simulations rather than identified hardware statistics.
- Actuator dynamics, latency, saturation, tyre contact, and physical wheel slip are not modelled.
- No physical-robot or hardware-in-the-loop validation has been completed.
- The project is educational portfolio software, not a safety-certified controller.

## Verification status

| Check | Status | Evidence or limitation |
|---|---|---|
| Python package installation | Automated in CI | `.github/workflows/python-core.yml` |
| Ruff lint | Automated in CI | Python 3.10–3.12 matrix |
| Unit tests | Automated in CI | `pytest` |
| Ideal-motion CLI artifacts | Automated in CI | CSV, JSON, and SVG existence checks |
| Combined-fault CLI artifacts | Automated in CI | CSV, JSON, and SVG existence checks |
| ROS 2 runtime | Not currently automated | Requires compatible ROS 2 environment |
| Physics simulation | Not implemented | Future milestone |
| Hardware validation | Not performed | Explicit approval and separate safety plan required |
| Dependency vulnerability scan | Not configured | Evaluate when dependency surface grows |
| Static type checking | Not configured | Consider before estimator complexity increases |

## Highest-priority next engineering tasks

1. Define the EKF state, process model, measurement models, covariance conventions, and reference cases in an architecture decision record.
2. Implement ROS-independent EKF prediction with deterministic analytical tests.
3. Add encoder and IMU updates, innovation statistics, and covariance validation.
4. Add Normalized Innovation Squared monitoring and configurable measurement rejection.
5. Compare raw wheel odometry, nominal EKF, and fault-aware EKF using reproducible experiments and reports.

## Risks and technical debt

- Estimator development can create convincing but incorrect results if frame, unit, covariance, or timestamp assumptions remain implicit.
- Placeholder ROS covariance values may be misread as validated uncertainty unless clearly labelled.
- Generated experiment artifacts can become stale when model behavior changes; regeneration and metric verification should accompany relevant changes.
- The current development dependencies use ranges rather than a lockfile; reproducibility should be reassessed as the dependency surface grows.
- GitHub CI validates the Python core but not a complete ROS 2 workspace.

## Deferred ideas

- URDF/Xacro and simulator integration;
- LiDAR fault injection;
- SLAM Toolbox and Nav2;
- fault supervisor and recovery policies;
- micro-ROS and hardware-in-the-loop rover;
- real-data calibration and parameter identification.
