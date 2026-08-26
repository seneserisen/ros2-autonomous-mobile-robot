# FaultNav Project Status

- Last updated: 26 August 2026
- Current maturity: Portfolio MVP in active development
- Current documented milestone: `v0.3.0` (not published as a GitHub release or tag)
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
- ROS-independent five-state EKF prediction with exact constant-twist mean propagation;
- analytical state-transition Jacobians and continuous process-noise covariance propagation;
- covariance shape, finiteness, symmetry, and positive-semi-definite validation;
- encoder-count-derived forward body-velocity EKF updates in m/s;
- gyroscope yaw-rate EKF updates in rad/s, with missing measurements rejected rather than zero-filled;
- diagnostic innovation, innovation covariance, Kalman gain, and measurement-model outputs;
- Joseph-form posterior covariance with analytical, correlated-state, uncertainty, and fault-boundary tests;
- Windows and POSIX setup, demo, test, and doctor launchers backed by one tested helper;
- deterministic one-command combined-fault demo output under `artifacts/demo`;
- Python unit and integration tests;
- Ruff linting and Python 3.10–3.12 GitHub Actions validation.

## Current limitations

- ROS odometry still integrates commanded motion rather than simulated encoder measurements.
- ROS covariance values remain placeholders.
- EKF measurement updates are not yet orchestrated into a continuous estimator, CLI, report, or ROS node.
- Raw innovations are exposed, but NIS, monitoring summaries, gating, rejection, and fault
  classification are not implemented.
- Measurement-noise variances are explicit engineering/test values rather than calibrated hardware
  statistics.
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
| Unit tests | Automated in CI | `pytest`, including analytical EKF prediction and measurement-update references |
| Ideal-motion CLI artifacts | Automated in CI | CSV, JSON, and SVG existence checks |
| Combined-fault CLI artifacts | Automated in CI | CSV, JSON, and SVG existence checks |
| Local workflow helper | Automated in tests | Idempotence fingerprint, commands, artifacts, wrappers |
| Windows launchers | Automated and locally verified | Clean-runner setup, demo, tests, and doctor |
| Linux launchers | Automated in CI | Clean-runner setup, demo, tests, and doctor |
| macOS launchers | Syntax and unit tested | Native execution still requires macOS |
| ROS 2 runtime | Not currently automated | Requires compatible ROS 2 environment |
| Physics simulation | Not implemented | Future milestone |
| Hardware validation | Not performed | Explicit approval and separate safety plan required |
| Dependency vulnerability scan | Not configured | Evaluate when dependency surface grows |
| Static type checking | Not configured | Consider before estimator complexity increases |

## Highest-priority next engineering tasks

1. Add Normalized Innovation Squared diagnostics in monitor-only mode without rejection.
2. Add configurable measurement rejection as a separate behavior change.
3. Build deterministic estimator orchestration and compare raw wheel odometry, ungated EKF, and
   later fault-aware EKF using reproducible experiments and reports.
4. Add a thin ROS 2 estimator adapter only after deterministic comparison evidence is accepted.

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
