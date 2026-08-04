# FaultNav Project Context

This file is the repository source of truth for project scope, maturity, and success criteria.

## Identity

- Project: FaultNav ROS 2
- Repository: `seneserisen/ros2-autonomous-mobile-robot`
- Owner: Sadik Enes Erisen
- Maturity target: Portfolio MVP
- Current documented milestone: `v0.3.0`
- Primary stack: Python 3.10–3.12, ROS 2 `ament_python`, NumPy, pytest, Ruff
- Repository visibility: Public
- License: MIT

## Problem

FaultNav provides a reproducible engineering testbed for mobile-robot kinematics, simulated encoder and IMU measurements, controlled fault injection, error analysis, and the staged development of fault-aware autonomous navigation.

The project is intended to demonstrate robotics modelling, numerical reasoning, ROS 2 integration, testing, reproducibility, and technical documentation. It is not a safety-certified controller or a claim of physical localisation accuracy.

## Intended users

- robotics and autonomy students;
- engineers evaluating estimation and fault-monitoring concepts;
- reviewers assessing Enes's autonomy-software portfolio;
- future contributors extending the system toward state estimation, simulation, and Nav2.

## Current validated scope

- exact planar differential-drive/unicycle integration;
- typed straight, circle, square, and figure-eight scenarios;
- deterministic experiment artifacts;
- wheel-rate conversion and encoder quantisation;
- encoder-derived wheel odometry;
- seeded encoder and IMU noise;
- scale error, asymmetric wheel slip, gyro bias, IMU dropout, and gyro outlier injection;
- CSV datasets, JSON metrics, and dependency-free SVG reports;
- ROS 2 `cmd_vel` subscription, `odom` publication, `odom -> base_link` TF, parameters, launch support, and stale-command protection;
- automated Python tests, Ruff linting, and Python 3.10–3.12 CI.

## Current non-goals

- safety certification;
- production deployment;
- physical-robot validation;
- identified tyre, actuator, latency, or contact-dynamics models;
- hardware commands from automated agents;
- claims that simulated errors equal real-world localisation performance;
- authentication, payments, cloud infrastructure, or user accounts;
- replacing the ROS-independent mathematical core with middleware-dependent code.

## Current milestone objective

Build the state-estimation and fault-monitoring layer while preserving the existing deterministic experiment and sensor-fault baselines.

### Planned acceptance direction

- Extended Kalman Filter for planar pose and velocity;
- explicit covariance propagation;
- encoder and IMU measurement updates;
- innovation and Normalized Innovation Squared monitoring;
- configurable measurement rejection;
- comparison of raw wheel odometry, nominal EKF, and fault-aware EKF;
- reproducible metrics and visual reports;
- tests against analytical or independently calculated reference cases.

These are roadmap items, not claims of implemented functionality.

## Core engineering invariants

1. Ground truth must remain independent from corrupted measurements and estimates.
2. Seeded experiments with identical configuration must be reproducible.
3. Units, frames, timestamp assumptions, and half-open fault windows must remain explicit.
4. ROS-independent mathematics should remain testable without a ROS installation.
5. Public artifacts must not contain secrets, private data, or unsupported claims.
6. Simulated results must be labelled as simulation results.

## Interfaces requiring compatibility review

- Python package imports under `faultnav_robot`;
- `faultnav-experiment` CLI arguments and generated artifact names;
- CSV and JSON fields;
- YAML configuration fields;
- ROS topics, message types, parameters, frames, and launch behavior;
- documented metric definitions.

Breaking changes require explicit approval and documentation.

## Definition of done for a meaningful change

- [ ] Acceptance criteria are explicit and satisfied.
- [ ] Relevant normal, boundary, and failure behavior is tested.
- [ ] Analytical or independent reference checks are used for important mathematics.
- [ ] `docs/testing.md` commands pass, or existing failures are documented accurately.
- [ ] Documentation matches actual behavior.
- [ ] Generated artifacts and public claims are reproducible.
- [ ] No secrets, private data, placeholder results, or unsupported claims remain.
- [ ] Enes can explain the architecture, equations, assumptions, tests, and limitations.
