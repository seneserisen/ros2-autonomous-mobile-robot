# FaultNav ROS 2

[![Python core validation](https://github.com/seneserisen/ros2-autonomous-mobile-robot/actions/workflows/python-core.yml/badge.svg)](https://github.com/seneserisen/ros2-autonomous-mobile-robot/actions/workflows/python-core.yml)

Python-first ROS 2 project for building and evaluating a fault-aware autonomous mobile robot in
simulation. The first milestone provides a tested differential-drive kinematics core and a ROS 2
node that converts velocity commands into odometry and TF output.

## Current milestone

Implemented:

- exact constant-twist integration for a planar differential-drive/unicycle model;
- typed and immutable robot-state representation;
- `cmd_vel` subscription using `geometry_msgs/Twist`;
- `odom` publication using `nav_msgs/Odometry`;
- `odom` to `base_link` TF broadcasting;
- configurable update rate, frame names, TF output, and command timeout;
- automatic zero-velocity fallback when commands become stale;
- ROS 2 launch file and YAML configuration;
- ROS-independent unit tests across Python 3.10–3.12;
- Ruff linting and GitHub Actions validation.

This milestone is a software-in-the-loop foundation. It is not yet a complete autonomous-navigation
stack and has not yet been validated against a physics simulator or physical robot.

## Architecture

```text
geometry_msgs/Twist (`cmd_vel`)
              |
              v
     CommandOdometryNode
       |              |
       |              +--> stale-command safety stop
       v
 exact unicycle integration
       |
       +--> nav_msgs/Odometry (`odom`)
       |
       +--> TF: `odom` -> `base_link`
```

See [docs/architecture.md](docs/architecture.md) for design decisions, frame ownership, limitations,
and planned milestones.

## Repository structure

```text
.
├── .github/workflows/python-core.yml
├── config/faultnav.yaml
├── docs/architecture.md
├── launch/faultnav.launch.py
├── resource/faultnav_robot
├── src/faultnav_robot/
│   ├── __init__.py
│   ├── differential_drive.py
│   └── odometry_node.py
├── tests/test_differential_drive.py
├── package.xml
├── pyproject.toml
├── setup.cfg
└── setup.py
```

## Build in a ROS 2 workspace

The package follows the `ament_python` layout. Place or clone it inside the `src` directory of a
ROS 2 workspace:

```bash
mkdir -p ~/faultnav_ws/src
cd ~/faultnav_ws/src
git clone https://github.com/seneserisen/ros2-autonomous-mobile-robot.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Launch the node:

```bash
ros2 launch faultnav_robot faultnav.launch.py
```

Send a forward and rotational velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

Inspect the generated odometry:

```bash
ros2 topic echo /odom
```

The configured command timeout is `0.5 s`. After the one-shot command becomes stale, published
velocity returns to zero and pose integration stops.

## Run the Python core tests without ROS 2

The mathematical core has no ROS dependencies:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
python -m pip install -r dev-requirements.txt
ruff check src tests setup.py launch
pytest
```

## Technical model

For body-frame linear velocity `v`, yaw rate `omega`, heading `theta`, and interval `dt`, the
straight-motion case is:

```text
x_next     = x + v cos(theta) dt
y_next     = y + v sin(theta) dt
theta_next = wrap(theta + omega dt)
```

For non-zero yaw rate, the implementation uses the analytical circular-arc solution:

```text
R          = v / omega
x_next     = x + R [sin(theta + omega dt) - sin(theta)]
y_next     = y - R [cos(theta + omega dt) - cos(theta)]
theta_next = wrap(theta + omega dt)
```

This avoids the accumulated approximation error of first-order Euler integration when a constant
command describes a circular arc.

## Learning objectives

This repository is structured to develop practical competence in:

- Python package design, typing, dataclasses, and unit testing;
- ROS 2 nodes, topics, messages, parameters, timers, and launch files;
- odometry frames and TF publication;
- differential-drive modelling and numerical integration;
- repeatable experiments and measurable engineering results;
- sensor fusion, fault injection, localisation, SLAM, and navigation in later milestones.

## Roadmap

### Milestone 2 — deterministic motion experiments

- command-sequence generator;
- CSV trajectory export;
- path and error plots;
- repeatability and runtime metrics.

### Milestone 3 — sensor simulation and fault injection

- wheel encoders and IMU;
- Gaussian noise, bias, dropout, wheel slip, and outliers;
- configurable deterministic scenarios.

### Milestone 4 — state estimation

- Extended Kalman Filter;
- covariance propagation;
- innovation and Normalized Innovation Squared monitoring;
- measurement rejection and fault-detection metrics.

### Milestone 5 — robot simulation

- URDF/Xacro model;
- differential-drive controller;
- laser scanner and IMU integration;
- RViz and physics-simulator launch configuration.

### Milestone 6 — autonomous navigation

- SLAM Toolbox;
- localisation;
- Nav2 planner and controller configuration;
- navigation success rate, path length, completion time, and recovery metrics.

## Engineering limitations

- Current odometry is derived from commanded rather than measured motion.
- Wheel slip, encoder resolution, latency, dynamics, and actuator saturation are not yet modelled.
- Published covariance entries are initial placeholders and are not identified from physical data.
- No safety or real-time guarantees are provided.
- The project is educational portfolio software, not a production robot controller.

## Author

Sadik Enes Erisen — M.Sc. Autonomy Technologies, FAU Erlangen-Nürnberg; B.Sc. Electrical and
Electronics Engineering.
