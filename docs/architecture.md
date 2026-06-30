# Architecture

## Current data flows

```text
Deterministic experiment path                   ROS 2 command path

Built-in MotionScenario                         geometry_msgs/Twist (`cmd_vel`)
          |                                               |
          v                                               v
CommandSegment sequence                         CommandOdometryNode
          |                                               |
          +------------------+----------------------------+
                             |
                             v
                  exact unicycle integration
                             |
          +------------------+-----------------------------+
          |                  |                             |
          v                  v                             v
CSV trajectory        JSON metrics / SVG report    nav_msgs/Odometry + TF
```

## Module responsibilities

| Module | Responsibility |
|---|---|
| `differential_drive.py` | ROS-independent planar state and exact constant-twist integration |
| `scenarios.py` | Typed command segments and reusable motion scenarios |
| `experiments.py` | Deterministic simulation, metrics, CSV/JSON export, and SVG generation |
| `experiment_cli.py` | Installed command-line interface for reproducible experiments |
| `odometry_node.py` | ROS 2 command subscription, timeout handling, odometry, and TF output |

## Design decisions

### Separate mathematics from middleware

The kinematics, scenarios, metrics, and reports contain no ROS imports. They can be tested with standard Python tooling and reused by future simulation, state-estimation, or hardware-interface nodes.

### Exact constant-twist integration

For non-zero yaw rate, the pose update follows the analytical circular-arc solution rather than a first-order Euler approximation. Straight motion is handled separately near zero yaw rate to avoid numerical division problems.

### Exact command-segment boundaries

The experiment runner shortens the final integration step whenever a fixed step would cross a segment boundary. Scenario duration is therefore preserved even when the configured step is not an exact divisor of a command duration.

### Deterministic evidence

Experiments use no wall-clock timing or uncontrolled randomness. Re-running a scenario with the same configuration produces identical samples, metrics, and report geometry.

### Dependency-free reports

Trajectory previews are generated as SVG using the Python standard library. This keeps the experiment layer lightweight and ensures GitHub can render result figures directly.

### Command timeout

The ROS 2 node stops integrating the last commanded velocity when `cmd_vel` becomes stale. This prevents an old command from driving the software model indefinitely after a publisher disconnects.

### Explicit frame ownership

The current ROS 2 node owns the `odom` to `base_link` transform. A later localisation node must either replace this transform or publish a different frame relationship to avoid multiple TF publishers claiming the same transform.

## Validation boundary

The current automated validation covers the ROS-independent model, scenario, experiment, artifact, and CLI layers. Full ROS 2 runtime validation still requires a ROS 2 environment and later physics-simulator integration.

## Current limitations

- The ROS node integrates commanded motion rather than measured wheel motion.
- Wheel slip, encoder quantisation, sensor noise, latency, and actuator dynamics are not modelled.
- Covariance values are fixed placeholders rather than identified sensor statistics.
- There is no robot description, physics simulator, localisation filter, SLAM, or Nav2 integration yet.
- Numerical closure results demonstrate software consistency, not real-world localisation accuracy.

## Planned milestones

1. Add wheel-encoder and IMU simulation with configurable noise and faults.
2. Implement an Extended Kalman Filter with innovation monitoring.
3. Introduce a URDF/Xacro differential-drive model and physics simulation.
4. Add mapping, localisation, Nav2 configuration, and navigation metrics.
