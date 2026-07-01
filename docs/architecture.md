# Architecture

## Current data flows

```text
MotionScenario
      |
      v
exact unicycle integration ---------------------------> ground-truth pose and twist
      |
      +--> body-twist to wheel-rate conversion
      |          |
      |          +--> encoder scale error
      |          +--> time-window wheel slip
      |          +--> seeded Gaussian wheel-rate noise
      |          +--> cumulative integer encoder counts
      |          +--> encoder-derived wheel odometry
      |
      +--> ideal yaw rate and longitudinal acceleration
                 |
                 +--> gyro / accelerometer bias
                 +--> seeded Gaussian IMU noise
                 +--> dropout and outlier windows
                 +--> simulated IMU measurements

Outputs: CSV dataset + JSON metrics + SVG comparison report

ROS 2 path:
geometry_msgs/Twist -> CommandOdometryNode -> nav_msgs/Odometry + odom-to-base_link TF
```

## Module responsibilities

| Module | Responsibility |
|---|---|
| `differential_drive.py` | ROS-independent planar state and exact constant-twist integration |
| `scenarios.py` | Typed command segments and reusable motion scenarios |
| `experiments.py` | Deterministic ground-truth simulation and baseline reports |
| `sensors.py` | Geometry, noise, faults, encoder counts, IMU measurements, wheel odometry, and error metrics |
| `sensor_reports.py` | Sensor CSV, metrics JSON, comparison SVG, and end-to-end experiment workflow |
| `experiment_cli.py` | Installed CLI for baseline and sensor-fault experiments |
| `odometry_node.py` | ROS 2 command subscription, timeout handling, odometry, and TF output |

## Design decisions

### Separate ground truth from measurements

Ground-truth motion is generated first and remains immutable. Encoder and IMU models consume the truth samples but cannot alter them. Wheel odometry is reconstructed from quantised encoder counts rather than copied from the reference pose.

This separation is necessary for meaningful error analysis. If a corrupted measurement path also changed the reference state, faults could appear artificially harmless.

### Separate mathematics from middleware

Kinematics, scenarios, sensor simulation, metrics, and reports contain no ROS imports. They can be validated with standard Python tools and reused later by ROS 2 nodes, state estimators, physics simulation, or hardware interfaces.

### Exact constant-twist integration

For non-zero yaw rate, pose updates use the analytical circular-arc solution rather than first-order Euler integration. Straight motion is handled separately near zero yaw rate to avoid numerical division problems.

### Exact command boundaries

The ground-truth experiment shortens the final step when a fixed integration interval would cross a command-segment boundary. Scenario duration is preserved even when the configured step does not divide the segment duration exactly.

### Quantised encoder reconstruction

The sensor model integrates corrupted wheel angular rate into cumulative floating-point count position, rounds it to integer encoder counts, and reconstructs left and right wheel increments from count differences. Wheel odometry therefore includes encoder quantisation.

### Seeded stochastic simulation

Noise is generated through `numpy.random.Generator` with an explicit seed. Identical scenario, configuration, step size, and seed produce identical sensor data and metrics.

### Time-window fault injection

Wheel slip, IMU dropout, and gyro outliers use half-open time windows. Fault activation is evaluated at the midpoint of each integration interval, avoiding ambiguous exact-boundary behaviour.

### Dependency-free reports

Trajectory and sensor-comparison reports are generated as SVG with the Python standard library. GitHub can render the results directly without committing binary plotting outputs.

### Command timeout

The ROS 2 node stops integrating the last velocity command when `cmd_vel` becomes stale. This prevents an old command from driving the software model indefinitely after a publisher disconnects.

### Explicit frame ownership

The current ROS 2 node owns the `odom` to `base_link` transform. A later localisation node must replace that transform or publish a different frame relationship to avoid multiple TF publishers claiming the same transform.

## Sensor assumptions

Default geometry:

| Parameter | Value |
|---|---:|
| Wheel radius | 0.08 m |
| Wheel separation | 0.34 m |
| Encoder resolution | 2048 counts/revolution |

The current wheel-slip model distorts encoder-reported wheel rate. It is useful for estimator and fault-monitoring development, but it is not a tyre-contact or rigid-body physics model.

The IMU model currently produces yaw rate and longitudinal acceleration. It does not yet include three-axis orientation, gravity projection, temperature effects, vibration spectra, or axis misalignment.

## Validation boundary

Automated validation covers:

- analytical kinematics;
- scenario timing;
- seeded sensor repeatability;
- ideal zero-noise measurements;
- encoder reconstruction;
- wheel-slip degradation;
- gyro bias, dropout, and outliers;
- CSV, JSON, SVG, and installed CLI workflows.

Full ROS 2 runtime, physics-simulator, and physical-hardware validation remain separate future milestones.

## Current limitations

- The ROS node still integrates commanded motion rather than the new encoder measurements.
- Sensor parameters are controlled simulation values rather than identified hardware statistics.
- Actuator dynamics, latency, saturation, and contact physics are not modelled.
- ROS covariance values remain placeholders.
- There is no localisation filter, robot description, SLAM, or Nav2 integration yet.

## Planned milestones

1. Implement an Extended Kalman Filter with covariance propagation and innovation monitoring.
2. Add Normalized Innovation Squared thresholds and faulty-measurement rejection.
3. Introduce a URDF/Xacro differential-drive model and physics simulation.
4. Add mapping, localisation, Nav2 configuration, and navigation metrics.
5. Connect the model to a microcontroller-based hardware-in-the-loop rover.
