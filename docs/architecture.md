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

EKF prediction path:
five-state posterior estimate + covariance -> exact constant-twist prediction -> predicted estimate + covariance

EKF measurement path:
encoder count increments -> body forward velocity --+
                                                 +--> ungated scalar updates -> posterior estimate + covariance
finite IMU yaw rate -----------------------------+

Local demonstration path:
SETUP/RUN/TEST/DOCTOR wrapper -> standard-library workflow helper -> installed faultnav-experiment CLI
```

## Module responsibilities

| Module | Responsibility |
|---|---|
| `differential_drive.py` | ROS-independent planar state and exact constant-twist integration |
| `scenarios.py` | Typed command segments and reusable motion scenarios |
| `experiments.py` | Deterministic ground-truth simulation and baseline reports |
| `sensors.py` | Geometry, noise, faults, encoder counts, IMU measurements, wheel odometry, and error metrics |
| `ekf.py` | ROS-independent five-state EKF prediction and ungated scalar encoder-velocity/gyro updates |
| `sensor_reports.py` | Sensor CSV, metrics JSON, comparison SVG, and end-to-end experiment workflow |
| `experiment_cli.py` | Installed CLI for baseline and sensor-fault experiments |
| `odometry_node.py` | ROS 2 command subscription, timeout handling, odometry, and TF output |
| `scripts/faultnav_workflow.py` | Idempotent local setup, deterministic demo, tests, and environment diagnostics |

## Design decisions

### Separate ground truth from measurements

Ground-truth motion is generated first and remains immutable. Encoder and IMU models consume the truth samples but cannot alter them. Wheel odometry is reconstructed from quantised encoder counts rather than copied from the reference pose.

This separation is necessary for meaningful error analysis. If a corrupted measurement path also changed the reference state, faults could appear artificially harmless.

### Separate mathematics from middleware

Kinematics, scenarios, sensor simulation, metrics, and reports contain no ROS imports. They can be validated with standard Python tools and reused later by ROS 2 nodes, state estimators, physics simulation, or hardware interfaces.

The EKF core follows the same boundary. It operates only on a five-element estimate and
five-by-five covariance in the ordering `[p_x_m, p_y_m, yaw_rad, linear_velocity_m_s,
yaw_rate_rad_s]`; it does not consume ground truth, fault flags, sensor-simulation objects, or ROS
messages. Measurement functions receive independent numeric values.

### Encoder measurement definition

The current encoder update observes forward body velocity along `base_link` +x in m/s:

```text
z_v = v_encoder
h_v(x) = linear_velocity_m_s
H_v = [0, 0, 0, 1, 0]
```

`v_encoder` is calculated from consecutive quantised left/right encoder counts, the elapsed time,
encoder resolution, and wheel radius. The same count-to-body-twist helper feeds wheel odometry, so
there is no parallel kinematic convention. Pre-quantisation simulated wheel rates, ground truth,
truth pose, fault flags, and integrated wheel-odometry pose are not EKF measurements.

### Gyroscope measurement definition

The gyro update observes yaw rate about `base_link` +z in rad/s:

```text
z_gyro = imu_yaw_rate_rad_s
h_gyro(x) = yaw_rate_rad_s
H_gyro = [0, 0, 0, 0, 1]
```

Velocity and yaw-rate innovations are ordinary scalar differences and are not angle-wrapped. A
missing IMU value is unavailable data: the caller skips the update, and the low-level function
rejects `None` rather than treating it as zero. No gyro-bias state or privileged fault correction is
present.

### Measurement covariance and posterior update

Each public measurement function requires one finite, strictly positive variance: `(m/s)^2` for
encoder-derived velocity or `(rad/s)^2` for gyro yaw rate. These values are separate from simulated
sensor-noise settings and are not identified or calibrated hardware parameters.

The shared scalar update computes innovation, innovation covariance, and the full Kalman gain. State
cross-covariances are preserved, so observing velocity or yaw rate may legitimately change correlated
pose components. Posterior covariance uses Joseph form:

```text
P_plus = (I - K H) P_minus (I - K H).T + K R K.T
```

The posterior is then checked with the existing finite, symmetry, and positive-semi-definite
validation. Innovations are diagnostic outputs only. NIS, thresholds, acceptance/rejection, adaptive
covariance, sensor disabling, and fault classification are not implemented.

### Continuous process-noise discretisation

EKF prediction uses configurable longitudinal-acceleration and yaw-acceleration white-noise
densities. The standard continuous white-noise acceleration covariance is discretised over each
interval. Longitudinal position noise is projected into odometry x/y using the nominal midpoint
body heading; yaw acceleration is discretised over the yaw/yaw-rate double integrator. This is a
local linearisation assumption, not an identified physical disturbance model.

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

### Thin local workflow wrappers

Root `.bat` and `.sh` files contain no robotics or reporting logic. They locate Python and delegate
to one standard-library workflow helper, which in turn creates the documented environment and calls
the installed `faultnav-experiment` CLI. A setup fingerprint covers Python's major/minor version and
the package/development dependency metadata, allowing unchanged environments to be reused. Git and
GitHub checks are diagnostic only; no launcher modifies repository history or publishes changes.

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
- analytical EKF prediction, finite-difference Jacobians, scalar measurement references,
  correlated-state updates, and Joseph-form covariance propagation;
- wheel-slip degradation;
- gyro bias, dropout, and outliers;
- CSV, JSON, SVG, and installed CLI workflows.

Full ROS 2 runtime, physics-simulator, and physical-hardware validation remain separate future milestones.

## Current limitations

- The ROS node still integrates commanded motion rather than the new encoder measurements.
- Sensor parameters are controlled simulation values rather than identified hardware statistics.
- Actuator dynamics, latency, saturation, and contact physics are not modelled.
- ROS covariance values remain placeholders.
- Raw measurement innovations are available, but NIS monitoring, gating, rejection, estimator
  orchestration/reports, and ROS integration are not implemented.
- Measurement covariance values are explicit engineering inputs, not hardware-calibrated statistics.
- There is no global localisation, robot description, SLAM, or Nav2 integration yet.

## Planned milestones

1. Add monitor-only Normalized Innovation Squared diagnostics without rejection.
2. Add configurable faulty-measurement rejection as a separate behavior change.
3. Add deterministic raw-odometry versus ungated-EKF versus gated-EKF comparison artifacts.
4. Introduce a URDF/Xacro differential-drive model and physics simulation.
5. Add mapping, localisation, Nav2 configuration, and navigation metrics.
6. Connect the model to a microcontroller-based hardware-in-the-loop rover.
