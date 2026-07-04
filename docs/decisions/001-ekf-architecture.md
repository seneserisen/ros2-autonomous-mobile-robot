# ADR-001: ROS-independent EKF architecture

- Status: Proposed
- Date: 2026-07-04
- Decision owner: Sadik Enes Erisen
- Related issue or milestone: FaultNav state-estimation and fault-monitoring milestone

## Context

FaultNav currently has deterministic planar kinematics, differential-drive motion scenarios,
encoder and IMU simulation, controlled fault injection, sensor-fault reports, and a ROS 2
command-odometry interface. No Extended Kalman Filter (EKF), innovation monitoring, or
measurement gating is implemented yet.

The next estimator milestone needs an EKF architecture that keeps the mathematical core
independent of ROS 2, preserves the separation between ground truth, measurements, and estimates,
and makes units, frames, timestamps, covariance ordering, and fault-monitoring assumptions explicit.

The EKF is a local filtered-odometry estimator. It is not a map localizer and does not estimate a
`map -> odom` transform.

## Decision drivers

- correctness and mathematical validity;
- explicit units, frames, timestamps, and transform ownership;
- ROS-independent estimator core with thin future ROS adapters;
- compatibility with existing differential-drive kinematics and sensor simulation;
- reproducible tests against analytical and independent reference cases;
- visible covariance, innovation, and NIS assumptions;
- clear separation between measurement rejection and physical fault diagnosis;
- Portfolio MVP scope without hardware or safety claims.

## Options considered

### Option A: Three-state pose EKF

- Description: Estimate `[x, y, yaw]` and apply encoder-derived pose increments as inputs or pseudo-measurements.
- Benefits: Smallest state and easiest to explain.
- Disadvantages: Encourages fusing derived odometry pose rather than direct sensor quantities; makes velocity and yaw-rate innovation monitoring weaker.
- Risks: Can hide encoder and gyro disagreement behind already-integrated pose updates.
- Verification approach: Analytical pose propagation tests and pose residual tests.

### Option B: Five-state planar odometry EKF

- Description: Estimate `[x, y, yaw, v, yaw_rate]` using exact constant-twist prediction, encoder wheel-rate updates, and gyro yaw-rate updates.
- Benefits: Keeps pose, velocity, and yaw rate explicit; supports direct encoder and gyro residuals; aligns with existing planar kinematics.
- Disadvantages: Requires covariance ordering, process noise, Jacobians, and measurement gating discipline.
- Risks: Poor covariance tuning can make NIS misleading.
- Verification approach: Analytical prediction cases, finite-difference Jacobian checks, covariance checks, and independent innovation/NIS references.

### Option C: Bias-augmented EKF

- Description: Extend the state with gyro bias and possibly encoder scale or accelerometer bias terms.
- Benefits: Can model persistent bias more realistically later.
- Disadvantages: Increases complexity before the base EKF is validated; can absorb simulated faults that should initially appear in innovations.
- Risks: Bias states may make fault-monitoring demonstrations look better while weakening interpretability.
- Verification approach: Bias observability tests and long-run consistency tests with independent reference cases.

## Decision

Select **Option B: five-state planar odometry EKF**.

The state vector is:

```text
x = [p_x_m, p_y_m, yaw_rad, linear_velocity_m_s, yaw_rate_rad_s]^T
```

where:

| Index | Symbol | Meaning | Unit |
|---:|---|---|---|
| 0 | `p_x_m` | `base_link` origin x-coordinate in `odom` | m |
| 1 | `p_y_m` | `base_link` origin y-coordinate in `odom` | m |
| 2 | `yaw_rad` | planar yaw of `base_link` relative to `odom` | rad |
| 3 | `linear_velocity_m_s` | forward body velocity along `base_link` +x | m/s |
| 4 | `yaw_rate_rad_s` | yaw rate about `base_link` +z | rad/s |

The state pose is the pose of `base_link` expressed in `odom`. The yaw mean is wrapped to
`[-pi, pi)` after prediction and update.

Coordinate conventions:

- `odom` is the local continuous dead-reckoning frame;
- `base_link` is the planar robot body frame;
- +x is forward, +y is left, +z is upward;
- positive yaw and positive yaw rate are counter-clockwise about +z;
- positive wheel angular velocity means forward wheel rotation.

The initial EKF does not estimate global localization, `map -> odom`, gyro bias, encoder scale,
accelerometer bias, actuator dynamics, contact physics, or hardware behaviour.

## Prediction model

For a prediction interval:

```text
T = t[k+1] - t[k]
```

`T` must be finite and positive. The estimator state at `t[k]` is propagated to `t[k+1]` using a
constant forward body velocity and constant yaw rate over the interval.

Let:

```text
delta_yaw = yaw_rate_rad_s * T
yaw_1 = yaw_rad + delta_yaw
```

For `abs(yaw_rate_rad_s) >= straight_line_epsilon`:

```text
p_x_next = p_x + (v / yaw_rate) * (sin(yaw_1) - sin(yaw))
p_y_next = p_y + (v / yaw_rate) * (cos(yaw) - cos(yaw_1))
yaw_next = wrap(yaw_1)
v_next = v
yaw_rate_next = yaw_rate
```

For `abs(yaw_rate_rad_s) < straight_line_epsilon`:

```text
p_x_next = p_x + v * cos(yaw) * T
p_y_next = p_y + v * sin(yaw) * T
yaw_next = wrap(yaw + yaw_rate * T)
v_next = v
yaw_rate_next = yaw_rate
```

The default `straight_line_epsilon` is `1e-9 rad/s`, matching the existing kinematic convention.

## Prediction Jacobian

The state-transition Jacobian is:

```text
F = df/dx
```

All unlisted entries are zero. State-preserving diagonal entries are one.

For the turning branch, define:

```text
A = sin(yaw + yaw_rate*T) - sin(yaw)
B = cos(yaw) - cos(yaw + yaw_rate*T)
delta_x = (v / yaw_rate) * A
delta_y = (v / yaw_rate) * B
```

Then:

```text
F[p_x, yaw]      = -delta_y
F[p_y, yaw]      =  delta_x
F[p_x, v]        = A / yaw_rate
F[p_y, v]        = B / yaw_rate
F[p_x, yaw_rate] = v * (T*yaw_rate*cos(yaw + yaw_rate*T) - A) / yaw_rate^2
F[p_y, yaw_rate] = v * (T*yaw_rate*sin(yaw + yaw_rate*T) - B) / yaw_rate^2
F[yaw, yaw_rate] = T
```

For the near-straight branch:

```text
F[p_x, yaw]      = -v*T*sin(yaw)
F[p_y, yaw]      =  v*T*cos(yaw)
F[p_x, v]        =  T*cos(yaw)
F[p_y, v]        =  T*sin(yaw)
F[p_x, yaw_rate] = -0.5*v*T^2*sin(yaw)
F[p_y, yaw_rate] =  0.5*v*T^2*cos(yaw)
F[yaw, yaw_rate] =  T
```

The straight-limit yaw-rate derivatives must not be dropped.

## Process covariance

The state covariance uses the same ordering as the state vector:

```text
[p_x_m, p_y_m, yaw_rad, linear_velocity_m_s, yaw_rate_rad_s]
```

Diagonal units are:

| Entry | Unit |
|---|---|
| `P[0,0]` | m^2 |
| `P[1,1]` | m^2 |
| `P[2,2]` | rad^2 |
| `P[3,3]` | (m/s)^2 |
| `P[4,4]` | (rad/s)^2 |

Prediction covariance is:

```text
P_minus = F * P_plus * F.T + Q
```

The process-noise model uses explicit continuous white-noise densities:

| Symbol | Meaning | Unit |
|---|---|---|
| `q_linear_accel` | longitudinal acceleration noise density | m^2/s^3 |
| `q_yaw_accel` | yaw acceleration noise density | rad^2/s^3 |

These values must be configuration, not hidden constants or final-scenario tuning.

The implementation must validate covariance shape, finite values, symmetry, and positive
semi-definiteness within documented tolerances.

## Encoder measurement model

Encoder updates use count differences or equivalent measured wheel angular-rate inputs. They must
not use ground truth or the already integrated wheel-odometry pose as measurements.

For encoder resolution `N` counts/revolution and interval `T`:

```text
delta_count_left  = count_left[k]  - count_left[k-1]
delta_count_right = count_right[k] - count_right[k-1]
z_left_rad_s  = 2*pi*delta_count_left  / (N*T)
z_right_rad_s = 2*pi*delta_count_right / (N*T)
```

The encoder measurement vector is:

```text
z_encoder = [z_left_rad_s, z_right_rad_s]^T
```

The predicted measurement is:

```text
h_left  = (v - 0.5*b*yaw_rate) / r
h_right = (v + 0.5*b*yaw_rate) / r
```

where:

- `r` is wheel radius in metres;
- `b` is wheel separation in metres.

The encoder Jacobian is:

```text
H_encoder = [
  [0, 0, 0, 1/r, -b/(2*r)],
  [0, 0, 0, 1/r,  b/(2*r)],
]
```

The encoder measurement covariance `R_encoder` has ordering `[left, right]` and units `(rad/s)^2`.
A full symmetric 2-by-2 matrix is permitted; the core must not assume independent wheel errors.

The first encoder sample establishes previous counts and does not produce an encoder update.

## IMU measurement model

The version-1 IMU update uses gyroscope yaw rate only.

```text
z_gyro = imu_yaw_rate_rad_s
h_gyro = yaw_rate_rad_s
H_gyro = [[0, 0, 0, 0, 1]]
R_gyro = [[gyro_variance_rad_s2]]
```

A missing gyroscope value means the measurement is unavailable. It must not be replaced with zero.

The existing longitudinal accelerometer channel is not fused in version 1. It remains available in
simulation samples and reports. Accelerometer fusion requires a later decision covering timestamp
semantics, discontinuous velocity segments, gravity handling for future ROS IMU data, bias handling,
and process/measurement covariance coupling.

## Innovation and NIS

For each candidate measurement group:

```text
innovation = z - h(x_minus)
S = H * P_minus * H.T + R
NIS = innovation.T * solve(S, innovation)
```

The implementation shall solve linear systems. It shall not explicitly invert `S`.

Measurement groups are:

| Group | Dimension | Degrees of freedom |
|---|---:|---:|
| Encoder pair | 2 | 2 |
| Gyroscope | 1 | 1 |

A singular, non-finite, or non-positive-definite innovation covariance is a numerical error, not a
normal rejection decision.

Every candidate measurement should record:

- timestamp;
- group name;
- innovation;
- innovation covariance;
- NIS;
- threshold;
- accepted, rejected, unavailable, or invalid status;
- reason.

NIS is a consistency signal. It is not proof of a specific physical fault.

## Gating behaviour

The initial configurable gate confidence is 99%.

Proposed chi-square thresholds:

| Group | Degrees of freedom | 99% threshold |
|---|---:|---:|
| Gyroscope | 1 | 6.635 |
| Encoder pair | 2 | 9.210 |

Acceptance rule:

```text
accepted if NIS <= threshold
rejected if NIS > threshold
```

Equality is accepted.

Encoder and gyroscope candidate NIS values are calculated independently against the same predicted
state and covariance. Accepted groups are then stacked into one Joseph-form update. Rejected groups
do not alter the state or covariance. If no group is accepted, the predicted state and covariance
become the posterior.

Version 1 shall not silently inflate measurement covariance, reset from ground truth, adapt noise
online, blacklist a sensor permanently, or classify a rejected measurement as a proven physical fault.

## Measurement update

For accepted stacked measurements:

```text
K = P_minus * H.T * solve(S, I)
x_plus = x_minus + K * innovation
yaw_plus = wrap(yaw_plus)
```

Covariance shall use Joseph form:

```text
P_plus = (I - K*H) * P_minus * (I - K*H).T + K * R * K.T
```

The result may be symmetrized numerically:

```text
P_plus = 0.5 * (P_plus + P_plus.T)
```

Symmetrization must not hide materially negative eigenvalues or large numerical defects.

## Consequences

### Positive

- Keeps the estimator core ROS-independent and directly testable with Python.
- Reuses the existing exact constant-twist motion convention.
- Fuses direct wheel-rate and gyro measurements instead of derived pose.
- Makes covariance ordering and units explicit.
- Creates clear raw-odometry, ungated-EKF, and gated-EKF comparison paths.
- Keeps simulated gyro bias and wheel-slip effects visible in innovation records.

### Negative

- No accelerometer fusion in the first EKF implementation.
- No gyro-bias or encoder-scale state in version 1.
- No absolute pose correction is possible with the selected sensors.
- NIS depends strongly on covariance assumptions and must not be overinterpreted.
- Encoder quantization is only approximately Gaussian.

### Compatibility and migration

- No existing CLI, CSV, JSON, SVG, ROS topic, TF, launch, or configuration behaviour changes in this ADR.
- Runtime implementation should start in a new ROS-independent module, likely `src/faultnav_robot/ekf.py`.
- Tests should start in `tests/test_ekf_prediction.py` and later expand to measurement and NIS tests.
- Future ROS integration must ensure only one component owns `odom -> base_link`.
- Future ROS adapters must transform sensor data into the documented frames before calling the EKF core.

## Verification

Required reference cases before accepting implementation:

1. Zero interval preserves state and covariance when process noise is zero.
2. Stationary robot keeps pose unchanged.
3. Straight motion with `v=2 m/s`, `yaw_rate=0`, `T=1.5 s`, `yaw=0` gives `x=3 m`.
4. Rotation in place with `v=0`, `yaw_rate=pi/2 rad/s`, `T=1 s` changes yaw by `pi/2`.
5. Constant-radius arc with `v=1 m/s`, `yaw_rate=1 rad/s`, `T=pi/2 s` gives `x=1 m`, `y=1 m`, `yaw=pi/2`.
6. Yaw wrapping maps `pi` to `-pi`.
7. Prediction Jacobian matches central finite differences for straight, turning, negative-yaw-rate, and near-branch cases.
8. Prediction and update covariance remain finite, symmetric, and positive semi-definite within tolerance.
9. Encoder geometry reference with `r=0.1 m`, `b=0.4 m`, `v=1.0 m/s`, `yaw_rate=0.5 rad/s` gives left/right wheel rates of `9` and `11 rad/s`.
10. Scalar NIS reference: innovation `2`, covariance `4`, NIS `1`.
11. Two-dimensional NIS reference: innovation `[1, 2]`, covariance `diag([1, 4])`, NIS `2`.
12. Measurement exactly at the threshold is accepted.
13. Measurement above the threshold is rejected.
14. Missing gyro sample is unavailable and does not create a zero residual.
15. Ground truth is used only for evaluation and never enters prediction, update, gating, reset, or measurement construction.
16. Identical scenario, configuration, and seed produce identical estimator states, covariance histories, innovations, NIS values, decisions, metrics, and artifacts.

## Implementation milestones

1. Accept this ADR.
2. Implement ROS-independent prediction core only.
3. Add analytical prediction, finite-difference Jacobian, and covariance validation tests.
4. Add encoder and gyroscope measurement updates.
5. Add innovation and NIS monitoring in monitor-only mode.
6. Add configurable gating as a separate behaviour change.
7. Add deterministic raw-odometry versus ungated-EKF versus gated-EKF comparison artifacts.
8. Revisit accelerometer fusion in a separate ADR.
9. Add a thin ROS 2 adapter only after the mathematical core and deterministic comparison are validated.

## Revisit when

Reconsider this architecture when:

- accelerometer fusion is required;
- gyro bias or encoder scale must be estimated rather than only detected;
- a physics simulator introduces more realistic actuator, latency, or contact dynamics;
- real sensor data or rosbag data becomes available;
- absolute localization, SLAM, landmarks, GNSS, or `map -> odom` estimation is introduced;
- hardware-in-the-loop or physical robot validation is planned;
- NIS statistics are inconsistent under nominal seeded experiments despite documented covariance tuning.
