# Architecture

## Milestone 1 data flow

```text
geometry_msgs/Twist (`cmd_vel`)
              |
              v
     CommandOdometryNode
       |              |
       |              +--> command timeout / zero-velocity fallback
       v
 exact unicycle integration
       |
       +--> nav_msgs/Odometry (`odom`)
       |
       +--> TF: `odom` -> `base_link`
```

## Design decisions

### Separate mathematical core from ROS 2 interfaces

`differential_drive.py` contains no ROS imports. The kinematics can therefore be tested with normal
Python tooling and later reused by simulation, state-estimation, or hardware-interface nodes.

### Exact constant-twist integration

For non-zero yaw rate, the pose update follows the analytical circular-arc solution rather than a
first-order Euler approximation. Straight motion is handled separately near zero yaw rate to avoid
numerical division problems.

### Command timeout

The ROS 2 node stops integrating the last commanded velocity when `cmd_vel` becomes stale. This
prevents an old command from driving the software model indefinitely after a publisher disconnects.

### Explicit frame ownership

This node currently owns the `odom` to `base_link` transform. A later localisation node must either
replace this transform or publish a different frame relationship to avoid multiple TF publishers
claiming the same transform.

## Current limitations

- The node integrates commanded motion, not measured wheel motion.
- It does not model wheel slip, encoder quantisation, latency, or actuator dynamics.
- Covariance values are fixed placeholders rather than identified sensor statistics.
- There is no robot description, physics simulator, localisation filter, SLAM, or Nav2 integration.
- The Python CI validates only the ROS-independent mathematical core; full ROS 2 integration testing
  remains a later milestone.

## Planned milestones

1. Add deterministic command scenarios and generated trajectory reports.
2. Add wheel-encoder and IMU simulation with configurable noise and faults.
3. Implement an Extended Kalman Filter with innovation monitoring.
4. Introduce a URDF/Xacro differential-drive model and physics simulation.
5. Add mapping, localisation, Nav2 configuration, and navigation metrics.
