# ROS 2 Autonomous Mobile Robot

A planned simulation-first robotics project demonstrating mapping, localisation, navigation, obstacle handling, and reproducible ROS 2 development practices.

## Project goals

- Build a differential-drive mobile robot in simulation
- Publish realistic odometry, laser-scan, and transform data
- Create a map and localise the robot within it
- Configure Nav2 for goal-based autonomous navigation
- Evaluate success rate, path length, and navigation time
- Package repeatable launch files, tests, and container setup

## Planned architecture

```text
Gazebo simulated robot
      |          |
      |          +--> Laser scan
      +--> Odometry
             |
             v
        TF + localisation
             |
             v
        Nav2 planner/controller
             |
             v
     Goal execution and metrics
```

## Planned technology

- ROS 2
- Python and C++
- Gazebo
- Nav2
- SLAM Toolbox
- TF2 and RViz
- Docker
- GitHub Actions

## Intended evidence

The finished repository will demonstrate ROS 2 package design, coordinate frames, sensor integration, navigation configuration, debugging, testing, and measurable experiment results.

## Status

Project specification and milestone planning are in progress. Development will follow the embedded BMS and CAN simulator.

## Author

Sadik Enes Erisen — M.Sc. Autonomy Technologies, FAU Erlangen-Nürnberg; B.Sc. Electrical and Electronics Engineering.
