# UAV_experiment

ROS 1 Noetic catkin workspace for the thesis project:

`不确定动静态混合环境下四旋翼无人机感知、规划与安全控制方法研究`

This repository is currently in M0-C2: read-only trajectory preview.

Current scope:
- WSL 2 Ubuntu 20.04 development environment.
- ROS 1 Noetic.
- System Python 3.8 at `/usr/bin/python3`.
- MAVROS and Gazebo Classic 11 are installed in the development environment.
- No flight controller, motors, real sensors, Jetson target, or PX4 SITL is
  connected by this stage.

M0-B provides:
- Catkin workspace skeleton.
- `uav_msgs` shared algorithm messages.
- `uav_px4_bridge` frame conversion utility library and unit tests.
- `uav_bringup` frame convention configuration.
- Documentation for environment, coordinate frames, and interfaces.

M0-C1 adds:
- PX4 SITL + MAVROS UDP launch configuration.
- A read-only MAVROS state bridge from `/mavros/local_position/odom` to
  `/uav/state`.
- Unit and ROS integration tests for state mapping and timeout behavior.

M0-C2 adds:
- `uav_trajectory` trajectory validation, buffering, and time-based preview.
- `uav_msgs/SetpointPreview` for sampled algorithm-layer setpoint previews.
- `/uav/trajectory` to `/uav/setpoint_preview` sampling at a configurable
  default of 30 Hz.
- Unit and ROS integration tests for Hermite interpolation, yaw wraparound,
  cache replacement, preview publication, and control-boundary checks.

Recommended build:
```bash
source scripts/env/ros_noetic_wsl.bash
catkin config --extend /opt/ros/noetic --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
source devel/setup.bash
catkin run_tests
catkin_test_results build
```

This stage does not provide flight capability and does not publish MAVROS
control setpoints. `/uav/setpoint_preview` is not a MAVROS command topic.

M0-C1 does not clone or vendor PX4-Autopilot into this repository. PX4 should be
kept as external source, for example `/home/tom/third_party/PX4-Autopilot`.
