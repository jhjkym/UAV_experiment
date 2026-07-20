# AGENTS.md

This repository is a ROS 1 Noetic catkin workspace for UAV perception,
planning, and control research.

Current development environment:
- WSL 2 Ubuntu 20.04 x86_64 for development, build, unit tests, offline data
  processing, and PX4 SITL preparation.
- The Jetson Xavier NX onboard computer is a separate target and has not been
  audited in this repository yet.
- ROS uses system Python 3.8 at `/usr/bin/python3`. Conda/Anaconda must not be
  active for ROS builds.

Safety rules:
- Do not arm a flight controller.
- Do not switch PX4 to Offboard mode.
- Do not modify PX4 parameters.
- Do not flash firmware.
- Do not start motors.
- Do not add launch files that connect to real flight hardware without explicit
  review.

Architecture rules:
- Algorithm packages use ENU for the world frame and FLU for the body frame.
- PX4 uses NED for the world frame and FRD for the body frame.
- MAVROS owns the standard ROS/PX4 frame adaptation for standard MAVROS topics.
- Custom frame conversion code is only for non-MAVROS data, simulation ground
  truth adapters, offline logs, or explicitly bypassed MAVROS interfaces.
- Future algorithm packages must not directly depend on `mavros_msgs` or Gazebo
  message packages.
- Simulation and real hardware are decoupled through `uav_msgs` interfaces.

Build baseline:
```bash
source scripts/env/ros_noetic_wsl.bash
catkin config --extend /opt/ros/noetic --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
catkin run_tests
catkin_test_results build
```
