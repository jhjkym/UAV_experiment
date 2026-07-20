# WSL Development Environment

This repository is currently built in a WSL 2 development environment:
- Ubuntu 20.04.
- x86_64 architecture.
- ROS 1 Noetic.
- MAVROS 1.20.1.
- Gazebo Classic 11.

The WSL environment is used for:
- Code development.
- Catkin builds.
- Unit tests.
- Offline data analysis.
- PX4 SITL preparation.

It is not the Jetson Xavier NX onboard computer. Jetson-specific JetPack, L4T,
CUDA, TensorRT, power mode, serial permissions, and hardware drivers must be
audited later on the real onboard computer.

WSL display policy:
- Prefer headless PX4 SITL and Gazebo Classic validation first.
- Gazebo GUI is optional. WSLg/OpenGL behavior can vary by Windows GPU driver
  and should not block M0-C1.
- In restricted shells, Gazebo may fail to write `~/.gazebo` logs. That is a
  shell permission issue, not a reason to connect real hardware.

ROS Python policy:
- ROS Noetic uses system Python 3.8.
- ROS builds must use `/usr/bin/python3`.
- Conda/Anaconda must not be active for catkin builds.
- Use `scripts/env/ros_noetic_wsl.bash` before building.

External PX4 source policy:
- Do not copy PX4-Autopilot into this repository.
- Keep PX4 source in an external directory such as
  `/home/tom/third_party/PX4-Autopilot`.
- Record the exact tag and commit in `docs/04_dependency_matrix.md`.
