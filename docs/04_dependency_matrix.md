# Dependency Matrix

Frozen baseline for WSL development:

| Component | Version or policy |
|---|---|
| OS | Ubuntu 20.04 |
| ROS | ROS 1 Noetic |
| Python for ROS | `/usr/bin/python3`, Python 3.8 |
| Build tool | `catkin_tools` |
| MAVROS | 1.20.1 |
| Gazebo | Gazebo Classic 11 |
| C++ | GCC/G++ 9 on Ubuntu 20.04 |
| PX4 SITL baseline | PX4-Autopilot `v1.14.3`, short commit `1dacb4c`; verify full SHA after clone |

Repository package policy:

| Package | May depend on MAVROS? | May depend on Gazebo messages? |
|---|---:|---:|
| `uav_msgs` | No | No |
| `uav_px4_bridge` | Yes, in later stages | No |
| `uav_bringup` | Yes, in hardware-specific launch files later | Yes, only in simulation launch files later |
| Future perception/planning/control packages | No | No |
| Future simulation package | No MAVROS requirement | Yes |

Jetson Xavier NX dependencies are not audited in M0-B and must be recorded in a
separate Jetson environment document after SSH access.

M0-C1 PX4 baseline:
- PX4 tag: `v1.14.3`.
- PX4 commit: `1dacb4c` short tag commit; after cloning, run
  `git rev-parse v1.14.3^{commit}` and replace this with the full SHA.
- Reason: formal release tag, suitable for Ubuntu 20.04, ROS Noetic, MAVROS,
  and Gazebo Classic 11 workflows.
- PX4 source location policy: external checkout at
  `/home/tom/third_party/PX4-Autopilot`, not vendored into this repository.
