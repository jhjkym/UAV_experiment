# Dependency Matrix

Frozen baseline for WSL development:

| Component | Version or policy |
|---|---|
| OS | Ubuntu 20.04 |
| ROS | ROS 1 Noetic |
| Python for ROS | `/usr/bin/python3`, Python 3.8 |
| Build tool | `catkin_tools` |
| MAVROS | 1.20.1 |
| Gazebo | Gazebo Classic 11.15.1 |
| C++ | GCC/G++ 9 on Ubuntu 20.04 |
| PX4 SITL baseline | PX4-Autopilot `v1.14.3`, commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40` |

Repository package policy:

| Package | May depend on MAVROS? | May depend on Gazebo messages? |
|---|---:|---:|
| `uav_msgs` | No | No |
| `uav_px4_bridge` | Yes, in later stages | No |
| `uav_offboard` | Yes, for `mavros_msgs/PositionTarget` and `mavros_msgs/State` | No |
| `uav_bringup` | Yes, in hardware-specific launch files later | Yes, only in simulation launch files later |
| Future perception/planning/control packages | No | No |
| Future simulation package | No MAVROS requirement | Yes |

Jetson Xavier NX dependencies are not audited in M0-B and must be recorded in a
separate Jetson environment document after SSH access.

M0-C1 PX4 baseline:
- PX4 tag: `v1.14.3`.
- PX4 commit: `1dacb4cdef2d7145754fc788fa8dc482eed74b40`.
- Reason: formal release tag, suitable for Ubuntu 20.04, ROS Noetic, MAVROS,
  and Gazebo Classic 11 workflows.
- PX4 source location policy: external checkout at
  `/home/tom/third_party/PX4-Autopilot`, not vendored into this repository.

M0-C1 measured read-only link:
- SITL command: `HEADLESS=1 make px4_sitl gazebo-classic`.
- MAVROS FCU URL: `udp://:14540@127.0.0.1:14557`.
- MAVROS state: connected, not armed, mode `AUTO.LOITER`.
- Odometry topic: `/mavros/local_position/odom`, `map` to `base_link`,
  measured at about 29.98 Hz.
- Unified state topic: `/uav/state`, measured at about 28.30 Hz during the
  validation window, with one transient sampling gap observed by `rostopic hz`.
- State validity: `pose_valid=true`, `twist_valid=true`,
  `acceleration_valid=false` during fresh odometry.
- Timeout behavior: after stopping the MAVROS input launch, pose and twist are
  marked invalid; after restarting MAVROS, both recover to valid.
- Control boundary: no project node published MAVROS setpoint topics, and no
  arming, mode switch, takeoff, or PX4 parameter write was executed.
