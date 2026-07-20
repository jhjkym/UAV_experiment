# PX4 SITL + MAVROS Read-Only State Link

M0-C1 goal:
- Run PX4 SITL with Gazebo Classic.
- Connect MAVROS to SITL over UDP.
- Read `/mavros/state` and `/mavros/local_position/odom`.
- Publish unified `/uav/state`.
- Do not publish setpoints, arm, switch modes, take off, or modify PX4
  parameters.

## Version Baseline

Recommended PX4 baseline:
- PX4-Autopilot tag: `v1.14.3`.
- Short tag commit observed from public tag index: `1dacb4c`.
- Full commit must be verified after clone:

```bash
git -C /home/tom/third_party/PX4-Autopilot rev-parse v1.14.3^{commit}
```

Installed development stack:
- Ubuntu 20.04 in WSL 2.
- ROS 1 Noetic.
- MAVROS 1.20.1.
- Gazebo Classic 11.15.1.
- gazebo_ros 2.9.3.

## Manual PX4 Setup Commands

These commands require network and possibly package installation. They are not
executed automatically by this repository.

```bash
mkdir -p /home/tom/third_party
cd /home/tom/third_party
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.14.3
git submodule update --init --recursive
git rev-parse HEAD
```

If PX4 reports missing system dependencies, install them manually after review.
The current WSL audit found `genromfs` and `exiftool` missing.

## Headless SITL First

Prefer headless validation in WSL:

```bash
cd /home/tom/third_party/PX4-Autopilot
HEADLESS=1 make px4_sitl gazebo-classic
```

Gazebo GUI is optional:

```bash
cd /home/tom/third_party/PX4-Autopilot
make px4_sitl gazebo-classic
```

WSLg/OpenGL stability depends on the Windows graphics stack. GUI failure should
not block the read-only state-link milestone if `gzserver`, PX4 SITL, MAVROS,
and ROS topics work.

## MAVROS UDP Link

Launch MAVROS for SITL:

```bash
source scripts/env/ros_noetic_wsl.bash
source devel/setup.bash
roslaunch "$(rospack find uav_bringup)/launch/sim/mavros_sitl.launch"
```

Default `fcu_url`:

```text
udp://:14540@127.0.0.1:14557
```

Meaning:
- `:14540`: local UDP bind port used by MAVROS.
- `127.0.0.1:14557`: PX4 SITL UDP endpoint.
- No `/dev/ttyACM0` or other serial device is used.

Default `gcs_url`:

```text
udp://@127.0.0.1:14550
```

Meaning:
- Optional local UDP forwarding endpoint for QGroundControl or packet
  inspection.
- QGroundControl is not required for M0-C1 validation.

The launch file keeps SITL MAVROS config separate from future real-hardware
MAVROS config.

## State Bridge

Launch the read-only state bridge:

```bash
source scripts/env/ros_noetic_wsl.bash
source devel/setup.bash
roslaunch "$(rospack find uav_bringup)/launch/sim/state_bridge_sitl.launch"
```

Data flow:

```text
PX4 SITL
  |
  v
MAVLink UDP
  |
  v
MAVROS
  |
  +--> /mavros/state
  +--> /mavros/local_position/odom
          |
          v
    mavros_state_bridge_node
          |
          v
       /uav/state
```

The bridge:
- Copies pose and twist from `nav_msgs/Odometry`.
- Preserves `header.stamp` and `header.frame_id`.
- Sets zero acceleration and `acceleration_valid=false`.
- Does not repeat ENU/NED conversion on standard MAVROS odometry.
- Marks pose and twist invalid after an odometry timeout.

## Verification Commands

```bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/local_position/odom
rostopic echo -n 1 /uav/state

rostopic hz /mavros/local_position/odom
rostopic hz /uav/state

rosnode list
rostopic list
rosparam get /mavros/fcu_url
```

Expected state bridge behavior:
- `/mavros/state.connected=true` after MAVROS connects to SITL.
- `/uav/state.pose_valid=true` while odometry is fresh.
- `/uav/state.twist_valid=true` while odometry is fresh.
- `/uav/state.acceleration_valid=false`.
- `/uav/state.header.stamp` matches the MAVROS odometry input stamp.
- `/uav/state.header.frame_id` matches the MAVROS odometry input frame.

## Unsupported in M0-C1

M0-C1 does not support:
- `/mavros/setpoint_*` publication.
- `/mavros/cmd/arming`.
- `/mavros/set_mode`.
- OFFBOARD switching.
- Takeoff.
- PX4 parameter modification.
- Real flight controller serial links.
- Planners or controllers.

## Troubleshooting

MAVROS does not connect:
- Confirm PX4 SITL is running.
- Confirm `rosparam get /mavros/fcu_url` is `udp://:14540@127.0.0.1:14557`.
- Check that no other process is using the UDP ports.

No `/uav/state`:
- Confirm `/mavros/local_position/odom` is publishing.
- Confirm `state_bridge_sitl.launch` is running.
- Check node logs for odometry timeout warnings.

Gazebo GUI fails:
- Retry headless mode with `HEADLESS=1`.
- Use topic-level verification instead of GUI visualization.
