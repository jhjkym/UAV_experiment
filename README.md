# UAV_experiment

ROS 1 Noetic catkin workspace for the thesis project:

`不确定动静态混合环境下四旋翼无人机感知、规划与安全控制方法研究`

This repository is currently in M0-C4: PX4 SITL-only manually authorized
OFFBOARD hover validation.

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

M0-C3 adds:
- `uav_offboard` dry-run MAVROS `PositionTarget` adapter.
- Double output gates: static `allow_mavros_output=false` by default plus a
  runtime `SetBool` gate.
- `/uav/mavros_target_preview` and `/uav/offboard_status` diagnostics.
- Test-only remapped output validation without PX4, arming, mode switching, or
  takeoff.

M0-C4 adds:
- A SITL-only experiment script for manually authorized OFFBOARD hover.
- A C4 launch that connects the existing state bridge, trajectory preview, and
  offboard adapter to the real SITL `/mavros/setpoint_raw/local` stream.
- Runtime checks that reject execution unless `UAV_ALLOW_SITL_FLIGHT=YES`,
  PX4 is the local SITL binary, MAVROS uses UDP, and the vehicle is connected
  and disarmed before control.
- Bag and log recording under `/tmp/uav_m0c4/`.
- Validated PX4 SITL OFFBOARD rise, 15 second hover, `AUTO.LAND`, and final
  disarmed state in `/tmp/uav_m0c4/run_20260727_190516`.

M0-C5A adds:
- Offline dynamic trajectory generation for smooth line, circle, and figure
  eight patterns in ENU coordinates.
- A read-only dynamic trajectory publisher and preview launch that do not start
  PX4, Gazebo, MAVROS, OFFBOARD, arming, or landing.
- Dynamic constraint validation with whole-trajectory time scaling.
- Offline tracking metrics for CSV or structured JSON time series.

M0-C5B1 adds a guarded PX4 SITL-only smooth line tracking experiment. It reuses
the M0-C5A dynamic trajectory generator, records outputs under
`/tmp/uav_m0_c5b1/`, and must not be run against real hardware. R1 uses a
two-stage protocol: prestream/OFFBOARD/arming on a fixed ground-hold trajectory,
then dynamic flight trajectory publication only after `armed=true`.

Recommended build:
```bash
source scripts/env/ros_noetic_wsl.bash
catkin config --extend /opt/ros/noetic --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
source devel/setup.bash
catkin run_tests
catkin_test_results build
```

Normal development and test launches do not provide flight capability and do
not publish real MAVROS control setpoints. M0-C4 flight validation is a separate
PX4 SITL-only experiment path guarded by `UAV_ALLOW_SITL_FLIGHT=YES`; it must
not be used with real flight hardware.

M0-C1 does not clone or vendor PX4-Autopilot into this repository. PX4 should be
kept as external source, for example `/home/tom/third_party/PX4-Autopilot`.
