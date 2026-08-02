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

M0-C4 SITL hover baseline:
- PX4 tag: `v1.14.3`.
- PX4 commit: `1dacb4cdef2d7145754fc788fa8dc482eed74b40`.
- SITL command: `HEADLESS=1 make px4_sitl gazebo-classic`.
- MAVROS FCU URL policy: UDP SITL URL only, default
  `udp://:14540@127.0.0.1:14557`; `serial://` and `/dev/tty*` are rejected.
- Control permission: the experiment script refuses to run unless
  `UAV_ALLOW_SITL_FLIGHT=YES`.
- Data policy: bags and logs are written under `/tmp/uav_m0c4/` and must not be
  committed.
- Passing run: `/tmp/uav_m0c4/run_20260727_190516`.
- Prestream setpoint rate: about 30.008 Hz.
- Real setpoint average rate during output: about 29.999 Hz.
- OFFBOARD switch confirmation: about 1.004 s.
- Arming confirmation: about 1.013 s.
- Hover horizontal RMS error: about 0.068 m.
- Maximum horizontal error: about 0.205 m.
- Hover height RMS error: about 0.171 m.
- Maximum altitude overshoot: about 0.059 m.
- Maximum speed: about 0.838 m/s.
- Landing to disarm: about 5.910 s.
- Adapter FAULT: false.
- Unexpected OFFBOARD exit: false.
- NaN or Inf: false.
- Final armed state: false.
- Test note: sandboxed ROS logging and network interface discovery can be
  restricted; tests pass with an independent temporary `ROS_HOME` under
  `/tmp/uav_m0c4/ros_home`.

M0-C5A dynamic trajectory baseline:
- `uav_trajectory` adds a pure C++ dynamic trajectory generator for line,
  circle, and figure-eight patterns.
- No new dependency on `mavros_msgs`, Gazebo messages, PX4, or serial devices.
- `scripts/analysis/trajectory_tracking_metrics.py` uses Python standard
  library only.
- This stage is offline: no PX4, Gazebo, MAVROS, arming, OFFBOARD, or landing
  execution is part of the baseline.

M0-C5B1 SITL line tracking:
- Uses PX4 v1.14.3 SITL, Gazebo Classic, UDP MAVROS, `uav_px4_bridge`,
  `uav_trajectory`, and `uav_offboard`.
- Flight service calls exist only in
  `scripts/experiments/m0_c5b1_sitl_line.py` after explicit SITL and
  `UAV_ALLOW_SITL_FLIGHT=YES` checks.
- R1 publishes a ground-hold trajectory for prestream, OFFBOARD, and arming,
  then starts the dynamic trajectory publisher only after `armed=true`; this
  keeps trajectory generation outside production nodes.
- `scripts/analysis/materialize_m0_c5b1_artifacts.py` is an offline Python
  standard-library tool. It reads existing M0-C5B1 run files from `/tmp` and
  materializes derived JSON artifacts without starting ROS, PX4, Gazebo,
  MAVROS, arming, OFFBOARD, or landing services.
- Run artifacts are written to `/tmp/uav_m0_c5b1/` and are not tracked by Git.

M0-C5B2A circle tracking preparation:
- Reuses the M0-C5A `uav_trajectory` C++ generator and the M0-C5B1 offline
  materialization pattern.
- Adds no new dependency on `mavros_msgs`, Gazebo messages, PX4 source, serial
  devices, or non-standard Python packages.
- `circle_metrics.json` is generated offline from existing summaries and
  tracking CSV data; it does not start ROS, PX4, Gazebo, MAVROS, arming,
  OFFBOARD, or landing services.
- The C5B2A stage did not perform a circle SITL flight. Future circle flight
  execution remains PX4 SITL-only and must use the same authorization and
  hardware-rejection boundary as C5B1.

M0-C5B2B-PREP circle entry:
- Adds `scripts/experiments/m0_c5b2b_sitl_circle.py` and shell wrapper as a
  guarded entry for future circle SITL execution.
- The entry reuses the C5B1 experiment lifecycle for SITL identity checks,
  one-shot authorization, process cleanup, publisher delivery, pending handoff,
  landing reserve, recovery, and post-disarm bag recording.
- The only circle-specific inputs are `m0_c5b2_circle.yaml`, circle phase
  boundaries, circle artifact expectations, and circle acceptance gates.
- The PREP stage is code, dry-run, and mock-test only. It does not start PX4,
  Gazebo, MAVROS, OFFBOARD, arming, or real `AUTO.LAND`.
