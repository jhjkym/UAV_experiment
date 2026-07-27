# M0-C4 SITL Offboard Hover

M0-C4 is a PX4 SITL-only experiment. It validates the path:

```text
PX4 SITL
  -> MAVROS
  -> /uav/state
  -> /uav/trajectory
  -> /uav/setpoint_preview
  -> /mavros/setpoint_raw/local
```

No real flight controller, Jetson, serial device, motor, or real sensor is part
of this stage.

## Safety Gate

The experiment script refuses to start unless:

```bash
export UAV_ALLOW_SITL_FLIGHT=YES
```

It also verifies:
- PX4 is the local SITL process from `px4_sitl_default/bin/px4`.
- Gazebo Classic `gzserver` exists.
- MAVROS FCU URL uses UDP and does not contain `serial://` or `/dev/tty`.
- `/mavros/state.connected=true`.
- `/mavros/state.armed=false` before OFFBOARD and arming requests.

The normal `offboard_adapter_dry_run.launch` remains dry-run only. C4 uses the
separate launch:

```bash
roslaunch uav_bringup m0_c4_offboard_hover.launch
```

That launch starts only project-side nodes. It does not start PX4, MAVROS, arm,
set mode, take off, land, disarm, or write PX4 parameters.

## Experiment Script

Entry point:

```bash
scripts/experiments/m0_c4_sitl_hover.sh
```

The script starts, records, and later stops only the processes it created:
- `roscore`
- PX4 SITL with `HEADLESS=1 make px4_sitl gazebo-classic`
- MAVROS SITL launch
- C4 project launch
- `rosbag record`

Logs and bags are written under `/tmp/uav_m0c4/run_*`.

## Trajectory

The script reads the latest valid `/uav/state` and constructs a relative
trajectory:

```text
x_target = current_x
y_target = current_y
z_target = current_z + 1.0 m
yaw_target = current_yaw
```

The trajectory rises over 5 seconds, then holds the target for 120 seconds so
the sampler keeps `finished=false` during the 15 second hover window.

The trajectory uses `map`/ENU algorithm coordinates. M0-C4 does not add any
manual ENU/NED or FLU/FRD conversion. MAVROS remains responsible for the
standard PX4 interface adaptation.

## Control Sequence

After at least 2 seconds of setpoint prestream at no less than 20 Hz:

```text
request OFFBOARD
confirm /mavros/state.mode == OFFBOARD
request arming
confirm /mavros/state.armed == true
rise for 5 seconds
hover for 15 seconds
request AUTO.LAND
wait for armed=false
disable runtime output gate
```

Service calls are limited and explicit. The adapter node, trajectory preview
node, and state bridge node do not call arming, `set_mode`, OFFBOARD, takeoff,
landing, disarming, or parameter services.

## Abort Conditions

If the vehicle is armed and any abort condition triggers, the script requests
`AUTO.LAND` first. It does not force disarm in the air.

Abort conditions include:
- MAVROS disconnect.
- Stale `/uav/state`.
- Adapter `FAULT`.
- Setpoint stream below 10 Hz.
- Unexpected exit from OFFBOARD while armed.
- NaN or Inf.
- Horizontal offset greater than 2 m.
- Height error greater than 1 m.
- Roll or pitch greater than 30 degrees.

## Recorded Topics

The bag records:
- `/mavros/state`
- `/mavros/local_position/odom`
- `/mavros/setpoint_raw/local`
- `/mavros/setpoint_raw/target_local`
- `/uav/state`
- `/uav/trajectory`
- `/uav/setpoint_preview`
- `/uav/mavros_target_preview`
- `/uav/offboard_status`

## Metrics

The script writes `summary.json` with:
- Setpoint prestream frequency.
- OFFBOARD switch time.
- Arming time.
- Maximum height overshoot.
- Hover height mean and RMS error.
- Horizontal position maximum and RMS error.
- Maximum speed.
- OFFBOARD exit flag.
- Adapter FAULT flag.
- Landing to disarm time.
- Final armed state.

First-pass targets:
- Setpoint average frequency at least 20 Hz.
- Horizontal maximum error at most 0.5 m.
- Hover position RMS error at most 0.25 m.
- Height overshoot at most 0.3 m.
- No NaN or Inf.
- No adapter FAULT.
- Final `armed=false`.

## Current Result

Passing run:

```text
/tmp/uav_m0c4/run_20260727_190516
```

PX4:
- Tag: `v1.14.3`.
- Commit: `1dacb4cdef2d7145754fc788fa8dc482eed74b40`.

SITL identity and connection:
- PX4 process came from `px4_sitl_default/bin/px4`.
- Gazebo Classic headless `gzserver` was running.
- MAVROS FCU URL was UDP SITL, `udp://:14540@127.0.0.1:14557`.
- No `serial://` or `/dev/tty*` FCU URL was used.
- MAVROS was connected and disarmed before OFFBOARD and arming.

Start and target:
- Start: `(-0.002535, -0.002335, -0.063635)`.
- Target: `(-0.002535, -0.002335, 0.936365)`.
- Target yaw: about `-0.054657 rad`.

Measured result:
- Prestream setpoint rate: `30.008 Hz`.
- Real setpoint average rate: `29.999 Hz`.
- OFFBOARD switch confirmation: `1.004 s`.
- Arming confirmation: `1.013 s`.
- Rise and 15 second hover completed.
- `AUTO.LAND` was requested.
- Landing to disarm: `5.910 s`.
- Final `armed=false`.

Metrics:
- Maximum horizontal error: `0.205 m`.
- Hover horizontal RMS error: `0.068 m`.
- Hover height mean error: `-0.062 m`.
- Hover height RMS error: `0.171 m`.
- Maximum altitude overshoot: `0.059 m`.
- Maximum speed: `0.838 m/s`.
- Unexpected OFFBOARD exit before landing: false.
- Adapter FAULT: false.
- NaN or Inf: false.
- Abort condition: none.

Recorded bag:

```text
/tmp/uav_m0c4/run_20260727_190516/m0_c4.bag
```

Bag summary:
- Duration: about 44 seconds.
- `/mavros/setpoint_raw/local`: 905 messages.
- `/mavros/local_position/odom`: 1319 messages.
- `/uav/state`: 1321 messages.
- `/uav/setpoint_preview`: 1321 messages.
- `/uav/offboard_status`: 1320 messages.

The bag, temporary logs, and PX4 generated logs remain outside Git.

## Test Environment Note

In the managed sandbox, ROS logging under the default home directory and local
network interface discovery may be restricted. The final test baseline was run
with an independent temporary `ROS_HOME` under `/tmp/uav_m0c4/ros_home` and
passed with `96 tests, 0 errors, 0 failures, 0 skipped`.
