# M0-C3 Offboard Adapter Dry Run

M0-C3 receives `/uav/setpoint_preview`, `/uav/state`, and `/mavros/state`,
checks input freshness and validity, applies two output gates, converts the
preview to `mavros_msgs/PositionTarget`, and publishes dry-run diagnostics by
default.

M0-C3 does not arm, call `set_mode`, switch OFFBOARD, take off, modify PX4
parameters, or publish to the real `/mavros/setpoint_raw/local` in the normal
launch.

## Package Responsibility

`uav_offboard` owns the control boundary:
- Setpoint adaptation.
- Input health checks.
- Output gates.
- Dry-run target preview.
- Future connection to MAVROS control output.

It does not implement trajectory planning, trajectory sampling, or the read-only
MAVROS state bridge.

## Interfaces

Inputs:
- `/uav/setpoint_preview` (`uav_msgs/SetpointPreview`)
- `/uav/state` (`uav_msgs/UavState`)
- `/mavros/state` (`mavros_msgs/State`)

Default outputs:
- `/uav/mavros_target_preview` (`mavros_msgs/PositionTarget`)
- `/uav/offboard_status` (`uav_msgs/OffboardStatus`)

Runtime service:
- `/offboard_adapter_node/set_output_enabled` (`std_srvs/SetBool`)

Potential MAVROS output:
- `/mavros/setpoint_raw/local`

The normal dry-run launch does not create the real MAVROS output publisher.

## Double Gate

Static gate:

```yaml
allow_mavros_output: false
```

The static gate is read on startup. With the default `false`, no real MAVROS
setpoint publisher is created. The normal launch explicitly writes `false`.

Runtime gate:
- Initial state is disabled.
- Controlled by `std_srvs/SetBool`.
- Can only be enabled when the static gate is true and all health checks pass.
- Disabling the service never arms, switches mode, or takes off.
- Any health failure while output is enabled disables output and reports fault.

Real output is allowed only when:

```text
allow_mavros_output == true
runtime_output_enabled == true
health == healthy
```

## State Machine

`uav_msgs/OffboardStatus.state`:
- `DISABLED`: static gate or runtime gate is closed.
- `WAITING_INPUTS`: output was requested but required inputs are absent or
  MAVROS is disconnected.
- `READY_DRY_RUN`: inputs are healthy, but real output is not enabled.
- `STREAMING`: both gates are open and all health checks pass.
- `FAULT`: output was requested and invalid data, timeout, or time rollback was
  detected.

Every status includes a `reason` string.

## Health Checks

M0-C3 requires:
- A `SetpointPreview` has been received.
- `trajectory_valid=true`.
- `started=true`.
- `finished=false`.
- `state_fresh=true`.
- Preview stamp is non-zero.
- Preview age is within `preview_timeout_sec`.
- `/uav/state` age is within `state_timeout_sec`.
- `/uav/state.pose_valid=true`.
- `/uav/state.twist_valid=true`.
- MAVROS is connected.
- Position, velocity, acceleration, yaw, and yaw-rate are finite.
- Preview and state frames match and are supported.
- ROS time has not moved backward relative to the previous adapter update.

Default parameters:
- `publish_rate_hz: 30.0`
- `preview_timeout_sec: 0.2`
- `state_timeout_sec: 0.2`
- `allow_mavros_output: false`
- `supported_frames: [map]`

Invalid parameter values are clamped to safe defaults.

## PositionTarget Mapping

`SetpointPreview.point` maps to `mavros_msgs/PositionTarget`:
- `position -> position`
- `velocity -> velocity`
- `acceleration -> acceleration_or_force`
- `yaw -> yaw`
- `yaw_rate -> yaw_rate`

`coordinate_frame` is `FRAME_LOCAL_NED` because this is the MAVROS
`setpoint_raw/local` message convention. The numeric fields are not manually
converted from ENU to NED in this package; MAVROS remains the PX4 interface
adapter.

`type_mask` is `0`, which means position, velocity, acceleration, yaw, and
yaw-rate are all intentionally populated. `FORCE` is not set, so the
acceleration field is interpreted as acceleration, not force.

The output `PositionTarget.header.stamp` is the adapter publish time. The input
preview stamp is still validated for freshness, but MAVROS setpoint streams are
time-sensitive command streams and are stamped when published.

## Frames

Algorithm-layer inputs remain ENU/FLU with `map` as the default world frame.
M0-C3 does not perform ENU/NED or FLU/FRD conversion. PX4-specific conversion
is still delegated to MAVROS.

## Failure Behavior

When output is disabled, failures update `/uav/offboard_status` and logs only.

When test output is enabled, any of these stop output within the adapter cycle:
- Preview timeout.
- State timeout.
- Invalid trajectory flag.
- Finished trajectory.
- Invalid vehicle state.
- MAVROS disconnect.
- NaN or Inf.
- Frame mismatch.
- ROS time rollback.
- Runtime gate disabled.

M0-C3 does not implement position hold, landing, mode switching, arming, or
disarming. Those belong to later flight safety state machines.

## Launch

Normal dry run:

```bash
source scripts/env/ros_noetic_wsl.bash
source devel/setup.bash
roslaunch uav_offboard offboard_adapter_dry_run.launch
```

The normal launch starts only `offboard_adapter_node` and does not start PX4,
MAVROS, arming, `set_mode`, OFFBOARD, or takeoff.

## Testing

Unit tests cover gate defaults, gate combinations, health checks, state machine
transitions, reason strings, finite checks, frame checks, time rollback, and
`PositionTarget` mapping.

ROS integration tests cover:
- Normal dry-run target preview and status publication.
- Absence of project publishers on the real `/mavros/setpoint_raw/local`.
- Test-only output to `/test/mavros/setpoint_raw/local` when both gates are
  enabled.
- Stopping output when runtime gate closes, preview times out, state is
  invalid, or MAVROS disconnects.
- Absence of arming and `set_mode` service use.

M0-C4 validates a PX4 SITL OFFBOARD hover through a separate experiment script.
The adapter node itself remains unchanged in responsibility: it only adapts,
gates, and publishes setpoints when both gates are open. It still does not call
arming, `set_mode`, OFFBOARD, takeoff, landing, disarming, or parameter
services.
