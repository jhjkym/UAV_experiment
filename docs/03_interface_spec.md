# Interface Specification

All algorithm packages communicate through `uav_msgs`.

Rules:
- Messages crossing package boundaries must include a `std_msgs/Header` when
  they describe time-varying state.
- `header.stamp` is the observation or generation time of the message.
- Trajectory point timing uses `time_from_start`, relative to
  `Trajectory.header.stamp`.
- Algorithm packages use ENU/FLU.
- Future algorithm packages must not directly depend on `mavros_msgs` or Gazebo
  message packages.

Trajectory execution architecture:

```text
uav_msgs/Trajectory
        |
        v
trajectory buffer
        |
        v
interpolated sampling from ros::Time
        |
        v 20-50 Hz
mavros_msgs/PositionTarget
        |
        v
MAVROS
```

`uav_msgs/Trajectory` represents a complete time-parameterized trajectory. It
must not be published directly as a one-shot command to
`/mavros/setpoint_raw/local`.

M0-B does not implement the trajectory buffer, sampler, or MAVROS setpoint
publisher.

M0-C2 implements only the read-only trajectory buffer and preview sampler:

```text
uav_msgs/Trajectory
        |
        v
trajectory_preview_node
        |
        v
/uav/setpoint_preview  (uav_msgs/SetpointPreview)
```

`/uav/setpoint_preview` is an algorithm-layer preview. It is not a MAVROS
setpoint stream and must not be remapped to `/mavros/setpoint_*`.

Trajectory time semantics:
- `Trajectory.header.stamp` is the execution start time in ROS time.
- `TrajectoryPoint.time_from_start` is relative to `Trajectory.header.stamp`.
- Before start, the preview holds the first point and reports
  `started=false`.
- After the final point, the preview holds the last point and reports
  `finished=true`.

Trajectory frame semantics:
- M0-C2 accepts `map` by default.
- Algorithm-layer trajectory data remains ENU/FLU.
- PX4 NED/FRD conversion is reserved for a later MAVROS output adapter.

M0-C3 adds a dry-run offboard adapter:

```text
/uav/setpoint_preview
/uav/state
/mavros/state
        |
        v
offboard_adapter_node
        |
        +--> /uav/mavros_target_preview  (mavros_msgs/PositionTarget)
        +--> /uav/offboard_status        (uav_msgs/OffboardStatus)
```

The adapter converts preview points to `mavros_msgs/PositionTarget` without
performing an extra ENU/NED or FLU/FRD conversion. The normal launch has
`allow_mavros_output=false`, so no real `/mavros/setpoint_raw/local` publisher
is created. Test launches may remap the MAVROS output topic to
`/test/mavros/setpoint_raw/local`.

`PositionTarget.header.stamp` is the adapter publish time. The input preview
stamp is checked for freshness before any output gate can open.

M0-C4 uses the same interfaces for a PX4 SITL-only hover experiment:

```text
/uav/trajectory
        |
        v
trajectory_preview_node
        |
        v
/uav/setpoint_preview
        |
        v
offboard_adapter_node
        |
        v
/mavros/setpoint_raw/local
```

The real MAVROS setpoint stream is enabled only by the C4 experiment launch and
only after the experiment script has checked `UAV_ALLOW_SITL_FLIGHT=YES`, PX4
SITL process identity, UDP MAVROS configuration, MAVROS connection, and
disarmed state. The adapter still does not arm, call `set_mode`, take off, land,
or modify PX4 parameters. Those calls are limited to the explicit C4 SITL
experiment script.

MAVROS state bridge:

```text
/mavros/state
/mavros/local_position/odom
        |
        v
mavros_state_bridge_node
        |
        v
/uav/state  (uav_msgs/UavState)
```

The state bridge is read-only:
- It does not publish `/mavros/setpoint_*`.
- It does not call `/mavros/cmd/arming`.
- It does not call `/mavros/set_mode`.
- It does not modify PX4 parameters.

`/mavros/local_position/odom` is treated as a standard MAVROS ROS-frame output.
The bridge copies pose and twist into `uav_msgs/UavState` and does not apply an
extra ENU/NED conversion.

When no reliable acceleration source is configured, `acceleration` is zero and
`acceleration_valid=false`.
