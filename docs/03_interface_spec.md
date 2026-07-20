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
