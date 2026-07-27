# M0-C2 Trajectory Preview

M0-C2 goal:
- Receive `uav_msgs/Trajectory` on `/uav/trajectory`.
- Validate and cache the latest valid trajectory.
- Sample the cached trajectory by ROS time.
- Publish read-only previews on `/uav/setpoint_preview`.

M0-C2 explicitly does not:
- Publish `/mavros/setpoint_*`.
- Call arming or mode services.
- Switch OFFBOARD.
- Take off.
- Modify PX4 parameters or PX4 source.

## Time Semantics

`uav_msgs/Trajectory.header.stamp` is the intended execution start time in ROS
time. Each `TrajectoryPoint.time_from_start` is relative to that start time.

Sampling time is:

```text
t = now_ros - trajectory.header.stamp
```

Rules:
- Before the start time, preview publishes the first point with
  `started=false` and `finished=false`.
- During the trajectory interval, preview publishes the interpolated point with
  `started=true` and `finished=false`.
- After the final point time, preview holds the last point with
  `started=true` and `finished=true`.
- If ROS time moves backward relative to the previous sample, preview holds the
  first point and emits a throttled warning.
- Wall time and ROS time are not mixed in trajectory math.

## Validation

A trajectory is accepted only when:
- It has at least one point.
- `header.frame_id` is supported. M0-C2 supports `map` by default.
- The first point starts at `time_from_start=0`.
- Point times are strictly increasing after the first point.
- All time, position, velocity, acceleration, yaw, and yaw-rate fields are
  finite.

Rejected trajectories do not overwrite the last valid cached trajectory.

## Interpolation

Position uses piecewise cubic Hermite interpolation with endpoint position and
velocity:

```text
p(s) = h00(s) p0 + h10(s) dt v0 + h01(s) p1 + h11(s) dt v1
s = (t - t0) / dt
h00 =  2s^3 - 3s^2 + 1
h10 =   s^3 - 2s^2 + s
h01 = -2s^3 + 3s^2
h11 =   s^3 - s^2
```

Velocity and acceleration are analytic derivatives of the same polynomial.
Position, velocity, and acceleration are not independently linearly
interpolated.

Yaw uses the same Hermite basis over the shortest angular difference:

```text
yaw1_unwrapped = yaw0 + atan2(sin(yaw1 - yaw0), cos(yaw1 - yaw0))
```

The sampled yaw is normalized back to `[-pi, pi]`. `yaw_rate` is the analytic
derivative of the yaw Hermite curve. A yaw-only quaternion utility normalizes
the generated quaternion.

## Topics

Inputs:
- `/uav/trajectory` (`uav_msgs/Trajectory`)
- `/uav/state` (`uav_msgs/UavState`, optional health input)

Output:
- `/uav/setpoint_preview` (`uav_msgs/SetpointPreview`)

`SetpointPreview` contains:
- `header`: sampled time and frame.
- `point`: sampled `TrajectoryPoint`.
- `trajectory_valid`: whether a valid cached trajectory exists.
- `started`: whether ROS time has reached the trajectory start.
- `finished`: whether ROS time is past the final point.
- `state_fresh`: whether the optional `/uav/state` input is fresh and valid.
- `trajectory_id`: source trajectory ID.

## State Freshness

The preview node may subscribe to `/uav/state`. State freshness affects only
the `state_fresh` diagnostic flag and warnings. It does not change the
mathematical trajectory sample and does not trigger any flight-controller
action.

M0-C1 observed one `/uav/state` `rostopic hz` sampling gap of about 1.071 s in
the WSL validation window. M0-C2 records state freshness explicitly so later
stages can react to stale state without pretending that this transient has been
resolved.

## Frames

M0-C2 is algorithm-layer only:
- World frame convention: ENU.
- Body frame convention: FLU.
- Default frame ID: `map`.
- No ENU/NED or FLU/FRD conversion is performed.

PX4-facing conversion is reserved for a later output adapter. The preview topic
is not a MAVROS command topic.

## Launch

```bash
source scripts/env/ros_noetic_wsl.bash
source devel/setup.bash
roslaunch uav_trajectory trajectory_preview.launch
```

Default parameters:
- `trajectory_topic: /uav/trajectory`
- `preview_topic: /uav/setpoint_preview`
- `uav_state_topic: /uav/state`
- `subscribe_uav_state: true`
- `publish_rate: 30.0`
- `state_timeout: 0.5`
- `supported_frames: [map]`

The node clamps invalid publish rates to 30 Hz. Accepted configured rates are
1 to 100 Hz.

M0-C5A also provides an offline dynamic preview launch:

```bash
roslaunch uav_trajectory dynamic_trajectory_preview.launch trajectory_type:=line
```

Supported `trajectory_type` values are `line`, `circle`, and `figure8`.
Supported yaw modes are `fixed` and `velocity_aligned`. This launch starts only
`dynamic_trajectory_publisher_node` and `trajectory_preview_node`; it does not
start PX4, Gazebo, MAVROS, or any output adapter.

## Verification

```bash
catkin build
source devel/setup.bash
catkin run_tests
catkin_test_results build
```

Tests cover trajectory validation, Hermite sampling, yaw wraparound, cache
replacement, invalid trajectory rejection, time rollback handling, quaternion
normalization, dynamic trajectory generation, ROS preview publication,
end-of-trajectory behavior, dynamic preview continuity, and absence of project
publishers on MAVROS setpoint topics.
