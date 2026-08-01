# M0-C5A Dynamic Trajectory Baseline

M0-C5A adds offline dynamic trajectory generation and tracking metrics. It does
not run PX4, Gazebo, MAVROS, OFFBOARD, arming, landing, or any SITL flight.

## Data Path

```text
dynamic trajectory config
  -> uav_trajectory dynamic generator
  -> uav_msgs/Trajectory
  -> trajectory_preview_node
  -> /uav/setpoint_preview
  -> offline continuity checks
  -> trajectory_tracking_metrics.py
```

The generator is a pure C++ library and can be unit-tested without starting a
ROS node. `dynamic_trajectory_publisher_node` is only a thin ROS parameter and
publisher wrapper.

## Time And Frames

- World frame: ENU.
- Body frame: FLU.
- `Trajectory.header.stamp`: future execution start.
- `TrajectoryPoint.time_from_start`: relative time from that start.
- All generated trajectories are relative to a supplied start pose.
- M0-C5A performs no ENU/NED or FLU/FRD conversion.

## Five-Order Time Scaling

Smooth scalar motion uses:

```text
s(u)   = 10u^3 - 15u^4 + 6u^5
s'(u)  = 30u^2(1-u)^2
s''(u) = 60u(1-u)(1-2u)
u      = t / T
```

This gives zero velocity and acceleration at segment endpoints, so line
segments and dynamic-pattern entry/exit are C2 continuous.

## Trajectories

Line:

```text
(0, 0) -> (+L, 0) -> (-L, 0) -> (0, 0)
```

Each segment uses the five-order time scaling above. Defaults are `L=1.0 m`
and `line_segment_duration_sec=5.0`.

Optional line, circle, and figure-eight prefaces support SITL experiment
protocols without moving trajectory mathematics into shell scripts:

```yaml
initial_hold_sec: 0.0
initial_climb_duration_sec: 0.0
post_climb_hold_sec: 0.0
```

When enabled, the generated trajectory first holds the supplied start pose,
then applies a fifth-order vertical climb by `altitude_offset_m`, then holds at
the climbed altitude before horizontal motion begins. Position, velocity, and
acceleration are continuous at each boundary.

Circle:

```text
x = R cos(theta)
y = R sin(theta)
theta = 2*pi*s(t/T)
```

Velocity and acceleration are analytic derivatives using `theta_dot` and
`theta_ddot`. The phase completes one full revolution with zero endpoint
velocity and acceleration. Defaults are `R=1.0 m` and nominal tangent speed no
more than `0.5 m/s`.

Figure eight:

```text
x = A sin(theta)
y = B sin(2 theta)
theta = 2*pi*s(t/T)
```

The default Gerono-style figure eight uses `A=1.0 m` and `B=0.5 m`. Velocity
and acceleration are analytic derivatives of the same parameterization.

## Yaw

`fixed` keeps the supplied start yaw and sets `yaw_rate=0`.

`velocity_aligned` uses horizontal velocity direction when speed is above the
low-speed threshold. Below the threshold it holds the last valid yaw. Yaw is
unwrapped against the previous sample to avoid `+/-pi` jumps, and yaw-rate is
computed from the unwrapped sequence.

## Dynamic Constraints

Generated trajectories are checked for:
- finite fields;
- strictly increasing time;
- frame consistency;
- maximum velocity;
- maximum acceleration;
- maximum jerk from adjacent acceleration samples;
- yaw continuity.

Default limits:

```yaml
max_velocity_mps: 1.0
max_acceleration_mps2: 1.5
max_jerk_mps3: 4.0
```

M0-C5A uses whole-trajectory time scaling for dynamic-limit violations. The
generator never clips individual position, velocity, acceleration, or yaw
fields. If a trajectory still fails validation after scaling attempts, it is
rejected.

## Preview Verification

The ROS integration test starts only `trajectory_preview_node`, publishes line,
circle, and figure-eight `uav_msgs/Trajectory` messages, and validates
`/uav/setpoint_preview` continuity. It verifies preview frequency, valid
trajectory flags, before/during/after state behavior, finite fields, position
and velocity finite-difference consistency, yaw continuity, quaternion
normalization by yaw construction, and absence of MAVROS setpoint publishers.

If a valid replacement trajectory has a future `Trajectory.header.stamp` while
another trajectory is active, `trajectory_preview_node` queues it as pending and
continues sampling the active trajectory until the new start time. This prevents
a one-second handoff gap from producing a not-started preview during guarded
SITL experiments.

The pending handoff path logs the active trajectory ID, pending trajectory ID,
planned switch time, actual switch time, and position/velocity/acceleration
jumps at promotion. M0-C5B1-R1A validated this behavior in a ground-only SITL
rehearsal without arming, OFFBOARD, takeoff, or landing commands. The measured
handoff at `/tmp/uav_m0_c5b1/handoff_20260801_125214` switched once with
`0.026675 s` timing error, `0.000000001 m` position jump, zero velocity and
acceleration jump, `30.003862 Hz` average setpoint rate, no adapter FAULT, no
OFFBOARD state, and `armed=false` throughout.

## Tracking Metrics

`scripts/analysis/trajectory_tracking_metrics.py` reads CSV or structured JSON
with target and actual position/velocity columns. It computes metrics only on
the common target/actual time range using linear interpolation.

Summary fields include sample count, valid duration, axis mean and RMS position
errors, 3D and horizontal RMS/max errors, height mean/RMS error, velocity RMS
error, maximum actual speed, estimated maximum actual acceleration, coverage,
abnormal time gaps, and NaN/Inf counts.

Optional delay estimation uses discrete cross-correlation of target and actual
horizontal displacement magnitudes on the common resampled grid. This is only a
descriptive estimate under matching-motion conditions; it is not proof of true
system latency.
