# M0-C5B2 SITL Circle Tracking Preparation

M0-C5B2A defines the PX4 SITL-only circle tracking protocol, metrics, and
offline tests. This stage did not start PX4, Gazebo, MAVROS, OFFBOARD, arming,
`AUTO.LAND`, or any flight.

M0-C5B2B-PREP adds the executable circle experiment entry without performing a
flight. Commit `afcd69f` prepared the offline protocol and metrics only; it did
not include an executable circle SITL entry. The PREP entry reuses the accepted
C5B1 safety lifecycle and supplies only the circle configuration, phase
boundaries, artifact expectations, and circle acceptance rules.

## Reused Lifecycle

Circle tracking reuses the M0-C5B1 guarded dynamic tracking lifecycle:

```text
ground-hold
-> setpoint prestream
-> OFFBOARD
-> arming
-> pending handoff
-> dynamic flight trajectory
-> landing reserve
-> AUTO.LAND
-> automatic disarm
-> post-disarm bag recording
```

The future C5B2 flight entry must call the same core protocol used by C5B1 for
SITL identity checks, one-shot authorization, process cleanup, publisher
delivery hard gates, handoff validation, setpoint and adapter monitoring,
landing reserve lifecycle, abnormal landing recovery, atomic JSON writes, and
cross-file consistency checks. Circle-specific behavior is supplied by
trajectory configuration and circle metrics; it must not duplicate the state
machine or circle mathematics in an experiment script.

The guarded entry points prepared for the next flight are:

```text
scripts/experiments/m0_c5b2b_sitl_circle.py
scripts/experiments/m0_c5b2b_sitl_circle.sh
src/uav_bringup/launch/sim/m0_c5b2b_circle_tracking.launch
```

The Python entry consumes an explicit `--auth-file` containing the one-shot
token `M0_C5B2B_CIRCLE_SITL_ONLY`. It requires a regular non-symlink file owned
by the current user, mode `600`, a matching token on line 1, and a Unix
timestamp no older than ten minutes on line 2. The file is deleted before any
process can be started. The dry-run mode validates the same authorization and
configuration but starts no processes and calls no services.

## Geometry

The C5B2A default trajectory is ENU and relative to the latest valid
`/uav/state` pose:

```yaml
trajectory_type: circle
initial_hold_sec: 1.0
initial_climb_duration_sec: 8.0
post_climb_hold_sec: 2.0
circle_radius_m: 1.0
circle_tangent_speed_mps: 0.40
circle_laps: 1.0
transition_duration_sec: 4.0
center_hold_evaluation_sec: 10.0
landing_reserve_hold_sec: 60.0
yaw_mode: fixed
```

The circle center is the horizontal start point `(x0, y0)` at altitude
`z0 + 1.0 m`. The entry segment moves smoothly from the center to the circle
start point `(x0 + R, y0)`. The default direction is counter-clockwise, so the
initial circle tangent points toward positive ENU `y`. One lap ends back at
`(x0 + R, y0)`, then the exit segment returns smoothly to the center. The final
center hold and landing reserve keep that same center target with zero velocity
and acceleration. Fixed yaw holds the yaw observed at takeoff.

The trajectory generator implements the circle phase as:

```text
theta = 2*pi*circle_laps*s(t/T)
x = x0 + R*cos(theta)
y = y0 + R*sin(theta)
```

Velocity and acceleration use analytic derivatives of `theta`. Entry and exit
use the fifth-order time law from `docs/09_dynamic_trajectory_baseline.md`, so
position, velocity, and acceleration are continuous at phase boundaries.

## Phases

Circle tracking records:

```text
PREFLIGHT
PRESTREAM
OFFBOARD_PREARM
ARMED_HOLD
PENDING_HANDOFF
CLIMB
CLIMB_HOLD
CIRCLE_ENTRY
CIRCLE_LAP
CIRCLE_EXIT
CENTER_HOLD
LANDING_PREP
LANDING
COMPLETE
ABORT
```

Performance aggregation for circle flight uses `CIRCLE_LAP` and
`CENTER_HOLD`. `CIRCLE_ENTRY`, `CIRCLE_EXIT`, `LANDING_PREP`, and `LANDING` are
reported separately and are not mixed into lap RMS metrics.

## Circle Metrics

`circle_metrics.json` is a derived offline artifact with `schema_version=1`.
It records the target center, radius, direction, target laps, actual continuous
angle coverage, completed laps, radial mean/RMS/max error, along-track RMS/max
error, lap horizontal RMS/max error, closure error, entry/lap and lap/exit
continuity, entry/lap/exit maximum errors, and center endpoint error after
exit.

Angle coverage uses continuous unwrap with the declared direction. Samples too
close to the circle center are excluded from angle coverage because their angle
is not meaningful. Completion is based on the unwrapped coverage during
`CIRCLE_LAP`, not only on first and last angle difference.

## Offline Geometry Gates

The generator-level circle checks are mathematical continuity gates, not flight
tracking gates:

```text
circle radius RMS error <= 0.01 m
circle max radius error <= 0.03 m
one-lap closure error <= 0.02 m
entry/exit position jump <= 1e-6 m
entry/exit velocity jump <= 1e-6 m/s
entry/exit acceleration jump <= 1e-5 m/s^2
```

Generated trajectories must also pass finite-field, strict-time, max velocity,
max acceleration, max jerk, frame, and yaw-continuity checks. Limit violations
use whole-trajectory time scaling; individual fields are not clipped.

## Flight Acceptance Draft

The future C5B2 flight acceptance draft is:

```text
publisher exit code = 0
dynamic trajectory ID delivered
setpoint average rate >= 20 Hz
max setpoint interval <= 0.10 s
target/actual coverage >= 95%
circle horizontal RMS <= 0.30 m
circle max horizontal error <= 0.60 m
circle height RMS <= 0.25 m
actual radial RMS <= 0.25 m
max radial error <= 0.50 m
angle coverage >= 350 deg
completed laps >= 0.97
circle closure error <= 0.30 m
exit center endpoint error <= 0.25 m
handoff switch count = 1
adapter FAULT = 0
unexpected OFFBOARD exit = 0
NaN/Inf = 0
CENTER_HOLD complete
AUTO.LAND reserve remaining >= 30 s
output gate open until AUTO.LAND confirmation
output gate closed within 0.5 s after AUTO.LAND confirmation
final armed=false
post-disarm bag recording >= 2 s
```

This stage only implements the offline definitions and tests; it does not use
these gates to validate a flight.

## Data Products

Circle runs should materialize:

```text
summary.json
tracking_metrics.json
delivery_diagnostics.json
handoff_metrics.json
phase_metrics.json
circle_metrics.json
landing_lifecycle_metrics.json
recovery_metrics.json
```

Derived JSON files are written atomically, are deterministic on repeated runs,
and refuse conflicting overwrites unless explicitly using the derived-only
overwrite mode. Bags, logs, PX4 artifacts, ULog files, and core files remain in
`/tmp` and are not tracked by Git.

M0-C5B2B runs use their own directory:

```text
/tmp/uav_m0_c5b2b/run_<timestamp>/
```

The rosbag name is `m0_c5b2b.bag`. Circle runs must not write into the C5B1
line-tracking directory.

## PREP Validation

The PREP dry-run verifies the circle config path
`src/uav_trajectory/config/m0_c5b2_circle.yaml`, `trajectory_type=circle`,
publisher parameters for radius/laps/speed/transition, phase boundaries derived
from the circle configuration, expected JSON artifacts, the C5B2B run root, and
the C5B2B bag name. It does not start PX4, Gazebo, MAVROS, OFFBOARD, arming,
real `AUTO.LAND`, or any flight. The next stage may use a fresh one-shot
authorization to execute the actual PX4 SITL circle tracking experiment.
