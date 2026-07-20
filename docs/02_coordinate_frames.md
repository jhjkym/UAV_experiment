# Coordinate Frames

Algorithm convention:
- World frame: ENU.
- Body frame: FLU.
- Default world frame id: `map`.
- Default body frame id: `base_link`.

PX4 convention:
- World frame: NED.
- Body frame: FRD.

MAVROS convention:
- Standard MAVROS topics expose ROS coordinate conventions where supported by
  MAVROS plugins.
- Do not repeat ENU/NED conversions on standard MAVROS topic data that MAVROS
  already adapts.

Custom conversion utilities in `uav_px4_bridge` are only for:
- Non-MAVROS data.
- Simulation ground truth adapters.
- Offline logs.
- Explicitly bypassed MAVROS interfaces.

Implemented vector conversions:
- ENU vector to NED vector: `(x_e, y_n, z_u) -> (y_n, x_e, -z_u)`.
- NED vector to ENU vector: `(x_n, y_e, z_d) -> (y_e, x_n, -z_d)`.
- FLU vector to FRD vector: `(x_f, y_l, z_u) -> (x_f, -y_l, -z_u)`.
- FRD vector to FLU vector: `(x_f, y_r, z_d) -> (x_f, -y_r, -z_d)`.

Quaternion frame conversion is intentionally not implemented in M0-B. It must
be added only after the exact input and output attitude conventions are written
and reviewed.
