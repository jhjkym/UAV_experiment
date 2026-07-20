# Project Scope

The project studies quadrotor perception, planning, and safety control in
uncertain mixed static-dynamic environments.

The planned technical route is:
1. Dynamic obstacle perception, tracking, short-horizon prediction, and online
   uncertainty calibration.
2. Cross-layer dynamic safety tube integrating perception error, prediction
   error, system delay, and closed-loop tracking error.
3. Coordinated nominal and emergency trajectory planning based on the safety
   tube.
4. Risk-adaptive NMPC-HOCBF safety control with control-margin feedback.

M0-B is limited to the engineering skeleton:
- ROS 1 Noetic catkin workspace.
- Shared messages.
- Frame convention documentation.
- Pure frame conversion tests.

M0-B does not implement perception, prediction, planning, control, PX4 SITL,
MAVROS setpoint publication, arming, Offboard switching, or real hardware
connection.
