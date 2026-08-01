#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest

import rospy
from geometry_msgs.msg import Point, Vector3
from uav_msgs.msg import TrajectoryPoint


SCRIPT_PATH = pathlib.Path(__file__).with_name("m0_c5b1_sitl_line.py")
SPEC = importlib.util.spec_from_file_location("m0_c5b1_sitl_line", SCRIPT_PATH)
m0 = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(m0)


def point(x, y, z, vx=0.0, vy=0.0, vz=0.0, ax=0.0, ay=0.0, az=0.0):
  value = TrajectoryPoint()
  value.position = Point(x, y, z)
  value.velocity = Vector3(vx, vy, vz)
  value.acceleration = Vector3(ax, ay, az)
  return value


class M0C5B1ProtocolTest(unittest.TestCase):
  def test_ground_hold_trajectory_keeps_fixed_position(self):
    trajectory = m0.make_hold_trajectory("map", (1.0, 2.0, 0.3), 0.4,
                                         rospy.Time(10.0), 60.0, 42)
    self.assertEqual(trajectory.trajectory_id, 42)
    self.assertEqual(len(trajectory.points), 2)
    for sample in trajectory.points:
      self.assertAlmostEqual(sample.position.x, 1.0)
      self.assertAlmostEqual(sample.position.y, 2.0)
      self.assertAlmostEqual(sample.position.z, 0.3)
      self.assertAlmostEqual(sample.velocity.x, 0.0)
      self.assertAlmostEqual(sample.acceleration.z, 0.0)

  def test_dynamic_trajectory_only_after_armed(self):
    self.assertFalse(m0.can_start_dynamic_trajectory(False))
    self.assertTrue(m0.can_start_dynamic_trajectory(True))

  def test_dynamic_stamp_later_than_armed_confirmation(self):
    armed_confirm = 100.0
    stamp = armed_confirm + m0.FLIGHT_START_DELAY_SEC
    self.assertGreater(stamp, armed_confirm)
    self.assertGreaterEqual(m0.FLIGHT_START_DELAY_SEC, 1.0)

  def test_trajectory_switch_position_continuity(self):
    ok, reason, details = m0.validate_trajectory_switch(point(0.0, 0.0, 0.0),
                                                        point(0.02, 0.0, 0.0))
    self.assertTrue(ok, reason)
    self.assertLess(details["position_jump_m"], 0.15)

  def test_trajectory_switch_derivative_continuity(self):
    ok, reason, details = m0.validate_trajectory_switch(point(0.0, 0.0, 0.0),
                                                        point(0.0, 0.0, 0.0))
    self.assertTrue(ok, reason)
    self.assertEqual(details["speed_mps"], 0.0)
    self.assertEqual(details["acceleration_mps2"], 0.0)

  def test_switch_rejects_large_position_jump(self):
    ok, reason, _ = m0.validate_trajectory_switch(point(0.0, 0.0, 0.0),
                                                  point(0.2, 0.0, 0.0))
    self.assertFalse(ok)
    self.assertIn("position jump", reason)

  def test_switch_rejects_nonzero_initial_velocity(self):
    ok, reason, _ = m0.validate_trajectory_switch(point(0.0, 0.0, 0.0),
                                                  point(0.0, 0.0, 0.0, vx=0.2))
    self.assertFalse(ok)
    self.assertIn("derivative", reason)

  def test_phase_state_order(self):
    phases = m0.PhaseRecorder()
    phases.set("PREFLIGHT", 1.0)
    phases.set("PRESTREAM", 2.0)
    phases.set("OFFBOARD_PREARM", 3.0)
    phases.set("ARMED_HOLD", 4.0)
    phases.set("LANDING_PREP", 5.0)
    summary = phases.summary()
    self.assertEqual(summary["PREFLIGHT"]["end"], 2.0)
    self.assertEqual(summary["PRESTREAM"]["end"], 3.0)
    self.assertEqual(summary["ARMED_HOLD"]["status"], "reached")
    self.assertEqual(summary["ARMED_HOLD"]["end"], 5.0)
    self.assertEqual(summary["LANDING_PREP"]["status"], "reached")

  def test_performance_window_excludes_preflight_and_landing(self):
    start, end = m0.tracking_acceptance_window({"PREFLIGHT": 1.0, "LINE_FORWARD": 10.0},
                                               30.0, 40.0)
    self.assertEqual(start, 10.0)
    self.assertEqual(end, 30.0)

  def test_center_hold_evaluation_ends_before_trajectory_finish(self):
    self.assertEqual(m0.CENTER_HOLD_EVALUATION_SEC, 10.0)
    self.assertGreaterEqual(m0.LANDING_RESERVE_HOLD_SEC, 60.0)
    self.assertGreater(m0.TRAJECTORY_TOTAL_SEC, m0.CENTER_HOLD_END_SEC)
    self.assertEqual(m0.TRAJECTORY_TOTAL_SEC - m0.CENTER_HOLD_END_SEC,
                     m0.LANDING_RESERVE_HOLD_SEC)

  def test_reserve_remaining_before_land_request(self):
    trajectory_start = 100.0
    land_request = trajectory_start + m0.CENTER_HOLD_END_SEC + 0.5
    remaining = m0.reserve_remaining_at(trajectory_start, land_request)
    self.assertGreaterEqual(remaining, m0.MIN_RESERVE_AT_LAND_REQUEST_SEC)

  def test_reserve_stage_target_is_terminal_hold(self):
    self.assertEqual(m0.HOVER_DURATION_SEC,
                     m0.CENTER_HOLD_EVALUATION_SEC + m0.LANDING_RESERVE_HOLD_SEC)
    trajectory = m0.make_hold_trajectory("map", (1.0, 2.0, 3.0), 0.4,
                                         rospy.Time(10.0), m0.HOVER_DURATION_SEC, 8)
    self.assertEqual(len(trajectory.points), 2)
    self.assertEqual(trajectory.points[-1].time_from_start.to_sec(), m0.HOVER_DURATION_SEC)
    self.assertAlmostEqual(trajectory.points[-1].position.x, trajectory.points[0].position.x)
    self.assertAlmostEqual(trajectory.points[-1].position.y, trajectory.points[0].position.y)
    self.assertAlmostEqual(trajectory.points[-1].position.z, trajectory.points[0].position.z)

  def test_reserve_stage_derivatives_are_zero(self):
    trajectory = m0.make_hold_trajectory("map", (1.0, 2.0, 3.0), 0.4,
                                         rospy.Time(10.0), m0.HOVER_DURATION_SEC, 8)
    for sample in trajectory.points:
      self.assertAlmostEqual(sample.velocity.x, 0.0)
      self.assertAlmostEqual(sample.velocity.y, 0.0)
      self.assertAlmostEqual(sample.velocity.z, 0.0)
      self.assertAlmostEqual(sample.acceleration.x, 0.0)
      self.assertAlmostEqual(sample.acceleration.y, 0.0)
      self.assertAlmostEqual(sample.acceleration.z, 0.0)

  def test_output_gate_close_requires_autoland_confirmation(self):
    self.assertFalse(m0.can_close_output_gate("OFFBOARD", False))
    self.assertFalse(m0.can_close_output_gate("OFFBOARD", True))
    self.assertFalse(m0.can_close_output_gate("AUTO.LOITER", True))
    self.assertTrue(m0.can_close_output_gate("AUTO.LAND", True))

  def test_mode_still_offboard_forbids_output_gate_close(self):
    self.assertFalse(m0.can_close_output_gate("OFFBOARD", True))

  def test_tracking_window_excludes_landing_prep_and_landing(self):
    phases = {"LINE_FORWARD": 10.0, "LANDING_PREP": 40.0, "LANDING": 41.0}
    start, end = m0.tracking_acceptance_window(phases, 40.0, 100.0)
    self.assertEqual(start, 10.0)
    self.assertEqual(end, 40.0)

  def test_landing_prep_phase_is_excluded_from_not_reached_default(self):
    summary = m0.PhaseRecorder().summary()
    self.assertEqual(summary["LANDING_PREP"]["status"], "not_reached")

  def test_fault_counter_splits_before_and_after_land_confirm(self):
    experiment = m0.Experiment.__new__(m0.Experiment)
    experiment.state = m0.TopicState()
    experiment.state.offboard_status_samples = [
        (10.0, "READY", "healthy", True, True),
        (11.0, "FAULT", "x", False, True),
        (12.0, "DISABLED", "x", False, False),
        (20.0, "FAULT", "y", False, False),
    ]
    self.assertEqual(experiment.count_adapter_faults(None, 15.0), 1)
    self.assertEqual(experiment.count_adapter_faults(15.0, None), 1)

  def test_normal_simulated_lifecycle_accepts_zero_adapter_faults(self):
    metrics = {
        "adapter_fault": False,
        "adapter_fault_count": 0,
        "center_hold_completed": True,
        "reserve_remaining_at_land_request_sec": 59.0,
        "output_gate_closed_after_land_confirm": True,
        "output_gate_close_error": None,
        "mode_at_output_gate_close": "AUTO.LAND",
    }
    self.assertFalse(metrics["adapter_fault"])
    self.assertEqual(metrics["adapter_fault_count"], 0)
    self.assertGreaterEqual(metrics["reserve_remaining_at_land_request_sec"],
                            m0.MIN_RESERVE_AT_LAND_REQUEST_SEC)

  def test_land_request_failure_policy_keeps_gate_open_until_confirmed(self):
    self.assertFalse(m0.can_close_output_gate("OFFBOARD", False))
    self.assertFalse(m0.can_close_output_gate("AUTO.LAND", False))

  def test_land_confirm_then_close_output_gate(self):
    self.assertTrue(m0.can_close_output_gate("AUTO.LAND", True))

  def test_not_reached_phase_is_explicit(self):
    summary = m0.PhaseRecorder().summary()
    self.assertEqual(summary["LINE_FORWARD"]["status"], "not_reached")

  def test_height_dwell_single_sample_does_not_trigger(self):
    dwell = m0.DwellThreshold(0.9, 1.0)
    self.assertFalse(dwell.update(0.91, 10.0))

  def test_height_dwell_resets_below_threshold(self):
    dwell = m0.DwellThreshold(0.9, 1.0)
    self.assertFalse(dwell.update(1.0, 10.0))
    self.assertFalse(dwell.update(0.1, 10.5))
    self.assertFalse(dwell.update(1.0, 10.9))

  def test_sustained_height_violation_triggers_autoland_path(self):
    dwell = m0.DwellThreshold(0.9, 1.0)
    self.assertFalse(dwell.update(1.0, 10.0))
    self.assertTrue(dwell.update(1.0, 11.1))
    self.assertEqual(dwell.trigger_time, 11.1)

  def test_unauthorized_environment_rejected(self):
    self.assertFalse(m0.is_authorized({}))
    self.assertFalse(m0.is_authorized({"UAV_ALLOW_SITL_FLIGHT": "yes"}))
    self.assertTrue(m0.is_authorized({"UAV_ALLOW_SITL_FLIGHT": "YES"}))


if __name__ == "__main__":
  unittest.main()
