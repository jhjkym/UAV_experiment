#!/usr/bin/env python3
import importlib.util
import pathlib
import tempfile
import time
import unittest

import rospy
from geometry_msgs.msg import Point, Vector3
from uav_msgs.msg import TrajectoryPoint
from mavros_msgs.msg import State


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


def make_state(armed, mode):
  value = State()
  value.connected = True
  value.armed = armed
  value.mode = mode
  return value


def make_recovery_experiment(armed=True, mode="OFFBOARD"):
  experiment = m0.Experiment.__new__(m0.Experiment)
  experiment.run_dir = pathlib.Path(tempfile.mkdtemp(prefix="m0_c5b1_protocol_"))
  experiment.processes = []
  experiment.state = m0.TopicState()
  experiment.state.mavros_state = make_state(armed, mode)
  experiment.state.offboard_status = None
  experiment.start_position = None
  experiment.target_position = None
  experiment.target_yaw = 0.0
  experiment.mode_request_time = None
  experiment.mode_confirm_time = None
  experiment.arm_request_time = None
  experiment.arm_confirm_time = time.time() if armed else None
  experiment.land_request_time = None
  experiment.land_confirm_time = None
  experiment.land_service_call_started_at = None
  experiment.land_service_response_at = None
  experiment.land_mode_first_observed_at = None
  experiment.offboard_last_observed_at = None
  experiment.disarm_time = None
  experiment.flight_start_time = None
  experiment.line_end_time = None
  experiment.land_complete_time = None
  experiment.output_enabled_time = None
  experiment.output_gate_close_requested_at = None
  experiment.output_disabled_time = None
  experiment.output_gate_close_error = None
  experiment.mode_at_output_gate_close = None
  experiment.land_request_setpoint_rate = None
  experiment.land_request_max_setpoint_gap = None
  experiment.liftoff_time = None
  experiment.height_error_dwell = m0.DwellThreshold(m0.ABORT_HEIGHT_ERROR_M,
                                                   m0.ABORT_HEIGHT_ERROR_DWELL_SEC)
  experiment.horizontal_error_dwell = m0.DwellThreshold(1.0, 0.5)
  experiment.abort_reason = None
  experiment.service_calls = []
  experiment.phase = m0.PhaseRecorder()
  experiment.flight_trajectory_id = None
  experiment.flight_trajectory_stamp = None
  experiment.flight_trajectory_publisher_started_at = None
  experiment.trajectory_end_time = None
  experiment.adapter_fault_first_at = None
  experiment.center_hold_completed = False
  experiment.observed_armed_true = armed
  experiment.final_disarm_confirmed = False
  experiment.abort_recovery = {}
  experiment.compute_metrics = lambda prestream_rate=0.0: {}

  def set_phase(phase):
    experiment.phase.set(phase, time.time())

  def call_mode(mode_name, require_mode=True):
    experiment.service_calls.append({"service": "set_mode", "mode": mode_name})
    if mode_name == "AUTO.LAND":
      now = time.time()
      experiment.land_service_call_started_at = now
      experiment.land_service_response_at = now
      experiment.land_request_time = now
      experiment.land_confirm_time = now
      experiment.state.mavros_state.mode = "AUTO.LAND"

  def call_set_output(enabled):
    experiment.service_calls.append({"service": "set_output_enabled", "value": enabled})
    if not enabled:
      experiment.output_gate_close_requested_at = time.time()
      experiment.output_disabled_time = time.time()
      experiment.mode_at_output_gate_close = experiment.state.mavros_state.mode

  def wait_disarmed(stable_sec=2.0, timeout=90.0):
    experiment.state.mavros_state.armed = False
    experiment.disarm_time = time.time()
    experiment.final_disarm_confirmed = True
    return True

  experiment.set_phase = set_phase
  experiment.call_mode = call_mode
  experiment.call_set_output = call_set_output
  experiment.wait_for_disarm_stable = wait_disarmed
  return experiment


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

  def test_publisher_sigsegv_before_arming_does_not_request_land(self):
    experiment = make_recovery_experiment(armed=False, mode="")
    recovery = experiment.recover_after_failure("dynamic publisher exit_code=-11")
    self.assertFalse(recovery["ever_armed"])
    self.assertFalse(recovery["land_requested"])
    self.assertFalse(any(call.get("mode") == "AUTO.LAND" for call in experiment.service_calls))
    self.assertTrue((experiment.run_dir / "summary.json").exists())

  def test_publisher_sigsegv_after_arming_requests_autoland(self):
    experiment = make_recovery_experiment(armed=True, mode="OFFBOARD")
    recovery = experiment.recover_after_failure("dynamic publisher exit_code=-11")
    self.assertTrue(recovery["ever_armed"])
    self.assertTrue(recovery["land_requested"])
    self.assertTrue(recovery["land_confirmed"])
    self.assertTrue(recovery["final_disarm_confirmed"])
    self.assertIn({"service": "set_mode", "mode": "AUTO.LAND"}, experiment.service_calls)

  def test_recovery_closes_output_gate_only_after_autoland_confirm(self):
    experiment = make_recovery_experiment(armed=True, mode="OFFBOARD")
    experiment.recover_after_failure("python exception")
    self.assertEqual(experiment.mode_at_output_gate_close, "AUTO.LAND")
    self.assertIsNotNone(experiment.land_confirm_time)
    self.assertIsNotNone(experiment.output_disabled_time)
    self.assertGreaterEqual(experiment.output_disabled_time, experiment.land_confirm_time)

  def test_recovery_keeps_original_failure_reason(self):
    experiment = make_recovery_experiment(armed=True, mode="OFFBOARD")
    recovery = experiment.recover_after_failure("metrics calculation failed")
    self.assertEqual(recovery["original_error"], "metrics calculation failed")
    self.assertEqual(experiment.abort_reason, "metrics calculation failed")

  def test_disarm_timeout_is_written_as_failure(self):
    experiment = make_recovery_experiment(armed=True, mode="OFFBOARD")

    def no_disarm(stable_sec=2.0, timeout=90.0):
      return False

    experiment.wait_for_disarm_stable = no_disarm
    recovery = experiment.recover_after_failure("dynamic publisher exit_code=-11")
    self.assertFalse(recovery["final_disarm_confirmed"])
    self.assertTrue(experiment.state.mavros_state.armed)
    summary = (experiment.run_dir / "summary.json").read_text(encoding="utf-8")
    self.assertIn('"final_disarm_confirmed": false', summary)

  def test_recovery_never_forces_disarm(self):
    experiment = make_recovery_experiment(armed=True, mode="OFFBOARD")
    experiment.recover_after_failure("dynamic publisher exit_code=-11")
    forced_disarm_calls = [
        call for call in experiment.service_calls
        if call.get("service") == "arming" and call.get("value") is False
    ]
    self.assertEqual(forced_disarm_calls, [])


if __name__ == "__main__":
  unittest.main()
