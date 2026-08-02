#!/usr/bin/env python3
import importlib.util
import json
import math
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

MATERIALIZER_PATH = SCRIPT_PATH.parents[1] / "analysis" / "materialize_m0_c5b1_artifacts.py"
MATERIALIZER_SPEC = importlib.util.spec_from_file_location("materialize_m0_c5b1_artifacts",
                                                           MATERIALIZER_PATH)
materializer = importlib.util.module_from_spec(MATERIALIZER_SPEC)
assert MATERIALIZER_SPEC.loader is not None
MATERIALIZER_SPEC.loader.exec_module(materializer)


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


def write_text(path, text):
  path.write_text(text, encoding="utf-8")


def make_materializer_run(success=True):
  run_dir = pathlib.Path(tempfile.mkdtemp(prefix="run_20260802_162751_"))
  metrics = {
      "setpoint_average_rate_hz": 30.0,
      "max_setpoint_gap_before_land_request": 0.034,
      "adapter_fault_count": 0,
      "adapter_fault_before_land_request": 0,
      "adapter_fault_before_land_confirm": 0,
      "final_armed": False,
      "landing_reserve_sec": 60.0,
      "reserve_remaining_at_land_request_sec": 59.929,
      "reserve_remaining_at_land_confirm_sec": 58.925,
      "land_request_to_confirm_sec": 1.004,
      "land_confirm_to_output_gate_close_sec": 0.0101,
      "mode_at_output_gate_close": "AUTO.LAND",
      "landing_to_disarm_sec": 9.076,
      "nan_or_inf_count": 0,
      "phase_metrics": {
          "LINE_FORWARD": {
              "status": "reached", "sample_count": 2, "duration_sec": 5.0,
              "horizontal_rms_m": 0.05, "height_mean_error_m": 0.01,
              "height_rms_m": 0.02, "max_speed_mps": 0.4, "coverage": 1.0,
          },
          "LINE_REVERSE": {
              "status": "reached", "sample_count": 2, "duration_sec": 5.0,
              "horizontal_rms_m": 0.06, "height_mean_error_m": 0.01,
              "height_rms_m": 0.02, "max_speed_mps": 0.5, "coverage": 1.0,
          },
          "LINE_RETURN": {
              "status": "reached", "sample_count": 2, "duration_sec": 5.0,
              "horizontal_rms_m": 0.07, "height_mean_error_m": 0.01,
              "height_rms_m": 0.02, "max_speed_mps": 0.4, "coverage": 1.0,
          },
          "CENTER_HOLD": {
              "status": "reached", "sample_count": 2, "duration_sec": 10.0,
              "horizontal_rms_m": 0.04, "height_mean_error_m": 0.01,
              "height_rms_m": 0.02, "max_speed_mps": 0.1, "coverage": 1.0,
          },
      },
  }
  phase_times = {
      "PREFLIGHT": {"status": "reached", "start": 1.0, "end": 2.0},
      "PRESTREAM": {"status": "reached", "start": 2.0, "end": 4.0},
      "OFFBOARD_PREARM": {"status": "reached", "start": 4.0, "end": 8.0},
      "ARMED_HOLD": {"status": "reached", "start": 8.0, "end": 10.0},
      "CLIMB": {"status": "reached", "start": 10.0, "end": 20.0},
      "LINE_FORWARD": {"status": "reached", "start": 20.0, "end": 25.0},
      "LINE_REVERSE": {"status": "reached", "start": 25.0, "end": 30.0},
      "LINE_RETURN": {"status": "reached", "start": 30.0, "end": 35.0},
      "CENTER_HOLD": {"status": "reached", "start": 35.0, "end": 45.0},
      "LANDING_PREP": {"status": "reached", "start": 45.0, "end": 46.0},
      "LANDING": {"status": "reached", "start": 46.0, "end": 58.0},
      "COMPLETE": {"status": "reached", "start": 58.0, "end": 58.1},
  }
  summary = {
      "run_dir": str(run_dir),
      "status": "complete" if success else "failed",
      "ground_hold_trajectory_id": 11,
      "flight_trajectory_id": 22 if success else None,
      "flight_trajectory_stamp": 9.5,
      "center_hold_end_time": 45.0,
      "land_request_time": 45.1 if success else None,
      "land_confirm_time": 46.0 if success else None,
      "land_service_call_started_at": 45.1 if success else None,
      "land_service_response_at": 45.2 if success else None,
      "land_mode_first_observed_at": 46.0 if success else None,
      "offboard_last_observed_at": 45.9 if success else None,
      "output_gate_close_requested_at": 46.01 if success else None,
      "output_gate_closed_at": 46.02 if success else None,
      "disarm_at": 55.0 if success else None,
      "phase_times": phase_times if success else {"PREFLIGHT": {"status": "reached", "start": 1.0, "end": 2.0}},
      "metrics": metrics if success else {"final_armed": False, "nan_or_inf_count": 0},
  }
  tracking = {
      "horizontal_rms_error_m": 0.069,
      "horizontal_max_error_m": 0.16,
      "height_mean_error_m": 0.05,
      "height_rms_error_m": 0.064,
      "position_3d_rms_error_m": 0.094,
      "velocity_rms_error_mps": 0.078,
      "input_quality": {"total_nan_or_inf_values": 0},
  }
  write_text(run_dir / "summary.json", json.dumps(summary))
  write_text(run_dir / "tracking_metrics.json", json.dumps(tracking))
  if success:
    write_text(run_dir / "dynamic_flight_trajectory.log",
               "[INFO] generated x id=22 points=1 topic=/uav/trajectory\n"
               "[INFO] publish_once_ready trajectory_id=22 subscriber_count=1 "
               "wait_sec=0.180876 planned_publish_count=3 header_stamp=9.5\n"
               "[INFO] publish_once_message trajectory_id=22 publish_index=1 "
               "publish_wall_time=8.1 subscriber_count=1 header_stamp=9.5\n"
               "[INFO] publish_once_message trajectory_id=22 publish_index=2 "
               "publish_wall_time=8.2 subscriber_count=1 header_stamp=9.5\n"
               "[INFO] publish_once_message trajectory_id=22 publish_index=3 "
               "publish_wall_time=8.3 subscriber_count=1 header_stamp=9.5\n"
               "[INFO] publish_once_result exit_reason=published trajectory_id=22 "
               "subscriber_count=1 wait_sec=0.180876 publish_count=3\n")
    write_text(run_dir / "m0_c5b1_project_nodes.log",
               "[INFO] Queued pending trajectory active_id=11 pending_id=22 planned_switch=9.5\n"
               "[INFO] Promoted pending trajectory id=22 planned_switch=9.5 "
               "actual_switch=9.5178 position_jump=0.028456 velocity_jump=0.000000 "
               "acceleration_jump=0.000000\n")
  else:
    write_text(run_dir / "dynamic_flight_trajectory.log", "")
    write_text(run_dir / "m0_c5b1_project_nodes.log", "")
  write_text(run_dir / "rosbag.log", "[INFO] Recording to 'm0_c5b1.bag'.\n")
  write_text(run_dir / "m0_c5b1.bag", "do not overwrite")
  return run_dir


def make_circle_materializer_run(direction="ccw", include_near_center=False):
  run_dir = make_materializer_run(True)
  summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
  summary["trajectory_type"] = "circle"
  summary["circle"] = {
      "center": {"x": 0.0, "y": 0.0, "z": 1.0},
      "radius_m": 1.0,
      "direction": direction,
      "target_laps": 1.0,
  }
  for line_phase in ["LINE_FORWARD", "LINE_REVERSE", "LINE_RETURN"]:
    summary["phase_times"].pop(line_phase, None)
  summary["phase_times"].update({
      "CLIMB": {"status": "reached", "start": 10.0, "end": 18.0},
      "CLIMB_HOLD": {"status": "reached", "start": 18.0, "end": 20.0},
      "CIRCLE_ENTRY": {"status": "reached", "start": 20.0, "end": 24.0},
      "CIRCLE_LAP": {"status": "reached", "start": 24.0, "end": 40.0},
      "CIRCLE_EXIT": {"status": "reached", "start": 40.0, "end": 44.0},
      "CENTER_HOLD": {"status": "reached", "start": 44.0, "end": 54.0},
      "LANDING_PREP": {"status": "reached", "start": 54.0, "end": 55.0},
      "LANDING": {"status": "reached", "start": 55.0, "end": 66.0},
      "COMPLETE": {"status": "reached", "start": 66.0, "end": 66.1},
  })
  summary["metrics"]["phase_metrics"].update({
      "CIRCLE_ENTRY": {"status": "reached", "sample_count": 3, "coverage": 1.0,
                       "horizontal_rms_m": 0.01, "height_mean_error_m": 0.0,
                       "height_rms_m": 0.01, "max_speed_mps": 0.4,
                       "horizontal_max_m": 0.02},
      "CIRCLE_LAP": {"status": "reached", "sample_count": 33, "coverage": 1.0,
                     "horizontal_rms_m": 0.05, "height_mean_error_m": 0.0,
                     "height_rms_m": 0.02, "max_speed_mps": 0.4,
                     "horizontal_max_m": 0.08},
      "CIRCLE_EXIT": {"status": "reached", "sample_count": 3, "coverage": 1.0,
                      "horizontal_rms_m": 0.01, "height_mean_error_m": 0.0,
                      "height_rms_m": 0.01, "max_speed_mps": 0.4,
                      "horizontal_max_m": 0.02},
  })
  write_text(run_dir / "summary.json", json.dumps(summary))
  sign = -1.0 if direction == "cw" else 1.0
  rows = ["time,target_x,target_y,target_z,actual_x,actual_y,actual_z,target_vx,target_vy,target_vz,actual_vx,actual_vy,actual_vz\n"]
  rows.append("20.0,0.0,0.0,1.0,0.0,0.0,1.0,0,0,0,0,0,0\n")
  first_actual_y = 0.01 if direction == "ccw" else -0.01
  rows.append(f"23.999,1.0,0.0,1.0,1.05,{first_actual_y},1.0,0,0,0,0,0,0\n")
  if include_near_center:
    rows.append("24.5,0.0,0.0,1.0,0.0,0.0,1.0,0,0,0,0,0,0\n")
  for index in range(33):
    theta = sign * 2.0 * math.pi * index / 32.0
    tx = math.cos(theta)
    ty = math.sin(theta)
    # Add a small deterministic radial and along-track error.
    radial_error = 0.05
    tangent_x = -math.sin(theta) * sign
    tangent_y = math.cos(theta) * sign
    ax = (1.0 + radial_error) * tx + 0.01 * tangent_x
    ay = (1.0 + radial_error) * ty + 0.01 * tangent_y
    rows.append(f"{24.0 + 16.0 * index / 32.0:.3f},{tx:.9f},{ty:.9f},1.0,"
                f"{ax:.9f},{ay:.9f},1.0,0,0,0,0,0,0\n")
  rows.append("44.0,0.0,0.0,1.0,0.02,0.0,1.0,0,0,0,0,0,0\n")
  rows.append("54.0,0.0,0.0,1.0,0.02,0.0,1.0,0,0,0,0,0,0\n")
  write_text(run_dir / "tracking_samples.csv", "".join(rows))
  return run_dir


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

  def test_materializer_complete_success_generates_five_json_files(self):
    run_dir = make_materializer_run(True)
    result = materializer.materialize_run_dir(run_dir)
    self.assertEqual(set(result["written"]), materializer.BASE_DERIVED_FILES)
    for name in materializer.BASE_DERIVED_FILES:
      payload = json.loads((run_dir / name).read_text(encoding="utf-8"))
      self.assertEqual(payload["schema_version"], 1)
      self.assertTrue(payload["generated_offline"])
      self.assertEqual(payload["experiment_id"], "run_20260802_162751")

  def test_materializer_delivery_fields_for_success(self):
    run_dir = make_materializer_run(True)
    materializer.materialize_run_dir(run_dir)
    delivery = json.loads((run_dir / "delivery_diagnostics.json").read_text(encoding="utf-8"))
    self.assertEqual(delivery["trajectory_id"], 22)
    self.assertEqual(delivery["publish_repeat_count"], 3)
    self.assertEqual(delivery["publisher_exit_code"], 0)
    self.assertFalse(delivery["sigsegv"])
    self.assertTrue(delivery["delivery_passed"])

  def test_materializer_handoff_fields_for_success(self):
    run_dir = make_materializer_run(True)
    materializer.materialize_run_dir(run_dir)
    handoff = json.loads((run_dir / "handoff_metrics.json").read_text(encoding="utf-8"))
    self.assertEqual(handoff["switch_count"], 1)
    self.assertAlmostEqual(handoff["switch_timing_error_sec"], 0.0178)
    self.assertAlmostEqual(handoff["position_jump_m"], 0.028456)
    self.assertTrue(handoff["handoff_passed"])

  def test_materializer_phase_and_landing_outputs(self):
    run_dir = make_materializer_run(True)
    materializer.materialize_run_dir(run_dir)
    phase = json.loads((run_dir / "phase_metrics.json").read_text(encoding="utf-8"))
    lifecycle = json.loads((run_dir / "landing_lifecycle_metrics.json").read_text(encoding="utf-8"))
    self.assertIn("PENDING_HANDOFF", phase["phases"])
    self.assertEqual(phase["performance_phases"], materializer.PERFORMANCE_PHASES)
    self.assertGreaterEqual(lifecycle["reserve_remaining_at_land_request_sec"], 30.0)
    self.assertLessEqual(lifecycle["land_confirm_to_output_gate_close_sec"], 0.5)
    self.assertTrue(lifecycle["lifecycle_passed"])

  def test_materializer_recovery_not_triggered_file(self):
    run_dir = make_materializer_run(True)
    materializer.materialize_run_dir(run_dir)
    recovery = json.loads((run_dir / "recovery_metrics.json").read_text(encoding="utf-8"))
    self.assertFalse(recovery["recovery_triggered"])
    self.assertFalse(recovery["forced_disarm_called"])
    self.assertTrue(recovery["final_disarm_confirmed"])

  def test_materializer_early_failure_generates_applicable_files(self):
    run_dir = make_materializer_run(False)
    result = materializer.materialize_run_dir(run_dir)
    self.assertIn("delivery_diagnostics.json", result["written"])
    self.assertIn("recovery_metrics.json", result["written"])
    handoff = json.loads((run_dir / "handoff_metrics.json").read_text(encoding="utf-8"))
    self.assertEqual(handoff["switch_count"], 0)
    self.assertFalse(handoff["handoff_passed"])

  def test_materializer_repeat_is_idempotent(self):
    run_dir = make_materializer_run(True)
    materializer.materialize_run_dir(run_dir)
    before = {
        name: (run_dir / name).read_text(encoding="utf-8")
        for name in materializer.BASE_DERIVED_FILES
    }
    result = materializer.materialize_run_dir(run_dir)
    self.assertEqual(set(result["unchanged"]), materializer.BASE_DERIVED_FILES)
    after = {
        name: (run_dir / name).read_text(encoding="utf-8")
        for name in materializer.BASE_DERIVED_FILES
    }
    self.assertEqual(before, after)

  def test_materializer_conflicting_file_refuses_without_overwrite(self):
    run_dir = make_materializer_run(True)
    write_text(run_dir / "delivery_diagnostics.json", "{}\n")
    with self.assertRaises(FileExistsError):
      materializer.materialize_run_dir(run_dir)

  def test_materializer_overwrite_derived_does_not_touch_bag(self):
    run_dir = make_materializer_run(True)
    bag_before = (run_dir / "m0_c5b1.bag").read_text(encoding="utf-8")
    write_text(run_dir / "delivery_diagnostics.json", "{}\n")
    materializer.materialize_run_dir(run_dir, overwrite_derived_only=True)
    self.assertEqual((run_dir / "m0_c5b1.bag").read_text(encoding="utf-8"), bag_before)

  def test_materializer_non_finite_rejected(self):
    run_dir = make_materializer_run(True)
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    summary["metrics"]["setpoint_average_rate_hz"] = float("nan")
    write_text(run_dir / "summary.json", json.dumps(summary, allow_nan=True))
    with self.assertRaises(ValueError):
      materializer.materialize_run_dir(run_dir)

  def test_materializer_consistency_detects_trajectory_id_mismatch(self):
    run_dir = make_materializer_run(True)
    text = (run_dir / "m0_c5b1_project_nodes.log").read_text(encoding="utf-8")
    write_text(run_dir / "m0_c5b1_project_nodes.log", text.replace("pending_id=22", "pending_id=23"))
    with self.assertRaises(ValueError):
      materializer.materialize_run_dir(run_dir)

  def test_circle_materializer_generates_circle_json(self):
    run_dir = make_circle_materializer_run()
    result = materializer.materialize_run_dir(run_dir)
    self.assertIn("circle_metrics.json", result["written"])
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertEqual(circle["schema_version"], 1)
    self.assertEqual(circle["target_radius_m"], 1.0)
    self.assertEqual(circle["direction"], "ccw")
    self.assertTrue(circle["circle_passed"])

  def test_circle_radius_rms_and_radial_error(self):
    run_dir = make_circle_materializer_run()
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertAlmostEqual(circle["radial_mean_error_m"], 0.05, places=3)
    self.assertAlmostEqual(circle["radial_rms_error_m"], 0.05, places=3)
    self.assertAlmostEqual(circle["radial_max_error_m"], 0.05, places=3)

  def test_circle_along_track_error(self):
    run_dir = make_circle_materializer_run()
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertAlmostEqual(circle["along_track_rms_error_m"], 0.01, places=6)
    self.assertAlmostEqual(circle["along_track_max_error_m"], 0.01, places=6)

  def test_circle_ccw_angle_coverage(self):
    run_dir = make_circle_materializer_run("ccw")
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertGreaterEqual(circle["actual_angle_coverage_deg"], 359.0)
    self.assertGreaterEqual(circle["completed_laps"], 0.99)

  def test_circle_cw_angle_coverage(self):
    run_dir = make_circle_materializer_run("cw")
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertGreaterEqual(circle["actual_angle_coverage_deg"], 359.0)
    self.assertGreaterEqual(circle["completed_laps"], 0.99)

  def test_circle_closure_error(self):
    run_dir = make_circle_materializer_run()
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertLessEqual(circle["closure_error_m"], 0.02)

  def test_circle_low_speed_near_center_samples_are_not_angles(self):
    run_dir = make_circle_materializer_run(include_near_center=True)
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertGreaterEqual(circle["near_center_rejected_count"], 1)
    self.assertGreaterEqual(circle["actual_angle_coverage_deg"], 359.0)

  def test_circle_lap_phase_excludes_entry_and_exit(self):
    run_dir = make_circle_materializer_run()
    materializer.materialize_run_dir(run_dir)
    phase = json.loads((run_dir / "phase_metrics.json").read_text(encoding="utf-8"))
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertEqual(phase["performance_phases"], materializer.CIRCLE_PERFORMANCE_PHASES)
    self.assertEqual(circle["angle_sample_count"], 33)

  def test_circle_phase_order_is_monotonic(self):
    run_dir = make_circle_materializer_run()
    result = materializer.materialize_run_dir(run_dir)
    self.assertEqual(result["consistency_errors"], [])

  def test_circle_early_failure_does_not_forge_metrics(self):
    run_dir = make_materializer_run(False)
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    summary["trajectory_type"] = "circle"
    summary["circle"] = {"center": {"x": 0.0, "y": 0.0, "z": 1.0},
                         "radius_m": 1.0, "direction": "ccw", "target_laps": 1.0}
    write_text(run_dir / "summary.json", json.dumps(summary))
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertFalse(circle["circle_available"])
    self.assertFalse(circle["circle_passed"])

  def test_circle_missing_radius_or_laps_are_not_defaulted(self):
    run_dir = make_materializer_run(False)
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    summary["trajectory_type"] = "circle"
    summary["circle"] = {"center": {"x": 0.0, "y": 0.0, "z": 1.0}, "direction": "ccw"}
    write_text(run_dir / "summary.json", json.dumps(summary))
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    self.assertFalse(circle["circle_available"])
    self.assertIn("radius", circle["reason"])
    self.assertNotIn("target_radius_m", circle)
    self.assertNotIn("target_laps", circle)

  def test_circle_metrics_fields_complete(self):
    run_dir = make_circle_materializer_run()
    materializer.materialize_run_dir(run_dir)
    circle = json.loads((run_dir / "circle_metrics.json").read_text(encoding="utf-8"))
    for key in [
        "circle_center", "target_radius_m", "direction", "target_laps",
        "actual_angle_coverage_deg", "completed_laps", "radial_rms_error_m",
        "radial_max_error_m", "along_track_rms_error_m", "closure_error_m",
        "entry_to_lap_continuity", "lap_to_exit_continuity", "circle_passed",
    ]:
      self.assertIn(key, circle)

  def test_materializer_is_offline_only(self):
    source = MATERIALIZER_PATH.read_text(encoding="utf-8")
    for forbidden in ["rospy", "roslaunch", "set_mode", "arming"]:
      self.assertNotIn(forbidden, source)


if __name__ == "__main__":
  unittest.main()
