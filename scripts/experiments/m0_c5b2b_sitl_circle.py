#!/usr/bin/env python3
"""M0-C5B2B PX4 SITL-only circle tracking experiment entry.

The real flight path reuses the guarded M0-C5B1 lifecycle by subclassing its
Experiment. Circle-specific behavior is limited to configuration, phase
boundaries, summary fields, and acceptance checks.
"""

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Dict, Optional

import rospy

import dynamic_tracking_mission as mission
import m0_c5b1_sitl_line as line


REPO_DIR = Path("/home/tom/UAV_experiment")
LOG_ROOT = Path("/tmp/uav_m0_c5b2b")
CONFIG_PATH = REPO_DIR / "src/uav_trajectory/config/m0_c5b2_circle.yaml"
AUTH_TOKEN = "M0_C5B2B_CIRCLE_SITL_ONLY"
PHASE_ORDER = (
    "PREFLIGHT",
    "PRESTREAM",
    "OFFBOARD_PREARM",
    "ARMED_HOLD",
    "PENDING_HANDOFF",
    "CLIMB",
    "CLIMB_HOLD",
    "CIRCLE_ENTRY",
    "CIRCLE_LAP",
    "CIRCLE_EXIT",
    "CENTER_HOLD",
    "LANDING_PREP",
    "LANDING",
    "COMPLETE",
    "ABORT",
)

CIRCLE_SPEC = mission.MissionSpec(
    name="M0-C5B2B",
    trajectory_type="circle",
    log_root=LOG_ROOT,
    bag_name="m0_c5b2b.bag",
    config_path=CONFIG_PATH,
    project_launch='roslaunch "$(rospack find uav_bringup)/launch/sim/m0_c5b2b_circle_tracking.launch"',
    process_name="m0_c5b2b_project_nodes",
    ros_node_name="m0_c5b2b_sitl_circle_experiment",
    auth_token=AUTH_TOKEN,
    phase_order=PHASE_ORDER,
    performance_phases=("CIRCLE_LAP", "CENTER_HOLD"),
)


class CircleExperiment(line.Experiment):
  def __init__(self, auth_file: Path) -> None:
    self.config = load_circle_config()
    self.boundaries = mission.circle_phase_boundaries(self.config)
    self.auth_details = mission.consume_one_shot_auth(auth_file, AUTH_TOKEN)
    os.environ["UAV_ALLOW_SITL_FLIGHT"] = "YES"
    super().__init__()
    self.phase = self.phase_recorder()

  def log_root(self) -> Path:
    return CIRCLE_SPEC.log_root

  def phase_recorder(self):
    return line.phase_recorder_with_order(list(CIRCLE_SPEC.phase_order))()

  def ros_node_name(self) -> str:
    return CIRCLE_SPEC.ros_node_name

  def project_launch_command(self) -> str:
    return CIRCLE_SPEC.project_launch

  def project_process_name(self) -> str:
    return CIRCLE_SPEC.process_name

  def bag_name(self) -> str:
    return CIRCLE_SPEC.bag_name

  def center_hold_end_sec(self) -> float:
    return self.boundaries["CENTER_HOLD_END"]

  def trajectory_total_sec(self) -> float:
    return self.boundaries["TRAJECTORY_TOTAL"]

  def dynamic_trajectory_command(self, frame_id: str) -> str:
    assert self.start_position is not None
    return build_circle_publisher_command(self.config, frame_id, self.start_position,
                                          self.target_yaw)

  def update_phase_from_time(self, now: float) -> None:
    if self.flight_trajectory_stamp is None:
      return
    elapsed = now - self.flight_trajectory_stamp
    if elapsed < self.boundaries["ARMED_HOLD_END"]:
      self.set_phase("ARMED_HOLD")
    elif elapsed < self.boundaries["CLIMB_END"]:
      self.set_phase("CLIMB")
    elif elapsed < self.boundaries["CLIMB_HOLD_END"]:
      self.set_phase("CLIMB_HOLD")
    elif elapsed < self.boundaries["CIRCLE_ENTRY_END"]:
      self.set_phase("CIRCLE_ENTRY")
    elif elapsed < self.boundaries["CIRCLE_LAP_END"]:
      self.set_phase("CIRCLE_LAP")
    elif elapsed < self.boundaries["CIRCLE_EXIT_END"]:
      self.set_phase("CIRCLE_EXIT")
    elif elapsed < self.boundaries["CENTER_HOLD_END"]:
      self.set_phase("CENTER_HOLD")
    else:
      self.set_phase("LANDING_PREP")

  def mission_summary_fields(self) -> Dict[str, object]:
    radius = float(self.config["circle_radius_m"])
    altitude = self.target_position[2] if self.target_position else None
    center = None
    entry = None
    if self.start_position:
      center = {"x": self.start_position[0], "y": self.start_position[1], "z": altitude}
      entry = {"x": self.start_position[0] + radius, "y": self.start_position[1], "z": altitude}
    return {
        "trajectory_type": "circle",
        "circle": {
            "center": center,
            "entry_point": entry,
            "radius_m": radius,
            "direction": "ccw",
            "target_laps": float(self.config["circle_laps"]),
            "initial_tangent": "+y",
            "flight_altitude_m": altitude,
            "final_hold_point": center,
        },
        "circle_radius_m": radius,
        "circle_laps": float(self.config["circle_laps"]),
        "circle_tangent_speed_mps": float(self.config["circle_tangent_speed_mps"]),
        "transition_duration_sec": float(self.config["transition_duration_sec"]),
        "circle_phase_boundaries": self.boundaries,
        "config_path": str(CONFIG_PATH),
    }

  def compute_turnpoint_metrics(self) -> Dict[str, object]:
    if not self.start_position or not self.target_position or not self.flight_trajectory_stamp:
      return {}
    center_time = self.flight_trajectory_stamp + self.boundaries["CIRCLE_EXIT_END"]
    nearest = self.nearest_actual(center_time)
    if nearest is None:
      return {}
    error = math.sqrt((nearest[1] - self.start_position[0]) ** 2 +
                      (nearest[2] - self.start_position[1]) ** 2 +
                      (nearest[3] - self.target_position[2]) ** 2)
    return {"exit_center_endpoint_error_m": error, "center_endpoint_error_m": error}

  def mission_passed(self, summary: Dict[str, object]) -> bool:
    metrics = summary["metrics"]
    circle_metrics = {}
    circle_path = self.run_dir / "circle_metrics.json"
    if circle_path.exists():
      circle_metrics = json.loads(circle_path.read_text(encoding="utf-8"))
    return (
        metrics.get("abort_reason") is None and
        metrics.get("setpoint_average_rate_hz", 0.0) >= 20.0 and
        metrics.get("data_coverage_target", 0.0) >= 0.95 and
        metrics.get("data_coverage_actual", 0.0) >= 0.95 and
        metrics.get("height_rms_error_m", 99.0) <= 0.25 and
        not metrics.get("nan_or_inf") and
        metrics.get("nan_or_inf_count", 1) == 0 and
        not metrics.get("adapter_fault") and
        metrics.get("adapter_fault_count", 1) == 0 and
        metrics.get("unexpected_offboard_exit_count", 1) == 0 and
        metrics.get("center_hold_completed") is True and
        metrics.get("reserve_remaining_at_land_request_sec", 0.0) >= line.MIN_RESERVE_AT_LAND_REQUEST_SEC and
        metrics.get("output_gate_closed_after_land_confirm") is True and
        metrics.get("output_gate_close_error") is None and
        metrics.get("mode_at_output_gate_close") == "AUTO.LAND" and
        metrics.get("final_armed") is False and
        summary["phase_times"].get("CIRCLE_ENTRY", {}).get("status") == "reached" and
        summary["phase_times"].get("CIRCLE_LAP", {}).get("status") == "reached" and
        summary["phase_times"].get("CIRCLE_EXIT", {}).get("status") == "reached" and
        summary["phase_times"].get("CENTER_HOLD", {}).get("status") == "reached" and
        circle_metrics.get("circle_horizontal_rms_error_m", 99.0) <= 0.30 and
        circle_metrics.get("circle_horizontal_max_error_m", 99.0) <= 0.60 and
        circle_metrics.get("radial_rms_error_m", 99.0) <= 0.25 and
        circle_metrics.get("radial_max_error_m", 99.0) <= 0.50 and
        circle_metrics.get("actual_angle_coverage_deg", 0.0) >= 350.0 and
        circle_metrics.get("completed_laps", 0.0) >= 0.97 and
        circle_metrics.get("closure_error_m", 99.0) <= 0.30 and
        circle_metrics.get("exit_center_endpoint_error_m", 99.0) <= 0.25)


def load_circle_config() -> Dict[str, object]:
  config = mission.parse_simple_yaml(CONFIG_PATH)
  required = {
      "trajectory_type": "circle",
      "yaw_mode": "fixed",
  }
  for key, expected in required.items():
    if config.get(key) != expected:
      raise RuntimeError(f"{CONFIG_PATH} must set {key}: {expected}")
  for key in [
      "initial_hold_sec",
      "initial_climb_duration_sec",
      "post_climb_hold_sec",
      "circle_radius_m",
      "circle_laps",
      "circle_tangent_speed_mps",
      "transition_duration_sec",
      "center_hold_evaluation_sec",
      "landing_reserve_hold_sec",
  ]:
    mission.require_float(config, key)
  if float(config["circle_radius_m"]) != 1.0:
    raise RuntimeError("M0-C5B2B requires circle_radius_m=1.0")
  if float(config["circle_laps"]) != 1.0:
    raise RuntimeError("M0-C5B2B requires circle_laps=1.0")
  if float(config["circle_tangent_speed_mps"]) != 0.4:
    raise RuntimeError("M0-C5B2B requires circle_tangent_speed_mps=0.4")
  if float(config["transition_duration_sec"]) != 4.0:
    raise RuntimeError("M0-C5B2B requires transition_duration_sec=4.0")
  return config


def build_circle_publisher_command(config: Dict[str, object], frame_id: str,
                                   start_position, yaw: float) -> str:
  hold_end = (float(config["center_hold_evaluation_sec"]) +
              float(config["landing_reserve_hold_sec"]))
  return (
      "rosrun uav_trajectory dynamic_trajectory_publisher_node "
      "_trajectory_type:=circle "
      f"_frame_id:={frame_id} "
      f"_start_delay_sec:={line.FLIGHT_START_DELAY_SEC:.3f} "
      f"_start_x:={start_position[0]:.9f} "
      f"_start_y:={start_position[1]:.9f} "
      f"_start_z:={start_position[2]:.9f} "
      f"_start_yaw:={yaw:.9f} "
      f"_altitude_offset_m:={float(config['altitude_offset_m']):.3f} "
      f"_initial_hold_sec:={float(config['initial_hold_sec']):.3f} "
      f"_initial_climb_duration_sec:={float(config['initial_climb_duration_sec']):.3f} "
      f"_post_climb_hold_sec:={float(config['post_climb_hold_sec']):.3f} "
      f"_circle_radius_m:={float(config['circle_radius_m']):.3f} "
      f"_circle_tangent_speed_mps:={float(config['circle_tangent_speed_mps']):.3f} "
      f"_circle_tangential_speed_mps:={float(config['circle_tangent_speed_mps']):.3f} "
      f"_circle_laps:={float(config['circle_laps']):.3f} "
      f"_transition_duration_sec:={float(config['transition_duration_sec']):.3f} "
      f"_hold_end_sec:={hold_end:.3f} "
      "_yaw_mode:=fixed "
      "_publish_once:=true "
      "_subscriber_wait_timeout_sec:=2.000 "
      "_publish_repeat_count:=3 "
      "_publish_repeat_interval_sec:=0.050 "
      "_post_publish_grace_sec:=0.200"
  )


def dry_run(auth_file: Path) -> Dict[str, object]:
  auth = mission.consume_one_shot_auth(auth_file, AUTH_TOKEN)
  config = load_circle_config()
  boundaries = mission.circle_phase_boundaries(config)
  command = build_circle_publisher_command(config, "map", (1.0, 2.0, 0.3), 0.4)
  forbidden_line_type = "".join(["_trajectory_type:=", "li", "ne"])
  if forbidden_line_type in command:
    raise RuntimeError("circle publisher command contains line trajectory type")
  result = {
      "dry_run": True,
      "auth": auth,
      "git_expected_commit": "afcd69f",
      "trajectory_type": config["trajectory_type"],
      "config_path": str(CONFIG_PATH),
      "log_root": str(LOG_ROOT),
      "bag_name": CIRCLE_SPEC.bag_name,
      "project_launch": CIRCLE_SPEC.project_launch,
      "publisher_command": command,
      "phase_order": list(CIRCLE_SPEC.phase_order),
      "phase_boundaries": boundaries,
      "circle_geometry": {
          "center": "(x_start, y_start)",
          "entry_point": "(x_start + R, y_start)",
          "direction": "ccw",
          "initial_tangent": "+y",
          "flight_altitude": "z_start + 1.0",
          "final_hold_point": "circle center",
      },
      "expected_artifacts": mission.expected_circle_artifacts(),
      "started_processes": [],
      "service_calls": [],
  }
  print(json.dumps(result, indent=2, sort_keys=True), flush=True)
  return result


def main(argv: Optional[list] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--auth-file", type=Path, required=True)
  parser.add_argument("--dry-run", action="store_true")
  args = parser.parse_args(argv)
  if args.dry_run:
    dry_run(args.auth_file)
    return 0
  experiment = CircleExperiment(args.auth_file)
  recovery_attempted = False
  try:
    summary = experiment.run()
    return 0 if experiment.mission_passed(summary) else 4
  except Exception as exc:
    print(f"M0-C5B2B experiment failed: {exc}", file=sys.stderr, flush=True)
    try:
      recovery_attempted = True
      experiment.recover_after_failure(str(exc))
    except Exception as recovery_exc:
      print(f"failure recovery failed: {recovery_exc}", file=sys.stderr, flush=True)
    return 1
  finally:
    try:
      if (not recovery_attempted and not experiment.was_ever_armed() and
          experiment.output_disabled_time is None):
        try:
          experiment.call_set_output(False)
        except Exception:
          pass
    finally:
      experiment.stop_processes()


if __name__ == "__main__":
  sys.exit(main())
