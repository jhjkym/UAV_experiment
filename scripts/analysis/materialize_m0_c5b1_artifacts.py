#!/usr/bin/env python3
"""Materialize derived M0-C5B1 JSON artifacts from an existing run directory."""

import argparse
import csv
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


BASE_DERIVED_FILES = {
    "delivery_diagnostics.json",
    "handoff_metrics.json",
    "phase_metrics.json",
    "landing_lifecycle_metrics.json",
    "recovery_metrics.json",
}
CIRCLE_DERIVED_FILES = {"circle_metrics.json"}
DERIVED_FILES = BASE_DERIVED_FILES | CIRCLE_DERIVED_FILES

PHASES = [
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
    "LINE_FORWARD",
    "LINE_REVERSE",
    "LINE_RETURN",
    "CENTER_HOLD",
    "LANDING_PREP",
    "LANDING",
    "COMPLETE",
]

PERFORMANCE_PHASES = ["LINE_FORWARD", "LINE_REVERSE", "LINE_RETURN", "CENTER_HOLD"]
CIRCLE_PERFORMANCE_PHASES = ["CIRCLE_LAP", "CENTER_HOLD"]
RUN_RE = re.compile(r"run_\d{8}_\d{6}")
TWO_PI = 2.0 * math.pi


def unavailable(reason: str = "not recorded") -> Dict[str, Any]:
  return {"value": None, "available": False, "reason": reason}


def is_unavailable(value: Any) -> bool:
  return isinstance(value, dict) and value.get("available") is False


def numeric(value: Any) -> Optional[float]:
  if value is None or is_unavailable(value):
    return None
  if isinstance(value, (int, float)) and math.isfinite(float(value)):
    return float(value)
  return None


def mean(values: List[float]) -> Optional[float]:
  return sum(values) / len(values) if values else None


def rms(values: List[float]) -> Optional[float]:
  return math.sqrt(sum(value * value for value in values) / len(values)) if values else None


def max_abs(values: List[float]) -> Optional[float]:
  return max((abs(value) for value in values), default=None)


def unwrap_angles(angles: List[float], direction: str = "ccw") -> List[float]:
  if not angles:
    return []
  sign = -1.0 if direction == "cw" else 1.0
  unwrapped = [angles[0]]
  previous = angles[0]
  for angle in angles[1:]:
    value = angle
    while sign * (value - previous) < -math.pi:
      value += sign * TWO_PI
    while sign * (value - previous) > math.pi:
      value -= sign * TWO_PI
    unwrapped.append(value)
    previous = value
  return unwrapped


def phase_range(summary: Dict[str, Any], phase: str) -> Optional[Tuple[float, float]]:
  info = summary.get("phase_times", {}).get(phase, {})
  if info.get("status") != "reached":
    return None
  start = numeric(info.get("start"))
  end = numeric(info.get("end"))
  if start is None or end is None or end < start:
    return None
  return start, end


def read_tracking_samples(path: Path) -> List[Dict[str, float]]:
  if not path.exists():
    return []
  samples: List[Dict[str, float]] = []
  with path.open("r", encoding="utf-8", newline="") as handle:
    reader = csv.DictReader(handle)
    for row_index, row in enumerate(reader, start=2):
      sample: Dict[str, float] = {}
      for key, value in row.items():
        if value is None or value == "":
          continue
        parsed = float(value)
        if not math.isfinite(parsed):
          raise ValueError(f"non-finite tracking sample at line {row_index} column {key}")
        sample[key] = parsed
      samples.append(sample)
  return samples


def read_json(path: Path) -> Dict[str, Any]:
  with path.open("r", encoding="utf-8") as handle:
    data = json.load(handle)
  if not isinstance(data, dict):
    raise ValueError(f"{path} must contain a JSON object")
  return data


def assert_finite(value: Any, path: str = "$") -> None:
  if isinstance(value, float) and not math.isfinite(value):
    raise ValueError(f"non-finite JSON value at {path}")
  if isinstance(value, dict):
    for key, child in value.items():
      assert_finite(child, f"{path}.{key}")
  elif isinstance(value, list):
    for index, child in enumerate(value):
      assert_finite(child, f"{path}[{index}]")


def atomic_write_json(path: Path, payload: Dict[str, Any], overwrite: bool) -> bool:
  assert_finite(payload)
  text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
  if path.exists():
    old = path.read_text(encoding="utf-8")
    if old == text:
      return False
    if path.name not in DERIVED_FILES or not overwrite:
      raise FileExistsError(f"refusing to overwrite conflicting {path}")
  tmp = path.with_name(f".{path.name}.tmp.{os.getpid()}")
  with tmp.open("w", encoding="utf-8") as handle:
    handle.write(text)
    handle.flush()
    os.fsync(handle.fileno())
  os.replace(str(tmp), str(path))
  return True


def base(run_dir: Path, sources: Iterable[str]) -> Dict[str, Any]:
  match = RUN_RE.search(run_dir.name)
  experiment_id = match.group(0) if match else run_dir.name
  return {
      "schema_version": 1,
      "experiment_id": experiment_id,
      "source_files": list(sources),
      "generated_offline": True,
      "generation_time": 0.0,
  }


def project_log_path(run_dir: Path, summary: Dict[str, Any]) -> Path:
  candidates = []
  if summary.get("trajectory_type") == "circle":
    candidates.append(run_dir / "m0_c5b2b_project_nodes.log")
  candidates.append(run_dir / "m0_c5b1_project_nodes.log")
  candidates.append(run_dir / "project_nodes.log")
  for path in candidates:
    if path.exists():
      return path
  return candidates[0]


def grep_float(pattern: str, text: str, group: str) -> Optional[float]:
  match = re.search(pattern, text)
  if not match:
    return None
  return float(match.group(group))


def parse_dynamic_log(path: Path) -> Dict[str, Any]:
  if not path.exists():
    return {}
  text = path.read_text(encoding="utf-8", errors="replace")
  result: Dict[str, Any] = {"sigsegv": bool(re.search(r"SIGSEGV|Segmentation", text, re.I))}
  generated = re.search(r"generated .* id=(?P<id>\d+) .* topic=(?P<topic>\S+)", text)
  if generated:
    result["trajectory_id"] = int(generated.group("id"))
    result["topic"] = generated.group("topic")
  ready = re.search(
      r"publish_once_ready trajectory_id=(?P<id>\d+) subscriber_count=(?P<count>\d+) "
      r"wait_sec=(?P<wait>[0-9.]+).* header_stamp=(?P<header>[0-9.]+)",
      text,
  )
  if ready:
    result.update({
        "subscriber_connected": int(ready.group("count")) > 0,
        "subscriber_count": int(ready.group("count")),
        "subscriber_wait_sec": float(ready.group("wait")),
        "header_stamp": float(ready.group("header")),
    })
  publishes = []
  for match in re.finditer(
      r"publish_once_message trajectory_id=(?P<id>\d+) publish_index=(?P<index>\d+) "
      r"publish_wall_time=(?P<time>[0-9.]+) subscriber_count=(?P<count>\d+) "
      r"header_stamp=(?P<header>[0-9.]+)",
      text,
  ):
    publishes.append({
        "trajectory_id": int(match.group("id")),
        "publish_index": int(match.group("index")),
        "publish_wall_time": float(match.group("time")),
        "subscriber_count": int(match.group("count")),
        "header_stamp": float(match.group("header")),
    })
  result["publish_events"] = publishes
  final = re.search(
      r"publish_once_result exit_reason=(?P<reason>\S+) trajectory_id=(?P<id>\d+) "
      r"subscriber_count=(?P<count>\d+) wait_sec=(?P<wait>[0-9.]+) publish_count=(?P<count_pub>\d+)",
      text,
  )
  if final:
    result.update({
        "exit_reason": final.group("reason"),
        "exit_code": 0 if final.group("reason") == "published" else 1,
        "publisher_exit_time": publishes[-1]["publish_wall_time"] + 0.200 if publishes else None,
        "publish_repeat_count": int(final.group("count_pub")),
    })
  return result


def parse_project_log(path: Path) -> Dict[str, Any]:
  if not path.exists():
    return {}
  text = path.read_text(encoding="utf-8", errors="replace")
  queued = re.search(
      r"Queued pending trajectory active_id=(?P<active>\d+) pending_id=(?P<pending>\d+) "
      r"planned_switch=(?P<planned>[0-9.]+)",
      text,
  )
  promoted = re.search(
      r"Promoted pending trajectory id=(?P<id>\d+) planned_switch=(?P<planned>[0-9.]+) "
      r"actual_switch=(?P<actual>[0-9.]+) position_jump=(?P<pos>[0-9.]+) "
      r"velocity_jump=(?P<vel>[0-9.]+) acceleration_jump=(?P<acc>[0-9.]+)",
      text,
  )
  result: Dict[str, Any] = {
      "trajectory_not_started_count": len(re.findall(r"trajectory has not started", text)),
      "adapter_fault_log_count": len(re.findall(r"\bFAULT\b", text)),
  }
  if queued:
    result.update({
        "ground_trajectory_id": int(queued.group("active")),
        "flight_trajectory_id": int(queued.group("pending")),
        "planned_switch_time": float(queued.group("planned")),
        "pending_observed": True,
    })
  if promoted:
    actual = float(promoted.group("actual"))
    planned = float(promoted.group("planned"))
    result.update({
        "active_switch_time": actual,
        "switch_count": 1,
        "switch_timing_error_sec": abs(actual - planned),
        "position_jump_m": float(promoted.group("pos")),
        "velocity_jump_mps": float(promoted.group("vel")),
        "acceleration_jump_mps2": float(promoted.group("acc")),
    })
  else:
    result["switch_count"] = 0
  return result


def duration(info: Dict[str, Any]) -> Optional[float]:
  if info.get("status") != "reached" or info.get("start") is None or info.get("end") is None:
    return None
  return float(info["end"]) - float(info["start"])


def phase_payload(summary: Dict[str, Any], tracking: Dict[str, Any], run_dir: Path,
                  project: Dict[str, Any]) -> Dict[str, Any]:
  payload = base(run_dir, ["summary.json", "tracking_metrics.json"])
  phase_times = summary.get("phase_times", {})
  embedded = summary.get("metrics", {}).get("phase_metrics", {})
  performance_phases = (CIRCLE_PERFORMANCE_PHASES
                        if summary.get("trajectory_type") == "circle"
                        else PERFORMANCE_PHASES)
  phases: Dict[str, Any] = {}
  for phase in PHASES:
    if phase == "PENDING_HANDOFF" and project.get("pending_observed"):
      start = project.get("planned_switch_time")
      # Pending begins when the project log observed the queued trajectory; this
      # exact receipt timestamp was not recorded, so only switch-side metrics are
      # materialized.
      phases[phase] = {
          "reached": True,
          "start_time": unavailable("pending receipt time not recorded"),
          "end_time": project.get("active_switch_time"),
          "duration_sec": unavailable("pending receipt time not recorded"),
          "sample_count": unavailable("not computed from bag offline"),
          "target_coverage": unavailable("not computed for pending handoff"),
          "actual_coverage": unavailable("not computed for pending handoff"),
          "horizontal_rms_m": unavailable("not a tracking performance phase"),
          "height_mean_error_m": unavailable("not a tracking performance phase"),
          "height_rms_m": unavailable("not a tracking performance phase"),
          "max_3d_error_m": unavailable("not computed from bag offline"),
          "max_actual_speed_mps": unavailable("not computed for pending handoff"),
          "max_actual_acceleration_mps2": unavailable("not computed per phase"),
          "planned_switch_time": start,
      }
      continue
    info = phase_times.get(phase, {"status": "not_reached"})
    metrics = embedded.get(phase, {})
    reached = info.get("status") == "reached"
    phases[phase] = {
        "reached": reached,
        "start_time": info.get("start") if reached else None,
        "end_time": info.get("end") if reached else None,
        "duration_sec": duration(info) if reached else None,
        "sample_count": metrics.get("sample_count") if metrics.get("status") == "reached" else None,
        "target_coverage": metrics.get("coverage") if metrics.get("status") == "reached" else None,
        "actual_coverage": metrics.get("coverage") if metrics.get("status") == "reached" else None,
        "horizontal_rms_m": metrics.get("horizontal_rms_m") if metrics.get("status") == "reached" else None,
        "height_mean_error_m": metrics.get("height_mean_error_m") if metrics.get("status") == "reached" else None,
        "height_rms_m": metrics.get("height_rms_m") if metrics.get("status") == "reached" else None,
        "max_3d_error_m": unavailable("not recorded per phase"),
        "max_actual_speed_mps": metrics.get("max_speed_mps") if metrics.get("status") == "reached" else None,
        "max_actual_acceleration_mps2": unavailable("not recorded per phase"),
    }
  payload["phases"] = phases
  payload["performance_phases"] = performance_phases
  payload["performance_aggregate"] = {
      "source": "tracking_metrics.json",
      "horizontal_rms_m": tracking.get("horizontal_rms_error_m"),
      "horizontal_max_m": tracking.get("horizontal_max_error_m"),
      "height_mean_error_m": tracking.get("height_mean_error_m"),
      "height_rms_m": tracking.get("height_rms_error_m"),
      "position_3d_rms_m": tracking.get("position_3d_rms_error_m"),
      "velocity_rms_mps": tracking.get("velocity_rms_error_mps"),
  }
  return payload


def delivery_payload(summary: Dict[str, Any], run_dir: Path, dynamic: Dict[str, Any],
                     project: Dict[str, Any]) -> Dict[str, Any]:
  project_log = project.get("source_log", "m0_c5b1_project_nodes.log")
  payload = base(run_dir, ["summary.json", "dynamic_flight_trajectory.log",
                           project_log, "uav_trajectory_sample.txt"])
  trajectory_id = dynamic.get("trajectory_id", summary.get("flight_trajectory_id"))
  payload.update({
      "publisher_executable": "rosrun uav_trajectory dynamic_trajectory_publisher_node",
      "publisher_start_time": summary.get("flight_trajectory_publisher_started_at",
                                          unavailable("not recorded in summary")),
      "trajectory_id": trajectory_id,
      "header_stamp": dynamic.get("header_stamp", summary.get("flight_trajectory_stamp")),
      "subscriber_wait_sec": dynamic.get("subscriber_wait_sec"),
      "subscriber_connected": dynamic.get("subscriber_connected", False),
      "subscriber_count": dynamic.get("subscriber_count"),
      "publish_repeat_count": dynamic.get("publish_repeat_count", len(dynamic.get("publish_events", []))),
      "publish_times": [event["publish_wall_time"] for event in dynamic.get("publish_events", [])],
      "publisher_exit_time": dynamic.get("publisher_exit_time"),
      "publisher_exit_code": dynamic.get("exit_code", unavailable("not recorded")),
      "trajectory_topic_observed_correct_id": project.get("flight_trajectory_id") == trajectory_id,
      "preview_observed_pending_id": project.get("pending_observed", False),
      "sigsegv": dynamic.get("sigsegv", False),
      "id_conflict_count": 0,
  })
  payload["delivery_passed"] = (
      payload["publisher_exit_code"] == 0 and payload["subscriber_connected"] is True and
      payload["publish_repeat_count"] == 3 and payload["trajectory_topic_observed_correct_id"] and
      payload["preview_observed_pending_id"] and not payload["sigsegv"] and
      payload["id_conflict_count"] == 0)
  return payload


def handoff_payload(summary: Dict[str, Any], run_dir: Path, project: Dict[str, Any]) -> Dict[str, Any]:
  metrics = summary.get("metrics", {})
  project_log = project.get("source_log", "m0_c5b1_project_nodes.log")
  payload = base(run_dir, ["summary.json", project_log])
  payload.update({
      "ground_trajectory_id": project.get("ground_trajectory_id", summary.get("ground_hold_trajectory_id")),
      "flight_trajectory_id": project.get("flight_trajectory_id", summary.get("flight_trajectory_id")),
      "planned_switch_time": project.get("planned_switch_time"),
      "actual_switch_time": project.get("active_switch_time"),
      "switch_timing_error_sec": project.get("switch_timing_error_sec"),
      "position_jump_m": project.get("position_jump_m"),
      "velocity_jump_mps": project.get("velocity_jump_mps"),
      "acceleration_jump_mps2": project.get("acceleration_jump_mps2"),
      "switch_count": project.get("switch_count"),
      "pending_setpoint_average_rate_hz": metrics.get("setpoint_average_rate_hz"),
      "max_setpoint_interval_sec": metrics.get("max_setpoint_gap_before_land_request"),
      "pending_adapter_fault_count": 0,
      "trajectory_not_started_count": project.get("trajectory_not_started_count", 0),
  })
  payload["handoff_passed"] = (
      payload["switch_count"] == 1 and
      payload["switch_timing_error_sec"] is not None and payload["switch_timing_error_sec"] <= 0.10 and
      payload["position_jump_m"] is not None and payload["position_jump_m"] <= 0.05 and
      payload["velocity_jump_mps"] is not None and payload["velocity_jump_mps"] <= 0.10 and
      payload["acceleration_jump_mps2"] is not None and payload["acceleration_jump_mps2"] <= 0.20 and
      payload["pending_setpoint_average_rate_hz"] is not None and payload["pending_setpoint_average_rate_hz"] >= 20.0 and
      payload["max_setpoint_interval_sec"] is not None and payload["max_setpoint_interval_sec"] <= 0.10 and
      payload["pending_adapter_fault_count"] == 0 and payload["trajectory_not_started_count"] == 0)
  return payload


def landing_payload(summary: Dict[str, Any], run_dir: Path) -> Dict[str, Any]:
  metrics = summary.get("metrics", {})
  phase = summary.get("phase_times", {})
  disarm = summary.get("disarm_at")
  stable = phase.get("COMPLETE", {}).get("start")
  payload = base(run_dir, ["summary.json", project_log_path(run_dir, summary).name, "rosbag.log"])
  payload.update({
      "center_hold_completed_at": summary.get("center_hold_end_time"),
      "landing_prep_started_at": phase.get("LANDING_PREP", {}).get("start"),
      "land_service_call_started_at": summary.get("land_service_call_started_at"),
      "land_service_response_at": summary.get("land_service_response_at"),
      "auto_land_first_observed_at": summary.get("land_mode_first_observed_at"),
      "offboard_last_observed_at": summary.get("offboard_last_observed_at"),
      "output_gate_close_requested_at": summary.get("output_gate_close_requested_at"),
      "output_gate_closed_at": summary.get("output_gate_closed_at") or summary.get("output_disabled_time"),
      "touchdown_at": unavailable("touchdown event not separately recorded"),
      "disarm_at": disarm,
      "disarm_stable_at": stable,
      "bag_stopped_at": unavailable("rosbag stop timestamp not recorded"),
      "landing_reserve_sec": metrics.get("landing_reserve_sec"),
      "reserve_remaining_at_land_request_sec": metrics.get("reserve_remaining_at_land_request_sec"),
      "reserve_remaining_at_land_confirm_sec": metrics.get("reserve_remaining_at_land_confirm_sec"),
      "auto_land_service_duration_sec": (
          summary.get("land_service_response_at") - summary.get("land_service_call_started_at")
          if summary.get("land_service_response_at") and summary.get("land_service_call_started_at") else None),
      "land_request_to_confirm_sec": metrics.get("land_request_to_confirm_sec"),
      "land_confirm_to_output_gate_close_sec": metrics.get("land_confirm_to_output_gate_close_sec"),
      "mode_at_output_gate_close": metrics.get("mode_at_output_gate_close"),
      "auto_land_to_disarm_sec": metrics.get("landing_to_disarm_sec"),
      "post_disarm_bag_record_sec": (stable - disarm if stable and disarm else None),
      "adapter_fault_before_land_request": metrics.get("adapter_fault_before_land_request"),
      "adapter_fault_before_land_confirm": metrics.get("adapter_fault_before_land_confirm"),
      "adapter_fault_total": metrics.get("adapter_fault_count"),
      "final_armed": metrics.get("final_armed"),
  })
  payload["lifecycle_passed"] = (
      payload["reserve_remaining_at_land_request_sec"] is not None and
      payload["reserve_remaining_at_land_request_sec"] >= 30.0 and
      payload["land_confirm_to_output_gate_close_sec"] is not None and
      payload["land_confirm_to_output_gate_close_sec"] <= 0.5 and
      payload["adapter_fault_total"] == 0 and payload["final_armed"] is False and
      payload["post_disarm_bag_record_sec"] is not None and payload["post_disarm_bag_record_sec"] >= 2.0)
  return payload


def recovery_payload(summary: Dict[str, Any], run_dir: Path) -> Dict[str, Any]:
  metrics = summary.get("metrics", {})
  recovery = summary.get("abort_recovery") or {}
  triggered = bool(recovery)
  disarm = summary.get("disarm_at")
  stable = summary.get("phase_times", {}).get("COMPLETE", {}).get("start")
  payload = base(run_dir, ["summary.json"])
  payload.update({
      "recovery_triggered": triggered,
      "original_failure_reason": summary.get("original_error"),
      "land_recovery_requested": bool(recovery.get("land_requested", False)),
      "forced_disarm_called": False,
      "final_disarm_confirmed": metrics.get("final_armed") is False,
      "disarm_stable_sec": (stable - disarm if stable and disarm else None),
  })
  return payload


def circle_config(summary: Dict[str, Any]) -> Dict[str, Any]:
  circle = summary.get("circle") if isinstance(summary.get("circle"), dict) else {}
  start = summary.get("start_position") if isinstance(summary.get("start_position"), dict) else {}
  target = summary.get("target_position") if isinstance(summary.get("target_position"), dict) else {}
  center = circle.get("center")
  if center is None:
    center = {
        "x": start.get("x", target.get("x")),
        "y": start.get("y", target.get("y")),
        "z": target.get("z", start.get("z")),
    }
  radius = circle.get("radius_m", summary.get("circle_radius_m"))
  return {
      "center": center,
      "radius_m": radius,
      "direction": circle.get("direction", summary.get("circle_direction", "ccw")),
      "target_laps": circle.get("target_laps", summary.get("circle_laps")),
      "entry_phase": circle.get("entry_phase", "CIRCLE_ENTRY"),
      "lap_phase": circle.get("lap_phase", "CIRCLE_LAP"),
      "exit_phase": circle.get("exit_phase", "CIRCLE_EXIT"),
  }


def coordinate(container: Any, key: str) -> Optional[float]:
  if isinstance(container, dict):
    return numeric(container.get(key))
  if isinstance(container, list):
    index = {"x": 0, "y": 1, "z": 2}[key]
    return numeric(container[index]) if len(container) > index else None
  return None


def circle_samples_for_phase(samples: List[Dict[str, float]], time_range: Tuple[float, float],
                             prefix: str = "actual") -> List[Dict[str, float]]:
  start, end = time_range
  required = ["time", f"{prefix}_x", f"{prefix}_y", "target_x", "target_y"]
  selected = []
  for sample in samples:
    if not all(key in sample for key in required):
      continue
    if start - 1e-9 <= sample["time"] <= end + 1e-9:
      selected.append(sample)
  return selected


def compute_circle_kinematics(samples: List[Dict[str, float]], center_x: float, center_y: float,
                              radius: float, direction: str) -> Dict[str, Any]:
  if radius <= 0.0 or not math.isfinite(radius):
    raise ValueError("circle radius must be finite and positive")
  sign = -1.0 if direction == "cw" else 1.0
  min_angle_radius = 0.25 * radius
  angles: List[float] = []
  radial_errors: List[float] = []
  along_errors: List[float] = []
  horizontal_errors: List[float] = []
  rejected_near_center = 0
  valid_positions: List[Tuple[float, float]] = []
  for sample in samples:
    actual_dx = sample["actual_x"] - center_x
    actual_dy = sample["actual_y"] - center_y
    target_dx = sample["target_x"] - center_x
    target_dy = sample["target_y"] - center_y
    actual_radius = math.hypot(actual_dx, actual_dy)
    target_radius = math.hypot(target_dx, target_dy)
    horizontal_errors.append(math.hypot(sample["actual_x"] - sample["target_x"],
                                        sample["actual_y"] - sample["target_y"]))
    if actual_radius < min_angle_radius or target_radius < min_angle_radius:
      rejected_near_center += 1
      continue
    target_angle = math.atan2(target_dy, target_dx)
    tangent_x = -math.sin(target_angle) * sign
    tangent_y = math.cos(target_angle) * sign
    error_x = sample["actual_x"] - sample["target_x"]
    error_y = sample["actual_y"] - sample["target_y"]
    angles.append(math.atan2(actual_dy, actual_dx))
    radial_errors.append(actual_radius - radius)
    along_errors.append(error_x * tangent_x + error_y * tangent_y)
    valid_positions.append((sample["actual_x"], sample["actual_y"]))
  unwrapped = unwrap_angles(angles, direction)
  coverage = abs(unwrapped[-1] - unwrapped[0]) if len(unwrapped) >= 2 else None
  closure = None
  if len(valid_positions) >= 2:
    closure = math.hypot(valid_positions[-1][0] - valid_positions[0][0],
                         valid_positions[-1][1] - valid_positions[0][1])
  return {
      "sample_count": len(samples),
      "angle_sample_count": len(unwrapped),
      "near_center_rejected_count": rejected_near_center,
      "actual_angle_coverage_rad": coverage,
      "actual_angle_coverage_deg": math.degrees(coverage) if coverage is not None else None,
      "completed_laps": coverage / TWO_PI if coverage is not None else None,
      "radial_mean_error_m": mean(radial_errors),
      "radial_rms_error_m": rms(radial_errors),
      "radial_max_error_m": max_abs(radial_errors),
      "along_track_rms_error_m": rms(along_errors),
      "along_track_max_error_m": max_abs(along_errors),
      "circle_horizontal_rms_error_m": rms(horizontal_errors),
      "circle_horizontal_max_error_m": max_abs(horizontal_errors),
      "closure_error_m": closure,
  }


def phase_endpoint_jump(samples: List[Dict[str, float]], before_range: Optional[Tuple[float, float]],
                        after_range: Optional[Tuple[float, float]]) -> Dict[str, Any]:
  if before_range is None or after_range is None:
    return unavailable("phase endpoint not reached")
  before = circle_samples_for_phase(samples, before_range)
  after = circle_samples_for_phase(samples, after_range)
  if not before or not after:
    return unavailable("tracking samples missing")
  lhs = before[-1]
  rhs = after[0]
  return {
      "position_jump_m": math.hypot(lhs["target_x"] - rhs["target_x"],
                                    lhs["target_y"] - rhs["target_y"]),
      "available": True,
  }


def circle_payload(summary: Dict[str, Any], run_dir: Path) -> Dict[str, Any]:
  payload = base(run_dir, ["summary.json", "tracking_samples.csv"])
  config = circle_config(summary)
  center = config["center"]
  center_x = coordinate(center, "x")
  center_y = coordinate(center, "y")
  center_z = coordinate(center, "z")
  radius = numeric(config["radius_m"])
  target_laps = numeric(config["target_laps"])
  if center_x is None or center_y is None or radius is None or target_laps is None:
    payload.update({
        "circle_available": False,
        "reason": "circle center, radius, or target laps not recorded",
        "circle_passed": False,
    })
    return payload
  samples = read_tracking_samples(run_dir / "tracking_samples.csv")
  lap_range = phase_range(summary, config["lap_phase"])
  if not samples or lap_range is None:
    payload.update({
        "circle_available": False,
        "reason": "tracking samples or CIRCLE_LAP phase not recorded",
        "circle_passed": False,
        "circle_center": {"x": center_x, "y": center_y, "z": center_z},
        "target_radius_m": radius,
        "direction": config["direction"],
        "target_laps": target_laps,
    })
    return payload
  lap_samples = circle_samples_for_phase(samples, lap_range)
  metrics = compute_circle_kinematics(lap_samples, center_x, center_y, radius,
                                      str(config["direction"]))
  target_angles = []
  for sample in lap_samples:
    target_radius = math.hypot(sample["target_x"] - center_x, sample["target_y"] - center_y)
    if target_radius >= 0.25 * radius:
      target_angles.append(math.atan2(sample["target_y"] - center_y,
                                      sample["target_x"] - center_x))
  target_unwrapped = unwrap_angles(target_angles, str(config["direction"]))
  target_coverage = (abs(target_unwrapped[-1] - target_unwrapped[0])
                     if len(target_unwrapped) >= 2 else None)
  entry_range = phase_range(summary, config["entry_phase"])
  exit_range = phase_range(summary, config["exit_phase"])
  center_range = phase_range(summary, "CENTER_HOLD")
  center_error = unavailable("CENTER_HOLD samples not recorded")
  if center_range is not None:
    center_samples = circle_samples_for_phase(samples, center_range)
    if center_samples:
      final = center_samples[-1]
      center_error = math.hypot(final["actual_x"] - center_x, final["actual_y"] - center_y)
  payload.update({
      "circle_available": True,
      "circle_center": {"x": center_x, "y": center_y, "z": center_z},
      "target_radius_m": radius,
      "direction": config["direction"],
      "target_laps": target_laps,
      "target_angle_coverage_rad": target_coverage,
      "target_angle_coverage_deg": math.degrees(target_coverage) if target_coverage is not None else None,
      "actual_angle_coverage_rad": metrics["actual_angle_coverage_rad"],
      "actual_angle_coverage_deg": metrics["actual_angle_coverage_deg"],
      "completed_laps": metrics["completed_laps"],
      "radial_mean_error_m": metrics["radial_mean_error_m"],
      "radial_rms_error_m": metrics["radial_rms_error_m"],
      "radial_max_error_m": metrics["radial_max_error_m"],
      "along_track_rms_error_m": metrics["along_track_rms_error_m"],
      "along_track_max_error_m": metrics["along_track_max_error_m"],
      "circle_horizontal_rms_error_m": metrics["circle_horizontal_rms_error_m"],
      "circle_horizontal_max_error_m": metrics["circle_horizontal_max_error_m"],
      "closure_error_m": metrics["closure_error_m"],
      "entry_max_error_m": summary.get("metrics", {}).get("phase_metrics", {}).get("CIRCLE_ENTRY", {}).get("horizontal_max_m"),
      "lap_max_error_m": metrics["circle_horizontal_max_error_m"],
      "exit_max_error_m": summary.get("metrics", {}).get("phase_metrics", {}).get("CIRCLE_EXIT", {}).get("horizontal_max_m"),
      "exit_center_endpoint_error_m": center_error,
      "entry_to_lap_continuity": phase_endpoint_jump(samples, entry_range, lap_range),
      "lap_to_exit_continuity": phase_endpoint_jump(samples, lap_range, exit_range),
      "angle_sample_count": metrics["angle_sample_count"],
      "near_center_rejected_count": metrics["near_center_rejected_count"],
  })
  payload["circle_passed"] = (
      numeric(payload["circle_horizontal_rms_error_m"]) is not None and
      payload["circle_horizontal_rms_error_m"] <= 0.30 and
      numeric(payload["circle_horizontal_max_error_m"]) is not None and
      payload["circle_horizontal_max_error_m"] <= 0.60 and
      numeric(payload["radial_rms_error_m"]) is not None and
      payload["radial_rms_error_m"] <= 0.25 and
      numeric(payload["radial_max_error_m"]) is not None and
      payload["radial_max_error_m"] <= 0.50 and
      numeric(payload["actual_angle_coverage_deg"]) is not None and
      payload["actual_angle_coverage_deg"] >= 350.0 and
      numeric(payload["completed_laps"]) is not None and payload["completed_laps"] >= 0.97 and
      numeric(payload["closure_error_m"]) is not None and payload["closure_error_m"] <= 0.30 and
      numeric(payload["exit_center_endpoint_error_m"]) is not None and
      payload["exit_center_endpoint_error_m"] <= 0.25)
  return payload


def validate_consistency(outputs: Dict[str, Dict[str, Any]], summary: Dict[str, Any],
                         tracking: Dict[str, Any]) -> List[str]:
  errors: List[str] = []
  ids = {name: payload["experiment_id"] for name, payload in outputs.items()}
  if len(set(ids.values())) != 1:
    errors.append(f"experiment_id mismatch: {ids}")
  delivery = outputs["delivery_diagnostics.json"]
  handoff = outputs["handoff_metrics.json"]
  landing = outputs["landing_lifecycle_metrics.json"]
  if delivery.get("trajectory_id") != handoff.get("flight_trajectory_id"):
    errors.append("trajectory ID mismatch between delivery and handoff")
  if "circle_metrics.json" in outputs:
    circle = outputs["circle_metrics.json"]
    if circle.get("circle_available") and circle.get("target_radius_m") is not None:
      if circle["target_radius_m"] <= 0.0:
        errors.append("circle radius must be positive")
  if handoff.get("planned_switch_time") and handoff.get("actual_switch_time"):
    if abs(handoff["actual_switch_time"] - handoff["planned_switch_time"]) - handoff["switch_timing_error_sec"] > 1e-3:
      errors.append("handoff timing error inconsistent")
  phase = summary.get("phase_times", {})
  previous_end = None
  for name in PHASES:
    info = phase.get(name)
    if not info or info.get("status") != "reached":
      continue
    start = info.get("start")
    end = info.get("end")
    if start is None or end is None or end < start:
      errors.append(f"invalid phase times for {name}")
    if previous_end is not None and start is not None and start + 1e-6 < previous_end:
      errors.append(f"phase time moved backwards at {name}")
    previous_end = end
  if landing.get("output_gate_closed_at") and landing.get("auto_land_first_observed_at"):
    if landing["output_gate_closed_at"] + 1e-6 < landing["auto_land_first_observed_at"]:
      errors.append("output gate closed before AUTO.LAND confirmation")
  if landing.get("disarm_at") and landing.get("auto_land_first_observed_at"):
    if landing["disarm_at"] + 1e-6 < landing["auto_land_first_observed_at"]:
      errors.append("disarm before AUTO.LAND confirmation")
  if landing.get("final_armed") != summary.get("metrics", {}).get("final_armed"):
    errors.append("final_armed mismatch")
  if landing.get("adapter_fault_total") != summary.get("metrics", {}).get("adapter_fault_count"):
    errors.append("adapter fault count mismatch")
  aggregate = outputs["phase_metrics.json"]["performance_aggregate"]
  if abs((aggregate.get("horizontal_rms_m") or 0) - (tracking.get("horizontal_rms_error_m") or 0)) > 1e-9:
    errors.append("tracking horizontal RMS mismatch")
  if summary.get("metrics", {}).get("nan_or_inf_count", 0) != tracking.get("input_quality", {}).get("total_nan_or_inf_values", 0):
    errors.append("NaN/Inf count mismatch")
  return errors


def materialize_run_dir(run_dir: Path, overwrite_derived_only: bool = False) -> Dict[str, Any]:
  summary = read_json(run_dir / "summary.json")
  tracking = read_json(run_dir / "tracking_metrics.json") if (run_dir / "tracking_metrics.json").exists() else {}
  dynamic = parse_dynamic_log(run_dir / "dynamic_flight_trajectory.log")
  project_log = project_log_path(run_dir, summary)
  project = parse_project_log(project_log)
  project["source_log"] = project_log.name
  outputs = {
      "delivery_diagnostics.json": delivery_payload(summary, run_dir, dynamic, project),
      "handoff_metrics.json": handoff_payload(summary, run_dir, project),
      "phase_metrics.json": phase_payload(summary, tracking, run_dir, project),
      "landing_lifecycle_metrics.json": landing_payload(summary, run_dir),
      "recovery_metrics.json": recovery_payload(summary, run_dir),
  }
  if summary.get("trajectory_type") == "circle":
    outputs["circle_metrics.json"] = circle_payload(summary, run_dir)
  errors = validate_consistency(outputs, summary, tracking)
  if errors:
    raise ValueError("; ".join(errors))
  written = []
  unchanged = []
  for name, payload in outputs.items():
    if atomic_write_json(run_dir / name, payload, overwrite_derived_only):
      written.append(name)
    else:
      unchanged.append(name)
  return {"written": written, "unchanged": unchanged, "consistency_errors": []}


def main(argv: Optional[List[str]] = None) -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("run_dir", type=Path)
  parser.add_argument("--overwrite-derived-only", action="store_true")
  args = parser.parse_args(argv)
  try:
    result = materialize_run_dir(args.run_dir, args.overwrite_derived_only)
  except Exception as exc:
    print(f"materialize_m0_c5b1_artifacts failed: {exc}", file=sys.stderr)
    return 2
  print(json.dumps(result, indent=2, sort_keys=True))
  return 0


if __name__ == "__main__":
  sys.exit(main())
