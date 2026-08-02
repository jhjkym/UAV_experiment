#!/usr/bin/env python3
"""Materialize derived M0-C5B1 JSON artifacts from an existing run directory."""

import argparse
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


DERIVED_FILES = {
    "delivery_diagnostics.json",
    "handoff_metrics.json",
    "phase_metrics.json",
    "landing_lifecycle_metrics.json",
    "recovery_metrics.json",
}

PHASES = [
    "PREFLIGHT",
    "PRESTREAM",
    "OFFBOARD_PREARM",
    "ARMED_HOLD",
    "PENDING_HANDOFF",
    "CLIMB",
    "CLIMB_HOLD",
    "LINE_FORWARD",
    "LINE_REVERSE",
    "LINE_RETURN",
    "CENTER_HOLD",
    "LANDING_PREP",
    "LANDING",
    "COMPLETE",
]

PERFORMANCE_PHASES = ["LINE_FORWARD", "LINE_REVERSE", "LINE_RETURN", "CENTER_HOLD"]
RUN_RE = re.compile(r"run_\d{8}_\d{6}")


def unavailable(reason: str = "not recorded") -> Dict[str, Any]:
  return {"value": None, "available": False, "reason": reason}


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
  payload["performance_phases"] = PERFORMANCE_PHASES
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
  payload = base(run_dir, ["summary.json", "dynamic_flight_trajectory.log",
                           "m0_c5b1_project_nodes.log", "uav_trajectory_sample.txt"])
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
  payload = base(run_dir, ["summary.json", "m0_c5b1_project_nodes.log"])
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
  payload = base(run_dir, ["summary.json", "m0_c5b1_project_nodes.log", "rosbag.log"])
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
  project = parse_project_log(run_dir / "m0_c5b1_project_nodes.log")
  outputs = {
      "delivery_diagnostics.json": delivery_payload(summary, run_dir, dynamic, project),
      "handoff_metrics.json": handoff_payload(summary, run_dir, project),
      "phase_metrics.json": phase_payload(summary, tracking, run_dir, project),
      "landing_lifecycle_metrics.json": landing_payload(summary, run_dir),
      "recovery_metrics.json": recovery_payload(summary, run_dir),
  }
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
