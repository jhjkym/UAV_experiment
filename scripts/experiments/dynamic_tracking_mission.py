#!/usr/bin/env python3
"""Shared helpers for guarded dynamic tracking SITL experiment entries."""

import os
import stat
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


AUTH_MAX_AGE_SEC = 600.0


@dataclass(frozen=True)
class MissionSpec:
  name: str
  trajectory_type: str
  log_root: Path
  bag_name: str
  config_path: Path
  project_launch: str
  process_name: str
  ros_node_name: str
  auth_token: str
  phase_order: Tuple[str, ...]
  performance_phases: Tuple[str, ...]


def parse_simple_yaml(path: Path) -> Dict[str, object]:
  values: Dict[str, object] = {}
  for raw in path.read_text(encoding="utf-8").splitlines():
    line = raw.split("#", 1)[0].strip()
    if not line or ":" not in line:
      continue
    key, value = line.split(":", 1)
    text = value.strip()
    if text == "":
      continue
    lowered = text.lower()
    if lowered in ("true", "false"):
      parsed: object = lowered == "true"
    else:
      try:
        parsed = float(text)
      except ValueError:
        parsed = text
    values[key.strip()] = parsed
  return values


def require_float(config: Dict[str, object], key: str) -> float:
  value = config.get(key)
  if not isinstance(value, (int, float)):
    raise ValueError(f"{key} must be numeric")
  parsed = float(value)
  if not (parsed == parsed and abs(parsed) != float("inf")):
    raise ValueError(f"{key} must be finite")
  return parsed


def validate_one_shot_auth(auth_file: Path, expected_token: str,
                           now: float = None) -> Dict[str, object]:
  now = time.time() if now is None else now
  if not auth_file.exists():
    raise RuntimeError(f"authorization file does not exist: {auth_file}")
  if auth_file.is_symlink():
    raise RuntimeError(f"authorization file must not be a symlink: {auth_file}")
  info = auth_file.stat()
  if not stat.S_ISREG(info.st_mode):
    raise RuntimeError(f"authorization file must be a regular file: {auth_file}")
  if info.st_uid != os.getuid():
    raise RuntimeError(f"authorization file owner uid mismatch: {info.st_uid}")
  mode = stat.S_IMODE(info.st_mode)
  if mode != 0o600:
    raise RuntimeError(f"authorization file mode must be 600, got {mode:o}")
  lines = auth_file.read_text(encoding="utf-8").splitlines()
  if len(lines) < 2 or lines[0] != expected_token:
    raise RuntimeError("authorization token mismatch")
  try:
    timestamp = int(lines[1])
  except ValueError as exc:
    raise RuntimeError("authorization timestamp is not a Unix integer") from exc
  age = now - float(timestamp)
  if age < 0.0 or age > AUTH_MAX_AGE_SEC:
    raise RuntimeError(f"authorization file age outside 10 minute window: {age:.1f} s")
  return {
      "file": str(auth_file),
      "owner_uid": info.st_uid,
      "mode": f"{mode:o}",
      "mtime": info.st_mtime,
      "timestamp": timestamp,
      "age_sec": age,
      "token": lines[0],
  }


def consume_one_shot_auth(auth_file: Path, expected_token: str) -> Dict[str, object]:
  details = validate_one_shot_auth(auth_file, expected_token)
  auth_file.unlink()
  if auth_file.exists():
    raise RuntimeError(f"authorization file was not consumed: {auth_file}")
  details["consumed"] = True
  return details


def circle_phase_boundaries(config: Dict[str, object]) -> Dict[str, float]:
  initial_hold = require_float(config, "initial_hold_sec")
  climb = require_float(config, "initial_climb_duration_sec")
  climb_hold = require_float(config, "post_climb_hold_sec")
  transition = require_float(config, "transition_duration_sec")
  radius = require_float(config, "circle_radius_m")
  speed = require_float(config, "circle_tangent_speed_mps")
  laps = require_float(config, "circle_laps")
  center_hold = require_float(config, "center_hold_evaluation_sec")
  landing_reserve = require_float(config, "landing_reserve_hold_sec")
  if radius <= 0.0 or speed <= 0.0 or laps <= 0.0:
    raise ValueError("circle radius, speed, and laps must be positive")
  lap_duration = 2.0 * 3.14159265358979323846 * radius * laps / speed
  values = {
      "ARMED_HOLD_END": initial_hold,
      "CLIMB_END": initial_hold + climb,
      "CLIMB_HOLD_END": initial_hold + climb + climb_hold,
  }
  values["CIRCLE_ENTRY_END"] = values["CLIMB_HOLD_END"] + transition
  values["CIRCLE_LAP_END"] = values["CIRCLE_ENTRY_END"] + lap_duration
  values["CIRCLE_EXIT_END"] = values["CIRCLE_LAP_END"] + transition
  values["CENTER_HOLD_END"] = values["CIRCLE_EXIT_END"] + center_hold
  values["TRAJECTORY_TOTAL"] = values["CENTER_HOLD_END"] + landing_reserve
  values["LAP_DURATION"] = lap_duration
  return values


def expected_circle_artifacts() -> List[str]:
  return [
      "summary.json",
      "tracking_metrics.json",
      "delivery_diagnostics.json",
      "handoff_metrics.json",
      "phase_metrics.json",
      "circle_metrics.json",
      "landing_lifecycle_metrics.json",
      "recovery_metrics.json",
  ]
