#!/usr/bin/env python3
"""M0-C5B1 PX4 SITL-only offboard line tracking experiment.

This script is intentionally outside normal catkin tests. It starts only the
processes needed for a local PX4 SITL experiment, checks SITL identity before
arming or mode changes, records logs under /tmp, and cleans up only processes
that it started.
"""

import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import Point, Vector3
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from uav_msgs.msg import OffboardStatus, SetpointPreview, Trajectory, TrajectoryPoint, UavState


REPO_DIR = Path("/home/tom/UAV_experiment")
PX4_DIR = Path("/home/tom/third_party/PX4-Autopilot")
LOG_ROOT = Path("/tmp/uav_m0_c5b1")
PX4_BIN_TOKEN = "build/px4_sitl_default/bin/px4"
TARGET_ALTITUDE_DELTA_M = 1.0
PREFLIGHT_HOLD_SEC = 60.0
FLIGHT_START_DELAY_SEC = 1.0
INITIAL_HOLD_SEC = 1.0
RISE_DURATION_SEC = 8.0
ALTITUDE_SETTLE_SEC = 2.0
LINE_SEGMENT_DURATION_SEC = 5.0
LINE_LENGTH_M = 1.0
CENTER_HOLD_EVALUATION_SEC = 10.0
LANDING_RESERVE_HOLD_SEC = 60.0
HOVER_DURATION_SEC = CENTER_HOLD_EVALUATION_SEC + LANDING_RESERVE_HOLD_SEC
TRAJECTORY_ACTIVE_SEC = (
    INITIAL_HOLD_SEC + RISE_DURATION_SEC + ALTITUDE_SETTLE_SEC +
    3.0 * LINE_SEGMENT_DURATION_SEC)
CENTER_HOLD_END_SEC = TRAJECTORY_ACTIVE_SEC + CENTER_HOLD_EVALUATION_SEC
TRAJECTORY_TOTAL_SEC = CENTER_HOLD_END_SEC + LANDING_RESERVE_HOLD_SEC
MIN_RESERVE_AT_LAND_REQUEST_SEC = 30.0
OUTPUT_GATE_CLOSE_AFTER_LAND_CONFIRM_SEC = 0.5
ABORT_HEIGHT_ERROR_M = 0.9
ABORT_HEIGHT_ERROR_DWELL_SEC = 1.0
ABORT_HEIGHT_OVERSHOOT_M = 0.5


def is_authorized(env: Dict[str, str]) -> bool:
  return env.get("UAV_ALLOW_SITL_FLIGHT") == "YES"


def can_start_dynamic_trajectory(armed: bool) -> bool:
  return armed


def validate_trajectory_switch(previous_point,
                               next_point,
                               max_position_jump_m: float = 0.15,
                               max_speed_mps: float = 0.05,
                               max_acceleration_mps2: float = 0.05) -> Tuple[bool, str, Dict[str, float]]:
  jump = math.sqrt(
      (next_point.position.x - previous_point.position.x) ** 2 +
      (next_point.position.y - previous_point.position.y) ** 2 +
      (next_point.position.z - previous_point.position.z) ** 2)
  speed0 = math.sqrt(next_point.velocity.x ** 2 + next_point.velocity.y ** 2 +
                     next_point.velocity.z ** 2)
  accel0 = math.sqrt(next_point.acceleration.x ** 2 + next_point.acceleration.y ** 2 +
                     next_point.acceleration.z ** 2)
  details = {"position_jump_m": jump, "speed_mps": speed0, "acceleration_mps2": accel0}
  if jump > max_position_jump_m:
    return False, f"trajectory switch position jump too large: {jump:.3f} m", details
  if speed0 > max_speed_mps or accel0 > max_acceleration_mps2:
    return False, f"trajectory switch derivative discontinuity speed={speed0:.3f} accel={accel0:.3f}", details
  return True, "ok", details


def tracking_acceptance_window(phase_starts: Dict[str, float],
                               land_request_time: Optional[float],
                               line_end_time: Optional[float]) -> Tuple[Optional[float], Optional[float]]:
  return phase_starts.get("LINE_FORWARD"), land_request_time or line_end_time


def reserve_remaining_at(trajectory_start: float, stamp: float) -> float:
  return trajectory_start + TRAJECTORY_TOTAL_SEC - stamp


def can_close_output_gate(mode: str, land_confirmed: bool) -> bool:
  return land_confirmed and mode == "AUTO.LAND"


class DwellThreshold:
  def __init__(self, threshold: float, dwell_sec: float) -> None:
    self.threshold = threshold
    self.dwell_sec = dwell_sec
    self.first_violation_time: Optional[float] = None
    self.trigger_time: Optional[float] = None

  def update(self, value: float, now: float) -> bool:
    if value <= self.threshold:
      self.first_violation_time = None
      return False
    if self.first_violation_time is None:
      self.first_violation_time = now
      return False
    if now - self.first_violation_time >= self.dwell_sec:
      self.trigger_time = now
      return True
    return False


class PhaseRecorder:
  ORDER = [
      "PREFLIGHT", "PRESTREAM", "OFFBOARD_PREARM", "ARMED_HOLD", "CLIMB",
      "LINE_FORWARD", "LINE_REVERSE", "LINE_RETURN", "CENTER_HOLD",
      "LANDING_PREP", "LANDING", "COMPLETE", "ABORT",
  ]

  def __init__(self) -> None:
    self.current: Optional[str] = None
    self.starts: Dict[str, float] = {}
    self.ends: Dict[str, float] = {}

  def set(self, phase: str, stamp: float) -> None:
    if phase not in self.ORDER:
      raise ValueError(f"unknown phase {phase}")
    if self.current == phase:
      return
    if self.current is not None and self.current not in self.ends:
      self.ends[self.current] = stamp
    self.current = phase
    self.starts.setdefault(phase, stamp)

  def finish(self, stamp: float) -> None:
    if self.current is not None and self.current not in self.ends:
      self.ends[self.current] = stamp

  def summary(self) -> Dict[str, Dict[str, object]]:
    result: Dict[str, Dict[str, object]] = {}
    for phase in self.ORDER:
      if phase not in self.starts:
        result[phase] = {"status": "not_reached"}
      else:
        result[phase] = {
            "status": "reached",
            "start": self.starts[phase],
            "end": self.ends.get(phase),
        }
    return result


def make_hold_trajectory(frame_id: str,
                         position: Tuple[float, float, float],
                         yaw: float,
                         start_stamp: rospy.Time,
                         hold_sec: float,
                         trajectory_id: int) -> Trajectory:
  trajectory = Trajectory()
  trajectory.header.stamp = start_stamp
  trajectory.header.frame_id = frame_id
  trajectory.mode = Trajectory.MODE_NOMINAL
  trajectory.trajectory_id = trajectory_id
  first = TrajectoryPoint()
  first.time_from_start = rospy.Duration(0.0)
  first.position = Point(*position)
  first.velocity = Vector3(0.0, 0.0, 0.0)
  first.acceleration = Vector3(0.0, 0.0, 0.0)
  first.yaw = yaw
  first.yaw_rate = 0.0
  second = TrajectoryPoint()
  second.time_from_start = rospy.Duration(hold_sec)
  second.position = Point(*position)
  second.velocity = Vector3(0.0, 0.0, 0.0)
  second.acceleration = Vector3(0.0, 0.0, 0.0)
  second.yaw = yaw
  second.yaw_rate = 0.0
  trajectory.points = [first, second]
  return trajectory


@dataclass
class ManagedProcess:
  name: str
  process: subprocess.Popen
  log_path: Path


@dataclass
class TopicState:
  mavros_state: Optional[State] = None
  uav_state: Optional[UavState] = None
  odom: Optional[Odometry] = None
  setpoint_preview: Optional[SetpointPreview] = None
  offboard_status: Optional[OffboardStatus] = None
  target: Optional[PositionTarget] = None
  target_times: List[float] = field(default_factory=list)
  target_times_all: List[float] = field(default_factory=list)
  target_samples: List[Tuple[float, float, float, float, float, float, float]] = field(default_factory=list)
  uav_samples: List[Tuple[float, float, float, float, float, float, float]] = field(default_factory=list)
  attitude_samples: List[Tuple[float, float, float]] = field(default_factory=list)
  offboard_status_samples: List[Tuple[float, str, str, bool, bool]] = field(default_factory=list)
  adapter_fault: bool = False
  adapter_fault_count: int = 0
  exited_offboard: bool = False
  unexpected_offboard_exit_count: int = 0
  nan_or_inf: bool = False


class Experiment:
  def __init__(self) -> None:
    if not is_authorized(os.environ):
      raise RuntimeError("UAV_ALLOW_SITL_FLIGHT must be exactly YES")
    self.run_dir = LOG_ROOT / time.strftime("run_%Y%m%d_%H%M%S")
    self.run_dir.mkdir(parents=True, exist_ok=False)
    self.processes: List[ManagedProcess] = []
    self.state = TopicState()
    self.start_position: Optional[Tuple[float, float, float]] = None
    self.target_position: Optional[Tuple[float, float, float]] = None
    self.target_yaw = 0.0
    self.mode_request_time: Optional[float] = None
    self.mode_confirm_time: Optional[float] = None
    self.arm_request_time: Optional[float] = None
    self.arm_confirm_time: Optional[float] = None
    self.land_request_time: Optional[float] = None
    self.land_confirm_time: Optional[float] = None
    self.land_service_call_started_at: Optional[float] = None
    self.land_service_response_at: Optional[float] = None
    self.land_mode_first_observed_at: Optional[float] = None
    self.offboard_last_observed_at: Optional[float] = None
    self.disarm_time: Optional[float] = None
    self.flight_start_time: Optional[float] = None
    self.line_end_time: Optional[float] = None
    self.land_complete_time: Optional[float] = None
    self.output_enabled_time: Optional[float] = None
    self.output_gate_close_requested_at: Optional[float] = None
    self.output_disabled_time: Optional[float] = None
    self.output_gate_close_error: Optional[str] = None
    self.mode_at_output_gate_close: Optional[str] = None
    self.land_request_setpoint_rate: Optional[float] = None
    self.land_request_max_setpoint_gap: Optional[float] = None
    self.liftoff_time: Optional[float] = None
    self.height_error_dwell = DwellThreshold(ABORT_HEIGHT_ERROR_M, ABORT_HEIGHT_ERROR_DWELL_SEC)
    self.horizontal_error_dwell = DwellThreshold(1.0, 0.5)
    self.abort_reason: Optional[str] = None
    self.service_calls: List[Dict[str, object]] = []
    self.phase = PhaseRecorder()
    self.flight_trajectory_id: Optional[int] = None
    self.flight_trajectory_stamp: Optional[float] = None
    self.flight_trajectory_publisher_started_at: Optional[float] = None
    self.trajectory_end_time: Optional[float] = None
    self.adapter_fault_first_at: Optional[float] = None
    self.center_hold_completed = False

  def start_process(self, name: str, command: str, cwd: Path) -> None:
    log_path = self.run_dir / f"{name}.log"
    log_file = open(log_path, "w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", command],
        cwd=str(cwd),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    self.processes.append(ManagedProcess(name, process, log_path))
    print(f"started {name}: pid={process.pid} log={log_path}", flush=True)

  def stop_processes(self) -> None:
    for managed in reversed(self.processes):
      if managed.process.poll() is not None:
        continue
      print(f"stopping {managed.name}: pid={managed.process.pid}", flush=True)
      try:
        os.killpg(os.getpgid(managed.process.pid), signal.SIGINT)
      except ProcessLookupError:
        continue
    time.sleep(4.0)
    for managed in reversed(self.processes):
      if managed.process.poll() is not None:
        continue
      try:
        os.killpg(os.getpgid(managed.process.pid), signal.SIGTERM)
      except ProcessLookupError:
        continue
    time.sleep(2.0)

  def stop_process(self, name: str) -> None:
    for managed in reversed(self.processes):
      if managed.name != name or managed.process.poll() is not None:
        continue
      print(f"stopping {managed.name}: pid={managed.process.pid}", flush=True)
      try:
        os.killpg(os.getpgid(managed.process.pid), signal.SIGINT)
      except ProcessLookupError:
        return
      deadline = time.time() + 6.0
      while time.time() < deadline:
        if managed.process.poll() is not None:
          return
        time.sleep(0.2)
      try:
        os.killpg(os.getpgid(managed.process.pid), signal.SIGTERM)
      except ProcessLookupError:
        return

  def managed_process(self, name: str) -> Optional[ManagedProcess]:
    for managed in reversed(self.processes):
      if managed.name == name:
        return managed
    return None

  def run_cmd(self, command: str, cwd: Path = REPO_DIR, timeout: float = 15.0) -> str:
    result = subprocess.run(
        ["bash", "-lc", command],
        cwd=str(cwd),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=False,
    )
    if result.returncode != 0:
      raise RuntimeError(f"command failed ({result.returncode}): {command}\n{result.stdout}")
    return result.stdout

  def wait_for(self, description: str, predicate, timeout: float, period: float = 0.2) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline and not rospy.is_shutdown():
      if predicate():
        print(f"ready: {description}", flush=True)
        return
      time.sleep(period)
    raise RuntimeError(f"timeout waiting for {description}")

  def wait_log_contains(self, name: str, required: List[str], timeout: float) -> None:
    managed = next(p for p in self.processes if p.name == name)
    deadline = time.time() + timeout
    while time.time() < deadline:
      if managed.process.poll() is not None:
        tail = self.tail(managed.log_path)
        raise RuntimeError(f"{name} exited before ready\n{tail}")
      content = managed.log_path.read_text(encoding="utf-8", errors="replace")
      if all(token in content for token in required):
        print(f"ready: {name} log contains required startup markers", flush=True)
        return
      time.sleep(0.5)
    raise RuntimeError(f"timeout waiting for {name} startup markers\n{self.tail(managed.log_path)}")

  def tail(self, path: Path, lines: int = 120) -> str:
    if not path.exists():
      return ""
    content = path.read_text(encoding="utf-8", errors="replace").splitlines()
    return "\n".join(content[-lines:])

  def setup_ros_subscribers(self) -> None:
    rospy.Subscriber("/mavros/state", State, self.on_mavros_state, queue_size=20)
    rospy.Subscriber("/uav/state", UavState, self.on_uav_state, queue_size=20)
    rospy.Subscriber("/mavros/local_position/odom", Odometry, self.on_odom, queue_size=20)
    rospy.Subscriber("/uav/setpoint_preview", SetpointPreview, self.on_preview, queue_size=20)
    rospy.Subscriber("/uav/offboard_status", OffboardStatus, self.on_status, queue_size=20)
    rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, self.on_target, queue_size=200)
    self.trajectory_pub = rospy.Publisher("/uav/trajectory", Trajectory, queue_size=1, latch=True)
    self.phase_pub = rospy.Publisher("/uav/experiment_phase", String, queue_size=1, latch=True)

  def set_phase(self, phase: str) -> None:
    stamp = rospy.Time.now().to_sec() if not rospy.is_shutdown() else time.time()
    self.phase.set(phase, stamp)
    if hasattr(self, "phase_pub"):
      self.phase_pub.publish(String(data=phase))
    print(f"phase: {phase}", flush=True)

  def on_mavros_state(self, msg: State) -> None:
    previous = self.state.mavros_state
    self.state.mavros_state = msg
    now = rospy.Time.now().to_sec()
    if msg.mode == "OFFBOARD":
      self.offboard_last_observed_at = now
    if msg.mode == "AUTO.LAND" and self.land_mode_first_observed_at is None:
      self.land_mode_first_observed_at = now
    if (previous is not None and previous.mode == "OFFBOARD" and
        msg.mode != "OFFBOARD" and msg.armed and self.land_request_time is None):
      self.state.exited_offboard = True
      self.state.unexpected_offboard_exit_count += 1

  def on_uav_state(self, msg: UavState) -> None:
    self.state.uav_state = msg
    if not self.finite_values([
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w,
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z,
    ]):
      self.state.nan_or_inf = True
    t = rospy.Time.now().to_sec()
    speed = math.sqrt(msg.twist.linear.x ** 2 + msg.twist.linear.y ** 2 + msg.twist.linear.z ** 2)
    self.state.uav_samples.append((
        t, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
    ))
    roll, pitch = self.roll_pitch_from_quaternion(msg.pose.orientation)
    self.state.attitude_samples.append((t, roll, pitch))

  def on_odom(self, msg: Odometry) -> None:
    self.state.odom = msg

  def on_preview(self, msg: SetpointPreview) -> None:
    self.state.setpoint_preview = msg

  def on_status(self, msg: OffboardStatus) -> None:
    self.state.offboard_status = msg
    now = rospy.Time.now().to_sec()
    self.state.offboard_status_samples.append((
        now,
        msg.state_name,
        msg.reason,
        bool(msg.output_active),
        bool(msg.runtime_gate_enabled),
    ))
    if msg.state_name == "FAULT":
      if not self.state.adapter_fault:
        self.state.adapter_fault_count += 1
        self.adapter_fault_first_at = now
      self.state.adapter_fault = True

  def on_target(self, msg: PositionTarget) -> None:
    self.state.target = msg
    now = rospy.Time.now().to_sec()
    self.state.target_times.append(now)
    self.state.target_times_all.append(now)
    self.state.target_samples.append((
        now, msg.position.x, msg.position.y, msg.position.z,
        msg.velocity.x, msg.velocity.y, msg.velocity.z,
    ))
    if not self.finite_values([
        msg.position.x, msg.position.y, msg.position.z,
        msg.velocity.x, msg.velocity.y, msg.velocity.z,
        msg.acceleration_or_force.x, msg.acceleration_or_force.y,
        msg.acceleration_or_force.z, msg.yaw, msg.yaw_rate,
    ]):
      self.state.nan_or_inf = True
    cutoff = now - 10.0
    self.state.target_times = [t for t in self.state.target_times if t >= cutoff]

  def finite_values(self, values: List[float]) -> bool:
    return all(math.isfinite(v) for v in values)

  def yaw_from_quaternion(self, q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

  def roll_pitch_from_quaternion(self, q) -> Tuple[float, float]:
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch

  def verify_px4_version(self) -> Tuple[str, str]:
    tag = self.run_cmd("git describe --tags --exact-match", PX4_DIR).strip()
    commit = self.run_cmd("git rev-parse HEAD", PX4_DIR).strip()
    if tag != "v1.14.3":
      raise RuntimeError(f"PX4 tag mismatch: expected v1.14.3, got {tag}")
    if commit != "1dacb4cdef2d7145754fc788fa8dc482eed74b40":
      raise RuntimeError(f"PX4 commit mismatch: {commit}")
    return tag, commit

  def verify_sitl_identity(self) -> None:
    ps_output = self.run_cmd("ps -eo pid=,args=", timeout=5.0)
    if PX4_BIN_TOKEN not in ps_output:
      raise RuntimeError(f"PX4 process does not include {PX4_BIN_TOKEN}")
    if "gzserver" not in ps_output:
      raise RuntimeError("Gazebo Classic gzserver process was not found")
    fcu_url = self.run_cmd("rosparam get /mavros/fcu_url", timeout=5.0).strip()
    if "serial://" in fcu_url or "/dev/tty" in fcu_url:
      raise RuntimeError(f"refusing non-SITL FCU URL: {fcu_url}")
    if not fcu_url.startswith("udp://"):
      raise RuntimeError(f"refusing non-UDP FCU URL: {fcu_url}")
    serial_devices = list(Path("/dev").glob("ttyACM*")) + list(Path("/dev").glob("ttyUSB*"))
    if serial_devices:
      raise RuntimeError(f"refusing while serial flight-controller-like devices exist: {serial_devices}")

  def capture_start_state(self) -> Tuple[str, Tuple[float, float, float], float]:
    self.wait_for("fresh /uav/state", lambda: self.state.uav_state is not None and
                  self.state.uav_state.pose_valid and self.state.uav_state.twist_valid, 20.0)
    state = self.state.uav_state
    assert state is not None
    self.start_position = (state.pose.position.x, state.pose.position.y, state.pose.position.z)
    self.target_position = (
        state.pose.position.x,
        state.pose.position.y,
        state.pose.position.z + TARGET_ALTITUDE_DELTA_M,
    )
    self.target_yaw = self.yaw_from_quaternion(state.pose.orientation)
    print(
        "captured start state: "
        f"start={self.start_position} target_altitude={self.target_position[2]:.3f} "
        f"yaw={self.target_yaw:.3f}",
        flush=True,
    )
    return state.header.frame_id or "map", self.start_position, self.target_yaw

  def publish_ground_hold_trajectory(self, frame_id: str,
                                     position: Tuple[float, float, float],
                                     yaw: float) -> int:
    trajectory_id = int(time.time()) & 0xFFFFFFFF
    trajectory = make_hold_trajectory(
        frame_id, position, yaw, rospy.Time.now() + rospy.Duration(0.2),
        PREFLIGHT_HOLD_SEC, trajectory_id)
    for _ in range(5):
      self.trajectory_pub.publish(trajectory)
      rospy.sleep(0.1)
    print(f"ground hold trajectory published id={trajectory_id}", flush=True)
    return trajectory_id

  def start_dynamic_flight_trajectory(self, frame_id: str) -> None:
    self.wait_for("fresh armed /uav/state", lambda:
                  self.state.uav_state is not None and
                  self.state.uav_state.pose_valid and
                  self.state.uav_state.twist_valid and
                  self.state.mavros_state is not None and
                  self.state.mavros_state.armed, 5.0)
    state = self.state.uav_state
    preview = self.state.setpoint_preview
    assert state is not None
    assert preview is not None
    if not can_start_dynamic_trajectory(self.state.mavros_state is not None and
                                        self.state.mavros_state.armed):
      raise RuntimeError("dynamic flight trajectory requires confirmed armed=true")
    old_id = preview.trajectory_id
    self.start_position = (state.pose.position.x, state.pose.position.y, state.pose.position.z)
    self.target_position = (
        state.pose.position.x,
        state.pose.position.y,
        state.pose.position.z + TARGET_ALTITUDE_DELTA_M,
    )
    self.target_yaw = self.yaw_from_quaternion(state.pose.orientation)
    self.flight_trajectory_stamp = rospy.Time.now().to_sec() + FLIGHT_START_DELAY_SEC
    command = (
        "rosrun uav_trajectory dynamic_trajectory_publisher_node "
        "_trajectory_type:=line "
        f"_frame_id:={frame_id} "
        f"_start_delay_sec:={FLIGHT_START_DELAY_SEC:.3f} "
        f"_start_x:={self.start_position[0]:.9f} "
        f"_start_y:={self.start_position[1]:.9f} "
        f"_start_z:={self.start_position[2]:.9f} "
        f"_start_yaw:={self.target_yaw:.9f} "
        "_altitude_offset_m:=1.0 "
        f"_initial_hold_sec:={INITIAL_HOLD_SEC:.3f} "
        f"_initial_climb_duration_sec:={RISE_DURATION_SEC:.3f} "
        f"_post_climb_hold_sec:={ALTITUDE_SETTLE_SEC:.3f} "
        f"_line_length_m:={LINE_LENGTH_M:.3f} "
        f"_line_segment_duration_sec:={LINE_SEGMENT_DURATION_SEC:.3f} "
        f"_hold_end_sec:={HOVER_DURATION_SEC:.3f} "
        "_yaw_mode:=fixed "
        "_publish_once:=true "
        "_subscriber_wait_timeout_sec:=2.000 "
        "_publish_repeat_count:=3 "
        "_publish_repeat_interval_sec:=0.050 "
        "_post_publish_grace_sec:=0.200"
    )
    self.flight_trajectory_publisher_started_at = time.time()
    self.start_process("dynamic_flight_trajectory", command, REPO_DIR)
    managed = self.managed_process("dynamic_flight_trajectory")

    def flight_trajectory_delivered() -> bool:
      if managed is not None:
        exit_code = managed.process.poll()
        if exit_code is not None and exit_code != 0:
          self.write_delivery_diagnostics(old_id)
          raise RuntimeError(
              f"dynamic trajectory publisher exited before handoff exit_code={exit_code}")
      return (self.state.setpoint_preview is not None and
              self.state.setpoint_preview.trajectory_valid and
              self.state.setpoint_preview.trajectory_id != old_id)

    try:
      self.wait_for("flight trajectory id switch", flight_trajectory_delivered,
                    5.0, 0.05)
    except Exception:
      self.write_delivery_diagnostics(old_id)
      raise
    if managed is not None:
      exit_code = managed.process.poll()
      if exit_code not in (None, 0):
        self.write_delivery_diagnostics(old_id)
        raise RuntimeError(f"dynamic trajectory publisher failed exit_code={exit_code}")
    new_preview = self.state.setpoint_preview
    assert new_preview is not None
    self.flight_trajectory_id = new_preview.trajectory_id
    valid_switch, switch_reason, switch_details = validate_trajectory_switch(preview.point,
                                                                            new_preview.point)
    if not valid_switch:
      raise RuntimeError(switch_reason)
    print(
        f"flight trajectory switched old_id={old_id} new_id={self.flight_trajectory_id} "
        f"jump={switch_details['position_jump_m']:.3f} "
        f"speed0={switch_details['speed_mps']:.3f} "
        f"accel0={switch_details['acceleration_mps2']:.3f}",
        flush=True,
    )

  def write_delivery_diagnostics(self, old_id: int) -> None:
    preview_id = None
    preview_stamp = None
    if self.state.setpoint_preview is not None:
      preview_id = self.state.setpoint_preview.trajectory_id
      preview_stamp = self.state.setpoint_preview.header.stamp.to_sec()
    diagnostics = {
        "old_preview_trajectory_id": old_id,
        "current_preview_trajectory_id": preview_id,
        "current_preview_stamp": preview_stamp,
        "flight_trajectory_stamp": self.flight_trajectory_stamp,
        "flight_trajectory_publisher_started_at": self.flight_trajectory_publisher_started_at,
        "trajectory_topic_publishers": None,
        "trajectory_topic_subscribers": None,
        "preview_topic_publishers": None,
        "preview_topic_subscribers": None,
    }
    for topic, pub_key, sub_key in [
        ("/uav/trajectory", "trajectory_topic_publishers", "trajectory_topic_subscribers"),
        ("/uav/setpoint_preview", "preview_topic_publishers", "preview_topic_subscribers"),
    ]:
      try:
        output = self.run_cmd(f"rostopic info {topic}", timeout=5.0)
      except Exception as exc:
        output = f"failed: {exc}"
      diagnostics[pub_key] = output
      diagnostics[sub_key] = output
    (self.run_dir / "delivery_diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2, sort_keys=True), encoding="utf-8")

  def call_set_output(self, enabled: bool) -> None:
    rospy.wait_for_service("/offboard_adapter_node/set_output_enabled", timeout=5.0)
    service = rospy.ServiceProxy("/offboard_adapter_node/set_output_enabled", SetBool)
    request_time = time.time()
    if not enabled and self.output_gate_close_requested_at is None:
      self.output_gate_close_requested_at = request_time
    response = service(enabled)
    self.service_calls.append({"service": "set_output_enabled", "value": enabled,
                               "success": bool(response.success), "message": response.message})
    if enabled and not response.success:
      raise RuntimeError(f"failed to enable output gate: {response.message}")
    if enabled and response.success and self.output_enabled_time is None:
      self.output_enabled_time = time.time()
    if not enabled and response.success:
      self.output_disabled_time = time.time()
      self.mode_at_output_gate_close = (
          self.state.mavros_state.mode if self.state.mavros_state is not None else None)
    if not enabled and not response.success:
      self.output_gate_close_error = response.message

  def call_mode(self, mode: str, require_mode: bool = True) -> None:
    rospy.wait_for_service("/mavros/set_mode", timeout=5.0)
    service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    for attempt in range(1, 4):
      request_time = time.time()
      if mode == "AUTO.LAND" and self.land_service_call_started_at is None:
        self.land_service_call_started_at = request_time
      response = service(custom_mode=mode)
      response_time = time.time()
      if mode == "AUTO.LAND":
        self.land_service_response_at = response_time
      self.service_calls.append({"service": "set_mode", "mode": mode,
                                 "attempt": attempt, "mode_sent": bool(response.mode_sent),
                                 "request_time": request_time, "response_time": response_time})
      if mode == "OFFBOARD" and self.mode_request_time is None:
        self.mode_request_time = request_time
      if mode == "AUTO.LAND" and self.land_request_time is None:
        self.land_request_time = request_time
      deadline = time.time() + 3.0
      while time.time() < deadline:
        if self.state.mavros_state is not None and self.state.mavros_state.mode == mode:
          if mode == "OFFBOARD" and self.mode_confirm_time is None:
            self.mode_confirm_time = time.time()
          if mode == "AUTO.LAND" and self.land_confirm_time is None:
            self.land_confirm_time = time.time()
          return
        time.sleep(0.1)
      if not require_mode:
        return
    raise RuntimeError(f"mode {mode} was not confirmed")

  def call_arm(self, arm: bool) -> None:
    rospy.wait_for_service("/mavros/cmd/arming", timeout=5.0)
    service = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    for attempt in range(1, 4):
      request_time = time.time()
      response = service(value=arm)
      self.service_calls.append({"service": "arming", "value": arm,
                                 "attempt": attempt, "success": bool(response.success)})
      if arm and self.arm_request_time is None:
        self.arm_request_time = request_time
      deadline = time.time() + 3.0
      while time.time() < deadline:
        if self.state.mavros_state is not None and self.state.mavros_state.armed == arm:
          if arm and self.arm_confirm_time is None:
            self.arm_confirm_time = time.time()
          if not arm:
            self.disarm_time = time.time()
          return
        time.sleep(0.1)
    raise RuntimeError(f"arming state {arm} was not confirmed")

  def current_setpoint_rate(self, window_sec: float = 2.0) -> float:
    now = rospy.Time.now().to_sec()
    samples = [t for t in self.state.target_times if t >= now - window_sec]
    if len(samples) < 2:
      return 0.0
    return (len(samples) - 1) / (samples[-1] - samples[0])

  def current_setpoint_rate_and_gap(self, window_sec: float = 2.0) -> Tuple[float, float]:
    now = rospy.Time.now().to_sec()
    samples = [t for t in self.state.target_times if t >= now - window_sec]
    if len(samples) < 2:
      return 0.0, float("inf")
    intervals = [b - a for a, b in zip(samples[:-1], samples[1:])]
    return (len(samples) - 1) / (samples[-1] - samples[0]), max(intervals)

  def wait_prestream(self) -> float:
    self.wait_for("healthy setpoint preview", lambda:
                  self.state.setpoint_preview is not None and
                  self.state.setpoint_preview.trajectory_valid and
                  self.state.setpoint_preview.started and
                  not self.state.setpoint_preview.finished and
                  self.state.setpoint_preview.state_fresh, 10.0)
    self.wait_for("offboard adapter healthy", lambda:
                  self.state.offboard_status is not None and
                  self.state.offboard_status.mavros_connected and
                  self.state.offboard_status.reason == "healthy", 10.0)
    self.call_set_output(True)
    start = time.time()
    while time.time() - start < 2.2:
      if self.state.mavros_state and self.state.mavros_state.armed:
        raise RuntimeError("vehicle armed during prestream")
      time.sleep(0.1)
    rate = self.current_setpoint_rate(2.0)
    if rate < 20.0:
      raise RuntimeError(f"prestream setpoint rate too low: {rate:.2f} Hz")
    return rate

  def monitor_flight(self) -> None:
    assert self.start_position is not None
    assert self.target_position is not None
    assert self.flight_trajectory_stamp is not None
    self.flight_start_time = time.time()
    self.line_end_time = self.flight_trajectory_stamp + CENTER_HOLD_END_SEC
    self.trajectory_end_time = self.flight_trajectory_stamp + TRAJECTORY_TOTAL_SEC
    while time.time() < self.line_end_time and not rospy.is_shutdown():
      self.update_phase_from_time(rospy.Time.now().to_sec())
      reason = self.abort_condition(self.flight_start_time)
      if reason:
        self.abort_reason = reason
        print(f"abort condition: {reason}", flush=True)
        self.set_phase("ABORT")
        if self.state.mavros_state and self.state.mavros_state.armed:
          self.call_mode("AUTO.LAND", require_mode=False)
        return
      time.sleep(0.1)
    self.update_phase_from_time(rospy.Time.now().to_sec())
    self.center_hold_completed = True
    self.set_phase("LANDING_PREP")

  def update_phase_from_time(self, now: float) -> None:
    if self.flight_trajectory_stamp is None:
      return
    elapsed = now - self.flight_trajectory_stamp
    if elapsed < INITIAL_HOLD_SEC:
      self.set_phase("ARMED_HOLD")
    elif elapsed < INITIAL_HOLD_SEC + RISE_DURATION_SEC:
      self.set_phase("CLIMB")
    elif elapsed < INITIAL_HOLD_SEC + RISE_DURATION_SEC + ALTITUDE_SETTLE_SEC:
      self.set_phase("CLIMB")
    elif elapsed < INITIAL_HOLD_SEC + RISE_DURATION_SEC + ALTITUDE_SETTLE_SEC + LINE_SEGMENT_DURATION_SEC:
      self.set_phase("LINE_FORWARD")
    elif elapsed < INITIAL_HOLD_SEC + RISE_DURATION_SEC + ALTITUDE_SETTLE_SEC + 2.0 * LINE_SEGMENT_DURATION_SEC:
      self.set_phase("LINE_REVERSE")
    elif elapsed < INITIAL_HOLD_SEC + RISE_DURATION_SEC + ALTITUDE_SETTLE_SEC + 3.0 * LINE_SEGMENT_DURATION_SEC:
      self.set_phase("LINE_RETURN")
    elif elapsed < CENTER_HOLD_END_SEC:
      self.set_phase("CENTER_HOLD")
    else:
      self.set_phase("LANDING_PREP")

  def abort_condition(self, flight_start: float) -> Optional[str]:
    state = self.state.uav_state
    mavros = self.state.mavros_state
    status = self.state.offboard_status
    if mavros is None or not mavros.connected:
      return "MAVROS disconnected"
    if mavros.armed and mavros.mode != "OFFBOARD":
      return f"unexpected mode while armed: {mavros.mode}"
    if state is None or (rospy.Time.now() - state.header.stamp).to_sec() > 0.75:
      return "UAV state stale"
    if status is None or status.state_name == "FAULT":
      return "adapter FAULT"
    if self.current_setpoint_rate(1.0) < 10.0:
      return "setpoint rate below 10 Hz"
    if self.state.nan_or_inf:
      return "NaN or Inf detected"
    if self.state.target is not None:
      if self.liftoff_time is None and self.start_position is not None:
        height_above_start = state.pose.position.z - self.start_position[2]
        vertical_speed = state.twist.linear.z
        if height_above_start > 0.15 and vertical_speed > 0.10:
          self.liftoff_time = time.time()
      dx = state.pose.position.x - self.state.target.position.x
      dy = state.pose.position.y - self.state.target.position.y
      dz = state.pose.position.z - self.state.target.position.z
      horizontal_error = math.hypot(dx, dy)
      if horizontal_error > 1.0:
        if self.horizontal_error_dwell.update(horizontal_error, time.time()):
          return f"horizontal error {horizontal_error:.3f} m"
      else:
        self.horizontal_error_dwell.update(0.0, time.time())
      if state.pose.position.z > self.state.target.position.z + ABORT_HEIGHT_OVERSHOOT_M:
        return f"height overshoot {state.pose.position.z - self.state.target.position.z:.3f} m"
      if state.pose.position.z < self.start_position[2] - 0.25:
        return f"height below start envelope {state.pose.position.z - self.start_position[2]:.3f} m"
      if self.liftoff_time is not None and abs(dz) > ABORT_HEIGHT_ERROR_M:
        if self.height_error_dwell.update(abs(dz), time.time()):
          return f"height error {abs(dz):.3f} m"
      else:
        self.height_error_dwell.update(0.0, time.time())
    roll, pitch = self.roll_pitch_from_quaternion(state.pose.orientation)
    if abs(roll) > math.radians(30.0) or abs(pitch) > math.radians(30.0):
      return f"attitude limit roll={math.degrees(roll):.1f} pitch={math.degrees(pitch):.1f}"
    return None

  def land_and_wait_disarmed(self) -> None:
    if self.flight_trajectory_stamp is None:
      raise RuntimeError("flight trajectory timestamp is unavailable before landing")
    if self.state.mavros_state is None:
      raise RuntimeError("MAVROS state unavailable before landing")
    now = rospy.Time.now().to_sec()
    remaining = reserve_remaining_at(self.flight_trajectory_stamp, now)
    if remaining < MIN_RESERVE_AT_LAND_REQUEST_SEC:
      raise RuntimeError(f"landing reserve too short before AUTO.LAND request: {remaining:.3f} s")
    if self.state.offboard_status is None or self.state.offboard_status.state_name == "FAULT":
      raise RuntimeError("adapter is not healthy before AUTO.LAND request")
    self.land_request_setpoint_rate, self.land_request_max_setpoint_gap = (
        self.current_setpoint_rate_and_gap(2.0))
    if self.land_request_setpoint_rate < 20.0 or self.land_request_max_setpoint_gap > 0.10:
      raise RuntimeError(
          "setpoint stream unhealthy before AUTO.LAND request: "
          f"rate={self.land_request_setpoint_rate:.2f}Hz gap={self.land_request_max_setpoint_gap:.3f}s")
    self.call_mode("AUTO.LAND", require_mode=True)
    if self.state.mavros_state is None or self.state.mavros_state.mode != "AUTO.LAND":
      raise RuntimeError("AUTO.LAND was not confirmed")
    self.set_phase("LANDING")
    if not can_close_output_gate(self.state.mavros_state.mode, self.land_confirm_time is not None):
      raise RuntimeError(f"refusing to close output gate while mode={self.state.mavros_state.mode}")
    close_deadline = time.time() + OUTPUT_GATE_CLOSE_AFTER_LAND_CONFIRM_SEC
    self.call_set_output(False)
    if self.output_disabled_time is None or self.output_disabled_time > close_deadline:
      self.output_gate_close_error = "output gate was not closed within deadline after AUTO.LAND confirm"
    self.wait_for("runtime output gate closed", lambda:
                  self.state.offboard_status is not None and
                  not self.state.offboard_status.output_active and
                  self.state.offboard_status.state_name != "FAULT", 3.0, 0.05)
    deadline = time.time() + 90.0
    while time.time() < deadline:
      if self.state.mavros_state is not None and not self.state.mavros_state.armed:
        self.disarm_time = time.time()
        self.land_complete_time = self.disarm_time
        self.set_phase("COMPLETE")
        self.phase.finish(rospy.Time.now().to_sec())
        return
      time.sleep(0.2)
    if self.state.uav_state is not None:
      z = self.state.uav_state.pose.position.z
      vz = self.state.uav_state.twist.linear.z
      if self.start_position is not None and abs(z - self.start_position[2]) < 0.15 and abs(vz) < 0.1:
        raise RuntimeError("vehicle landed but PX4 did not auto-disarm; refusing forced in-air disarm")
    raise RuntimeError("vehicle did not disarm after AUTO.LAND")

  def write_samples(self) -> None:
    for topic, path in [
        ("/mavros/state", "mavros_state_sample.txt"),
        ("/mavros/local_position/odom", "mavros_odom_sample.txt"),
        ("/mavros/setpoint_raw/local", "mavros_setpoint_sample.txt"),
        ("/uav/state", "uav_state_sample.txt"),
        ("/uav/trajectory", "uav_trajectory_sample.txt"),
        ("/uav/setpoint_preview", "uav_setpoint_preview_sample.txt"),
        ("/uav/mavros_target_preview", "uav_mavros_target_preview_sample.txt"),
        ("/uav/offboard_status", "uav_offboard_status_sample.txt"),
    ]:
      try:
        output = self.run_cmd(f"timeout 5 rostopic echo -n 1 {topic}", timeout=8.0)
      except Exception as exc:
        output = f"failed to read {topic}: {exc}\n"
      (self.run_dir / path).write_text(output, encoding="utf-8")

  def compute_metrics(self, prestream_rate: float) -> Dict[str, object]:
    samples = self.state.uav_samples
    target_samples = self.state.target_samples
    all_target_times = self.state.target_times_all
    metrics: Dict[str, object] = {
        "prestream_setpoint_rate_hz": prestream_rate,
        "setpoint_average_rate_hz": (
            (len(all_target_times) - 1) / (all_target_times[-1] - all_target_times[0])
            if len(all_target_times) > 1 and all_target_times[-1] > all_target_times[0]
            else 0.0
        ),
        "adapter_fault": self.state.adapter_fault,
        "adapter_fault_count": self.state.adapter_fault_count,
        "exited_offboard": self.state.exited_offboard,
        "unexpected_offboard_exit_count": self.state.unexpected_offboard_exit_count,
        "nan_or_inf": self.state.nan_or_inf,
        "abort_reason": self.abort_reason,
        "final_armed": self.state.mavros_state.armed if self.state.mavros_state else None,
        "final_mode": self.state.mavros_state.mode if self.state.mavros_state else None,
        "service_calls": self.service_calls,
    }
    if self.mode_request_time and self.mode_confirm_time:
      metrics["offboard_switch_sec"] = self.mode_confirm_time - self.mode_request_time
    if self.arm_request_time and self.arm_confirm_time:
      metrics["arming_sec"] = self.arm_confirm_time - self.arm_request_time
    if self.land_request_time and self.disarm_time:
      metrics["landing_to_disarm_sec"] = self.disarm_time - self.land_request_time
    if self.land_request_time and self.land_confirm_time:
      metrics["land_request_to_confirm_sec"] = self.land_confirm_time - self.land_request_time
    if self.land_confirm_time and self.output_disabled_time:
      metrics["land_confirm_to_output_gate_close_sec"] = (
          self.output_disabled_time - self.land_confirm_time)
    if self.flight_trajectory_stamp:
      if self.land_request_time:
        metrics["reserve_remaining_at_land_request_sec"] = reserve_remaining_at(
            self.flight_trajectory_stamp, self.land_request_time)
      if self.land_confirm_time:
        metrics["reserve_remaining_at_land_confirm_sec"] = reserve_remaining_at(
            self.flight_trajectory_stamp, self.land_confirm_time)
    metrics.update({
        "center_hold_completed": self.center_hold_completed,
        "landing_reserve_sec": LANDING_RESERVE_HOLD_SEC,
        "land_service_call_started_at": self.land_service_call_started_at,
        "land_service_response_at": self.land_service_response_at,
        "land_mode_first_observed_at": self.land_mode_first_observed_at,
        "offboard_last_observed_at": self.offboard_last_observed_at,
        "output_gate_close_requested_at": self.output_gate_close_requested_at,
        "output_gate_closed_at": self.output_disabled_time,
        "trajectory_natural_end_at": self.trajectory_end_time,
        "adapter_fault_first_at": self.adapter_fault_first_at,
        "disarm_at": self.disarm_time,
        "setpoint_rate_before_land_request": self.land_request_setpoint_rate,
        "max_setpoint_gap_before_land_request": self.land_request_max_setpoint_gap,
        "output_gate_closed_after_land_confirm": (
            self.land_confirm_time is not None and self.output_disabled_time is not None and
            self.output_disabled_time >= self.land_confirm_time),
        "output_gate_close_error": self.output_gate_close_error,
        "mode_at_output_gate_close": self.mode_at_output_gate_close,
        "adapter_fault_before_land_request": self.count_adapter_faults(None, self.land_request_time),
        "adapter_fault_before_land_confirm": self.count_adapter_faults(None, self.land_confirm_time),
        "adapter_fault_after_land_confirm": self.count_adapter_faults(self.land_confirm_time, None),
    })
    if self.state.attitude_samples:
      metrics["max_roll_deg"] = max(abs(math.degrees(s[1])) for s in self.state.attitude_samples)
      metrics["max_pitch_deg"] = max(abs(math.degrees(s[2])) for s in self.state.attitude_samples)
    tracking_summary = self.compute_tracking_metrics()
    if tracking_summary:
      metrics.update({
          "data_coverage_target": tracking_summary["coverage"]["target_time_coverage"],
          "data_coverage_actual": tracking_summary["coverage"]["actual_time_coverage"],
          "position_3d_rms_error_m": tracking_summary["position_3d_rms_error_m"],
          "horizontal_rms_error_m": tracking_summary["horizontal_rms_error_m"],
          "horizontal_max_error_m": tracking_summary["horizontal_max_error_m"],
          "height_mean_error_m": tracking_summary["height_mean_error_m"],
          "height_rms_error_m": tracking_summary["height_rms_error_m"],
          "velocity_rms_error_mps": tracking_summary["velocity_rms_error_mps"],
          "max_actual_speed_mps": tracking_summary["max_actual_speed_mps"],
          "max_actual_acceleration_mps2": tracking_summary["max_actual_acceleration_mps2"],
          "dropped_or_anomalous_intervals": tracking_summary["dropped_or_anomalous_intervals"],
          "nan_or_inf_count": tracking_summary["input_quality"]["total_nan_or_inf_values"],
      })
    turnpoint_metrics = self.compute_turnpoint_metrics()
    metrics.update(turnpoint_metrics)
    metrics["phase_metrics"] = self.compute_phase_metrics()
    return metrics

  def count_adapter_faults(self, start: Optional[float], end: Optional[float]) -> int:
    count = 0
    in_fault = False
    for stamp, state_name, _, _, _ in self.state.offboard_status_samples:
      if start is not None and stamp < start:
        continue
      if end is not None and stamp > end:
        continue
      if state_name == "FAULT" and not in_fault:
        count += 1
        in_fault = True
      elif state_name != "FAULT":
        in_fault = False
    return count

  def nearest_actual(self, t: float) -> Optional[Tuple[float, float, float, float, float, float, float]]:
    if not self.state.uav_samples:
      return None
    return min(self.state.uav_samples, key=lambda sample: abs(sample[0] - t))

  def nearest_target(self, t: float) -> Optional[Tuple[float, float, float, float, float, float, float]]:
    if not self.state.target_samples:
      return None
    return min(self.state.target_samples, key=lambda sample: abs(sample[0] - t))

  def compute_turnpoint_metrics(self) -> Dict[str, object]:
    trajectory_start = self.flight_trajectory_stamp
    if not trajectory_start or not self.start_position or not self.target_position:
      return {}
    points = [
        ("climb_end", trajectory_start + INITIAL_HOLD_SEC + RISE_DURATION_SEC,
         (self.start_position[0], self.start_position[1], self.target_position[2])),
        ("x_plus", trajectory_start + INITIAL_HOLD_SEC + RISE_DURATION_SEC +
         ALTITUDE_SETTLE_SEC + LINE_SEGMENT_DURATION_SEC,
         (self.start_position[0] + LINE_LENGTH_M, self.start_position[1], self.target_position[2])),
        ("x_minus", trajectory_start + INITIAL_HOLD_SEC + RISE_DURATION_SEC +
         ALTITUDE_SETTLE_SEC + 2.0 * LINE_SEGMENT_DURATION_SEC,
         (self.start_position[0] - LINE_LENGTH_M, self.start_position[1], self.target_position[2])),
        ("center_return", trajectory_start + INITIAL_HOLD_SEC + RISE_DURATION_SEC +
         ALTITUDE_SETTLE_SEC + 3.0 * LINE_SEGMENT_DURATION_SEC,
         (self.start_position[0], self.start_position[1], self.target_position[2])),
    ]
    result: Dict[str, object] = {}
    for name, t, target in points:
      window = [s for s in self.state.uav_samples if abs(s[0] - t) <= 0.75]
      if not window:
        continue
      errors = [math.sqrt((s[1] - target[0]) ** 2 + (s[2] - target[1]) ** 2 +
                          (s[3] - target[2]) ** 2) for s in window]
      result[f"{name}_max_error_m"] = max(errors)
      nearest = self.nearest_actual(t)
      if nearest:
        result[f"{name}_nearest_error_m"] = math.sqrt(
            (nearest[1] - target[0]) ** 2 + (nearest[2] - target[1]) ** 2 +
            (nearest[3] - target[2]) ** 2)
        if name == "center_return":
          result["center_endpoint_error_m"] = result[f"{name}_nearest_error_m"]
    return result

  def compute_phase_metrics(self) -> Dict[str, Dict[str, object]]:
    result: Dict[str, Dict[str, object]] = {}
    for phase_name, info in self.phase.summary().items():
      if info["status"] == "not_reached" or info.get("end") is None:
        result[phase_name] = {"status": "not_reached"}
        continue
      start = float(info["start"])
      end = float(info["end"])
      rows = []
      for sample in self.state.uav_samples:
        if start <= sample[0] <= end:
          target = self.nearest_target(sample[0])
          if target is None:
            continue
          dx = sample[1] - target[1]
          dy = sample[2] - target[2]
          dz = sample[3] - target[3]
          speed = math.sqrt(sample[4] ** 2 + sample[5] ** 2 + sample[6] ** 2)
          rows.append((dx, dy, dz, speed))
      if not rows:
        result[phase_name] = {"status": "not_reached"}
        continue
      horizontal = [math.hypot(r[0], r[1]) for r in rows]
      height = [r[2] for r in rows]
      result[phase_name] = {
          "status": "reached",
          "sample_count": len(rows),
          "duration_sec": end - start,
          "horizontal_rms_m": math.sqrt(sum(v * v for v in horizontal) / len(horizontal)),
          "height_mean_error_m": sum(height) / len(height),
          "height_rms_m": math.sqrt(sum(v * v for v in height) / len(height)),
          "height_max_abs_m": max(abs(v) for v in height),
          "max_speed_mps": max(r[3] for r in rows),
          "coverage": 1.0,
      }
    return result

  def compute_tracking_metrics(self) -> Optional[Dict[str, object]]:
    if not self.state.target_samples or not self.state.uav_samples:
      return None
    csv_path = self.run_dir / "tracking_samples.csv"
    start, end = tracking_acceptance_window(self.phase.starts, self.land_request_time,
                                            self.line_end_time)
    target = sorted(s for s in self.state.target_samples
                    if (start is None or s[0] >= start) and (end is None or s[0] <= end))
    actual = sorted(s for s in self.state.uav_samples
                    if (start is None or s[0] >= start) and (end is None or s[0] <= end))
    if len(target) < 2 or len(actual) < 2:
      return None
    actual_times = [s[0] for s in actual]

    def interpolate_actual(t: float) -> Optional[Tuple[float, float, float, float, float, float]]:
      if t < actual_times[0] or t > actual_times[-1]:
        return None
      for i in range(1, len(actual)):
        if actual[i][0] >= t:
          before = actual[i - 1]
          after = actual[i]
          dt = after[0] - before[0]
          if dt <= 0.0:
            return None
          ratio = (t - before[0]) / dt
          return tuple(before[j] + ratio * (after[j] - before[j]) for j in range(1, 7))
      last = actual[-1]
      return (last[1], last[2], last[3], last[4], last[5], last[6])

    rows = ["time,target_x,target_y,target_z,actual_x,actual_y,actual_z,"
            "target_vx,target_vy,target_vz,actual_vx,actual_vy,actual_vz\n"]
    for sample in target:
      interpolated = interpolate_actual(sample[0])
      if interpolated is None:
        continue
      rows.append(
          f"{sample[0]:.9f},{sample[1]:.9f},{sample[2]:.9f},{sample[3]:.9f},"
          f"{interpolated[0]:.9f},{interpolated[1]:.9f},{interpolated[2]:.9f},"
          f"{sample[4]:.9f},{sample[5]:.9f},{sample[6]:.9f},"
          f"{interpolated[3]:.9f},{interpolated[4]:.9f},{interpolated[5]:.9f}\n")
    if len(rows) < 3:
      return None
    csv_path.write_text("".join(rows), encoding="utf-8")
    output = self.run_cmd(
        f"/usr/bin/python3 scripts/analysis/trajectory_tracking_metrics.py {csv_path} "
        f"--output {self.run_dir / 'tracking_metrics.json'}",
        REPO_DIR,
        timeout=20.0,
    )
    if output:
      print(output, flush=True)
    return json.loads((self.run_dir / "tracking_metrics.json").read_text(encoding="utf-8"))

  def run(self) -> Dict[str, object]:
    px4_tag, px4_commit = self.verify_px4_version()
    self.start_process("roscore", "roscore", REPO_DIR)
    self.wait_for("ROS master", lambda: subprocess.run(
        ["bash", "-lc", "rosnode list >/dev/null 2>&1"],
        cwd=str(REPO_DIR)).returncode == 0, 15.0)
    rospy.init_node("m0_c5b1_sitl_line_experiment", anonymous=True, disable_signals=True)
    self.setup_ros_subscribers()

    px4_cmd = (
        "conda deactivate >/dev/null 2>&1 || true; "
        "unset PYTHONHOME; "
        "export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH; "
        "HEADLESS=1 make px4_sitl gazebo-classic 2>&1 | "
        "tr '\\r' '\\n' | grep --line-buffered -v '^pxh>'"
    )
    self.start_process("px4_sitl", px4_cmd, PX4_DIR)
    self.wait_log_contains(
        "px4_sitl",
        ["Simulator connected on TCP port 4560", "Startup script returned successfully"],
        120.0,
    )

    self.start_process(
        "mavros",
        'roslaunch "$(rospack find uav_bringup)/launch/sim/mavros_sitl.launch"',
        REPO_DIR,
    )
    self.wait_for("MAVROS connected and disarmed", lambda:
                  self.state.mavros_state is not None and
                  self.state.mavros_state.connected and
                  not self.state.mavros_state.armed, 30.0)
    self.verify_sitl_identity()

    self.start_process(
        "state_bridge_prefetch",
        'roslaunch "$(rospack find uav_bringup)/launch/sim/state_bridge_sitl.launch"',
        REPO_DIR,
    )
    frame_id, start_position, start_yaw = self.capture_start_state()
    self.stop_process("state_bridge_prefetch")

    launch_cmd = 'roslaunch "$(rospack find uav_bringup)/launch/sim/m0_c5b1_line_tracking.launch"'
    self.start_process("m0_c5b1_project_nodes", launch_cmd, REPO_DIR)
    self.wait_for("project state and adapter topics", lambda:
                  self.state.uav_state is not None and
                  self.state.offboard_status is not None, 30.0)
    self.set_phase("PREFLIGHT")
    ground_hold_id = self.publish_ground_hold_trajectory(frame_id, start_position, start_yaw)

    self.start_process(
        "rosbag",
        "rosbag record -O m0_c5b1.bag "
        "/mavros/state /mavros/local_position/odom /mavros/setpoint_raw/local "
        "/mavros/setpoint_raw/target_local /uav/state /uav/trajectory "
        "/uav/setpoint_preview /uav/mavros_target_preview /uav/offboard_status "
        "/uav/experiment_phase",
        self.run_dir,
    )

    self.set_phase("PRESTREAM")
    prestream_rate = self.wait_prestream()
    if self.state.mavros_state is None or self.state.mavros_state.armed:
      raise RuntimeError("vehicle must remain disarmed before OFFBOARD request")

    self.set_phase("OFFBOARD_PREARM")
    self.call_mode("OFFBOARD")
    self.call_arm(True)
    self.wait_for("ground hold target still active after arming", lambda:
                  self.state.setpoint_preview is not None and
                  self.state.setpoint_preview.trajectory_id == ground_hold_id and
                  abs(self.state.setpoint_preview.point.position.z - start_position[2]) < 0.05, 2.0, 0.05)
    self.set_phase("ARMED_HOLD")
    self.start_dynamic_flight_trajectory(frame_id)
    self.monitor_flight()
    self.land_and_wait_disarmed()
    self.write_samples()
    metrics = self.compute_metrics(prestream_rate)
    summary = {
        "run_dir": str(self.run_dir),
        "px4_tag": px4_tag,
        "px4_commit": px4_commit,
        "start_position": self.start_position,
        "target_position": self.target_position,
        "line_waypoints": [
            self.start_position,
            (self.start_position[0] + LINE_LENGTH_M, self.start_position[1], self.target_position[2]),
            (self.start_position[0] - LINE_LENGTH_M, self.start_position[1], self.target_position[2]),
            (self.start_position[0], self.start_position[1], self.target_position[2]),
        ] if self.start_position and self.target_position else None,
        "target_yaw": self.target_yaw,
        "ground_hold_trajectory_id": ground_hold_id,
        "flight_trajectory_id": self.flight_trajectory_id,
        "flight_trajectory_stamp": self.flight_trajectory_stamp,
        "center_hold_end_time": self.line_end_time,
        "flight_trajectory_end_time": self.trajectory_end_time,
        "trajectory_natural_end_at": self.trajectory_end_time,
        "land_request_time": self.land_request_time,
        "land_confirm_time": self.land_confirm_time,
        "land_service_call_started_at": self.land_service_call_started_at,
        "land_service_response_at": self.land_service_response_at,
        "land_mode_first_observed_at": self.land_mode_first_observed_at,
        "offboard_last_observed_at": self.offboard_last_observed_at,
        "output_gate_close_requested_at": self.output_gate_close_requested_at,
        "output_disabled_time": self.output_disabled_time,
        "output_gate_closed_at": self.output_disabled_time,
        "adapter_fault_first_at": self.adapter_fault_first_at,
        "disarm_at": self.disarm_time,
        "liftoff_time": self.liftoff_time,
        "phase_times": self.phase.summary(),
        "mavros_connected": self.state.mavros_state.connected if self.state.mavros_state else None,
        "mavros_armed": self.state.mavros_state.armed if self.state.mavros_state else None,
        "mavros_mode": self.state.mavros_state.mode if self.state.mavros_state else None,
        "offboard_status": self.state.offboard_status.state_name if self.state.offboard_status else None,
        "metrics": metrics,
    }
    (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    return summary


def main() -> int:
  experiment = Experiment()
  try:
    summary = experiment.run()
    metrics = summary["metrics"]
    passed = (
        metrics.get("abort_reason") is None and
        metrics.get("setpoint_average_rate_hz", 0.0) >= 20.0 and
        metrics.get("data_coverage_target", 0.0) >= 0.95 and
        metrics.get("data_coverage_actual", 0.0) >= 0.95 and
        metrics.get("horizontal_rms_error_m", 99.0) <= 0.25 and
        metrics.get("horizontal_max_error_m", 99.0) <= 0.5 and
        metrics.get("height_rms_error_m", 99.0) <= 0.25 and
        metrics.get("center_endpoint_error_m", 99.0) <= 0.25 and
        not metrics.get("nan_or_inf") and
        metrics.get("nan_or_inf_count", 1) == 0 and
        not metrics.get("adapter_fault") and
        metrics.get("adapter_fault_count", 1) == 0 and
        metrics.get("unexpected_offboard_exit_count", 1) == 0 and
        metrics.get("center_hold_completed") is True and
        metrics.get("reserve_remaining_at_land_request_sec", 0.0) >= MIN_RESERVE_AT_LAND_REQUEST_SEC and
        metrics.get("output_gate_closed_after_land_confirm") is True and
        metrics.get("output_gate_close_error") is None and
        metrics.get("mode_at_output_gate_close") == "AUTO.LAND" and
        metrics.get("final_armed") is False and
        summary["phase_times"].get("LINE_FORWARD", {}).get("status") == "reached" and
        summary["phase_times"].get("LINE_REVERSE", {}).get("status") == "reached" and
        summary["phase_times"].get("LINE_RETURN", {}).get("status") == "reached" and
        summary["phase_times"].get("CENTER_HOLD", {}).get("status") == "reached"
    )
    return 0 if passed else 4
  except Exception as exc:
    print(f"M0-C5B1 experiment failed: {exc}", file=sys.stderr, flush=True)
    try:
      if experiment.state.mavros_state is not None and experiment.state.mavros_state.armed:
        experiment.call_mode("AUTO.LAND", require_mode=False)
    except Exception as land_exc:
      print(f"AUTO.LAND attempt after failure failed: {land_exc}", file=sys.stderr, flush=True)
    return 1
  finally:
    try:
      mavros_state = experiment.state.mavros_state if getattr(experiment, "state", None) else None
      can_disable_output = (
          mavros_state is not None and
          (not mavros_state.armed or mavros_state.mode == "AUTO.LAND"))
      if can_disable_output and experiment.output_disabled_time is None:
        try:
          experiment.call_set_output(False)
        except Exception:
          pass
    finally:
      experiment.stop_processes()


if __name__ == "__main__":
  sys.exit(main())
