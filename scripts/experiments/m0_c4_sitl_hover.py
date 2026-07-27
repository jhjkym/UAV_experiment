#!/usr/bin/env python3
"""M0-C4 PX4 SITL-only offboard hover experiment.

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
from std_srvs.srv import SetBool
from uav_msgs.msg import OffboardStatus, SetpointPreview, Trajectory, TrajectoryPoint, UavState


REPO_DIR = Path("/home/tom/UAV_experiment")
PX4_DIR = Path("/home/tom/third_party/PX4-Autopilot")
LOG_ROOT = Path("/tmp/uav_m0c4")
PX4_BIN_TOKEN = "build/px4_sitl_default/bin/px4"
TARGET_ALTITUDE_DELTA_M = 1.0
RISE_DURATION_SEC = 5.0
HOVER_DURATION_SEC = 15.0
TRAJECTORY_HOLD_SEC = 120.0


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
  uav_samples: List[Tuple[float, float, float, float, float, float, float]] = field(default_factory=list)
  adapter_fault: bool = False
  exited_offboard: bool = False
  nan_or_inf: bool = False


class Experiment:
  def __init__(self) -> None:
    if os.environ.get("UAV_ALLOW_SITL_FLIGHT") != "YES":
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
    self.disarm_time: Optional[float] = None
    self.flight_start_time: Optional[float] = None
    self.hover_end_time: Optional[float] = None
    self.abort_reason: Optional[str] = None
    self.service_calls: List[Dict[str, object]] = []

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

  def on_mavros_state(self, msg: State) -> None:
    previous = self.state.mavros_state
    self.state.mavros_state = msg
    if (previous is not None and previous.mode == "OFFBOARD" and
        msg.mode != "OFFBOARD" and msg.armed and self.land_request_time is None):
      self.state.exited_offboard = True

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

  def on_odom(self, msg: Odometry) -> None:
    self.state.odom = msg

  def on_preview(self, msg: SetpointPreview) -> None:
    self.state.setpoint_preview = msg

  def on_status(self, msg: OffboardStatus) -> None:
    self.state.offboard_status = msg
    if msg.state_name == "FAULT":
      self.state.adapter_fault = True

  def on_target(self, msg: PositionTarget) -> None:
    self.state.target = msg
    now = rospy.Time.now().to_sec()
    self.state.target_times.append(now)
    self.state.target_times_all.append(now)
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

  def publish_hover_trajectory(self) -> None:
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
    trajectory = Trajectory()
    trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
    trajectory.header.frame_id = state.header.frame_id or "map"
    trajectory.mode = Trajectory.MODE_NOMINAL
    trajectory.trajectory_id = int(time.time()) & 0xFFFFFFFF
    trajectory.points = [
        self.make_point(0.0, self.start_position, self.target_yaw),
        self.make_point(RISE_DURATION_SEC, self.target_position, self.target_yaw),
        self.make_point(RISE_DURATION_SEC + TRAJECTORY_HOLD_SEC, self.target_position, self.target_yaw),
    ]
    for _ in range(5):
      self.trajectory_pub.publish(trajectory)
      rospy.sleep(0.1)
    print(
        "trajectory published: "
        f"start={self.start_position} target={self.target_position} yaw={self.target_yaw:.3f}",
        flush=True,
    )

  def make_point(self, t: float, position: Tuple[float, float, float], yaw: float) -> TrajectoryPoint:
    point = TrajectoryPoint()
    point.time_from_start = rospy.Duration(t)
    point.position = Point(*position)
    point.velocity = Vector3(0.0, 0.0, 0.0)
    point.acceleration = Vector3(0.0, 0.0, 0.0)
    point.yaw = yaw
    point.yaw_rate = 0.0
    return point

  def call_set_output(self, enabled: bool) -> None:
    rospy.wait_for_service("/offboard_adapter_node/set_output_enabled", timeout=5.0)
    service = rospy.ServiceProxy("/offboard_adapter_node/set_output_enabled", SetBool)
    response = service(enabled)
    self.service_calls.append({"service": "set_output_enabled", "value": enabled,
                               "success": bool(response.success), "message": response.message})
    if enabled and not response.success:
      raise RuntimeError(f"failed to enable output gate: {response.message}")

  def call_mode(self, mode: str, require_mode: bool = True) -> None:
    rospy.wait_for_service("/mavros/set_mode", timeout=5.0)
    service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    for attempt in range(1, 4):
      request_time = time.time()
      response = service(custom_mode=mode)
      self.service_calls.append({"service": "set_mode", "mode": mode,
                                 "attempt": attempt, "mode_sent": bool(response.mode_sent)})
      if mode == "OFFBOARD" and self.mode_request_time is None:
        self.mode_request_time = request_time
      if mode == "AUTO.LAND" and self.land_request_time is None:
        self.land_request_time = request_time
      deadline = time.time() + 3.0
      while time.time() < deadline:
        if self.state.mavros_state is not None and self.state.mavros_state.mode == mode:
          if mode == "OFFBOARD" and self.mode_confirm_time is None:
            self.mode_confirm_time = time.time()
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
    self.flight_start_time = time.time()
    self.hover_end_time = self.flight_start_time + RISE_DURATION_SEC + HOVER_DURATION_SEC
    while time.time() < self.hover_end_time and not rospy.is_shutdown():
      reason = self.abort_condition(self.flight_start_time)
      if reason:
        self.abort_reason = reason
        print(f"abort condition: {reason}", flush=True)
        if self.state.mavros_state and self.state.mavros_state.armed:
          self.call_mode("AUTO.LAND", require_mode=False)
        return
      time.sleep(0.1)

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
    dx = state.pose.position.x - self.start_position[0]
    dy = state.pose.position.y - self.start_position[1]
    horizontal = math.hypot(dx, dy)
    if horizontal > 2.0:
      return f"horizontal offset {horizontal:.3f} m"
    elapsed = max(0.0, min(time.time() - flight_start, RISE_DURATION_SEC))
    expected_z = self.start_position[2] + TARGET_ALTITUDE_DELTA_M * (elapsed / RISE_DURATION_SEC)
    if abs(state.pose.position.z - expected_z) > 1.0:
      return f"height error {abs(state.pose.position.z - expected_z):.3f} m"
    roll, pitch = self.roll_pitch_from_quaternion(state.pose.orientation)
    if abs(roll) > math.radians(30.0) or abs(pitch) > math.radians(30.0):
      return f"attitude limit roll={math.degrees(roll):.1f} pitch={math.degrees(pitch):.1f}"
    return None

  def land_and_wait_disarmed(self) -> None:
    self.call_mode("AUTO.LAND", require_mode=False)
    deadline = time.time() + 90.0
    while time.time() < deadline:
      if self.state.mavros_state is not None and not self.state.mavros_state.armed:
        self.disarm_time = time.time()
        return
      time.sleep(0.2)
    if self.state.uav_state is not None:
      z = self.state.uav_state.pose.position.z
      vz = self.state.uav_state.twist.linear.z
      if self.start_position is not None and abs(z - self.start_position[2]) < 0.15 and abs(vz) < 0.1:
        self.call_arm(False)
        return
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
    all_target_times = self.state.target_times_all
    metrics: Dict[str, object] = {
        "prestream_setpoint_rate_hz": prestream_rate,
        "setpoint_average_rate_hz": (
            (len(all_target_times) - 1) / (all_target_times[-1] - all_target_times[0])
            if len(all_target_times) > 1 and all_target_times[-1] > all_target_times[0]
            else 0.0
        ),
        "adapter_fault": self.state.adapter_fault,
        "exited_offboard": self.state.exited_offboard,
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
    if self.start_position and self.target_position and samples:
      target_z = self.target_position[2]
      hover_start = (self.flight_start_time or self.arm_confirm_time or samples[0][0]) + RISE_DURATION_SEC
      hover_end = self.land_request_time or self.hover_end_time or samples[-1][0]
      hover_samples = [s for s in samples if hover_start <= s[0] <= hover_end]
      all_horizontal = [math.hypot(s[1] - self.start_position[0], s[2] - self.start_position[1])
                        for s in samples]
      all_speeds = [math.sqrt(s[4] ** 2 + s[5] ** 2 + s[6] ** 2) for s in samples]
      metrics["max_horizontal_error_m"] = max(all_horizontal) if all_horizontal else None
      metrics["max_speed_mps"] = max(all_speeds) if all_speeds else None
      metrics["max_altitude_overshoot_m"] = max([s[3] - target_z for s in samples] + [0.0])
      if hover_samples:
        z_errors = [s[3] - target_z for s in hover_samples]
        xy_errors = [math.hypot(s[1] - self.target_position[0], s[2] - self.target_position[1])
                     for s in hover_samples]
        metrics["hover_height_mean_error_m"] = sum(z_errors) / len(z_errors)
        metrics["hover_height_rms_error_m"] = math.sqrt(sum(e * e for e in z_errors) / len(z_errors))
        metrics["hover_position_rms_error_m"] = math.sqrt(sum(e * e for e in xy_errors) / len(xy_errors))
    return metrics

  def run(self) -> Dict[str, object]:
    px4_tag, px4_commit = self.verify_px4_version()
    self.start_process("roscore", "roscore", REPO_DIR)
    self.wait_for("ROS master", lambda: subprocess.run(
        ["bash", "-lc", "rosnode list >/dev/null 2>&1"],
        cwd=str(REPO_DIR)).returncode == 0, 15.0)
    rospy.init_node("m0_c4_sitl_hover_experiment", anonymous=True, disable_signals=True)
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
        "m0_c4_project_nodes",
        'roslaunch "$(rospack find uav_bringup)/launch/sim/m0_c4_offboard_hover.launch"',
        REPO_DIR,
    )
    self.wait_for("project state and adapter topics", lambda:
                  self.state.uav_state is not None and
                  self.state.offboard_status is not None, 30.0)

    self.start_process(
        "rosbag",
        "rosbag record -O m0_c4.bag "
        "/mavros/state /mavros/local_position/odom /mavros/setpoint_raw/local "
        "/mavros/setpoint_raw/target_local /uav/state /uav/trajectory "
        "/uav/setpoint_preview /uav/mavros_target_preview /uav/offboard_status",
        self.run_dir,
    )

    self.publish_hover_trajectory()
    prestream_rate = self.wait_prestream()
    if self.state.mavros_state is None or self.state.mavros_state.armed:
      raise RuntimeError("vehicle must remain disarmed before OFFBOARD request")

    self.call_mode("OFFBOARD")
    self.call_arm(True)
    self.monitor_flight()
    self.land_and_wait_disarmed()
    self.call_set_output(False)
    self.write_samples()
    metrics = self.compute_metrics(prestream_rate)
    summary = {
        "run_dir": str(self.run_dir),
        "px4_tag": px4_tag,
        "px4_commit": px4_commit,
        "start_position": self.start_position,
        "target_position": self.target_position,
        "target_yaw": self.target_yaw,
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
        metrics.get("prestream_setpoint_rate_hz", 0.0) >= 20.0 and
        (metrics.get("max_horizontal_error_m") is None or metrics["max_horizontal_error_m"] <= 0.5) and
        (metrics.get("hover_position_rms_error_m") is None or metrics["hover_position_rms_error_m"] <= 0.25) and
        (metrics.get("max_altitude_overshoot_m") is None or metrics["max_altitude_overshoot_m"] <= 0.3) and
        not metrics.get("nan_or_inf") and
        not metrics.get("adapter_fault") and
        metrics.get("final_armed") is False
    )
    return 0 if passed else 4
  except Exception as exc:
    print(f"M0-C4 experiment failed: {exc}", file=sys.stderr, flush=True)
    try:
      if experiment.state.mavros_state is not None and experiment.state.mavros_state.armed:
        experiment.call_mode("AUTO.LAND", require_mode=False)
    except Exception as land_exc:
      print(f"AUTO.LAND attempt after failure failed: {land_exc}", file=sys.stderr, flush=True)
    return 1
  finally:
    try:
      if getattr(experiment, "state", None) and experiment.state.mavros_state is not None:
        try:
          experiment.call_set_output(False)
        except Exception:
          pass
    finally:
      experiment.stop_processes()


if __name__ == "__main__":
  sys.exit(main())
