#!/usr/bin/env python3
"""M0-C5B1-R1A ground-only pending handoff rehearsal.

This script starts PX4 SITL, Gazebo Classic, MAVROS, and project-side nodes,
but it never arms, never requests OFFBOARD, and never calls PX4 mode services.
It verifies that a future-start trajectory can be queued while the adapter
continues streaming the active ground-hold setpoint.
"""

import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rospy
from geometry_msgs.msg import Point, Vector3
from mavros_msgs.msg import PositionTarget, State
from std_srvs.srv import SetBool
from uav_msgs.msg import OffboardStatus, SetpointPreview, Trajectory, TrajectoryPoint, UavState


REPO_DIR = Path("/home/tom/UAV_experiment")
PX4_DIR = Path("/home/tom/third_party/PX4-Autopilot")
LOG_ROOT = Path("/tmp/uav_m0_c5b1")
PX4_BIN_TOKEN = "build/px4_sitl_default/bin/px4"


@dataclass
class ManagedProcess:
  name: str
  process: subprocess.Popen
  log_path: Path


def finite(values: List[float]) -> bool:
  return all(math.isfinite(v) for v in values)


def norm3(values: Tuple[float, float, float]) -> float:
  return math.sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2])


def yaw_from_quaternion(q) -> float:
  siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
  cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
  return math.atan2(siny_cosp, cosy_cosp)


def make_hold_trajectory(frame_id: str,
                         position: Tuple[float, float, float],
                         yaw: float,
                         start_stamp: rospy.Time,
                         trajectory_id: int) -> Trajectory:
  trajectory = Trajectory()
  trajectory.header.stamp = start_stamp
  trajectory.header.frame_id = frame_id
  trajectory.mode = Trajectory.MODE_NOMINAL
  trajectory.trajectory_id = trajectory_id
  for t in (0.0, 60.0):
    point = TrajectoryPoint()
    point.time_from_start = rospy.Duration(t)
    point.position = Point(*position)
    point.velocity = Vector3(0.0, 0.0, 0.0)
    point.acceleration = Vector3(0.0, 0.0, 0.0)
    point.yaw = yaw
    point.yaw_rate = 0.0
    trajectory.points.append(point)
  return trajectory


class GroundHandoffRehearsal:
  def __init__(self) -> None:
    if os.environ.get("UAV_ALLOW_SITL_FLIGHT") != "YES":
      raise RuntimeError("UAV_ALLOW_SITL_FLIGHT must be exactly YES for SITL rehearsal")
    self.run_dir = LOG_ROOT / time.strftime("handoff_%Y%m%d_%H%M%S")
    self.run_dir.mkdir(parents=True, exist_ok=False)
    self.processes: List[ManagedProcess] = []
    self.mavros_state: Optional[State] = None
    self.uav_state: Optional[UavState] = None
    self.preview: Optional[SetpointPreview] = None
    self.status: Optional[OffboardStatus] = None
    self.last_trajectory: Optional[Trajectory] = None
    self.target_times: List[float] = []
    self.preview_samples: List[Tuple[float, SetpointPreview]] = []
    self.status_samples: List[Tuple[float, OffboardStatus]] = []
    self.state_samples: List[Tuple[float, State]] = []
    self.nan_or_inf_count = 0

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
      try:
        os.killpg(os.getpgid(managed.process.pid), signal.SIGINT)
      except ProcessLookupError:
        pass
    time.sleep(3.0)
    for managed in reversed(self.processes):
      if managed.process.poll() is not None:
        continue
      try:
        os.killpg(os.getpgid(managed.process.pid), signal.SIGTERM)
      except ProcessLookupError:
        pass

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

  def wait_for(self, description: str, predicate, timeout: float, period: float = 0.05) -> None:
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
        raise RuntimeError(f"{name} exited before ready")
      text = managed.log_path.read_text(encoding="utf-8", errors="replace")
      if all(token in text for token in required):
        print(f"ready: {name}", flush=True)
        return
      time.sleep(0.5)
    raise RuntimeError(f"timeout waiting for {name}")

  def setup_ros(self) -> None:
    rospy.Subscriber("/mavros/state", State, self.on_mavros_state, queue_size=50)
    rospy.Subscriber("/uav/state", UavState, self.on_uav_state, queue_size=50)
    rospy.Subscriber("/uav/setpoint_preview", SetpointPreview, self.on_preview, queue_size=200)
    rospy.Subscriber("/uav/offboard_status", OffboardStatus, self.on_status, queue_size=200)
    rospy.Subscriber("/mavros/setpoint_raw/local", PositionTarget, self.on_target, queue_size=300)
    rospy.Subscriber("/uav/trajectory", Trajectory, self.on_trajectory, queue_size=10)
    self.trajectory_pub = rospy.Publisher("/uav/trajectory", Trajectory, queue_size=1, latch=True)

  def on_mavros_state(self, msg: State) -> None:
    self.mavros_state = msg
    self.state_samples.append((rospy.Time.now().to_sec(), msg))

  def on_uav_state(self, msg: UavState) -> None:
    self.uav_state = msg
    if not finite([
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
    ]):
      self.nan_or_inf_count += 1

  def on_preview(self, msg: SetpointPreview) -> None:
    self.preview = msg
    self.preview_samples.append((rospy.Time.now().to_sec(), msg))
    if not finite([
        msg.point.position.x, msg.point.position.y, msg.point.position.z,
        msg.point.velocity.x, msg.point.velocity.y, msg.point.velocity.z,
        msg.point.acceleration.x, msg.point.acceleration.y, msg.point.acceleration.z,
        msg.point.yaw, msg.point.yaw_rate,
    ]):
      self.nan_or_inf_count += 1

  def on_status(self, msg: OffboardStatus) -> None:
    self.status = msg
    self.status_samples.append((rospy.Time.now().to_sec(), msg))

  def on_target(self, msg: PositionTarget) -> None:
    self.target_times.append(rospy.Time.now().to_sec())
    if not finite([
        msg.position.x, msg.position.y, msg.position.z,
        msg.velocity.x, msg.velocity.y, msg.velocity.z,
        msg.acceleration_or_force.x, msg.acceleration_or_force.y, msg.acceleration_or_force.z,
        msg.yaw, msg.yaw_rate,
    ]):
      self.nan_or_inf_count += 1

  def on_trajectory(self, msg: Trajectory) -> None:
    self.last_trajectory = msg

  def verify_px4(self) -> None:
    tag = self.run_cmd("git describe --tags --exact-match", PX4_DIR).strip()
    commit = self.run_cmd("git rev-parse HEAD", PX4_DIR).strip()
    if tag != "v1.14.3":
      raise RuntimeError(f"PX4 tag mismatch: {tag}")
    if commit != "1dacb4cdef2d7145754fc788fa8dc482eed74b40":
      raise RuntimeError(f"PX4 commit mismatch: {commit}")

  def verify_sitl_identity(self) -> None:
    ps_output = self.run_cmd("ps -eo pid=,args=", timeout=5.0)
    if PX4_BIN_TOKEN not in ps_output:
      raise RuntimeError("PX4 SITL binary was not found in process list")
    if "gzserver" not in ps_output:
      raise RuntimeError("Gazebo Classic gzserver was not found")
    fcu_url = self.run_cmd("rosparam get /mavros/fcu_url", timeout=5.0).strip()
    if not fcu_url.startswith("udp://") or "serial://" in fcu_url or "/dev/tty" in fcu_url:
      raise RuntimeError(f"refusing non-UDP SITL fcu_url: {fcu_url}")
    serial_devices = list(Path("/dev").glob("ttyACM*")) + list(Path("/dev").glob("ttyUSB*"))
    if serial_devices:
      raise RuntimeError(f"refusing while serial flight-controller-like devices exist: {serial_devices}")

  def set_output(self, enabled: bool) -> None:
    rospy.wait_for_service("/offboard_adapter_node/set_output_enabled", timeout=5.0)
    service = rospy.ServiceProxy("/offboard_adapter_node/set_output_enabled", SetBool)
    response = service(enabled)
    if not response.success:
      raise RuntimeError(f"set_output_enabled({enabled}) failed: {response.message}")

  def rate_summary(self, start: float, end: float) -> Tuple[float, float, int]:
    times = [t for t in self.target_times if start <= t <= end]
    if len(times) < 2:
      return 0.0, 999.0, 1
    intervals = [b - a for a, b in zip(times[:-1], times[1:])]
    rate = (len(times) - 1) / (times[-1] - times[0])
    low_windows = 0
    for i in range(len(times)):
      window = [t for t in times if times[i] <= t <= times[i] + 1.0]
      if len(window) >= 2:
        wrate = (len(window) - 1) / (window[-1] - window[0])
        if wrate < 20.0:
          low_windows += 1
    return rate, max(intervals), low_windows

  def execute(self) -> Dict[str, object]:
    self.verify_px4()
    self.start_process("roscore", "roscore", REPO_DIR)
    self.wait_for("ROS master", lambda: subprocess.run(
        ["bash", "-lc", "rosnode list >/dev/null 2>&1"], cwd=str(REPO_DIR)).returncode == 0, 15.0)
    rospy.init_node("m0_c5b1_r1a_handoff_ground", anonymous=True, disable_signals=True)
    self.setup_ros()

    px4_cmd = (
        "conda deactivate >/dev/null 2>&1 || true; "
        "unset PYTHONHOME; export PATH=/usr/bin:/bin:/usr/sbin:/sbin:$PATH; "
        "HEADLESS=1 make px4_sitl gazebo-classic 2>&1 | "
        "tr '\\r' '\\n' | grep --line-buffered -v '^pxh>'"
    )
    self.start_process("px4_sitl", px4_cmd, PX4_DIR)
    self.wait_log_contains("px4_sitl",
                           ["Simulator connected on TCP port 4560",
                            "Startup script returned successfully"], 120.0)
    self.start_process("mavros", 'roslaunch "$(rospack find uav_bringup)/launch/sim/mavros_sitl.launch"', REPO_DIR)
    self.wait_for("MAVROS connected disarmed not offboard", lambda:
                  self.mavros_state is not None and self.mavros_state.connected and
                  not self.mavros_state.armed and self.mavros_state.mode != "OFFBOARD", 30.0)
    self.verify_sitl_identity()
    self.start_process("project_nodes",
                       'roslaunch "$(rospack find uav_bringup)/launch/sim/m0_c5b1_line_tracking.launch"',
                       REPO_DIR)
    self.wait_for("fresh /uav/state and adapter", lambda:
                  self.uav_state is not None and self.uav_state.pose_valid and
                  self.uav_state.twist_valid and self.status is not None, 30.0)

    state = self.uav_state
    assert state is not None
    frame_id = state.header.frame_id or "map"
    start = (state.pose.position.x, state.pose.position.y, state.pose.position.z)
    yaw = yaw_from_quaternion(state.pose.orientation)
    active_id = int(time.time()) & 0xFFFFFFFF
    hold = make_hold_trajectory(frame_id, start, yaw, rospy.Time.now() + rospy.Duration(0.2), active_id)
    for _ in range(5):
      self.trajectory_pub.publish(hold)
      rospy.sleep(0.1)
    self.wait_for("active hold preview", lambda:
                  self.preview is not None and self.preview.trajectory_id == active_id and
                  self.preview.trajectory_valid and self.preview.started, 10.0)
    self.start_process(
        "rosbag",
        "rosbag record -O handoff.bag /mavros/state /mavros/setpoint_raw/local "
        "/uav/state /uav/trajectory /uav/setpoint_preview /uav/mavros_target_preview "
        "/uav/offboard_status",
        self.run_dir,
    )
    self.set_output(True)
    pre_start = rospy.Time.now().to_sec()
    rospy.sleep(3.2)
    pre_end = rospy.Time.now().to_sec()

    pending_start_count = len(self.preview_samples)
    command = (
        "rosrun uav_trajectory dynamic_trajectory_publisher_node "
        "_trajectory_type:=line "
        f"_frame_id:={frame_id} "
        "_start_delay_sec:=1.000 "
        f"_start_x:={start[0]:.9f} _start_y:={start[1]:.9f} _start_z:={start[2]:.9f} "
        f"_start_yaw:={yaw:.9f} "
        "_altitude_offset_m:=1.0 _initial_hold_sec:=1.000 "
        "_initial_climb_duration_sec:=5.000 _post_climb_hold_sec:=2.000 "
        "_line_length_m:=1.000 _line_segment_duration_sec:=5.000 "
        "_hold_end_sec:=10.000 _yaw_mode:=fixed _publish_once:=true"
    )
    self.start_process("pending_dynamic_trajectory", command, REPO_DIR)
    self.wait_for("pending trajectory message", lambda:
                  self.last_trajectory is not None and self.last_trajectory.trajectory_id != active_id,
                  5.0)
    pending = self.last_trajectory
    assert pending is not None
    pending_id = pending.trajectory_id
    planned_switch = pending.header.stamp.to_sec()

    wait_window_end = min(planned_switch - 0.05, rospy.Time.now().to_sec() + 0.9)
    while rospy.Time.now().to_sec() < wait_window_end:
      if self.preview is None or self.preview.trajectory_id != active_id:
        raise RuntimeError("preview switched before pending header.stamp")
      if self.status is None or not self.status.output_active or self.status.state_name == "FAULT":
        raise RuntimeError("adapter stopped output during pending wait")
      if self.mavros_state is None or self.mavros_state.armed or self.mavros_state.mode == "OFFBOARD":
        raise RuntimeError("vehicle entered forbidden control state during pending wait")
      rospy.sleep(0.02)

    self.wait_for("single switch to pending id", lambda:
                  self.preview is not None and self.preview.trajectory_id == pending_id and
                  self.preview.started, 2.0, 0.01)
    actual_switch_sample = next((item for item in self.preview_samples[pending_start_count:]
                                 if item[1].trajectory_id == pending_id and item[1].started),
                                None)
    if actual_switch_sample is None:
      raise RuntimeError("failed to locate actual switch sample")
    actual_switch = actual_switch_sample[0]
    before_candidates = [item for item in self.preview_samples if item[0] < actual_switch and
                         item[1].trajectory_id == active_id]
    if not before_candidates:
      raise RuntimeError("no active preview before switch")
    before = before_candidates[-1][1]
    after = actual_switch_sample[1]
    pos_jump = norm3((after.point.position.x - before.point.position.x,
                      after.point.position.y - before.point.position.y,
                      after.point.position.z - before.point.position.z))
    vel_jump = norm3((after.point.velocity.x - before.point.velocity.x,
                      after.point.velocity.y - before.point.velocity.y,
                      after.point.velocity.z - before.point.velocity.z))
    acc_jump = norm3((after.point.acceleration.x - before.point.acceleration.x,
                      after.point.acceleration.y - before.point.acceleration.y,
                      after.point.acceleration.z - before.point.acceleration.z))
    rospy.sleep(3.0)
    post_end = rospy.Time.now().to_sec()
    self.set_output(False)

    switch_ids = []
    last_id = None
    for _, msg in self.preview_samples[pending_start_count:]:
      if last_id is None:
        last_id = msg.trajectory_id
      elif msg.trajectory_id != last_id:
        switch_ids.append((last_id, msg.trajectory_id))
        last_id = msg.trajectory_id
    rate, max_gap, low_windows = self.rate_summary(pre_start, post_end)
    fault_count = sum(1 for _, msg in self.status_samples if msg.state_name == "FAULT")
    inactive_count = sum(1 for t, msg in self.status_samples
                         if pre_start <= t <= post_end and not msg.output_active)
    armed_any = any(msg.armed for _, msg in self.state_samples)
    offboard_any = any(msg.mode == "OFFBOARD" for _, msg in self.state_samples)
    summary = {
        "run_dir": str(self.run_dir),
        "active_trajectory_id": active_id,
        "pending_trajectory_id": pending_id,
        "planned_switch_time": planned_switch,
        "actual_switch_time": actual_switch,
        "switch_timing_error_sec": actual_switch - planned_switch,
        "position_jump_m": pos_jump,
        "velocity_jump_mps": vel_jump,
        "acceleration_jump_mps2": acc_jump,
        "setpoint_average_rate_hz": rate,
        "max_setpoint_gap_sec": max_gap,
        "low_rate_window_count": low_windows,
        "adapter_fault_count": fault_count,
        "output_active_false_count": inactive_count,
        "trajectory_id_switch_count": len(switch_ids),
        "armed_always_false": not armed_any,
        "offboard_ever": offboard_any,
        "nan_or_inf_count": self.nan_or_inf_count,
    }
    summary["passed"] = (
        rate >= 20.0 and max_gap <= 0.10 and fault_count == 0 and inactive_count == 0 and
        len(switch_ids) == 1 and abs(actual_switch - planned_switch) <= 0.10 and
        pos_jump <= 0.05 and vel_jump <= 0.10 and acc_jump <= 0.20 and
        not armed_any and not offboard_any and self.nan_or_inf_count == 0)
    (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True),
                                                encoding="utf-8")
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    return summary


def main() -> int:
  rehearsal = GroundHandoffRehearsal()
  try:
    summary = rehearsal.execute()
    return 0 if summary.get("passed") else 4
  except Exception as exc:
    print(f"M0-C5B1-R1A handoff rehearsal failed: {exc}", file=sys.stderr, flush=True)
    return 1
  finally:
    try:
      if rospy.core.is_initialized():
        try:
          rehearsal.set_output(False)
        except Exception:
          pass
    finally:
      rehearsal.stop_processes()


if __name__ == "__main__":
  sys.exit(main())
