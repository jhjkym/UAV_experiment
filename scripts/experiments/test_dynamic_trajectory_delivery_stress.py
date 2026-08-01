#!/usr/bin/env python3
import argparse
import json
import math
import os
import random
import shutil
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rospy
import rosgraph
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from uav_msgs.msg import SetpointPreview, Trajectory, TrajectoryPoint


DEFAULT_ROUNDS = {"A": 20, "B": 20, "C": 5, "D": 10, "E": 20}


def wait_until(predicate, timeout, interval=0.02):
  deadline = time.time() + timeout
  while time.time() < deadline:
    value = predicate()
    if value:
      return value
    time.sleep(interval)
  return None


def finite_preview(msg):
  values = [
      msg.point.position.x, msg.point.position.y, msg.point.position.z,
      msg.point.velocity.x, msg.point.velocity.y, msg.point.velocity.z,
      msg.point.acceleration.x, msg.point.acceleration.y, msg.point.acceleration.z,
      msg.point.yaw, msg.point.yaw_rate,
  ]
  return all(math.isfinite(v) for v in values)


def make_point(t, x, y=0.0, z=0.0):
  point = TrajectoryPoint()
  point.time_from_start = rospy.Duration.from_sec(t)
  point.position = Point(x, y, z)
  point.velocity = Vector3(0.0, 0.0, 0.0)
  point.acceleration = Vector3(0.0, 0.0, 0.0)
  point.yaw = 0.0
  point.yaw_rate = 0.0
  return point


def make_hold_trajectory(trajectory_id, stamp, duration=30.0, x=0.0):
  trajectory = Trajectory()
  trajectory.header = Header(stamp=stamp, frame_id="map")
  trajectory.mode = Trajectory.MODE_NOMINAL
  trajectory.trajectory_id = trajectory_id
  trajectory.points = [make_point(0.0, x), make_point(duration, x)]
  return trajectory


class PreviewObserver:
  def __init__(self):
    self.samples = []
    self.transitions = []
    self.last_id = None
    self.nan_inf = 0
    self.sub = rospy.Subscriber("/uav/setpoint_preview", SetpointPreview,
                                self._callback, queue_size=200)

  def _callback(self, msg):
    stamp = time.time()
    self.samples.append((stamp, msg.trajectory_id, msg.started, msg.finished,
                         msg.point.position.x, msg.point.velocity.x,
                         msg.point.acceleration.x))
    if not finite_preview(msg):
      self.nan_inf += 1
    if msg.trajectory_id != self.last_id:
      self.transitions.append((stamp, msg.trajectory_id))
      self.last_id = msg.trajectory_id

  def clear(self):
    self.samples.clear()
    self.transitions.clear()
    self.last_id = None
    self.nan_inf = 0

  def ids(self):
    return [sample[1] for sample in self.samples]

  def saw_id(self, trajectory_id):
    return trajectory_id in self.ids()

  def transition_count_to(self, trajectory_id):
    return sum(1 for _, value in self.transitions if value == trajectory_id)


class StressRunner:
  def __init__(self, run_dir, rounds):
    self.run_dir = Path(run_dir)
    self.rounds = rounds
    self.processes = []
    self.results = {key: [] for key in "ABCDE"}
    self.delivery_latencies = []
    self.nan_inf_count = 0
    self.wrong_id_count = 0
    self.duplicate_switch_count = 0

  def start(self):
    self.run_dir.mkdir(parents=True, exist_ok=False)
    self.ros_home = self.run_dir / "ros_home"
    (self.ros_home / "log").mkdir(parents=True, exist_ok=True)
    os.environ["ROS_HOME"] = str(self.ros_home)
    os.environ["ROS_LOG_DIR"] = str(self.ros_home / "log")
    self.roscore = self.start_process("roscore", ["roscore"])
    wait_until(self.master_online, 10.0)
    if not self.master_online():
      raise RuntimeError("roscore did not start")
    rospy.init_node("dynamic_trajectory_delivery_stress", anonymous=True,
                    disable_signals=True)
    self.observer = PreviewObserver()
    self.publisher = rospy.Publisher("/uav/trajectory", Trajectory,
                                     queue_size=10, latch=True)
    wait_until(lambda: self.publisher.get_num_connections() > 0, 2.0)

  def stop(self):
    for process in reversed(self.processes):
      if process.poll() is None:
        process.terminate()
    for process in reversed(self.processes):
      try:
        process.wait(timeout=3.0)
      except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3.0)

  def start_process(self, name, command):
    log = open(self.run_dir / f"{name}.log", "ab")
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                               cwd="/home/tom/UAV_experiment",
                               preexec_fn=os.setsid)
    process._stress_log = log
    self.processes.append(process)
    return process

  def stop_process(self, process):
    if process is None:
      return
    if process.poll() is None:
      os.killpg(os.getpgid(process.pid), signal.SIGTERM)
      try:
        process.wait(timeout=3.0)
      except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=3.0)
    if process in self.processes:
      self.processes.remove(process)
    if getattr(process, "_stress_log", None):
      process._stress_log.close()

  @staticmethod
  def master_online():
    try:
      rosgraph.Master("/dynamic_trajectory_delivery_stress").getPid()
      return True
    except Exception:
      return False

  def start_preview(self, name):
    return self.start_process(name, [
        "rosrun", "uav_trajectory", "trajectory_preview_node",
        "_subscribe_uav_state:=false",
        "_trajectory_topic:=/uav/trajectory",
        "_preview_topic:=/uav/setpoint_preview",
        "_publish_rate:=30.0",
    ])

  def start_dynamic_publisher(self, name, start_x, timeout=2.0, line_length=0.2):
    return self.start_process(name, [
        "rosrun", "uav_trajectory", "dynamic_trajectory_publisher_node",
        "_publish_once:=true",
        f"_subscriber_wait_timeout_sec:={timeout}",
        "_publish_repeat_count:=3",
        "_publish_repeat_interval_sec:=0.05",
        "_post_publish_grace_sec:=0.20",
        "_trajectory_type:=line",
        "_start_delay_sec:=1.0",
        f"_start_x:={start_x:.3f}",
        "_start_y:=0.0",
        "_start_z:=0.0",
        "_start_yaw:=0.0",
        "_altitude_offset_m:=0.0",
        "_duration_sec:=15.0",
        "_sample_period_sec:=0.1",
        "_hold_end_sec:=5.0",
        "_initial_hold_sec:=0.0",
        "_initial_climb_duration_sec:=0.0",
        "_post_climb_hold_sec:=0.0",
        f"_line_length_m:={line_length:.3f}",
        "_line_segment_duration_sec:=1.5",
        "_max_velocity_mps:=1.0",
        "_max_acceleration_mps2:=1.5",
        "_max_jerk_mps3:=4.0",
    ])

  def run_publisher_round(self, scenario, index, preview_first, delay=0.0,
                          expect_success=True):
    preview = None
    self.observer.clear()
    start_time = time.time()
    if preview_first:
      preview = self.start_preview(f"{scenario}_{index}_preview")
      wait_until(lambda: self.publisher.get_num_connections() > 0, 2.0)
    scenario_offset = {"A": 0, "B": 20, "C": 40}.get(scenario, 0)
    line_length = 0.20 + 0.001 * (scenario_offset + index)
    publisher = self.start_dynamic_publisher(f"{scenario}_{index}_publisher",
                                             10.0 * ord(scenario) + index,
                                             line_length=line_length)
    if not preview_first and delay is not None:
      time.sleep(delay)
      preview = self.start_preview(f"{scenario}_{index}_preview")
    exit_code = publisher.wait(timeout=5.0)
    observed = wait_until(lambda: self.observer.samples[-1] if self.observer.samples else None,
                          2.0)
    self.stop_process(publisher)
    if preview is not None:
      self.stop_process(preview)
    success = (exit_code == 0 and observed is not None) if expect_success else exit_code != 0
    latency = (observed[0] - start_time) if observed else None
    if latency is not None:
      self.delivery_latencies.append(latency)
    self.nan_inf_count += self.observer.nan_inf
    result = {
        "round": index,
        "success": bool(success),
        "publisher_exit_code": int(exit_code),
        "delivery_latency_sec": latency,
        "preview_samples": len(self.observer.samples),
        "observed_trajectory_id": int(observed[1]) if observed else None,
        "line_length_m": line_length,
        "delay_sec": delay,
    }
    self.results[scenario].append(result)
    return success

  def run_duplicate_round(self, index):
    preview = self.start_preview(f"D_{index}_preview")
    self.observer.clear()
    trajectory_id = 50000 + index
    stamp = rospy.Time.now() + rospy.Duration.from_sec(0.5)
    msg = make_hold_trajectory(trajectory_id, stamp, duration=5.0, x=float(index))
    for _ in range(3):
      self.publisher.publish(msg)
      time.sleep(0.05)
    observed = wait_until(lambda: self.observer.saw_id(trajectory_id), 2.0)
    transition_count = self.observer.transition_count_to(trajectory_id)
    self.stop_process(preview)
    success = observed and transition_count == 1
    if transition_count > 1:
      self.duplicate_switch_count += transition_count - 1
    self.nan_inf_count += self.observer.nan_inf
    self.results["D"].append({
        "round": index,
        "success": bool(success),
        "trajectory_id": trajectory_id,
        "logical_switches": transition_count,
        "preview_samples": len(self.observer.samples),
    })
    return success

  def run_experiment_order_round(self, index):
    preview = self.start_preview(f"E_{index}_preview")
    self.observer.clear()
    ground_id = 60000 + index
    self.publisher.publish(make_hold_trajectory(
        ground_id, rospy.Time.now(), duration=60.0, x=0.0))
    if not wait_until(lambda: self.observer.saw_id(ground_id), 2.0):
      self.stop_process(preview)
      self.results["E"].append({"round": index, "success": False,
                                "reason": "ground_not_observed"})
      return False
    before = len(self.observer.transitions)
    publisher = self.start_dynamic_publisher(f"E_{index}_publisher", 900.0 + index,
                                             line_length=0.50 + 0.001 * index)
    exit_code = publisher.wait(timeout=5.0)
    success_id = wait_until(lambda: self.observer.last_id != ground_id, 3.0)
    after = len(self.observer.transitions)
    self.stop_process(publisher)
    self.stop_process(preview)
    switches = after - before
    success = exit_code == 0 and success_id and switches == 1
    if switches > 1:
      self.duplicate_switch_count += switches - 1
    self.nan_inf_count += self.observer.nan_inf
    self.results["E"].append({
        "round": index,
        "success": bool(success),
        "publisher_exit_code": int(exit_code),
        "ground_id": ground_id,
        "switches_after_ground": switches,
        "final_id": self.observer.last_id,
        "preview_samples": len(self.observer.samples),
    })
    return success

  def run(self):
    rng = random.Random(424242)
    for i in range(self.rounds["A"]):
      self.run_publisher_round("A", i, preview_first=True)
    for i in range(self.rounds["B"]):
      self.run_publisher_round("B", i, preview_first=False,
                               delay=rng.uniform(0.05, 1.50))
    for i in range(self.rounds["C"]):
      self.run_publisher_round("C", i, preview_first=False,
                               delay=None, expect_success=False)
    for i in range(self.rounds["D"]):
      self.run_duplicate_round(i)
    for i in range(self.rounds["E"]):
      self.run_experiment_order_round(i)

  def summarize(self):
    latencies = self.delivery_latencies
    residual = [
        p for p in self.processes
        if p.poll() is None and p is not getattr(self, "roscore", None)
    ]
    summary = {
        "rounds_total": sum(len(v) for v in self.results.values()),
        "scenarios": {},
        "wrong_trajectory_id_count": self.wrong_id_count,
        "unexpected_duplicate_switch_count": self.duplicate_switch_count,
        "nan_inf_count": self.nan_inf_count,
        "residual_ros_process_count": len(residual),
        "max_delivery_latency_sec": max(latencies) if latencies else None,
        "mean_delivery_latency_sec": statistics.mean(latencies) if latencies else None,
        "p95_delivery_latency_sec": (
            sorted(latencies)[int(math.ceil(0.95 * len(latencies))) - 1]
            if latencies else None),
    }
    for scenario, rows in self.results.items():
      successes = sum(1 for row in rows if row["success"])
      summary["scenarios"][scenario] = {
          "rounds": len(rows),
          "successes": successes,
          "failures": len(rows) - successes,
          "success_rate": successes / len(rows) if rows else 0.0,
          "round_details": rows,
      }
    for scenario in ("A", "B"):
      ids = [row.get("observed_trajectory_id")
             for row in self.results[scenario]
             if row.get("observed_trajectory_id") is not None]
      summary["scenarios"][scenario]["unique_trajectory_ids"] = len(set(ids))
      if len(set(ids)) != len(ids):
        self.wrong_id_count += len(ids) - len(set(ids))
        summary["wrong_trajectory_id_count"] = self.wrong_id_count
    return summary


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--run-dir", default=None)
  parser.add_argument("--quick", action="store_true",
                      help="run one round per scenario for CI smoke coverage")
  args = parser.parse_args()

  timestamp = time.strftime("%Y%m%d_%H%M%S")
  run_dir = args.run_dir or f"/tmp/uav_m0_c5b1/delivery_stress_{timestamp}"
  rounds = {key: 1 for key in DEFAULT_ROUNDS} if args.quick else dict(DEFAULT_ROUNDS)
  runner = StressRunner(run_dir, rounds)
  try:
    runner.start()
    runner.run()
    summary = runner.summarize()
    accepted = (
        summary["scenarios"]["A"]["successes"] == rounds["A"] and
        summary["scenarios"]["B"]["successes"] == rounds["B"] and
        summary["scenarios"]["C"]["successes"] == rounds["C"] and
        summary["scenarios"]["D"]["successes"] == rounds["D"] and
        summary["scenarios"]["E"]["successes"] == rounds["E"] and
        summary["wrong_trajectory_id_count"] == 0 and
        summary["unexpected_duplicate_switch_count"] == 0 and
        summary["nan_inf_count"] == 0 and
        summary["residual_ros_process_count"] == 0)
    summary["accepted"] = accepted
    with open(Path(run_dir) / "summary.json", "w", encoding="utf-8") as stream:
      json.dump(summary, stream, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if accepted else 1
  finally:
    runner.stop()


if __name__ == "__main__":
  sys.exit(main())
