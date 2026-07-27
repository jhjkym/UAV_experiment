#!/usr/bin/env python3
import math
import unittest

import rosgraph
import rospy
from uav_msgs.msg import SetpointPreview, Trajectory, TrajectoryPoint


def smooth5(u):
    u = max(0.0, min(1.0, u))
    return u ** 3 * (10.0 - 15.0 * u + 6.0 * u * u)


def dsmooth5(u):
    u = max(0.0, min(1.0, u))
    return 30.0 * u * u * (1.0 - u) * (1.0 - u)


def d2smooth5(u):
    u = max(0.0, min(1.0, u))
    return 60.0 * u * (1.0 - u) * (1.0 - 2.0 * u)


def make_point(t, x, y, z, vx, vy, vz, ax, ay, az, yaw):
    point = TrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(t)
    point.position.x = x
    point.position.y = y
    point.position.z = z
    point.velocity.x = vx
    point.velocity.y = vy
    point.velocity.z = vz
    point.acceleration.x = ax
    point.acceleration.y = ay
    point.acceleration.z = az
    point.yaw = yaw
    point.yaw_rate = 0.0
    return point


def make_line(trajectory_id):
    points = []
    waypoints = [(0.0, 0.0), (0.8, 0.0), (-0.8, 0.0), (0.0, 0.0)]
    t = 0.0
    duration = 1.2
    dt = 0.05
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        steps = int(math.ceil(duration / dt))
        for i in range(0 if not points else 1, steps + 1):
            local = duration * i / steps
            u = local / duration
            s = smooth5(u)
            ds = dsmooth5(u) / duration
            d2s = d2smooth5(u) / (duration * duration)
            points.append(make_point(t + local, start[0] + dx * s, start[1] + dy * s, 1.0,
                                     dx * ds, dy * ds, 0.0, dx * d2s, dy * d2s, 0.0, 0.0))
        t += duration
    return make_trajectory(trajectory_id, points)


def make_circle(trajectory_id):
    points = []
    radius = 0.8
    duration = 4.0
    dt = 0.05
    steps = int(math.ceil(duration / dt))
    for i in range(steps + 1):
        t = duration * i / steps
        u = t / duration
        theta = 2.0 * math.pi * smooth5(u)
        omega = 2.0 * math.pi * dsmooth5(u) / duration
        alpha = 2.0 * math.pi * d2smooth5(u) / (duration * duration)
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        vx = -radius * omega * math.sin(theta)
        vy = radius * omega * math.cos(theta)
        ax = -radius * (alpha * math.sin(theta) + omega * omega * math.cos(theta))
        ay = radius * (alpha * math.cos(theta) - omega * omega * math.sin(theta))
        yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 0.02 else 0.0
        points.append(make_point(t, x, y, 1.0, vx, vy, 0.0, ax, ay, 0.0, yaw))
    return make_trajectory(trajectory_id, points)


def make_figure8(trajectory_id):
    points = []
    amp_x = 0.8
    amp_y = 0.4
    duration = 4.0
    dt = 0.05
    steps = int(math.ceil(duration / dt))
    previous_yaw = 0.0
    for i in range(steps + 1):
        t = duration * i / steps
        u = t / duration
        theta = 2.0 * math.pi * smooth5(u)
        omega = 2.0 * math.pi * dsmooth5(u) / duration
        alpha = 2.0 * math.pi * d2smooth5(u) / (duration * duration)
        x = amp_x * math.sin(theta)
        y = amp_y * math.sin(2.0 * theta)
        vx = amp_x * omega * math.cos(theta)
        vy = 2.0 * amp_y * omega * math.cos(2.0 * theta)
        ax = amp_x * (alpha * math.cos(theta) - omega * omega * math.sin(theta))
        ay = 2.0 * amp_y * (alpha * math.cos(2.0 * theta)
                             - 2.0 * omega * omega * math.sin(2.0 * theta))
        if math.hypot(vx, vy) > 0.02:
            raw = math.atan2(vy, vx)
            yaw = previous_yaw + math.atan2(math.sin(raw - previous_yaw),
                                            math.cos(raw - previous_yaw))
            previous_yaw = yaw
        else:
            yaw = previous_yaw
        points.append(make_point(t, x, y, 1.0, vx, vy, 0.0, ax, ay, 0.0, yaw))
    return make_trajectory(trajectory_id, points)


def make_trajectory(trajectory_id, points):
    trajectory = Trajectory()
    trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.4)
    trajectory.header.frame_id = "map"
    trajectory.mode = Trajectory.MODE_NOMINAL
    trajectory.trajectory_id = trajectory_id
    trajectory.points = points
    return trajectory


class DynamicTrajectoryPreviewIntegration(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_dynamic_trajectory_preview_integration", anonymous=True)
        self.previews = []
        self.sub = rospy.Subscriber("/test/uav/setpoint_preview", SetpointPreview,
                                    self.previews.append)
        self.pub = rospy.Publisher("/test/uav/trajectory", Trajectory, queue_size=1, latch=True)

    def wait_for_valid(self, trajectory_id, timeout=10.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(60)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            valid = [msg for msg in self.previews
                     if msg.trajectory_id == trajectory_id and msg.trajectory_valid]
            moving = [msg for msg in valid if msg.started and not msg.finished]
            if len(valid) >= 30 and any(not msg.started for msg in valid) \
                    and len(moving) >= 25:
                return valid
            rate.sleep()
        self.fail("timed out waiting for valid dynamic previews")

    def exercise(self, trajectory):
        del self.previews[:]
        trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.4)
        self.pub.publish(trajectory)
        valid = self.wait_for_valid(trajectory.trajectory_id)
        moving = [msg for msg in valid if msg.started and not msg.finished]
        self.assertGreater(len(moving), 20)

        times = [msg.header.stamp.to_sec() for msg in moving]
        if len(times) > 2:
            rate = (len(times) - 1) / (times[-1] - times[0])
            self.assertGreater(rate, 20.0)
            self.assertLess(rate, 40.0)

        for before, after in zip(moving[:-1], moving[1:]):
            dt = after.header.stamp.to_sec() - before.header.stamp.to_sec()
            if dt <= 0.0:
                continue
            fd_vx = (after.point.position.x - before.point.position.x) / dt
            fd_vy = (after.point.position.y - before.point.position.y) / dt
            avg_vx = 0.5 * (before.point.velocity.x + after.point.velocity.x)
            avg_vy = 0.5 * (before.point.velocity.y + after.point.velocity.y)
            self.assertLess(abs(fd_vx - avg_vx), 0.35)
            self.assertLess(abs(fd_vy - avg_vy), 0.35)
            fd_ax = (after.point.velocity.x - before.point.velocity.x) / dt
            fd_ay = (after.point.velocity.y - before.point.velocity.y) / dt
            avg_ax = 0.5 * (before.point.acceleration.x + after.point.acceleration.x)
            avg_ay = 0.5 * (before.point.acceleration.y + after.point.acceleration.y)
            self.assertLess(abs(fd_ax - avg_ax), 1.2)
            self.assertLess(abs(fd_ay - avg_ay), 1.2)
            dyaw = math.atan2(math.sin(after.point.yaw - before.point.yaw),
                              math.cos(after.point.yaw - before.point.yaw))
            self.assertLess(abs(dyaw), math.pi / 2.0)

        for msg in valid:
            values = [
                msg.point.position.x, msg.point.position.y, msg.point.position.z,
                msg.point.velocity.x, msg.point.velocity.y, msg.point.velocity.z,
                msg.point.acceleration.x, msg.point.acceleration.y, msg.point.acceleration.z,
                msg.point.yaw, msg.point.yaw_rate,
            ]
            self.assertTrue(all(math.isfinite(value) for value in values))
            qz = math.sin(0.5 * msg.point.yaw)
            qw = math.cos(0.5 * msg.point.yaw)
            self.assertAlmostEqual(qz * qz + qw * qw, 1.0, places=12)

    def test_three_dynamic_trajectories_and_no_mavros_publishers(self):
        for trajectory in (make_line(101), make_circle(102), make_figure8(103)):
            self.exercise(trajectory)

        master = rosgraph.Master("/test_dynamic_trajectory_preview_integration")
        publishers, _, _ = master.getSystemState()
        publisher_map = dict(publishers)
        for topic in (
            "/mavros/setpoint_raw/local",
            "/mavros/setpoint_position/local",
            "/mavros/setpoint_raw/attitude",
            "/mavros/setpoint_velocity/cmd_vel",
        ):
            self.assertEqual(publisher_map.get(topic, []), [])


if __name__ == "__main__":
    import rostest

    rostest.rosrun("uav_trajectory", "test_dynamic_trajectory_preview_integration",
                   DynamicTrajectoryPreviewIntegration)
