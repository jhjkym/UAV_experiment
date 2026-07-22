#!/usr/bin/env python3
import math
import unittest

import rospy
import rosgraph
from uav_msgs.msg import SetpointPreview, Trajectory, TrajectoryPoint, UavState


def make_point(t, x, vx):
    point = TrajectoryPoint()
    point.time_from_start = rospy.Duration.from_sec(t)
    point.position.x = x
    point.velocity.x = vx
    point.yaw = 0.0
    point.yaw_rate = 0.0
    return point


class TrajectoryPreviewIntegration(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_trajectory_preview_integration", anonymous=True)
        self.previews = []
        self.preview_sub = rospy.Subscriber(
            "/test/uav/setpoint_preview", SetpointPreview, self.previews.append
        )
        self.trajectory_pub = rospy.Publisher(
            "/test/uav/trajectory", Trajectory, queue_size=1, latch=True
        )
        self.state_pub = rospy.Publisher("/test/uav/state", UavState, queue_size=1)

    def publish_state(self):
        state = UavState()
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = "map"
        state.pose_valid = True
        state.twist_valid = True
        state.acceleration_valid = False
        self.state_pub.publish(state)

    def publish_trajectory(self):
        trajectory = Trajectory()
        trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.8)
        trajectory.header.frame_id = "map"
        trajectory.mode = Trajectory.MODE_NOMINAL
        trajectory.trajectory_id = 7
        trajectory.points = [make_point(0.0, 0.0, 0.0), make_point(1.0, 1.0, 0.0)]
        self.trajectory_pub.publish(trajectory)
        return trajectory

    def wait_for_previews(self, count, timeout=6.0):
        deadline = rospy.Time.now() + rospy.Duration.from_sec(timeout)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.publish_state()
            if len(self.previews) >= count:
                return
            rate.sleep()
        self.fail("timed out waiting for previews")

    def test_preview_publication_and_control_boundary(self):
        trajectory = self.publish_trajectory()

        deadline = rospy.Time.now() + rospy.Duration(4.0)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.publish_state()
            valid = [msg for msg in self.previews if msg.trajectory_valid]
            has_before = any(not msg.started and not msg.finished for msg in valid)
            has_middle = any(msg.started and not msg.finished for msg in valid)
            xs = [msg.point.position.x for msg in valid if msg.started and not msg.finished]
            has_motion = len(xs) >= 2 and (max(xs) - min(xs)) > 0.05
            if has_before and has_middle and has_motion and len(valid) >= 10:
                break
            rate.sleep()
        else:
            self.fail("timed out waiting for before-start and moving previews")

        valid = [msg for msg in self.previews if msg.trajectory_valid]
        xs = [msg.point.position.x for msg in valid if msg.started and not msg.finished]
        self.assertGreater(max(xs) - min(xs), 0.05)

        while rospy.Time.now() < trajectory.header.stamp + rospy.Duration(1.3):
            self.publish_state()
            rate.sleep()
        self.publish_state()
        self.wait_for_previews(len(self.previews) + 2)
        last = self.previews[-1]
        self.assertTrue(last.finished)
        self.assertAlmostEqual(last.point.position.x, 1.0, places=6)
        self.assertTrue(last.state_fresh)

        for topic in (
            "/mavros/setpoint_raw/local",
            "/mavros/setpoint_position/local",
            "/mavros/setpoint_raw/attitude",
            "/mavros/setpoint_velocity/cmd_vel",
        ):
            master = rosgraph.Master("/test_trajectory_preview_integration")
            publishers, _, _ = master.getSystemState()
            topic_publishers = dict(publishers).get(topic, [])
            self.assertNotIn("/trajectory_preview_node", topic_publishers)

        for msg in valid:
            values = [
                msg.point.position.x,
                msg.point.position.y,
                msg.point.position.z,
                msg.point.velocity.x,
                msg.point.velocity.y,
                msg.point.velocity.z,
                msg.point.acceleration.x,
                msg.point.acceleration.y,
                msg.point.acceleration.z,
                msg.point.yaw,
                msg.point.yaw_rate,
            ]
            self.assertTrue(all(math.isfinite(value) for value in values))


if __name__ == "__main__":
    import rostest

    rostest.rosrun("uav_trajectory", "test_trajectory_preview_integration",
                   TrajectoryPreviewIntegration)
