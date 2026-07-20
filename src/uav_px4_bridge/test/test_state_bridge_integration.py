#!/usr/bin/env python3

import math
import unittest

import nav_msgs.msg
import rospy
import rostest
import uav_msgs.msg


class StateBridgeIntegrationTest(unittest.TestCase):
    def make_odom(self):
        odom = nav_msgs.msg.Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 1.25
        odom.pose.pose.position.y = -2.5
        odom.pose.pose.position.z = 3.75
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.5
        odom.twist.twist.linear.y = -0.25
        odom.twist.twist.linear.z = 0.125
        odom.twist.twist.angular.z = 0.75
        return odom

    def wait_for_state(self, predicate, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        latest = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                latest = rospy.wait_for_message("/test/uav/state", uav_msgs.msg.UavState, timeout=0.5)
            except rospy.ROSException:
                continue
            if predicate(latest):
                return latest
        self.fail("timed out waiting for matching /test/uav/state; latest=%r" % latest)

    def test_odometry_mapping_and_timeout(self):
        pub = rospy.Publisher("/test/mavros/local_position/odom", nav_msgs.msg.Odometry, queue_size=10)
        rospy.sleep(0.5)

        odom = self.make_odom()
        for _ in range(5):
            odom.header.stamp = rospy.Time.now()
            pub.publish(odom)
            rospy.sleep(0.05)

        state = self.wait_for_state(lambda msg: msg.pose_valid and msg.twist_valid)
        self.assertEqual(state.header.frame_id, "map")
        self.assertEqual(state.header.stamp, odom.header.stamp)
        self.assertAlmostEqual(state.pose.position.x, 1.25)
        self.assertAlmostEqual(state.pose.position.y, -2.5)
        self.assertAlmostEqual(state.pose.position.z, 3.75)
        self.assertAlmostEqual(state.twist.linear.x, 0.5)
        self.assertAlmostEqual(state.twist.linear.y, -0.25)
        self.assertAlmostEqual(state.twist.linear.z, 0.125)
        self.assertAlmostEqual(state.twist.angular.z, 0.75)
        self.assertFalse(state.acceleration_valid)
        self.assertTrue(math.isfinite(state.pose.position.x))

        timed_out = self.wait_for_state(lambda msg: not msg.pose_valid and not msg.twist_valid, timeout=3.0)
        self.assertEqual(timed_out.header.frame_id, "map")
        self.assertFalse(timed_out.acceleration_valid)


if __name__ == "__main__":
    rospy.init_node("test_state_bridge_integration")
    rostest.rosrun("uav_px4_bridge", "test_state_bridge_integration", StateBridgeIntegrationTest)
