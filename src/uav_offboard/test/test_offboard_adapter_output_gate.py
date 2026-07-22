#!/usr/bin/env python3
import unittest

import rospy
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.msg import PositionTarget
from std_srvs.srv import SetBool
from uav_msgs.msg import OffboardStatus, SetpointPreview, UavState


def make_preview(valid=True, finished=False):
    msg = SetpointPreview()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.trajectory_valid = valid
    msg.started = True
    msg.finished = finished
    msg.state_fresh = True
    msg.trajectory_id = 9
    msg.point.position.x = 2.0
    msg.point.position.y = 3.0
    msg.point.position.z = 4.0
    msg.point.velocity.x = 0.2
    msg.point.acceleration.z = 0.04
    msg.point.yaw = 0.5
    msg.point.yaw_rate = 0.6
    return msg


def make_state(pose_valid=True, twist_valid=True):
    msg = UavState()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose_valid = pose_valid
    msg.twist_valid = twist_valid
    return msg


def make_mavros_state(connected=True):
    msg = MavrosState()
    msg.connected = connected
    msg.armed = False
    msg.mode = "AUTO.LOITER"
    return msg


class OffboardAdapterOutputGateTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_offboard_adapter_output_gate", anonymous=True)
        self.outputs = []
        self.statuses = []
        self.preview_pub = rospy.Publisher("/test/uav/setpoint_preview", SetpointPreview, queue_size=1)
        self.state_pub = rospy.Publisher("/test/uav/state", UavState, queue_size=1)
        self.mavros_pub = rospy.Publisher("/test/mavros/state", MavrosState, queue_size=1)
        self.output_sub = rospy.Subscriber(
            "/test/mavros/setpoint_raw/local", PositionTarget, self.outputs.append
        )
        self.status_sub = rospy.Subscriber(
            "/test/uav/offboard_status", OffboardStatus, self.statuses.append
        )
        rospy.wait_for_service("/offboard_adapter_node/set_output_enabled", timeout=5.0)
        self.set_enabled = rospy.ServiceProxy(
            "/offboard_adapter_node/set_output_enabled", SetBool
        )

    def publish_inputs(self, preview=None, state=None, mavros=None):
        self.preview_pub.publish(preview if preview is not None else make_preview())
        self.state_pub.publish(state if state is not None else make_state())
        self.mavros_pub.publish(mavros if mavros is not None else make_mavros_state(True))

    def wait_until(self, condition, timeout=5.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(30)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            if condition():
                return
            rate.sleep()
        self.fail("timed out waiting for condition")

    def test_output_gate_and_fault_stops(self):
        rate = rospy.Rate(30)
        for _ in range(10):
            self.publish_inputs()
            rate.sleep()

        response = self.set_enabled(True)
        self.assertTrue(response.success, response.message)
        start_count = len(self.outputs)
        self.wait_until(lambda: len(self.outputs) > start_count)
        self.assertTrue(any(status.state_name == "STREAMING" for status in self.statuses))
        self.assertAlmostEqual(self.outputs[-1].position.x, 2.0)
        self.assertEqual(self.outputs[-1].type_mask, 0)

        response = self.set_enabled(False)
        self.assertTrue(response.success)
        stopped_count = len(self.outputs)
        rospy.sleep(0.3)
        self.assertEqual(len(self.outputs), stopped_count)

        for _ in range(10):
            self.publish_inputs()
            rate.sleep()
        self.assertTrue(self.set_enabled(True).success)
        rospy.sleep(0.7)
        self.wait_until(lambda: self.statuses and self.statuses[-1].state_name in ("FAULT", "DISABLED"))
        fault_count = len(self.outputs)
        rospy.sleep(0.2)
        self.assertEqual(len(self.outputs), fault_count)

        for _ in range(10):
            self.publish_inputs(state=make_state(pose_valid=False))
            rate.sleep()
        self.assertFalse(self.set_enabled(True).success)

        for _ in range(10):
            self.publish_inputs(mavros=make_mavros_state(False))
            rate.sleep()
        self.assertFalse(self.set_enabled(True).success)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("uav_offboard", "test_offboard_adapter_output_gate",
                   OffboardAdapterOutputGateTest)
