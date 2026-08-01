#!/usr/bin/env python3
import unittest

import rosgraph
import rospy
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.msg import PositionTarget
from uav_msgs.msg import OffboardStatus, SetpointPreview, UavState


def make_preview():
    msg = SetpointPreview()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.trajectory_valid = True
    msg.started = True
    msg.finished = False
    msg.state_fresh = True
    msg.trajectory_id = 3
    msg.point.position.x = 1.0
    msg.point.velocity.x = 0.1
    msg.point.acceleration.x = 0.01
    msg.point.yaw = 0.2
    msg.point.yaw_rate = 0.3
    return msg


def make_state():
    msg = UavState()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose_valid = True
    msg.twist_valid = True
    return msg


def make_mavros_state(connected=True):
    msg = MavrosState()
    msg.connected = connected
    msg.armed = False
    msg.mode = "AUTO.LOITER"
    return msg


class OffboardAdapterDryRunTest(unittest.TestCase):
    def setUp(self):
        rospy.init_node("test_offboard_adapter_dry_run", anonymous=True)
        self.targets = []
        self.statuses = []
        self.preview_pub = rospy.Publisher("/test/uav/setpoint_preview", SetpointPreview, queue_size=1)
        self.state_pub = rospy.Publisher("/test/uav/state", UavState, queue_size=1)
        self.mavros_pub = rospy.Publisher("/test/mavros/state", MavrosState, queue_size=1)
        self.target_sub = rospy.Subscriber(
            "/test/uav/mavros_target_preview", PositionTarget, self.targets.append
        )
        self.status_sub = rospy.Subscriber(
            "/test/uav/offboard_status", OffboardStatus, self.statuses.append
        )

    def publish_inputs(self):
        self.preview_pub.publish(make_preview())
        self.state_pub.publish(make_state())
        self.mavros_pub.publish(make_mavros_state(True))

    def wait_for_outputs(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(30)
        while rospy.Time.now() < deadline and not rospy.is_shutdown():
            self.publish_inputs()
            if self.targets and self.statuses and self.statuses[-1].state_name == "READY_DRY_RUN":
                return
            rate.sleep()
        self.fail("timed out waiting for dry-run outputs")

    def publishers_for(self, topic):
        master = rosgraph.Master("/test_offboard_adapter_dry_run")
        publishers, _, _ = master.getSystemState()
        return dict(publishers).get(topic, [])

    def test_dry_run_outputs_and_no_real_mavros_publisher(self):
        self.wait_for_outputs()
        self.assertEqual(self.targets[-1].coordinate_frame, PositionTarget.FRAME_LOCAL_NED)
        self.assertEqual(self.targets[-1].type_mask, 0)
        self.assertEqual(self.statuses[-1].state_name, "READY_DRY_RUN")
        self.assertFalse(self.statuses[-1].static_gate_allowed)
        self.assertFalse(self.statuses[-1].runtime_gate_enabled)
        self.assertFalse(self.statuses[-1].output_active)
        self.assertNotIn("/offboard_adapter_node", self.publishers_for("/mavros/setpoint_raw/local"))
        service_names = [name for name, _ in rosgraph.Master("/test_offboard_adapter_dry_run").getSystemState()[2]]
        self.assertNotIn("/mavros/cmd/arming", service_names)
        self.assertNotIn("/mavros/set_mode", service_names)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("uav_offboard", "test_offboard_adapter_dry_run",
                   OffboardAdapterDryRunTest)
