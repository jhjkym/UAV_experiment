#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <uav_msgs/UavState.h>

#include "uav_px4_bridge/state_bridge.hpp"

namespace {

class MavrosStateBridgeNode {
 public:
  MavrosStateBridgeNode() : private_nh_("~") {
    private_nh_.param<std::string>("state_topic", state_topic_, "/mavros/state");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "/mavros/local_position/odom");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/mavros/imu/data");
    private_nh_.param<std::string>("uav_state_topic", uav_state_topic_, "/uav/state");
    private_nh_.param("subscribe_imu", subscribe_imu_, false);
    private_nh_.param("odom_timeout", odom_timeout_s_, 0.5);
    private_nh_.param("publish_rate", publish_rate_hz_, 30.0);

    state_sub_ = nh_.subscribe(state_topic_, 10, &MavrosStateBridgeNode::stateCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 10, &MavrosStateBridgeNode::odomCallback, this);
    if (subscribe_imu_) {
      imu_sub_ = nh_.subscribe(imu_topic_, 10, &MavrosStateBridgeNode::imuCallback, this);
    }

    state_pub_ = nh_.advertise<uav_msgs::UavState>(uav_state_topic_, 10);
    diagnostics_timer_ = nh_.createTimer(ros::Duration(1.0), &MavrosStateBridgeNode::diagnosticsTimer, this);
    publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_hz_), &MavrosStateBridgeNode::publishTimer, this);

    ROS_INFO_STREAM("mavros_state_bridge_node started"
                    << " state_topic=" << state_topic_
                    << " odom_topic=" << odom_topic_
                    << " uav_state_topic=" << uav_state_topic_
                    << " odom_timeout=" << odom_timeout_s_);
  }

 private:
  void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    last_mavros_state_ = *msg;
    have_mavros_state_ = true;
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!uav_px4_bridge::isFinite(*msg)) {
      ROS_WARN_THROTTLE(1.0, "Dropping non-finite odometry sample from %s", odom_topic_.c_str());
      return;
    }
    last_odom_ = *msg;
    last_odom_receive_time_ = ros::Time::now();
    have_odom_ = true;
    ++odom_count_;
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    last_imu_stamp_ = msg->header.stamp;
    have_imu_ = true;
  }

  void publishTimer(const ros::TimerEvent&) {
    if (!have_odom_) {
      ROS_WARN_THROTTLE(2.0, "No odometry received from %s; /uav/state is not published yet", odom_topic_.c_str());
      return;
    }

    const bool timed_out = uav_px4_bridge::isTimedOut(last_odom_receive_time_,
                                                     ros::Time::now(),
                                                     ros::Duration(odom_timeout_s_));
    const bool valid = !timed_out;
    uav_msgs::UavState state = uav_px4_bridge::odometryToUavState(last_odom_, valid, valid);
    state_pub_.publish(state);

    if (timed_out) {
      ROS_WARN_THROTTLE(1.0, "Odometry timeout: latest sample age %.3f s exceeds %.3f s",
                        (ros::Time::now() - last_odom_receive_time_).toSec(),
                        odom_timeout_s_);
    }
  }

  void diagnosticsTimer(const ros::TimerEvent&) {
    const ros::Time now = ros::Time::now();
    const double elapsed = std::max(1e-6, (now - last_diagnostics_time_).toSec());
    const double odom_hz = static_cast<double>(odom_count_ - last_odom_count_) / elapsed;
    last_diagnostics_time_ = now;
    last_odom_count_ = odom_count_;

    const double age = have_odom_ ? (now - last_odom_receive_time_).toSec() : -1.0;
    const bool timed_out = !have_odom_ || uav_px4_bridge::isTimedOut(last_odom_receive_time_, now, ros::Duration(odom_timeout_s_));
    const std::string frame_id = have_odom_ ? last_odom_.header.frame_id : "";

    ROS_INFO_STREAM_THROTTLE(1.0,
        "MAVROS state: connected=" << (have_mavros_state_ && last_mavros_state_.connected)
        << " armed=" << (have_mavros_state_ && last_mavros_state_.armed)
        << " mode=" << (have_mavros_state_ ? last_mavros_state_.mode : "unknown")
        << " system_status=" << (have_mavros_state_ ? static_cast<int>(last_mavros_state_.system_status) : -1)
        << " odom_hz=" << odom_hz
        << " odom_age=" << age
        << " pose_valid=" << (!timed_out)
        << " twist_valid=" << (!timed_out)
        << " frame_id=" << frame_id
        << " imu_seen=" << have_imu_);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber state_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher state_pub_;
  ros::Timer publish_timer_;
  ros::Timer diagnostics_timer_;

  std::string state_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string uav_state_topic_;
  bool subscribe_imu_{false};
  double odom_timeout_s_{0.5};
  double publish_rate_hz_{30.0};

  mavros_msgs::State last_mavros_state_;
  nav_msgs::Odometry last_odom_;
  ros::Time last_odom_receive_time_;
  ros::Time last_imu_stamp_;
  ros::Time last_diagnostics_time_{ros::Time::now()};
  bool have_mavros_state_{false};
  bool have_odom_{false};
  bool have_imu_{false};
  uint64_t odom_count_{0};
  uint64_t last_odom_count_{0};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mavros_state_bridge_node");
  MavrosStateBridgeNode node;
  ros::spin();
  return 0;
}
