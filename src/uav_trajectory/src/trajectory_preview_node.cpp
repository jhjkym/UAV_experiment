#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <uav_msgs/SetpointPreview.h>
#include <uav_msgs/Trajectory.h>
#include <uav_msgs/UavState.h>

#include "uav_trajectory/trajectory_sampler.hpp"

namespace {

class TrajectoryPreviewNode {
 public:
  TrajectoryPreviewNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param<std::string>("trajectory_topic", trajectory_topic_, "/uav/trajectory");
    private_nh_.param<std::string>("preview_topic", preview_topic_, "/uav/setpoint_preview");
    private_nh_.param<std::string>("uav_state_topic", uav_state_topic_, "/uav/state");
    private_nh_.param<bool>("subscribe_uav_state", subscribe_uav_state_, true);

    double publish_rate = 30.0;
    private_nh_.param<double>("publish_rate", publish_rate, 30.0);
    if (!std::isfinite(publish_rate) || publish_rate < 1.0 || publish_rate > 100.0) {
      ROS_WARN("Invalid publish_rate %.3f; using 30.0 Hz", publish_rate);
      publish_rate = 30.0;
    }

    double state_timeout = 0.5;
    private_nh_.param<double>("state_timeout", state_timeout, 0.5);
    if (!std::isfinite(state_timeout)) {
      ROS_WARN("Invalid state_timeout %.3f; using 0.5 s", state_timeout);
      state_timeout = 0.5;
    }
    state_timeout_ = ros::Duration(state_timeout);

    if (!private_nh_.getParam("supported_frames", supported_frames_)) {
      supported_frames_ = {"map"};
    }
    if (supported_frames_.empty()) {
      ROS_WARN("supported_frames is empty; using map");
      supported_frames_.push_back("map");
    }

    trajectory_sub_ = nh_.subscribe(trajectory_topic_, 1,
                                   &TrajectoryPreviewNode::trajectoryCallback, this);
    if (subscribe_uav_state_) {
      state_sub_ =
          nh_.subscribe(uav_state_topic_, 1, &TrajectoryPreviewNode::stateCallback, this);
    }
    preview_pub_ = nh_.advertise<uav_msgs::SetpointPreview>(preview_topic_, 1);
    timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate),
                             &TrajectoryPreviewNode::timerCallback, this);

    ROS_INFO("trajectory_preview_node started trajectory_topic=%s preview_topic=%s "
             "publish_rate=%.3f supported_frames=%zu subscribe_uav_state=%d",
             trajectory_topic_.c_str(), preview_topic_.c_str(), publish_rate,
             supported_frames_.size(), subscribe_uav_state_);
  }

 private:
  void trajectoryCallback(const uav_msgs::TrajectoryConstPtr& msg) {
    std::string reason;
    if (cache_.replaceIfValid(*msg, supported_frames_, &reason)) {
      last_sample_time_ = ros::Time(0);
      ROS_INFO("Accepted trajectory id=%u frame=%s points=%zu start=%.9f",
               msg->trajectory_id, msg->header.frame_id.c_str(), msg->points.size(),
               msg->header.stamp.toSec());
    } else {
      ROS_WARN_THROTTLE(1.0, "Rejected trajectory id=%u: %s",
                        msg->trajectory_id, reason.c_str());
    }
  }

  void stateCallback(const uav_msgs::UavStateConstPtr& msg) {
    last_state_ = *msg;
    have_state_ = true;
  }

  void timerCallback(const ros::TimerEvent& event) {
    uav_msgs::Trajectory trajectory;
    if (!cache_.get(&trajectory)) {
      ROS_WARN_THROTTLE(2.0, "No valid trajectory cached; preview is not published");
      return;
    }

    const ros::Time now = event.current_real;
    const auto sample = uav_trajectory::sampleTrajectory(trajectory, now, last_sample_time_);
    last_sample_time_ = now;
    if (sample.time_went_back) {
      ROS_WARN_THROTTLE(1.0, "ROS time moved backward; holding first trajectory point");
    }

    bool state_fresh = true;
    if (subscribe_uav_state_) {
      state_fresh = have_state_ &&
                    uav_trajectory::isStateFresh(last_state_, now, state_timeout_);
      if (!state_fresh) {
        ROS_WARN_THROTTLE(1.0, "UAV state is stale or invalid; preview remains read-only");
      }
    }

    const auto preview =
        uav_trajectory::makePreviewMessage(trajectory, sample, state_fresh);
    preview_pub_.publish(preview);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber state_sub_;
  ros::Publisher preview_pub_;
  ros::Timer timer_;

  std::string trajectory_topic_;
  std::string preview_topic_;
  std::string uav_state_topic_;
  bool subscribe_uav_state_ = true;
  bool have_state_ = false;
  ros::Duration state_timeout_{0.5};
  ros::Time last_sample_time_;
  std::vector<std::string> supported_frames_;
  uav_msgs::UavState last_state_;
  uav_trajectory::TrajectoryCache cache_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_preview_node");
  TrajectoryPreviewNode node(ros::NodeHandle{}, ros::NodeHandle{"~"});
  ros::spin();
  return 0;
}
