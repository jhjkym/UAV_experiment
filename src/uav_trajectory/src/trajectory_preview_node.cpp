#include <algorithm>
#include <cstdint>
#include <cmath>
#include <iomanip>
#include <sstream>
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
  static double norm3(const geometry_msgs::Vector3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
  }

  static double positionDelta(const geometry_msgs::Point& lhs,
                              const geometry_msgs::Point& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  static geometry_msgs::Vector3 vectorDelta(const geometry_msgs::Vector3& lhs,
                                            const geometry_msgs::Vector3& rhs) {
    geometry_msgs::Vector3 result;
    result.x = lhs.x - rhs.x;
    result.y = lhs.y - rhs.y;
    result.z = lhs.z - rhs.z;
    return result;
  }

  void trajectoryCallback(const uav_msgs::TrajectoryConstPtr& msg) {
    std::string reason;
    bool queued_pending = false;
    if (cache_.queueOrReplaceIfValid(*msg, supported_frames_, ros::Time::now(), &reason,
                                     &queued_pending)) {
      if (!queued_pending) {
        last_sample_time_ = ros::Time(0);
        pending_log_.clear();
      } else {
        uav_msgs::Trajectory active;
        const std::uint32_t active_id = cache_.get(&active) ? active.trajectory_id : 0u;
        std::ostringstream pending_log;
        pending_log << "Queued pending trajectory active_id=" << active_id
                    << " pending_id=" << msg->trajectory_id
                    << " planned_switch=" << std::fixed << std::setprecision(9)
                    << msg->header.stamp.toSec();
        pending_log_ = pending_log.str();
      }
      if (queued_pending) {
        ROS_INFO("%s frame=%s points=%zu", pending_log_.c_str(),
                 msg->header.frame_id.c_str(), msg->points.size());
      } else {
        ROS_INFO("Accepted trajectory id=%u frame=%s points=%zu start=%.9f",
                 msg->trajectory_id, msg->header.frame_id.c_str(), msg->points.size(),
                 msg->header.stamp.toSec());
      }
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
    const ros::Time now = event.current_real;
    bool promoted_pending = false;
    if (!cache_.getActiveForTime(now, &trajectory, &promoted_pending)) {
      ROS_WARN_THROTTLE(2.0, "No valid trajectory cached; preview is not published");
      return;
    }
    if (promoted_pending) {
      last_sample_time_ = ros::Time(0);
    }
    const auto sample = uav_trajectory::sampleTrajectory(trajectory, now, last_sample_time_);
    last_sample_time_ = now;
    if (sample.time_went_back) {
      ROS_WARN_THROTTLE(1.0, "ROS time moved backward; holding first trajectory point");
    }
    if (promoted_pending) {
      const double position_jump =
          have_last_preview_ ? positionDelta(sample.point.position, last_preview_.point.position) : 0.0;
      const double velocity_jump =
          have_last_preview_ ? norm3(vectorDelta(sample.point.velocity, last_preview_.point.velocity)) : 0.0;
      const double acceleration_jump =
          have_last_preview_ ? norm3(vectorDelta(sample.point.acceleration,
                                                last_preview_.point.acceleration)) : 0.0;
      ROS_INFO("Promoted pending trajectory id=%u planned_switch=%.9f actual_switch=%.9f "
               "position_jump=%.6f velocity_jump=%.6f acceleration_jump=%.6f",
               trajectory.trajectory_id, trajectory.header.stamp.toSec(), now.toSec(),
               position_jump, velocity_jump, acceleration_jump);
      pending_log_.clear();
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
    last_preview_ = preview;
    have_last_preview_ = true;
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
  bool have_last_preview_ = false;
  uav_msgs::SetpointPreview last_preview_;
  std::string pending_log_;
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
