#include <cstdint>
#include <cmath>
#include <stdexcept>
#include <string>

#include <ros/ros.h>
#include <uav_msgs/Trajectory.h>

#include "uav_trajectory/dynamic_trajectory_generator.hpp"

namespace {

class DynamicTrajectoryPublisherNode {
 public:
  DynamicTrajectoryPublisherNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh) {
    private_nh_.param<std::string>("trajectory_topic", trajectory_topic_, "/uav/trajectory");
    private_nh_.param<bool>("publish_once", publish_once_, true);
    private_nh_.param<double>("republish_rate_hz", republish_rate_hz_, 0.2);
    private_nh_.param<double>("subscriber_wait_timeout_sec",
                              subscriber_wait_timeout_sec_, 2.0);
    private_nh_.param<double>("publish_once_subscriber_wait_sec",
                              subscriber_wait_timeout_sec_, subscriber_wait_timeout_sec_);
    private_nh_.param<int>("publish_repeat_count", publish_repeat_count_, 3);
    private_nh_.param<double>("publish_repeat_interval_sec",
                              publish_repeat_interval_sec_, 0.05);
    private_nh_.param<double>("post_publish_grace_sec", post_publish_grace_sec_, 0.20);

    uav_trajectory::DynamicTrajectoryConfig config;
    std::string trajectory_type = "line";
    std::string yaw_mode = "fixed";
    private_nh_.param<std::string>("trajectory_type", trajectory_type, "line");
    private_nh_.param<std::string>("frame_id", config.frame_id, "map");
    private_nh_.param<double>("start_delay_sec", config.start_delay_sec, 2.0);
    start_delay_sec_ = config.start_delay_sec;
    private_nh_.param<double>("altitude_offset_m", config.altitude_offset_m, 1.0);
    private_nh_.param<double>("duration_sec", config.duration_sec, 20.0);
    private_nh_.param<double>("sample_period_sec", config.sample_period_sec, 0.05);
    private_nh_.param<double>("hold_end_sec", config.hold_end_sec, 5.0);
    private_nh_.param<double>("initial_hold_sec", config.initial_hold_sec, 0.0);
    private_nh_.param<double>("initial_climb_duration_sec",
                              config.initial_climb_duration_sec, 0.0);
    private_nh_.param<double>("post_climb_hold_sec", config.post_climb_hold_sec, 0.0);
    private_nh_.param<std::string>("yaw_mode", yaw_mode, "fixed");
    private_nh_.param<double>("line_length_m", config.line_length_m, 1.0);
    private_nh_.param<double>("line_segment_duration_sec",
                              config.line_segment_duration_sec, 5.0);
    private_nh_.param<double>("circle_radius_m", config.circle_radius_m, 1.0);
    private_nh_.param<double>("circle_tangent_speed_mps",
                              config.circle_tangent_speed_mps, 0.5);
    private_nh_.param<double>("transition_duration_sec",
                              config.transition_duration_sec, 2.0);
    private_nh_.param<double>("figure8_amplitude_x_m",
                              config.figure8_amplitude_x_m, 1.0);
    private_nh_.param<double>("figure8_amplitude_y_m",
                              config.figure8_amplitude_y_m, 0.5);
    private_nh_.param<double>("max_velocity_mps", config.max_velocity_mps, 1.0);
    private_nh_.param<double>("max_acceleration_mps2",
                              config.max_acceleration_mps2, 1.5);
    private_nh_.param<double>("max_jerk_mps3", config.max_jerk_mps3, 4.0);
    private_nh_.param<double>("low_speed_yaw_threshold_mps",
                              config.low_speed_yaw_threshold_mps, 0.05);

    uav_trajectory::StartPose start;
    private_nh_.param<double>("start_x", start.x, 0.0);
    private_nh_.param<double>("start_y", start.y, 0.0);
    private_nh_.param<double>("start_z", start.z, 0.0);
    private_nh_.param<double>("start_yaw", start.yaw, 0.0);

    if (!uav_trajectory::parseTrajectoryType(trajectory_type, &config.trajectory_type)) {
      ROS_FATAL("Invalid trajectory_type '%s'", trajectory_type.c_str());
      throw std::runtime_error("invalid trajectory_type");
    }
    if (!uav_trajectory::parseYawMode(yaw_mode, &config.yaw_mode)) {
      ROS_FATAL("Invalid yaw_mode '%s'", yaw_mode.c_str());
      throw std::runtime_error("invalid yaw_mode");
    }
    validatePublisherParameters();

    const auto result = uav_trajectory::generateDynamicTrajectory(config, start);
    if (!result.valid) {
      ROS_FATAL("Failed to generate dynamic trajectory: %s", result.reason.c_str());
      throw std::runtime_error(result.reason);
    }
    trajectory_ = result.trajectory;
    trace_id_ = result.trace_id;

    publisher_ = nh_.advertise<uav_msgs::Trajectory>(trajectory_topic_, 1, true);
    const double rate = std::isfinite(republish_rate_hz_) && republish_rate_hz_ > 0.0
                            ? republish_rate_hz_
                            : 0.2;
    ROS_INFO("dynamic_trajectory_publisher_node generated %s id=%u points=%zu "
             "time_scale=%.3f max_v=%.3f max_a=%.3f max_j=%.3f topic=%s",
             trace_id_.c_str(), trajectory_.trajectory_id, trajectory_.points.size(),
             result.time_scale, result.measured.max_velocity_mps,
             result.measured.max_acceleration_mps2, result.measured.max_jerk_mps3,
             trajectory_topic_.c_str());
    if (publish_once_) {
      trajectory_.header.stamp = ros::Time::now() + ros::Duration(start_delay_sec_);
      const ros::WallTime wait_start = ros::WallTime::now();
      while (ros::ok() && publisher_.getNumSubscribers() == 0 &&
             (ros::WallTime::now() - wait_start).toSec() < subscriber_wait_timeout_sec_) {
        ros::WallDuration(0.02).sleep();
      }
      const double wait_sec = (ros::WallTime::now() - wait_start).toSec();
      const uint32_t subscriber_count = publisher_.getNumSubscribers();
      if (subscriber_count == 0) {
        ROS_ERROR("publish_once_result exit_reason=no_subscriber trajectory_id=%u "
                  "subscriber_count=0 wait_sec=%.6f timeout_sec=%.6f publish_count=0",
                  trajectory_.trajectory_id, wait_sec, subscriber_wait_timeout_sec_);
        throw std::runtime_error("publish_once timed out waiting for subscriber");
      }
      ROS_INFO("publish_once_ready trajectory_id=%u subscriber_count=%u wait_sec=%.6f "
               "planned_publish_count=%d header_stamp=%.9f",
               trajectory_.trajectory_id, subscriber_count, wait_sec,
               publish_repeat_count_, trajectory_.header.stamp.toSec());
      for (int i = 0; ros::ok() && i < publish_repeat_count_; ++i) {
        publishTrajectory();
        ROS_INFO("publish_once_message trajectory_id=%u publish_index=%d publish_wall_time=%.9f "
                 "subscriber_count=%u header_stamp=%.9f",
                 trajectory_.trajectory_id, i + 1, ros::WallTime::now().toSec(),
                 publisher_.getNumSubscribers(), trajectory_.header.stamp.toSec());
        if (i + 1 < publish_repeat_count_) {
          ros::WallDuration(publish_repeat_interval_sec_).sleep();
        }
      }
      ros::WallDuration(post_publish_grace_sec_).sleep();
      ROS_INFO("publish_once_result exit_reason=published trajectory_id=%u "
               "subscriber_count=%u wait_sec=%.6f publish_count=%d "
               "post_publish_grace_sec=%.6f",
               trajectory_.trajectory_id, publisher_.getNumSubscribers(), wait_sec,
               publish_repeat_count_, post_publish_grace_sec_);
      ros::shutdown();
    } else {
      timer_ = nh_.createTimer(ros::Duration(1.0 / rate),
                               &DynamicTrajectoryPublisherNode::timerCallback, this,
                               false, true);
    }
  }

 private:
  void timerCallback(const ros::TimerEvent&) {
    trajectory_.header.stamp = ros::Time::now() + ros::Duration(start_delay_sec_);
    publishTrajectory();
  }

  void publishTrajectory() {
    publisher_.publish(trajectory_);
  }

  void validatePublisherParameters() const {
    if (!std::isfinite(republish_rate_hz_) || republish_rate_hz_ <= 0.0) {
      throw std::runtime_error("republish_rate_hz must be finite and positive");
    }
    if (!std::isfinite(subscriber_wait_timeout_sec_) ||
        subscriber_wait_timeout_sec_ < 0.0 ||
        subscriber_wait_timeout_sec_ > 60.0) {
      throw std::runtime_error("subscriber_wait_timeout_sec must be finite in [0, 60]");
    }
    if (publish_repeat_count_ < 1 || publish_repeat_count_ > 100) {
      throw std::runtime_error("publish_repeat_count must be in [1, 100]");
    }
    if (!std::isfinite(publish_repeat_interval_sec_) ||
        publish_repeat_interval_sec_ < 0.0 ||
        publish_repeat_interval_sec_ > 10.0) {
      throw std::runtime_error("publish_repeat_interval_sec must be finite in [0, 10]");
    }
    if (!std::isfinite(post_publish_grace_sec_) ||
        post_publish_grace_sec_ < 0.0 ||
        post_publish_grace_sec_ > 60.0) {
      throw std::runtime_error("post_publish_grace_sec must be finite in [0, 60]");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher publisher_;
  ros::Timer timer_;
  std::string trajectory_topic_;
  std::string trace_id_;
  bool publish_once_ = true;
  double republish_rate_hz_ = 0.2;
  double subscriber_wait_timeout_sec_ = 2.0;
  int publish_repeat_count_ = 3;
  double publish_repeat_interval_sec_ = 0.05;
  double post_publish_grace_sec_ = 0.20;
  double start_delay_sec_ = 2.0;
  uav_msgs::Trajectory trajectory_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_trajectory_publisher_node");
  try {
    DynamicTrajectoryPublisherNode node(ros::NodeHandle{}, ros::NodeHandle{"~"});
    ros::spin();
  } catch (const std::exception& exc) {
    ROS_FATAL("dynamic_trajectory_publisher_node failed: %s", exc.what());
    return 1;
  }
  return 0;
}
