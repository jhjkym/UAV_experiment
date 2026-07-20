#include "uav_px4_bridge/state_bridge.hpp"

#include <cmath>

namespace uav_px4_bridge {
namespace {

bool finite(const double value) {
  return std::isfinite(value);
}

}  // namespace

bool isFinite(const nav_msgs::Odometry& odom) {
  const auto& p = odom.pose.pose.position;
  const auto& q = odom.pose.pose.orientation;
  const auto& linear = odom.twist.twist.linear;
  const auto& angular = odom.twist.twist.angular;

  return finite(p.x) && finite(p.y) && finite(p.z) &&
         finite(q.x) && finite(q.y) && finite(q.z) && finite(q.w) &&
         finite(linear.x) && finite(linear.y) && finite(linear.z) &&
         finite(angular.x) && finite(angular.y) && finite(angular.z);
}

bool isTimedOut(const ros::Time& stamp,
                const ros::Time& now,
                const ros::Duration& timeout) {
  if (stamp.isZero()) {
    return true;
  }
  if (timeout.toSec() < 0.0) {
    return false;
  }
  return (now - stamp) > timeout;
}

uav_msgs::UavState odometryToUavState(const nav_msgs::Odometry& odom,
                                      const bool pose_valid,
                                      const bool twist_valid) {
  uav_msgs::UavState state;
  state.header = odom.header;
  state.pose = odom.pose.pose;
  state.twist = odom.twist.twist;
  state.acceleration.x = 0.0;
  state.acceleration.y = 0.0;
  state.acceleration.z = 0.0;
  state.pose_valid = pose_valid;
  state.twist_valid = twist_valid;
  state.acceleration_valid = false;
  return state;
}

}  // namespace uav_px4_bridge
