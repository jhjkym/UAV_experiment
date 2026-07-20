#ifndef UAV_PX4_BRIDGE_STATE_BRIDGE_HPP_
#define UAV_PX4_BRIDGE_STATE_BRIDGE_HPP_

#include <nav_msgs/Odometry.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <uav_msgs/UavState.h>

namespace uav_px4_bridge {

bool isFinite(const nav_msgs::Odometry& odom);

bool isTimedOut(const ros::Time& stamp,
                const ros::Time& now,
                const ros::Duration& timeout);

uav_msgs::UavState odometryToUavState(const nav_msgs::Odometry& odom,
                                      bool pose_valid,
                                      bool twist_valid);

}  // namespace uav_px4_bridge

#endif  // UAV_PX4_BRIDGE_STATE_BRIDGE_HPP_
