#include <limits>

#include <gtest/gtest.h>
#include <nav_msgs/Odometry.h>

#include "uav_px4_bridge/state_bridge.hpp"

namespace {

nav_msgs::Odometry makeOdom() {
  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time(123.0);
  odom.header.frame_id = "map";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.x = 0.1;
  odom.pose.pose.orientation.y = 0.2;
  odom.pose.pose.orientation.z = 0.3;
  odom.pose.pose.orientation.w = 0.9;
  odom.twist.twist.linear.x = 4.0;
  odom.twist.twist.linear.y = 5.0;
  odom.twist.twist.linear.z = 6.0;
  odom.twist.twist.angular.x = 7.0;
  odom.twist.twist.angular.y = 8.0;
  odom.twist.twist.angular.z = 9.0;
  return odom;
}

}  // namespace

TEST(StateBridge, OdometryToUavStateCopiesValues) {
  const nav_msgs::Odometry odom = makeOdom();
  const uav_msgs::UavState state = uav_px4_bridge::odometryToUavState(odom, true, true);

  EXPECT_EQ(state.header.stamp, odom.header.stamp);
  EXPECT_EQ(state.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(state.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(state.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(state.pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(state.pose.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(state.pose.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(state.pose.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(state.pose.orientation.w, 0.9);
  EXPECT_DOUBLE_EQ(state.twist.linear.x, 4.0);
  EXPECT_DOUBLE_EQ(state.twist.linear.y, 5.0);
  EXPECT_DOUBLE_EQ(state.twist.linear.z, 6.0);
  EXPECT_DOUBLE_EQ(state.twist.angular.x, 7.0);
  EXPECT_DOUBLE_EQ(state.twist.angular.y, 8.0);
  EXPECT_DOUBLE_EQ(state.twist.angular.z, 9.0);
  EXPECT_TRUE(state.pose_valid);
  EXPECT_TRUE(state.twist_valid);
  EXPECT_FALSE(state.acceleration_valid);
  EXPECT_DOUBLE_EQ(state.acceleration.x, 0.0);
  EXPECT_DOUBLE_EQ(state.acceleration.y, 0.0);
  EXPECT_DOUBLE_EQ(state.acceleration.z, 0.0);
}

TEST(StateBridge, ValidFlagsAreConfigurable) {
  const uav_msgs::UavState state = uav_px4_bridge::odometryToUavState(makeOdom(), false, false);
  EXPECT_FALSE(state.pose_valid);
  EXPECT_FALSE(state.twist_valid);
  EXPECT_FALSE(state.acceleration_valid);
}

TEST(StateBridge, TimeoutFunction) {
  EXPECT_TRUE(uav_px4_bridge::isTimedOut(ros::Time(), ros::Time(10.0), ros::Duration(1.0)));
  EXPECT_FALSE(uav_px4_bridge::isTimedOut(ros::Time(9.5), ros::Time(10.0), ros::Duration(1.0)));
  EXPECT_TRUE(uav_px4_bridge::isTimedOut(ros::Time(8.0), ros::Time(10.0), ros::Duration(1.0)));
  EXPECT_FALSE(uav_px4_bridge::isTimedOut(ros::Time(1.0), ros::Time(100.0), ros::Duration(-1.0)));
}

TEST(StateBridge, FiniteCheckRejectsNaNAndInfinity) {
  nav_msgs::Odometry odom = makeOdom();
  EXPECT_TRUE(uav_px4_bridge::isFinite(odom));

  odom.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(uav_px4_bridge::isFinite(odom));

  odom = makeOdom();
  odom.twist.twist.angular.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(uav_px4_bridge::isFinite(odom));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
