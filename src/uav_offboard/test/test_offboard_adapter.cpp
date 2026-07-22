#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "uav_offboard/offboard_adapter.hpp"

namespace {

uav_msgs::SetpointPreview makePreview() {
  uav_msgs::SetpointPreview preview;
  preview.header.stamp = ros::Time(10.0);
  preview.header.frame_id = "map";
  preview.trajectory_valid = true;
  preview.started = true;
  preview.finished = false;
  preview.state_fresh = true;
  preview.trajectory_id = 12;
  preview.point.position.x = 1.0;
  preview.point.position.y = 2.0;
  preview.point.position.z = 3.0;
  preview.point.velocity.x = 0.1;
  preview.point.velocity.y = 0.2;
  preview.point.velocity.z = 0.3;
  preview.point.acceleration.x = 0.01;
  preview.point.acceleration.y = 0.02;
  preview.point.acceleration.z = 0.03;
  preview.point.yaw = 0.4;
  preview.point.yaw_rate = 0.5;
  return preview;
}

uav_msgs::UavState makeUavState() {
  uav_msgs::UavState state;
  state.header.stamp = ros::Time(10.0);
  state.header.frame_id = "map";
  state.pose_valid = true;
  state.twist_valid = true;
  return state;
}

mavros_msgs::State makeMavrosState() {
  mavros_msgs::State state;
  state.connected = true;
  state.armed = false;
  state.mode = "AUTO.LOITER";
  return state;
}

uav_offboard::AdapterInputs makeInputs() {
  uav_offboard::AdapterInputs inputs;
  inputs.preview_received = true;
  inputs.vehicle_state_received = true;
  inputs.mavros_state_received = true;
  inputs.preview = makePreview();
  inputs.vehicle_state = makeUavState();
  inputs.mavros_state = makeMavrosState();
  return inputs;
}

uav_offboard::AdapterConfig makeConfig() {
  uav_offboard::AdapterConfig config;
  config.preview_timeout = ros::Duration(0.2);
  config.state_timeout = ros::Duration(0.2);
  config.supported_frames = {"map"};
  return config;
}

uav_offboard::HealthReport healthyReport() {
  return uav_offboard::evaluateHealth(makeInputs(), makeConfig(), ros::Time(10.1), ros::Time(10.0));
}

}  // namespace

TEST(OffboardConfig, DefaultStaticGateIsFalse) {
  uav_offboard::AdapterConfig config;
  EXPECT_FALSE(config.allow_mavros_output);
}

TEST(OffboardConfig, InvalidParametersAreClamped) {
  auto config = makeConfig();
  config.publish_rate_hz = -1.0;
  config.preview_timeout = ros::Duration(-1.0);
  config.state_timeout = ros::Duration(0.0);
  config.supported_frames.clear();
  std::string warning;
  EXPECT_FALSE(uav_offboard::validateConfig(&config, &warning));
  EXPECT_DOUBLE_EQ(config.publish_rate_hz, 30.0);
  EXPECT_DOUBLE_EQ(config.preview_timeout.toSec(), 0.2);
  EXPECT_DOUBLE_EQ(config.state_timeout.toSec(), 0.2);
  EXPECT_EQ(config.supported_frames.front(), "map");
  EXPECT_FALSE(warning.empty());
}

TEST(OffboardGates, StaticGateClosedCannotEnableOutput) {
  auto config = makeConfig();
  config.allow_mavros_output = false;
  std::string reason;
  EXPECT_FALSE(uav_offboard::canEnableRuntimeOutput(config, healthyReport(), &reason));
  EXPECT_EQ(reason, "static gate disabled");
}

TEST(OffboardGates, RuntimeGateInitialFalseAndDoubleGateRequired) {
  auto config = makeConfig();
  config.allow_mavros_output = true;
  uav_offboard::GateState gates;
  EXPECT_FALSE(gates.runtime_output_enabled);
  EXPECT_EQ(uav_offboard::decideState(config, gates, healthyReport()),
            uav_offboard::AdapterState::READY_DRY_RUN);
  gates.runtime_output_enabled = true;
  EXPECT_EQ(uav_offboard::decideState(config, gates, healthyReport()),
            uav_offboard::AdapterState::STREAMING);
}

TEST(OffboardGates, HealthyInputsEnterReadyDryRun) {
  auto config = makeConfig();
  uav_offboard::GateState gates;
  EXPECT_EQ(uav_offboard::decideState(config, gates, healthyReport()),
            uav_offboard::AdapterState::READY_DRY_RUN);
}

TEST(OffboardHealth, PreviewTimeoutStopsOutput) {
  const auto health =
      uav_offboard::evaluateHealth(makeInputs(), makeConfig(), ros::Time(10.3), ros::Time(10.2));
  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "preview timeout");
}

TEST(OffboardHealth, StateTimeoutStopsOutput) {
  auto inputs = makeInputs();
  inputs.preview.header.stamp = ros::Time(10.25);
  inputs.vehicle_state.header.stamp = ros::Time(10.0);
  const auto health =
      uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.25), ros::Time(10.2));
  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "vehicle state timeout");
}

TEST(OffboardHealth, MavrosDisconnectedStopsOutput) {
  auto inputs = makeInputs();
  inputs.mavros_state.connected = false;
  const auto health =
      uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0));
  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "MAVROS disconnected");
}

TEST(OffboardHealth, RejectsInvalidTrajectoryFlags) {
  auto inputs = makeInputs();
  inputs.preview.trajectory_valid = false;
  EXPECT_EQ(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).reason,
            "preview trajectory is invalid");
  inputs = makeInputs();
  inputs.preview.started = false;
  EXPECT_EQ(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).reason,
            "trajectory has not started");
  inputs = makeInputs();
  inputs.preview.finished = true;
  EXPECT_EQ(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).reason,
            "trajectory is finished");
}

TEST(OffboardHealth, RejectsInvalidVehicleStateFlags) {
  auto inputs = makeInputs();
  inputs.vehicle_state.pose_valid = false;
  EXPECT_EQ(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).reason,
            "vehicle pose invalid");
  inputs = makeInputs();
  inputs.vehicle_state.twist_valid = false;
  EXPECT_EQ(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).reason,
            "vehicle twist invalid");
}

TEST(OffboardHealth, RejectsNanInfAndFrameMismatch) {
  auto inputs = makeInputs();
  inputs.preview.point.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).reason,
            "preview contains NaN or Inf");
  inputs = makeInputs();
  inputs.preview.header.frame_id = "odom";
  EXPECT_EQ(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).reason,
            "frame mismatch or unsupported frame");
}

TEST(OffboardHealth, RejectsRosTimeBackwards) {
  const auto health =
      uav_offboard::evaluateHealth(makeInputs(), makeConfig(), ros::Time(9.9), ros::Time(10.0));
  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "ROS time moved backward");
}

TEST(OffboardHealth, AtomicNewMessageUpdateUsesLatestInput) {
  auto inputs = makeInputs();
  inputs.preview.trajectory_id = 1;
  EXPECT_TRUE(uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0)).healthy);
  inputs.preview.trajectory_valid = false;
  inputs.preview.trajectory_id = 2;
  const auto health =
      uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0));
  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "preview trajectory is invalid");
}

TEST(OffboardStatus, ReasonAndStateNameAreExplicit) {
  auto config = makeConfig();
  uav_offboard::GateState gates;
  const auto health = healthyReport();
  const auto status = uav_offboard::makeStatus(
      config, gates, health, uav_offboard::AdapterState::READY_DRY_RUN, ros::Time(11.0));
  EXPECT_EQ(status.state, uav_msgs::OffboardStatus::READY_DRY_RUN);
  EXPECT_EQ(status.state_name, "READY_DRY_RUN");
  EXPECT_EQ(status.reason, "healthy");
  EXPECT_FALSE(status.output_active);
}

TEST(PositionTargetMapping, MapsAllSupportedFields) {
  const auto preview = makePreview();
  const auto target = uav_offboard::previewToPositionTarget(preview, ros::Time(12.0));
  EXPECT_DOUBLE_EQ(target.header.stamp.toSec(), 12.0);
  EXPECT_EQ(target.header.frame_id, "map");
  EXPECT_EQ(target.coordinate_frame, mavros_msgs::PositionTarget::FRAME_LOCAL_NED);
  EXPECT_EQ(target.type_mask, 0);
  EXPECT_DOUBLE_EQ(target.position.x, 1.0);
  EXPECT_DOUBLE_EQ(target.velocity.y, 0.2);
  EXPECT_DOUBLE_EQ(target.acceleration_or_force.z, 0.03);
  EXPECT_FLOAT_EQ(target.yaw, 0.4f);
  EXPECT_FLOAT_EQ(target.yaw_rate, 0.5f);
}

TEST(PositionTargetMapping, TypeMaskUsesPositionVelocityAccelerationYawYawRate) {
  const auto target = uav_offboard::previewToPositionTarget(makePreview(), ros::Time(12.0));
  EXPECT_EQ(target.type_mask & mavros_msgs::PositionTarget::IGNORE_PX, 0);
  EXPECT_EQ(target.type_mask & mavros_msgs::PositionTarget::IGNORE_VX, 0);
  EXPECT_EQ(target.type_mask & mavros_msgs::PositionTarget::IGNORE_AFX, 0);
  EXPECT_EQ(target.type_mask & mavros_msgs::PositionTarget::IGNORE_YAW, 0);
  EXPECT_EQ(target.type_mask & mavros_msgs::PositionTarget::IGNORE_YAW_RATE, 0);
  EXPECT_EQ(target.type_mask & mavros_msgs::PositionTarget::FORCE, 0);
}

TEST(OffboardStateMachine, WaitingInputsAndFaultStates) {
  auto config = makeConfig();
  config.allow_mavros_output = true;
  uav_offboard::GateState gates;
  gates.runtime_output_enabled = true;
  uav_offboard::HealthReport waiting;
  waiting.preview_received = false;
  waiting.mavros_connected = false;
  EXPECT_EQ(uav_offboard::decideState(config, gates, waiting),
            uav_offboard::AdapterState::WAITING_INPUTS);
  auto fault = healthyReport();
  fault.healthy = false;
  fault.reason = "preview timeout";
  EXPECT_EQ(uav_offboard::decideState(config, gates, fault),
            uav_offboard::AdapterState::FAULT);
}

TEST(OffboardLogging, FaultReasonsSupportThrottledLogs) {
  auto inputs = makeInputs();
  inputs.preview.state_fresh = false;
  const auto health =
      uav_offboard::evaluateHealth(inputs, makeConfig(), ros::Time(10.1), ros::Time(10.0));
  EXPECT_FALSE(health.healthy);
  EXPECT_EQ(health.reason, "preview reports stale vehicle state");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
