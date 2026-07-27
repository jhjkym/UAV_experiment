#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <uav_trajectory/dynamic_trajectory_generator.hpp>

namespace {

constexpr double kPi = 3.14159265358979323846;

uav_trajectory::DynamicTrajectoryConfig baseConfig(
    const uav_trajectory::DynamicTrajectoryType type) {
  uav_trajectory::DynamicTrajectoryConfig config;
  config.trajectory_type = type;
  config.frame_id = "map";
  config.start_delay_sec = 0.1;
  config.altitude_offset_m = 1.0;
  config.duration_sec = 20.0;
  config.sample_period_sec = 0.05;
  config.hold_end_sec = 0.5;
  config.max_velocity_mps = 1.0;
  config.max_acceleration_mps2 = 1.5;
  config.max_jerk_mps3 = 4.0;
  return config;
}

uav_trajectory::StartPose startPose() {
  uav_trajectory::StartPose start;
  start.x = 2.0;
  start.y = -1.0;
  start.z = 0.25;
  start.yaw = 0.3;
  return start;
}

const uav_msgs::TrajectoryPoint& lastMovingPoint(const uav_msgs::Trajectory& trajectory,
                                                 const double hold_end_sec) {
  const double end_time = trajectory.points.back().time_from_start.toSec() - hold_end_sec;
  auto it = std::min_element(
      trajectory.points.begin(), trajectory.points.end(),
      [end_time](const auto& lhs, const auto& rhs) {
        return std::abs(lhs.time_from_start.toSec() - end_time) <
               std::abs(rhs.time_from_start.toSec() - end_time);
      });
  return *it;
}

double speed(const uav_msgs::TrajectoryPoint& point) {
  return std::sqrt(point.velocity.x * point.velocity.x +
                   point.velocity.y * point.velocity.y +
                   point.velocity.z * point.velocity.z);
}

double accel(const uav_msgs::TrajectoryPoint& point) {
  return std::sqrt(point.acceleration.x * point.acceleration.x +
                   point.acceleration.y * point.acceleration.y +
                   point.acceleration.z * point.acceleration.z);
}

double horizontalDistance(const uav_msgs::TrajectoryPoint& point,
                          const uav_trajectory::StartPose& start) {
  return std::hypot(point.position.x - start.x, point.position.y - start.y);
}

std::vector<double> yaws(const uav_msgs::Trajectory& trajectory) {
  std::vector<double> values;
  for (const auto& point : trajectory.points) {
    values.push_back(point.yaw);
  }
  return values;
}

}  // namespace

TEST(DynamicLine, StartEndAndC2Segments) {
  auto config = baseConfig(uav_trajectory::DynamicTrajectoryType::kLine);
  const auto start = startPose();
  const auto result = uav_trajectory::generateDynamicTrajectory(config, start);
  ASSERT_TRUE(result.valid) << result.reason;
  const auto& trajectory = result.trajectory;
  ASSERT_GT(trajectory.points.size(), 10u);
  EXPECT_NEAR(trajectory.points.front().position.x, start.x, 1e-9);
  EXPECT_NEAR(trajectory.points.front().position.y, start.y, 1e-9);
  EXPECT_NEAR(trajectory.points.front().position.z, start.z + 1.0, 1e-9);
  const auto& end = lastMovingPoint(trajectory, config.hold_end_sec);
  EXPECT_NEAR(end.position.x, start.x, 1e-6);
  EXPECT_NEAR(end.position.y, start.y, 1e-6);
  EXPECT_NEAR(speed(trajectory.points.front()), 0.0, 1e-9);
  EXPECT_NEAR(accel(trajectory.points.front()), 0.0, 1e-9);
  EXPECT_NEAR(speed(end), 0.0, 1e-6);
  EXPECT_NEAR(accel(end), 0.0, 1e-6);

  for (const double t : {config.line_segment_duration_sec,
                         2.0 * config.line_segment_duration_sec}) {
    auto it = std::min_element(
        trajectory.points.begin(), trajectory.points.end(),
        [t](const auto& lhs, const auto& rhs) {
          return std::abs(lhs.time_from_start.toSec() - t) <
                 std::abs(rhs.time_from_start.toSec() - t);
        });
    ASSERT_NE(it, trajectory.points.end());
    EXPECT_NEAR(speed(*it), 0.0, 1e-6);
    EXPECT_NEAR(accel(*it), 0.0, 1e-6);
  }
}

TEST(DynamicCircle, RadiusClosureVelocityAndAcceleration) {
  auto config = baseConfig(uav_trajectory::DynamicTrajectoryType::kCircle);
  config.hold_end_sec = 0.0;
  const auto start = startPose();
  const auto result = uav_trajectory::generateDynamicTrajectory(config, start);
  ASSERT_TRUE(result.valid) << result.reason;
  const auto& trajectory = result.trajectory;

  double max_radius_error = 0.0;
  bool saw_circle_motion = false;
  for (const auto& point : trajectory.points) {
    const double t = point.time_from_start.toSec();
    if (t < config.transition_duration_sec * result.time_scale ||
        t > trajectory.points.back().time_from_start.toSec() -
                config.transition_duration_sec * result.time_scale) {
      continue;
    }
    const double radius = horizontalDistance(point, start);
    if (radius > 0.95 * config.circle_radius_m) {
      saw_circle_motion = true;
      max_radius_error = std::max(max_radius_error, std::abs(radius - config.circle_radius_m));
      const double radial_velocity =
          ((point.position.x - start.x) * point.velocity.x +
           (point.position.y - start.y) * point.velocity.y) / radius;
      EXPECT_NEAR(radial_velocity, 0.0, 0.04);
    }
  }
  EXPECT_TRUE(saw_circle_motion);
  EXPECT_LT(max_radius_error, 1e-6);
  EXPECT_NEAR(trajectory.points.back().position.x, start.x, 1e-6);
  EXPECT_NEAR(trajectory.points.back().position.y, start.y, 1e-6);
  EXPECT_NEAR(speed(trajectory.points.front()), 0.0, 1e-9);
  EXPECT_NEAR(speed(trajectory.points.back()), 0.0, 1e-6);
}

TEST(DynamicFigure8, CenterCrossingVelocityAndAcceleration) {
  auto config = baseConfig(uav_trajectory::DynamicTrajectoryType::kFigure8);
  config.hold_end_sec = 0.0;
  const auto start = startPose();
  const auto result = uav_trajectory::generateDynamicTrajectory(config, start);
  ASSERT_TRUE(result.valid) << result.reason;
  int center_crossings = 0;
  for (const auto& point : result.trajectory.points) {
    if (std::hypot(point.position.x - start.x, point.position.y - start.y) < 0.02) {
      ++center_crossings;
    }
    EXPECT_TRUE(std::isfinite(speed(point)));
    EXPECT_TRUE(std::isfinite(accel(point)));
  }
  EXPECT_GE(center_crossings, 3);
  EXPECT_NEAR(result.trajectory.points.front().position.x, start.x, 1e-9);
  EXPECT_NEAR(result.trajectory.points.back().position.x, start.x, 1e-6);
}

TEST(DynamicYaw, FixedYawAndVelocityAlignedYaw) {
  auto fixed = baseConfig(uav_trajectory::DynamicTrajectoryType::kLine);
  fixed.yaw_mode = uav_trajectory::YawMode::kFixed;
  const auto start = startPose();
  auto result = uav_trajectory::generateDynamicTrajectory(fixed, start);
  ASSERT_TRUE(result.valid) << result.reason;
  for (const auto& point : result.trajectory.points) {
    EXPECT_NEAR(point.yaw, start.yaw, 1e-12);
    EXPECT_NEAR(point.yaw_rate, 0.0, 1e-12);
  }

  auto aligned = fixed;
  aligned.yaw_mode = uav_trajectory::YawMode::kVelocityAligned;
  result = uav_trajectory::generateDynamicTrajectory(aligned, start);
  ASSERT_TRUE(result.valid) << result.reason;
  bool saw_forward = false;
  bool saw_backward = false;
  bool saw_backward_aligned = false;
  for (const auto& point : result.trajectory.points) {
    if (point.velocity.x > 0.2) {
      saw_forward = true;
      EXPECT_NEAR(std::sin(point.yaw), 0.0, 1e-3);
    }
    if (point.velocity.x < -0.2) {
      saw_backward = true;
      if (std::abs(std::abs(point.yaw) - kPi) < 0.2) {
        saw_backward_aligned = true;
      }
    }
  }
  EXPECT_TRUE(saw_forward);
  EXPECT_TRUE(saw_backward);
  EXPECT_TRUE(saw_backward_aligned);
}

TEST(DynamicYaw, PiCrossingAndLowSpeedHold) {
  EXPECT_NEAR(uav_trajectory::unwrapNear(3.10, -3.10), 3.183185307179586, 1e-6);

  auto config = baseConfig(uav_trajectory::DynamicTrajectoryType::kLine);
  config.yaw_mode = uav_trajectory::YawMode::kVelocityAligned;
  const auto result = uav_trajectory::generateDynamicTrajectory(config, startPose());
  ASSERT_TRUE(result.valid) << result.reason;
  const auto values = yaws(result.trajectory);
  for (std::size_t i = 1; i < values.size(); ++i) {
    EXPECT_LT(std::abs(values[i] - values[i - 1]), kPi / 2.0);
  }
  EXPECT_TRUE(std::isfinite(result.trajectory.points.front().yaw));
}

TEST(DynamicConstraints, VelocityAccelerationJerkAndStrictTime) {
  auto config = baseConfig(uav_trajectory::DynamicTrajectoryType::kLine);
  config.line_segment_duration_sec = 0.5;
  const auto result = uav_trajectory::generateDynamicTrajectory(config, startPose());
  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_GT(result.time_scale, 1.0);
  EXPECT_LE(result.measured.max_velocity_mps, config.max_velocity_mps * 1.001);
  EXPECT_LE(result.measured.max_acceleration_mps2, config.max_acceleration_mps2 * 1.001);
  EXPECT_LE(result.measured.max_jerk_mps3, config.max_jerk_mps3 * 1.001);
  for (std::size_t i = 1; i < result.trajectory.points.size(); ++i) {
    EXPECT_GT(result.trajectory.points[i].time_from_start.toSec(),
              result.trajectory.points[i - 1].time_from_start.toSec());
  }
}

TEST(DynamicValidation, RejectsNanInfAndIllegalParameters) {
  auto config = baseConfig(uav_trajectory::DynamicTrajectoryType::kLine);
  config.line_length_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(uav_trajectory::generateDynamicTrajectory(config, startPose()).valid);
  config = baseConfig(uav_trajectory::DynamicTrajectoryType::kLine);
  config.sample_period_sec = -0.1;
  EXPECT_FALSE(uav_trajectory::generateDynamicTrajectory(config, startPose()).valid);
  config = baseConfig(uav_trajectory::DynamicTrajectoryType::kLine);
  config.max_velocity_mps = 0.0;
  EXPECT_FALSE(uav_trajectory::generateDynamicTrajectory(config, startPose()).valid);
}

TEST(DynamicIdentity, TrajectoryIdIsDeterministic) {
  auto config = baseConfig(uav_trajectory::DynamicTrajectoryType::kFigure8);
  const auto start = startPose();
  const auto first = uav_trajectory::generateDynamicTrajectory(config, start);
  const auto second = uav_trajectory::generateDynamicTrajectory(config, start);
  ASSERT_TRUE(first.valid) << first.reason;
  ASSERT_TRUE(second.valid) << second.reason;
  EXPECT_EQ(first.trace_id, second.trace_id);
  EXPECT_EQ(first.trajectory.trajectory_id, second.trajectory.trajectory_id);
  ASSERT_EQ(first.trajectory.points.size(), second.trajectory.points.size());
  for (std::size_t i = 0; i < first.trajectory.points.size(); ++i) {
    EXPECT_NEAR(first.trajectory.points[i].position.x,
                second.trajectory.points[i].position.x, 1e-12);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
