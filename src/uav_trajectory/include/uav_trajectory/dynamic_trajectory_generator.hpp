#ifndef UAV_TRAJECTORY_DYNAMIC_TRAJECTORY_GENERATOR_HPP_
#define UAV_TRAJECTORY_DYNAMIC_TRAJECTORY_GENERATOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <uav_msgs/Trajectory.h>

namespace uav_trajectory {

enum class DynamicTrajectoryType {
  kLine,
  kCircle,
  kFigure8,
};

enum class YawMode {
  kFixed,
  kVelocityAligned,
};

struct DynamicTrajectoryConfig {
  DynamicTrajectoryType trajectory_type = DynamicTrajectoryType::kLine;
  std::string frame_id = "map";
  double start_delay_sec = 2.0;
  double altitude_offset_m = 1.0;
  double duration_sec = 20.0;
  double sample_period_sec = 0.05;
  double hold_end_sec = 5.0;
  double initial_hold_sec = 0.0;
  double initial_climb_duration_sec = 0.0;
  double post_climb_hold_sec = 0.0;
  YawMode yaw_mode = YawMode::kFixed;

  double line_length_m = 1.0;
  double line_segment_duration_sec = 5.0;
  double circle_radius_m = 1.0;
  double circle_tangent_speed_mps = 0.5;
  double circle_laps = 1.0;
  double transition_duration_sec = 2.0;
  double figure8_amplitude_x_m = 1.0;
  double figure8_amplitude_y_m = 0.5;

  double max_velocity_mps = 1.0;
  double max_acceleration_mps2 = 1.5;
  double max_jerk_mps3 = 4.0;
  double low_speed_yaw_threshold_mps = 0.05;
};

struct StartPose {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

struct DynamicTrajectoryLimits {
  double max_velocity_mps = 0.0;
  double max_acceleration_mps2 = 0.0;
  double max_jerk_mps3 = 0.0;
};

struct DynamicTrajectoryResult {
  bool valid = false;
  std::string reason;
  std::string trace_id;
  double time_scale = 1.0;
  DynamicTrajectoryLimits measured;
  uav_msgs::Trajectory trajectory;
};

bool parseTrajectoryType(const std::string& value, DynamicTrajectoryType* type);
bool parseYawMode(const std::string& value, YawMode* mode);
std::string trajectoryTypeName(DynamicTrajectoryType type);
std::string yawModeName(YawMode mode);

DynamicTrajectoryResult generateDynamicTrajectory(const DynamicTrajectoryConfig& config,
                                                  const StartPose& start_pose);

bool validateDynamicTrajectory(const uav_msgs::Trajectory& trajectory,
                               const DynamicTrajectoryConfig& config,
                               DynamicTrajectoryLimits* measured,
                               std::string* reason);

double shortestAngleDelta(double from, double to);
double unwrapNear(double reference, double value);

}  // namespace uav_trajectory

#endif  // UAV_TRAJECTORY_DYNAMIC_TRAJECTORY_GENERATOR_HPP_
