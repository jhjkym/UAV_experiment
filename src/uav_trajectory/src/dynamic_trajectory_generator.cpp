#include "uav_trajectory/dynamic_trajectory_generator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>

#include <ros/duration.h>
#include <ros/time.h>

namespace uav_trajectory {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEpsilon = 1e-9;

struct FlatSample {
  double t = 0.0;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double vx = 0.0;
  double vy = 0.0;
  double vz = 0.0;
  double ax = 0.0;
  double ay = 0.0;
  double az = 0.0;
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

bool finite(const double value) {
  return std::isfinite(value);
}

bool finitePoint(const uav_msgs::TrajectoryPoint& point) {
  return finite(point.time_from_start.toSec()) &&
         finite(point.position.x) && finite(point.position.y) && finite(point.position.z) &&
         finite(point.velocity.x) && finite(point.velocity.y) && finite(point.velocity.z) &&
         finite(point.acceleration.x) && finite(point.acceleration.y) &&
         finite(point.acceleration.z) && finite(point.yaw) && finite(point.yaw_rate);
}

double clamp01(const double value) {
  return std::max(0.0, std::min(1.0, value));
}

double smooth5(const double u) {
  const double c = clamp01(u);
  return c * c * c * (10.0 + c * (-15.0 + 6.0 * c));
}

double dsmooth5(const double u) {
  const double c = clamp01(u);
  return 30.0 * c * c * (1.0 - c) * (1.0 - c);
}

double d2smooth5(const double u) {
  const double c = clamp01(u);
  return 60.0 * c * (1.0 - c) * (1.0 - 2.0 * c);
}

void addHold(std::vector<FlatSample>* samples, const FlatSample& last,
             const double hold_sec, const double sample_period) {
  if (hold_sec <= 0.0) {
    return;
  }
  const double end = last.t + hold_sec;
  for (double t = last.t + sample_period; t < end - 0.5 * sample_period; t += sample_period) {
    FlatSample sample = last;
    sample.t = t;
    sample.vx = sample.vy = sample.vz = 0.0;
    sample.ax = sample.ay = sample.az = 0.0;
    sample.yaw_rate = 0.0;
    samples->push_back(sample);
  }
  FlatSample end_sample = last;
  end_sample.t = end;
  end_sample.vx = end_sample.vy = end_sample.vz = 0.0;
  end_sample.ax = end_sample.ay = end_sample.az = 0.0;
  end_sample.yaw_rate = 0.0;
  samples->push_back(end_sample);
}

void addSmoothSegment(std::vector<FlatSample>* samples,
                      const double start_t,
                      const double duration,
                      const double x0,
                      const double y0,
                      const double z0,
                      const double x1,
                      const double y1,
                      const double z1,
                      const double sample_period) {
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double dz = z1 - z0;
  const std::size_t steps =
      std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(duration / sample_period)));
  for (std::size_t i = samples->empty() ? 0 : 1; i <= steps; ++i) {
    const double t = start_t + duration * static_cast<double>(i) / static_cast<double>(steps);
    const double u = duration > kEpsilon ? (t - start_t) / duration : 1.0;
    const double s = smooth5(u);
    const double ds = dsmooth5(u) / duration;
    const double d2s = d2smooth5(u) / (duration * duration);
    FlatSample sample;
    sample.t = t;
    sample.x = x0 + dx * s;
    sample.y = y0 + dy * s;
    sample.z = z0 + dz * s;
    sample.vx = dx * ds;
    sample.vy = dy * ds;
    sample.vz = dz * ds;
    sample.ax = dx * d2s;
    sample.ay = dy * d2s;
    sample.az = dz * d2s;
    samples->push_back(sample);
  }
}

std::string formatDouble(const double value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(3) << value;
  return out.str();
}

std::uint32_t deterministicId(const std::string& text) {
  std::uint32_t hash = 2166136261u;
  for (const unsigned char c : text) {
    hash ^= c;
    hash *= 16777619u;
  }
  return hash == 0u ? 1u : hash;
}

bool validateConfig(const DynamicTrajectoryConfig& config, std::string* reason) {
  const std::vector<double> finite_values = {
      config.start_delay_sec, config.altitude_offset_m, config.duration_sec,
      config.sample_period_sec, config.hold_end_sec, config.initial_hold_sec,
      config.initial_climb_duration_sec, config.post_climb_hold_sec, config.line_length_m,
      config.line_segment_duration_sec, config.circle_radius_m,
      config.circle_tangent_speed_mps, config.transition_duration_sec,
      config.figure8_amplitude_x_m, config.figure8_amplitude_y_m,
      config.max_velocity_mps, config.max_acceleration_mps2, config.max_jerk_mps3,
      config.low_speed_yaw_threshold_mps};
  if (!std::all_of(finite_values.begin(), finite_values.end(), finite)) {
    *reason = "dynamic trajectory parameters must be finite";
    return false;
  }
  if (config.frame_id.empty()) {
    *reason = "frame_id must not be empty";
    return false;
  }
  if (config.start_delay_sec < 0.0 || config.duration_sec <= 0.0 ||
      config.sample_period_sec <= 0.0 || config.hold_end_sec < 0.0 ||
      config.initial_hold_sec < 0.0 || config.initial_climb_duration_sec < 0.0 ||
      config.post_climb_hold_sec < 0.0) {
    *reason = "time parameters are outside valid ranges";
    return false;
  }
  if (config.sample_period_sec > config.duration_sec) {
    *reason = "sample_period_sec must not exceed duration_sec";
    return false;
  }
  if (config.max_velocity_mps <= 0.0 || config.max_acceleration_mps2 <= 0.0 ||
      config.max_jerk_mps3 <= 0.0) {
    *reason = "dynamic limits must be positive";
    return false;
  }
  if (config.line_length_m <= 0.0 || config.line_segment_duration_sec <= 0.0 ||
      config.circle_radius_m <= 0.0 || config.circle_tangent_speed_mps <= 0.0 ||
      config.transition_duration_sec <= 0.0 || config.figure8_amplitude_x_m <= 0.0 ||
      config.figure8_amplitude_y_m <= 0.0 || config.low_speed_yaw_threshold_mps < 0.0) {
    *reason = "shape parameters are outside valid ranges";
    return false;
  }
  return true;
}

std::string makeTraceId(const DynamicTrajectoryConfig& config) {
  std::ostringstream out;
  out << trajectoryTypeName(config.trajectory_type)
      << "_dur" << formatDouble(config.duration_sec)
      << "_dt" << formatDouble(config.sample_period_sec)
      << "_z" << formatDouble(config.altitude_offset_m)
      << "_hold0" << formatDouble(config.initial_hold_sec)
      << "_climb" << formatDouble(config.initial_climb_duration_sec)
      << "_holdz" << formatDouble(config.post_climb_hold_sec)
      << "_yaw" << yawModeName(config.yaw_mode);
  switch (config.trajectory_type) {
    case DynamicTrajectoryType::kLine:
      out << "_L" << formatDouble(config.line_length_m)
          << "_seg" << formatDouble(config.line_segment_duration_sec);
      break;
    case DynamicTrajectoryType::kCircle:
      out << "_R" << formatDouble(config.circle_radius_m)
          << "_vt" << formatDouble(config.circle_tangent_speed_mps);
      break;
    case DynamicTrajectoryType::kFigure8:
      out << "_A" << formatDouble(config.figure8_amplitude_x_m)
          << "_B" << formatDouble(config.figure8_amplitude_y_m);
      break;
  }
  return out.str();
}

void assignYaw(std::vector<FlatSample>* samples, const DynamicTrajectoryConfig& config,
               const StartPose& start_pose) {
  double previous_yaw = start_pose.yaw;
  for (std::size_t i = 0; i < samples->size(); ++i) {
    auto& sample = (*samples)[i];
    if (config.yaw_mode == YawMode::kFixed) {
      sample.yaw = start_pose.yaw;
      sample.yaw_rate = 0.0;
      continue;
    }
    const double speed_xy = std::hypot(sample.vx, sample.vy);
    if (speed_xy >= config.low_speed_yaw_threshold_mps) {
      const double desired = unwrapNear(previous_yaw, std::atan2(sample.vy, sample.vx));
      const double max_step = 0.25;
      const double delta = desired - previous_yaw;
      if (std::abs(delta) > max_step) {
        sample.yaw = previous_yaw + std::copysign(max_step, delta);
      } else {
        sample.yaw = desired;
      }
      previous_yaw = sample.yaw;
    } else {
      sample.yaw = previous_yaw;
    }
  }
  if (config.yaw_mode == YawMode::kVelocityAligned) {
    for (std::size_t i = 0; i < samples->size(); ++i) {
      if (i == 0 || i + 1 >= samples->size()) {
        samples->at(i).yaw_rate = 0.0;
      } else {
        const double dyaw = samples->at(i + 1).yaw - samples->at(i - 1).yaw;
        const double dt = samples->at(i + 1).t - samples->at(i - 1).t;
        samples->at(i).yaw_rate = dt > kEpsilon ? dyaw / dt : 0.0;
      }
    }
  }
}

std::vector<FlatSample> generateLineSamples(const DynamicTrajectoryConfig& config,
                                            const StartPose& start_pose,
                                            const double time_scale) {
  std::vector<FlatSample> samples;
  const double z = start_pose.z + config.altitude_offset_m;
  const double tseg = config.line_segment_duration_sec * time_scale;
  double t = 0.0;
  if (config.initial_hold_sec > 0.0) {
    FlatSample initial;
    initial.t = 0.0;
    initial.x = start_pose.x;
    initial.y = start_pose.y;
    initial.z = start_pose.z;
    samples.push_back(initial);
    addHold(&samples, initial, config.initial_hold_sec * time_scale, config.sample_period_sec);
    t += config.initial_hold_sec * time_scale;
  }
  if (config.initial_climb_duration_sec > 0.0) {
    const double climb = config.initial_climb_duration_sec * time_scale;
    addSmoothSegment(&samples, t, climb, start_pose.x, start_pose.y, start_pose.z,
                     start_pose.x, start_pose.y, z, config.sample_period_sec);
    t += climb;
    addHold(&samples, samples.back(), config.post_climb_hold_sec * time_scale,
            config.sample_period_sec);
    t += config.post_climb_hold_sec * time_scale;
  }
  addSmoothSegment(&samples, t, tseg, start_pose.x, start_pose.y, z,
                   start_pose.x + config.line_length_m, start_pose.y, z,
                   config.sample_period_sec);
  t += tseg;
  addSmoothSegment(&samples, t, tseg, start_pose.x + config.line_length_m, start_pose.y, z,
                   start_pose.x - config.line_length_m, start_pose.y, z,
                   config.sample_period_sec);
  t += tseg;
  addSmoothSegment(&samples, t, tseg, start_pose.x - config.line_length_m, start_pose.y, z,
                   start_pose.x, start_pose.y, z, config.sample_period_sec);
  addHold(&samples, samples.back(), config.hold_end_sec, config.sample_period_sec);
  return samples;
}

std::vector<FlatSample> generateCircleSamples(const DynamicTrajectoryConfig& config,
                                              const StartPose& start_pose,
                                              const double time_scale) {
  std::vector<FlatSample> samples;
  const double z = start_pose.z + config.altitude_offset_m;
  const double r = config.circle_radius_m;
  const double transition = config.transition_duration_sec * time_scale;
  const double circle_duration =
      std::max(config.duration_sec * time_scale - 2.0 * transition,
               kTwoPi * r / config.circle_tangent_speed_mps * time_scale);
  double t = 0.0;
  if (config.initial_hold_sec > 0.0) {
    FlatSample initial;
    initial.t = 0.0;
    initial.x = start_pose.x;
    initial.y = start_pose.y;
    initial.z = start_pose.z;
    samples.push_back(initial);
    addHold(&samples, initial, config.initial_hold_sec * time_scale, config.sample_period_sec);
    t += config.initial_hold_sec * time_scale;
  }
  if (config.initial_climb_duration_sec > 0.0) {
    const double climb = config.initial_climb_duration_sec * time_scale;
    addSmoothSegment(&samples, t, climb, start_pose.x, start_pose.y, start_pose.z,
                     start_pose.x, start_pose.y, z, config.sample_period_sec);
    t += climb;
    addHold(&samples, samples.back(), config.post_climb_hold_sec * time_scale,
            config.sample_period_sec);
    t += config.post_climb_hold_sec * time_scale;
  }
  addSmoothSegment(&samples, t, transition, start_pose.x, start_pose.y, z,
                   start_pose.x + r, start_pose.y, z, config.sample_period_sec);
  t += transition;
  const std::size_t steps =
      std::max<std::size_t>(8, static_cast<std::size_t>(std::ceil(circle_duration / config.sample_period_sec)));
  for (std::size_t i = 1; i <= steps; ++i) {
    const double local_t = circle_duration * static_cast<double>(i) / static_cast<double>(steps);
    const double u = local_t / circle_duration;
    const double theta = kTwoPi * smooth5(u);
    const double omega = kTwoPi * dsmooth5(u) / circle_duration;
    const double alpha = kTwoPi * d2smooth5(u) / (circle_duration * circle_duration);
    FlatSample sample;
    sample.t = t + local_t;
    sample.x = start_pose.x + r * std::cos(theta);
    sample.y = start_pose.y + r * std::sin(theta);
    sample.z = z;
    sample.vx = -r * omega * std::sin(theta);
    sample.vy = r * omega * std::cos(theta);
    sample.vz = 0.0;
    sample.ax = -r * (alpha * std::sin(theta) + omega * omega * std::cos(theta));
    sample.ay = r * (alpha * std::cos(theta) - omega * omega * std::sin(theta));
    sample.az = 0.0;
    samples.push_back(sample);
  }
  t += circle_duration;
  addSmoothSegment(&samples, t, transition, start_pose.x + r, start_pose.y, z,
                   start_pose.x, start_pose.y, z, config.sample_period_sec);
  addHold(&samples, samples.back(), config.hold_end_sec, config.sample_period_sec);
  return samples;
}

FlatSample figure8At(const DynamicTrajectoryConfig& config, const StartPose& start_pose,
                     const double t, const double total_duration) {
  const double a = config.figure8_amplitude_x_m;
  const double b = config.figure8_amplitude_y_m;
  const double u = t / total_duration;
  const double theta = kTwoPi * smooth5(u);
  const double omega = kTwoPi * dsmooth5(u) / total_duration;
  const double alpha = kTwoPi * d2smooth5(u) / (total_duration * total_duration);
  FlatSample sample;
  sample.t = t;
  sample.x = start_pose.x + a * std::sin(theta);
  sample.y = start_pose.y + b * std::sin(2.0 * theta);
  sample.z = start_pose.z + config.altitude_offset_m;
  sample.vx = a * omega * std::cos(theta);
  sample.vy = 2.0 * b * omega * std::cos(2.0 * theta);
  sample.vz = 0.0;
  sample.ax = a * (alpha * std::cos(theta) - omega * omega * std::sin(theta));
  sample.ay = 2.0 * b * (alpha * std::cos(2.0 * theta) -
                          2.0 * omega * omega * std::sin(2.0 * theta));
  sample.az = 0.0;
  return sample;
}

std::vector<FlatSample> generateFigure8Samples(const DynamicTrajectoryConfig& config,
                                               const StartPose& start_pose,
                                               const double time_scale) {
  std::vector<FlatSample> samples;
  const double z = start_pose.z + config.altitude_offset_m;
  const double active = config.duration_sec * time_scale;
  const double transition = config.transition_duration_sec * time_scale;
  double t = 0.0;
  if (config.initial_hold_sec > 0.0) {
    FlatSample initial;
    initial.t = 0.0;
    initial.x = start_pose.x;
    initial.y = start_pose.y;
    initial.z = start_pose.z;
    samples.push_back(initial);
    addHold(&samples, initial, config.initial_hold_sec * time_scale, config.sample_period_sec);
    t += config.initial_hold_sec * time_scale;
  }
  if (config.initial_climb_duration_sec > 0.0) {
    const double climb = config.initial_climb_duration_sec * time_scale;
    addSmoothSegment(&samples, t, climb, start_pose.x, start_pose.y, start_pose.z,
                     start_pose.x, start_pose.y, z, config.sample_period_sec);
    t += climb;
    addHold(&samples, samples.back(), config.post_climb_hold_sec * time_scale,
            config.sample_period_sec);
    t += config.post_climb_hold_sec * time_scale;
  }
  FlatSample first = figure8At(config, start_pose, 0.0, active);
  addSmoothSegment(&samples, t, transition, start_pose.x, start_pose.y, z,
                   first.x, first.y, z, config.sample_period_sec);
  t += transition;
  const std::size_t steps =
      std::max<std::size_t>(8, static_cast<std::size_t>(std::ceil(active / config.sample_period_sec)));
  for (std::size_t i = 1; i <= steps; ++i) {
    FlatSample sample = figure8At(config, start_pose,
                                  active * static_cast<double>(i) / static_cast<double>(steps),
                                  active);
    sample.t += t;
    samples.push_back(sample);
  }
  t += active;
  const FlatSample last = samples.back();
  addSmoothSegment(&samples, t, transition, last.x, last.y, z,
                   start_pose.x, start_pose.y, z, config.sample_period_sec);
  addHold(&samples, samples.back(), config.hold_end_sec, config.sample_period_sec);
  return samples;
}

uav_msgs::Trajectory makeMessage(const DynamicTrajectoryConfig& config,
                                 const StartPose& start_pose,
                                 const std::vector<FlatSample>& samples,
                                 const std::string& trace_id) {
  uav_msgs::Trajectory trajectory;
  trajectory.header.stamp = ros::Time(config.start_delay_sec);
  trajectory.header.frame_id = config.frame_id;
  trajectory.mode = uav_msgs::Trajectory::MODE_NOMINAL;
  trajectory.trajectory_id = deterministicId(trace_id);
  trajectory.points.reserve(samples.size());
  for (const auto& sample : samples) {
    uav_msgs::TrajectoryPoint point;
    point.time_from_start = ros::Duration(sample.t);
    point.position.x = sample.x;
    point.position.y = sample.y;
    point.position.z = sample.z;
    point.velocity.x = sample.vx;
    point.velocity.y = sample.vy;
    point.velocity.z = sample.vz;
    point.acceleration.x = sample.ax;
    point.acceleration.y = sample.ay;
    point.acceleration.z = sample.az;
    point.yaw = sample.yaw;
    point.yaw_rate = sample.yaw_rate;
    trajectory.points.push_back(point);
  }
  return trajectory;
}

std::vector<FlatSample> generateSamples(const DynamicTrajectoryConfig& config,
                                        const StartPose& start_pose,
                                        const double time_scale) {
  switch (config.trajectory_type) {
    case DynamicTrajectoryType::kLine:
      return generateLineSamples(config, start_pose, time_scale);
    case DynamicTrajectoryType::kCircle:
      return generateCircleSamples(config, start_pose, time_scale);
    case DynamicTrajectoryType::kFigure8:
      return generateFigure8Samples(config, start_pose, time_scale);
  }
  return {};
}

double norm3(const double x, const double y, const double z) {
  return std::sqrt(x * x + y * y + z * z);
}

}  // namespace

bool parseTrajectoryType(const std::string& value, DynamicTrajectoryType* type) {
  if (value == "line") {
    *type = DynamicTrajectoryType::kLine;
    return true;
  }
  if (value == "circle") {
    *type = DynamicTrajectoryType::kCircle;
    return true;
  }
  if (value == "figure8" || value == "figure_8") {
    *type = DynamicTrajectoryType::kFigure8;
    return true;
  }
  return false;
}

bool parseYawMode(const std::string& value, YawMode* mode) {
  if (value == "fixed") {
    *mode = YawMode::kFixed;
    return true;
  }
  if (value == "velocity_aligned") {
    *mode = YawMode::kVelocityAligned;
    return true;
  }
  return false;
}

std::string trajectoryTypeName(const DynamicTrajectoryType type) {
  switch (type) {
    case DynamicTrajectoryType::kLine:
      return "line";
    case DynamicTrajectoryType::kCircle:
      return "circle";
    case DynamicTrajectoryType::kFigure8:
      return "figure8";
  }
  return "unknown";
}

std::string yawModeName(const YawMode mode) {
  return mode == YawMode::kVelocityAligned ? "velocity_aligned" : "fixed";
}

double shortestAngleDelta(const double from, const double to) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

double unwrapNear(const double reference, const double value) {
  return reference + shortestAngleDelta(reference, value);
}

bool validateDynamicTrajectory(const uav_msgs::Trajectory& trajectory,
                               const DynamicTrajectoryConfig& config,
                               DynamicTrajectoryLimits* measured,
                               std::string* reason) {
  if (trajectory.header.frame_id != config.frame_id) {
    *reason = "trajectory frame does not match config frame";
    return false;
  }
  if (trajectory.points.size() < 2) {
    *reason = "trajectory must contain at least two points";
    return false;
  }
  DynamicTrajectoryLimits local;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto& point = trajectory.points[i];
    if (!finitePoint(point)) {
      *reason = "trajectory contains NaN or Inf";
      return false;
    }
    if (i > 0 && point.time_from_start <= trajectory.points[i - 1].time_from_start) {
      *reason = "trajectory time_from_start values are not strictly increasing";
      return false;
    }
    local.max_velocity_mps =
        std::max(local.max_velocity_mps,
                 norm3(point.velocity.x, point.velocity.y, point.velocity.z));
    local.max_acceleration_mps2 =
        std::max(local.max_acceleration_mps2,
                 norm3(point.acceleration.x, point.acceleration.y, point.acceleration.z));
    if (i > 0) {
      const double dt =
          (point.time_from_start - trajectory.points[i - 1].time_from_start).toSec();
      const auto& previous = trajectory.points[i - 1];
      const double jerk = norm3((point.acceleration.x - previous.acceleration.x) / dt,
                                (point.acceleration.y - previous.acceleration.y) / dt,
                                (point.acceleration.z - previous.acceleration.z) / dt);
      local.max_jerk_mps3 = std::max(local.max_jerk_mps3, jerk);
      const double previous_speed =
          norm3(previous.velocity.x, previous.velocity.y, previous.velocity.z);
      const double current_speed =
          norm3(point.velocity.x, point.velocity.y, point.velocity.z);
      if (previous_speed >= config.low_speed_yaw_threshold_mps &&
          current_speed >= config.low_speed_yaw_threshold_mps &&
          std::abs(point.yaw - previous.yaw) > kPi / 2.0) {
        *reason = "yaw has a discontinuous jump";
        return false;
      }
    }
  }
  if (measured != nullptr) {
    *measured = local;
  }
  if (local.max_velocity_mps > config.max_velocity_mps * (1.0 + 1e-6)) {
    *reason = "trajectory exceeds max velocity";
    return false;
  }
  if (local.max_acceleration_mps2 > config.max_acceleration_mps2 * (1.0 + 1e-6)) {
    *reason = "trajectory exceeds max acceleration";
    return false;
  }
  if (local.max_jerk_mps3 > config.max_jerk_mps3 * (1.0 + 1e-6)) {
    *reason = "trajectory exceeds max jerk";
    return false;
  }
  return true;
}

DynamicTrajectoryResult generateDynamicTrajectory(const DynamicTrajectoryConfig& config,
                                                  const StartPose& start_pose) {
  DynamicTrajectoryResult result;
  std::string reason;
  if (!validateConfig(config, &reason)) {
    result.reason = reason;
    return result;
  }
  if (!finite(start_pose.x) || !finite(start_pose.y) || !finite(start_pose.z) ||
      !finite(start_pose.yaw)) {
    result.reason = "start pose must be finite";
    return result;
  }

  result.trace_id = makeTraceId(config);
  double time_scale = 1.0;
  for (int attempt = 0; attempt < 16; ++attempt) {
    auto samples = generateSamples(config, start_pose, time_scale);
    assignYaw(&samples, config, start_pose);
    auto trajectory = makeMessage(config, start_pose, samples, result.trace_id);
    DynamicTrajectoryLimits measured;
    if (validateDynamicTrajectory(trajectory, config, &measured, &reason)) {
      result.valid = true;
      result.reason = "ok";
      result.time_scale = time_scale;
      result.measured = measured;
      result.trajectory = trajectory;
      return result;
    }
    const double velocity_scale = measured.max_velocity_mps > 0.0
                                      ? measured.max_velocity_mps / config.max_velocity_mps
                                      : 1.0;
    const double acceleration_scale = measured.max_acceleration_mps2 > 0.0
                                          ? std::sqrt(measured.max_acceleration_mps2 /
                                                      config.max_acceleration_mps2)
                                          : 1.0;
    const double jerk_scale = measured.max_jerk_mps3 > 0.0
                                  ? std::cbrt(measured.max_jerk_mps3 / config.max_jerk_mps3)
                                  : 1.0;
    const double extra_scale = std::max({1.0, velocity_scale, acceleration_scale, jerk_scale});
    const double next_scale = time_scale * extra_scale * 1.05;
    if (!finite(next_scale) || next_scale <= time_scale + 1e-6) {
      result.reason = reason;
      return result;
    }
    time_scale = next_scale;
  }
  result.reason = reason;
  return result;
}

}  // namespace uav_trajectory
