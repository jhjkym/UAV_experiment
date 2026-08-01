#include "uav_trajectory/trajectory_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace uav_trajectory {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

bool finite(const double value) {
  return std::isfinite(value);
}

bool supportedFrame(const std::string& frame,
                    const std::vector<std::string>& supported_frames) {
  return std::find(supported_frames.begin(), supported_frames.end(), frame) !=
         supported_frames.end();
}

double durationSec(const ros::Duration& duration) {
  return duration.toSec();
}

double hermiteValue(const double p0,
                    const double v0,
                    const double p1,
                    const double v1,
                    const double dt,
                    const double s) {
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  return h00 * p0 + h10 * dt * v0 + h01 * p1 + h11 * dt * v1;
}

double hermiteVelocity(const double p0,
                       const double v0,
                       const double p1,
                       const double v1,
                       const double dt,
                       const double s) {
  const double s2 = s * s;
  const double dh00 = 6.0 * s2 - 6.0 * s;
  const double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
  const double dh01 = -6.0 * s2 + 6.0 * s;
  const double dh11 = 3.0 * s2 - 2.0 * s;
  return (dh00 * p0 + dh10 * dt * v0 + dh01 * p1 + dh11 * dt * v1) / dt;
}

double hermiteAcceleration(const double p0,
                           const double v0,
                           const double p1,
                           const double v1,
                           const double dt,
                           const double s) {
  const double d2h00 = 12.0 * s - 6.0;
  const double d2h10 = 6.0 * s - 4.0;
  const double d2h01 = -12.0 * s + 6.0;
  const double d2h11 = 6.0 * s - 2.0;
  return (d2h00 * p0 + d2h10 * dt * v0 + d2h01 * p1 + d2h11 * dt * v1) /
         (dt * dt);
}

void sampleScalar(const double p0,
                  const double v0,
                  const double p1,
                  const double v1,
                  const double dt,
                  const double s,
                  double* value,
                  double* velocity,
                  double* acceleration) {
  *value = hermiteValue(p0, v0, p1, v1, dt, s);
  *velocity = hermiteVelocity(p0, v0, p1, v1, dt, s);
  *acceleration = hermiteAcceleration(p0, v0, p1, v1, dt, s);
}

}  // namespace

double normalizeAngle(double angle) {
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double shortestAngleDelta(const double from, const double to) {
  return std::atan2(std::sin(to - from), std::cos(to - from));
}

geometry_msgs::Quaternion yawToQuaternion(const double yaw) {
  geometry_msgs::Quaternion q;
  const double half_yaw = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half_yaw);
  q.w = std::cos(half_yaw);
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (norm > 0.0) {
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  }
  return q;
}

bool isFinite(const uav_msgs::TrajectoryPoint& point) {
  const auto& p = point.position;
  const auto& v = point.velocity;
  const auto& a = point.acceleration;
  return finite(point.time_from_start.toSec()) &&
         finite(p.x) && finite(p.y) && finite(p.z) &&
         finite(v.x) && finite(v.y) && finite(v.z) &&
         finite(a.x) && finite(a.y) && finite(a.z) &&
         finite(point.yaw) && finite(point.yaw_rate);
}

ValidationResult validateTrajectory(const uav_msgs::Trajectory& trajectory,
                                    const std::vector<std::string>& supported_frames) {
  if (trajectory.points.empty()) {
    return {false, "trajectory has no points"};
  }
  if (trajectory.header.frame_id.empty()) {
    return {false, "trajectory frame_id is empty"};
  }
  if (!supportedFrame(trajectory.header.frame_id, supported_frames)) {
    return {false, "unsupported trajectory frame_id: " + trajectory.header.frame_id};
  }

  double previous_time = -1.0;
  for (std::size_t i = 0; i < trajectory.points.size(); ++i) {
    const auto& point = trajectory.points[i];
    if (!isFinite(point)) {
      return {false, "trajectory point contains non-finite value"};
    }
    const double t = durationSec(point.time_from_start);
    if (t < 0.0) {
      return {false, "trajectory point time is negative"};
    }
    if (i == 0) {
      if (t != 0.0) {
        return {false, "first trajectory point must start at zero relative time"};
      }
    } else if (t <= previous_time) {
      return {false, "trajectory point times must be strictly increasing"};
    }
    previous_time = t;
  }
  return {true, ""};
}

SampleResult sampleTrajectory(const uav_msgs::Trajectory& trajectory,
                              const ros::Time& now,
                              const ros::Time& previous_sample_time) {
  SampleResult result;
  if (trajectory.points.empty()) {
    return result;
  }
  result.has_trajectory = true;
  result.time_went_back =
      !previous_sample_time.isZero() && !now.isZero() && now < previous_sample_time;

  if (trajectory.points.size() == 1) {
    result.point = trajectory.points.front();
    const ros::Duration elapsed = now - trajectory.header.stamp;
    result.started = elapsed.toSec() >= 0.0;
    result.finished = result.started;
    return result;
  }

  const ros::Duration elapsed_duration = now - trajectory.header.stamp;
  const double elapsed = elapsed_duration.toSec();
  if (result.time_went_back || elapsed <= 0.0) {
    result.point = trajectory.points.front();
    result.started = false;
    result.finished = false;
    return result;
  }

  const auto& points = trajectory.points;
  const double final_time = points.back().time_from_start.toSec();
  if (elapsed >= final_time) {
    result.point = points.back();
    result.started = true;
    result.finished = true;
    return result;
  }

  std::size_t segment = 0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double t0 = points[i].time_from_start.toSec();
    const double t1 = points[i + 1].time_from_start.toSec();
    if (elapsed >= t0 && elapsed < t1) {
      segment = i;
      break;
    }
  }

  const auto& p0 = points[segment];
  const auto& p1 = points[segment + 1];
  const double t0 = p0.time_from_start.toSec();
  const double t1 = p1.time_from_start.toSec();
  const double dt = t1 - t0;
  const double s = (elapsed - t0) / dt;

  result.point.time_from_start = ros::Duration(elapsed);
  sampleScalar(p0.position.x, p0.velocity.x, p1.position.x, p1.velocity.x, dt, s,
               &result.point.position.x, &result.point.velocity.x,
               &result.point.acceleration.x);
  sampleScalar(p0.position.y, p0.velocity.y, p1.position.y, p1.velocity.y, dt, s,
               &result.point.position.y, &result.point.velocity.y,
               &result.point.acceleration.y);
  sampleScalar(p0.position.z, p0.velocity.z, p1.position.z, p1.velocity.z, dt, s,
               &result.point.position.z, &result.point.velocity.z,
               &result.point.acceleration.z);

  const double yaw1_unwrapped = p0.yaw + shortestAngleDelta(p0.yaw, p1.yaw);
  double yaw_acceleration = 0.0;
  sampleScalar(p0.yaw, p0.yaw_rate, yaw1_unwrapped, p1.yaw_rate, dt, s,
               &result.point.yaw, &result.point.yaw_rate, &yaw_acceleration);
  result.point.yaw = normalizeAngle(result.point.yaw);

  result.started = true;
  result.finished = false;
  return result;
}

uav_msgs::SetpointPreview makePreviewMessage(const uav_msgs::Trajectory& trajectory,
                                             const SampleResult& sample,
                                             const bool state_fresh) {
  uav_msgs::SetpointPreview preview;
  preview.header.stamp = trajectory.header.stamp + sample.point.time_from_start;
  preview.header.frame_id = trajectory.header.frame_id;
  preview.point = sample.point;
  preview.trajectory_valid = sample.has_trajectory;
  preview.started = sample.started;
  preview.finished = sample.finished;
  preview.state_fresh = state_fresh;
  preview.trajectory_id = trajectory.trajectory_id;
  return preview;
}

bool isStateFresh(const uav_msgs::UavState& state,
                  const ros::Time& now,
                  const ros::Duration& timeout) {
  if (timeout.toSec() < 0.0) {
    return true;
  }
  if (state.header.stamp.isZero()) {
    return false;
  }
  if (!state.pose_valid || !state.twist_valid) {
    return false;
  }
  return (now - state.header.stamp) <= timeout;
}

bool TrajectoryCache::replaceIfValid(const uav_msgs::Trajectory& trajectory,
                                     const std::vector<std::string>& supported_frames,
                                     std::string* rejection_reason) {
  const auto validation = validateTrajectory(trajectory, supported_frames);
  if (!validation.valid) {
    if (rejection_reason != nullptr) {
      *rejection_reason = validation.reason;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  trajectory_ = trajectory;
  has_trajectory_ = true;
  has_pending_trajectory_ = false;
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  return true;
}

bool TrajectoryCache::queueOrReplaceIfValid(const uav_msgs::Trajectory& trajectory,
                                            const std::vector<std::string>& supported_frames,
                                            const ros::Time& now,
                                            std::string* rejection_reason,
                                            bool* queued_pending) {
  const auto validation = validateTrajectory(trajectory, supported_frames);
  if (!validation.valid) {
    if (rejection_reason != nullptr) {
      *rejection_reason = validation.reason;
    }
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (has_trajectory_ && trajectory.header.stamp > now) {
    pending_trajectory_ = trajectory;
    has_pending_trajectory_ = true;
    if (queued_pending != nullptr) {
      *queued_pending = true;
    }
  } else {
    trajectory_ = trajectory;
    has_trajectory_ = true;
    has_pending_trajectory_ = false;
    if (queued_pending != nullptr) {
      *queued_pending = false;
    }
  }
  if (rejection_reason != nullptr) {
    rejection_reason->clear();
  }
  return true;
}

bool TrajectoryCache::get(uav_msgs::Trajectory* trajectory) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_trajectory_) {
    return false;
  }
  *trajectory = trajectory_;
  return true;
}

bool TrajectoryCache::getActiveForTime(const ros::Time& now,
                                       uav_msgs::Trajectory* trajectory,
                                       bool* promoted_pending) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_pending_trajectory_ && pending_trajectory_.header.stamp <= now) {
    trajectory_ = pending_trajectory_;
    has_trajectory_ = true;
    has_pending_trajectory_ = false;
    if (promoted_pending != nullptr) {
      *promoted_pending = true;
    }
  } else if (promoted_pending != nullptr) {
    *promoted_pending = false;
  }
  if (!has_trajectory_) {
    return false;
  }
  *trajectory = trajectory_;
  return true;
}

}  // namespace uav_trajectory
