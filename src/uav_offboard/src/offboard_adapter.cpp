#include "uav_offboard/offboard_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace uav_offboard {
namespace {

bool finite(const double value) {
  return std::isfinite(value);
}

bool supportedFrame(const std::string& frame,
                    const std::vector<std::string>& supported_frames) {
  return std::find(supported_frames.begin(), supported_frames.end(), frame) !=
         supported_frames.end();
}

bool timedOut(const ros::Time& stamp,
              const ros::Time& now,
              const ros::Duration& timeout) {
  if (stamp.isZero()) {
    return true;
  }
  if (timeout.toSec() < 0.0) {
    return false;
  }
  if (now < stamp) {
    return true;
  }
  return (now - stamp) > timeout;
}

}  // namespace

bool validateConfig(AdapterConfig* config, std::string* warning) {
  std::ostringstream warnings;
  bool adjusted = false;

  if (!std::isfinite(config->publish_rate_hz) || config->publish_rate_hz < 1.0 ||
      config->publish_rate_hz > 100.0) {
    warnings << "publish_rate_hz out of range; using 30.0. ";
    config->publish_rate_hz = 30.0;
    adjusted = true;
  }
  if (!std::isfinite(config->preview_timeout.toSec()) ||
      config->preview_timeout.toSec() <= 0.0) {
    warnings << "preview_timeout invalid; using 0.2. ";
    config->preview_timeout = ros::Duration(0.2);
    adjusted = true;
  }
  if (!std::isfinite(config->state_timeout.toSec()) ||
      config->state_timeout.toSec() <= 0.0) {
    warnings << "state_timeout invalid; using 0.2. ";
    config->state_timeout = ros::Duration(0.2);
    adjusted = true;
  }
  if (config->supported_frames.empty()) {
    warnings << "supported_frames empty; using map. ";
    config->supported_frames = {"map"};
    adjusted = true;
  }
  if (config->mavros_output_topic.empty()) {
    warnings << "mavros_output_topic empty; using /mavros/setpoint_raw/local. ";
    config->mavros_output_topic = "/mavros/setpoint_raw/local";
    adjusted = true;
  }
  if (warning != nullptr) {
    *warning = warnings.str();
  }
  return !adjusted;
}

bool isFinite(const uav_msgs::SetpointPreview& preview) {
  const auto& point = preview.point;
  const auto& p = point.position;
  const auto& v = point.velocity;
  const auto& a = point.acceleration;
  return finite(p.x) && finite(p.y) && finite(p.z) &&
         finite(v.x) && finite(v.y) && finite(v.z) &&
         finite(a.x) && finite(a.y) && finite(a.z) &&
         finite(point.yaw) && finite(point.yaw_rate);
}

HealthReport evaluateHealth(const AdapterInputs& inputs,
                            const AdapterConfig& config,
                            const ros::Time& now,
                            const ros::Time& previous_update_time) {
  HealthReport report;
  report.preview_received = inputs.preview_received;
  report.mavros_connected =
      inputs.mavros_state_received && inputs.mavros_state.connected;

  if (!previous_update_time.isZero() && now < previous_update_time) {
    report.reason = "ROS time moved backward";
    return report;
  }
  if (!inputs.preview_received) {
    report.reason = "waiting for setpoint preview";
    return report;
  }
  if (!inputs.vehicle_state_received) {
    report.reason = "waiting for vehicle state";
    return report;
  }
  if (!inputs.mavros_state_received) {
    report.reason = "waiting for MAVROS state";
    return report;
  }
  if (!inputs.preview.trajectory_valid) {
    report.reason = "preview trajectory is invalid";
    return report;
  }
  if (!inputs.preview.started) {
    report.reason = "trajectory has not started";
    return report;
  }
  if (inputs.preview.finished) {
    report.reason = "trajectory is finished";
    return report;
  }
  if (!inputs.preview.state_fresh) {
    report.reason = "preview reports stale vehicle state";
    return report;
  }
  if (inputs.preview.header.stamp.isZero()) {
    report.reason = "preview stamp is zero";
    return report;
  }
  report.preview_fresh =
      !timedOut(inputs.preview.header.stamp, now, config.preview_timeout);
  if (!report.preview_fresh) {
    report.reason = "preview timeout";
    return report;
  }
  report.vehicle_state_fresh =
      !timedOut(inputs.vehicle_state.header.stamp, now, config.state_timeout);
  if (!report.vehicle_state_fresh) {
    report.reason = "vehicle state timeout";
    return report;
  }
  if (!inputs.vehicle_state.pose_valid) {
    report.reason = "vehicle pose invalid";
    return report;
  }
  if (!inputs.vehicle_state.twist_valid) {
    report.reason = "vehicle twist invalid";
    return report;
  }
  if (!report.mavros_connected) {
    report.reason = "MAVROS disconnected";
    return report;
  }
  if (!supportedFrame(inputs.preview.header.frame_id, config.supported_frames) ||
      inputs.preview.header.frame_id != inputs.vehicle_state.header.frame_id) {
    report.reason = "frame mismatch or unsupported frame";
    return report;
  }
  if (!isFinite(inputs.preview)) {
    report.reason = "preview contains NaN or Inf";
    return report;
  }

  report.healthy = true;
  report.reason = "healthy";
  return report;
}

bool canEnableRuntimeOutput(const AdapterConfig& config,
                            const HealthReport& health,
                            std::string* reason) {
  if (!config.allow_mavros_output) {
    if (reason != nullptr) {
      *reason = "static gate disabled";
    }
    return false;
  }
  if (!health.healthy) {
    if (reason != nullptr) {
      *reason = health.reason;
    }
    return false;
  }
  if (reason != nullptr) {
    *reason = "enabled";
  }
  return true;
}

AdapterState decideState(const AdapterConfig& config,
                         const GateState& gates,
                         const HealthReport& health) {
  if (!config.allow_mavros_output || !gates.runtime_output_enabled) {
    return health.healthy ? AdapterState::READY_DRY_RUN : AdapterState::DISABLED;
  }
  if (health.healthy) {
    return AdapterState::STREAMING;
  }
  if (!health.preview_received || !health.mavros_connected) {
    return AdapterState::WAITING_INPUTS;
  }
  return AdapterState::FAULT;
}

std::string stateName(const AdapterState state) {
  switch (state) {
    case AdapterState::DISABLED:
      return "DISABLED";
    case AdapterState::WAITING_INPUTS:
      return "WAITING_INPUTS";
    case AdapterState::READY_DRY_RUN:
      return "READY_DRY_RUN";
    case AdapterState::STREAMING:
      return "STREAMING";
    case AdapterState::FAULT:
      return "FAULT";
  }
  return "UNKNOWN";
}

uav_msgs::OffboardStatus makeStatus(const AdapterConfig& config,
                                    const GateState& gates,
                                    const HealthReport& health,
                                    const AdapterState state,
                                    const ros::Time& stamp) {
  uav_msgs::OffboardStatus status;
  status.header.stamp = stamp;
  status.header.frame_id = "";
  status.state = static_cast<uint8_t>(state);
  status.state_name = stateName(state);
  status.static_gate_allowed = config.allow_mavros_output;
  status.runtime_gate_enabled = gates.runtime_output_enabled;
  status.preview_received = health.preview_received;
  status.preview_fresh = health.preview_fresh;
  status.vehicle_state_fresh = health.vehicle_state_fresh;
  status.mavros_connected = health.mavros_connected;
  status.output_active = state == AdapterState::STREAMING;
  status.reason = health.reason;
  return status;
}

mavros_msgs::PositionTarget previewToPositionTarget(
    const uav_msgs::SetpointPreview& preview,
    const ros::Time& publish_time,
    const uint8_t coordinate_frame) {
  mavros_msgs::PositionTarget target;
  target.header.stamp = publish_time;
  target.header.frame_id = preview.header.frame_id;
  target.coordinate_frame = coordinate_frame;
  target.type_mask = 0;
  target.position = preview.point.position;
  target.velocity = preview.point.velocity;
  target.acceleration_or_force = preview.point.acceleration;
  target.yaw = static_cast<float>(preview.point.yaw);
  target.yaw_rate = static_cast<float>(preview.point.yaw_rate);
  return target;
}

}  // namespace uav_offboard
