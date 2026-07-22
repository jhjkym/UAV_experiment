#ifndef UAV_OFFBOARD_OFFBOARD_ADAPTER_HPP_
#define UAV_OFFBOARD_OFFBOARD_ADAPTER_HPP_

#include <string>
#include <vector>

#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <uav_msgs/OffboardStatus.h>
#include <uav_msgs/SetpointPreview.h>
#include <uav_msgs/UavState.h>

namespace uav_offboard {

struct AdapterConfig {
  double publish_rate_hz = 30.0;
  ros::Duration preview_timeout{0.2};
  ros::Duration state_timeout{0.2};
  bool allow_mavros_output = false;
  std::vector<std::string> supported_frames{"map"};
  std::string mavros_output_topic{"/mavros/setpoint_raw/local"};
};

struct AdapterInputs {
  bool preview_received = false;
  bool vehicle_state_received = false;
  bool mavros_state_received = false;
  uav_msgs::SetpointPreview preview;
  uav_msgs::UavState vehicle_state;
  mavros_msgs::State mavros_state;
};

struct HealthReport {
  bool healthy = false;
  bool preview_fresh = false;
  bool vehicle_state_fresh = false;
  bool mavros_connected = false;
  bool preview_received = false;
  std::string reason;
};

struct GateState {
  bool runtime_output_enabled = false;
};

enum class AdapterState : uint8_t {
  DISABLED = uav_msgs::OffboardStatus::DISABLED,
  WAITING_INPUTS = uav_msgs::OffboardStatus::WAITING_INPUTS,
  READY_DRY_RUN = uav_msgs::OffboardStatus::READY_DRY_RUN,
  STREAMING = uav_msgs::OffboardStatus::STREAMING,
  FAULT = uav_msgs::OffboardStatus::FAULT,
};

bool validateConfig(AdapterConfig* config, std::string* warning);

bool canEnableRuntimeOutput(const AdapterConfig& config,
                            const HealthReport& health,
                            std::string* reason);

HealthReport evaluateHealth(const AdapterInputs& inputs,
                            const AdapterConfig& config,
                            const ros::Time& now,
                            const ros::Time& previous_update_time);

AdapterState decideState(const AdapterConfig& config,
                         const GateState& gates,
                         const HealthReport& health);

std::string stateName(AdapterState state);

uav_msgs::OffboardStatus makeStatus(const AdapterConfig& config,
                                    const GateState& gates,
                                    const HealthReport& health,
                                    AdapterState state,
                                    const ros::Time& stamp);

mavros_msgs::PositionTarget previewToPositionTarget(
    const uav_msgs::SetpointPreview& preview,
    const ros::Time& publish_time,
    uint8_t coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED);

bool isFinite(const uav_msgs::SetpointPreview& preview);

}  // namespace uav_offboard

#endif  // UAV_OFFBOARD_OFFBOARD_ADAPTER_HPP_
