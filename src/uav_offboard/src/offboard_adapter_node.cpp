#include <mutex>
#include <string>
#include <vector>

#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <uav_msgs/OffboardStatus.h>
#include <uav_msgs/SetpointPreview.h>
#include <uav_msgs/UavState.h>

#include "uav_offboard/offboard_adapter.hpp"

namespace {

class OffboardAdapterNode {
 public:
  OffboardAdapterNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(nh), private_nh_(private_nh) {
    loadConfig();

    std::string warning;
    uav_offboard::validateConfig(&config_, &warning);
    if (!warning.empty()) {
      ROS_WARN("%s", warning.c_str());
    }

    private_nh_.param<std::string>("setpoint_preview_topic",
                                   setpoint_preview_topic_,
                                   "/uav/setpoint_preview");
    private_nh_.param<std::string>("uav_state_topic", uav_state_topic_, "/uav/state");
    private_nh_.param<std::string>("mavros_state_topic",
                                   mavros_state_topic_,
                                   "/mavros/state");
    private_nh_.param<std::string>("target_preview_topic",
                                   target_preview_topic_,
                                   "/uav/mavros_target_preview");
    private_nh_.param<std::string>("offboard_status_topic",
                                   offboard_status_topic_,
                                   "/uav/offboard_status");

    preview_sub_ = nh_.subscribe(setpoint_preview_topic_, 1,
                                 &OffboardAdapterNode::previewCallback, this);
    state_sub_ =
        nh_.subscribe(uav_state_topic_, 1, &OffboardAdapterNode::stateCallback, this);
    mavros_state_sub_ = nh_.subscribe(mavros_state_topic_, 1,
                                      &OffboardAdapterNode::mavrosStateCallback, this);

    target_preview_pub_ =
        nh_.advertise<mavros_msgs::PositionTarget>(target_preview_topic_, 1);
    status_pub_ = nh_.advertise<uav_msgs::OffboardStatus>(offboard_status_topic_, 1);
    if (config_.allow_mavros_output) {
      mavros_output_pub_ =
          nh_.advertise<mavros_msgs::PositionTarget>(config_.mavros_output_topic, 1);
      ROS_WARN("Static gate allows MAVROS output topic %s; runtime gate remains disabled",
               config_.mavros_output_topic.c_str());
    } else {
      ROS_INFO("Static gate disabled; real MAVROS output publisher is not created");
    }

    set_output_enabled_srv_ =
        private_nh_.advertiseService("set_output_enabled",
                                     &OffboardAdapterNode::setOutputEnabled, this);
    timer_ = nh_.createTimer(ros::Duration(1.0 / config_.publish_rate_hz),
                             &OffboardAdapterNode::timerCallback, this);

    ROS_INFO("offboard_adapter_node started preview=%s state=%s mavros_state=%s "
             "target_preview=%s status=%s allow_mavros_output=%d rate=%.3f",
             setpoint_preview_topic_.c_str(), uav_state_topic_.c_str(),
             mavros_state_topic_.c_str(), target_preview_topic_.c_str(),
             offboard_status_topic_.c_str(), config_.allow_mavros_output,
             config_.publish_rate_hz);
  }

 private:
  void loadConfig() {
    private_nh_.param<double>("publish_rate_hz", config_.publish_rate_hz, 30.0);
    double preview_timeout = 0.2;
    double state_timeout = 0.2;
    private_nh_.param<double>("preview_timeout_sec", preview_timeout, 0.2);
    private_nh_.param<double>("state_timeout_sec", state_timeout, 0.2);
    config_.preview_timeout = ros::Duration(preview_timeout);
    config_.state_timeout = ros::Duration(state_timeout);
    private_nh_.param<bool>("allow_mavros_output", config_.allow_mavros_output, false);
    private_nh_.param<std::string>("mavros_output_topic",
                                   config_.mavros_output_topic,
                                   "/mavros/setpoint_raw/local");
    if (!private_nh_.getParam("supported_frames", config_.supported_frames)) {
      config_.supported_frames = {"map"};
    }
  }

  void previewCallback(const uav_msgs::SetpointPreviewConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    inputs_.preview = *msg;
    inputs_.preview_received = true;
  }

  void stateCallback(const uav_msgs::UavStateConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    inputs_.vehicle_state = *msg;
    inputs_.vehicle_state_received = true;
  }

  void mavrosStateCallback(const mavros_msgs::StateConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    inputs_.mavros_state = *msg;
    inputs_.mavros_state_received = true;
  }

  bool setOutputEnabled(std_srvs::SetBool::Request& request,
                        std_srvs::SetBool::Response& response) {
    const ros::Time now = ros::Time::now();
    uav_offboard::AdapterInputs inputs;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs = inputs_;
    }

    if (!request.data) {
      gates_.runtime_output_enabled = false;
      response.success = true;
      response.message = "runtime output disabled";
      ROS_WARN("Runtime output gate disabled by service request");
      return true;
    }

    const auto health =
        uav_offboard::evaluateHealth(inputs, config_, now, last_update_time_);
    std::string reason;
    if (!uav_offboard::canEnableRuntimeOutput(config_, health, &reason)) {
      gates_.runtime_output_enabled = false;
      response.success = false;
      response.message = reason;
      ROS_WARN("Runtime output gate enable rejected: %s", reason.c_str());
      return true;
    }

    gates_.runtime_output_enabled = true;
    response.success = true;
    response.message = "runtime output enabled";
    ROS_WARN("Runtime output gate enabled; no arming or mode change was requested");
    return true;
  }

  void timerCallback(const ros::TimerEvent& event) {
    const ros::Time now = event.current_real;
    uav_offboard::AdapterInputs inputs;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inputs = inputs_;
    }

    const auto health =
        uav_offboard::evaluateHealth(inputs, config_, now, last_update_time_);
    auto state = uav_offboard::decideState(config_, gates_, health);

    if (gates_.runtime_output_enabled && !health.healthy) {
      ROS_WARN_THROTTLE(1.0, "Output disabled due to health failure: %s",
                        health.reason.c_str());
      gates_.runtime_output_enabled = false;
      state = uav_offboard::AdapterState::FAULT;
    }

    if (inputs.preview_received && uav_offboard::isFinite(inputs.preview)) {
      const auto target =
          uav_offboard::previewToPositionTarget(inputs.preview, now);
      target_preview_pub_.publish(target);
      if (state == uav_offboard::AdapterState::STREAMING &&
          config_.allow_mavros_output && mavros_output_pub_) {
        mavros_output_pub_.publish(target);
      }
    } else {
      ROS_WARN_THROTTLE(1.0, "Target preview not published: no finite preview input");
    }

    const auto status =
        uav_offboard::makeStatus(config_, gates_, health, state, now);
    status_pub_.publish(status);
    last_update_time_ = now;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber preview_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber mavros_state_sub_;
  ros::Publisher target_preview_pub_;
  ros::Publisher status_pub_;
  ros::Publisher mavros_output_pub_;
  ros::ServiceServer set_output_enabled_srv_;
  ros::Timer timer_;

  std::mutex mutex_;
  uav_offboard::AdapterInputs inputs_;
  uav_offboard::AdapterConfig config_;
  uav_offboard::GateState gates_;
  ros::Time last_update_time_;

  std::string setpoint_preview_topic_;
  std::string uav_state_topic_;
  std::string mavros_state_topic_;
  std::string target_preview_topic_;
  std::string offboard_status_topic_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "offboard_adapter_node");
  OffboardAdapterNode node(ros::NodeHandle{}, ros::NodeHandle{"~"});
  ros::spin();
  return 0;
}
