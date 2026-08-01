#ifndef UAV_TRAJECTORY_TRAJECTORY_SAMPLER_HPP_
#define UAV_TRAJECTORY_TRAJECTORY_SAMPLER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <uav_msgs/SetpointPreview.h>
#include <uav_msgs/Trajectory.h>
#include <uav_msgs/TrajectoryPoint.h>
#include <uav_msgs/UavState.h>

namespace uav_trajectory {

struct ValidationResult {
  bool valid = false;
  std::string reason;
};

struct SampleResult {
  bool has_trajectory = false;
  bool started = false;
  bool finished = false;
  bool time_went_back = false;
  uav_msgs::TrajectoryPoint point;
};

enum class TrajectoryUpdateAction {
  kRejected,
  kAcceptedActive,
  kQueuedPending,
  kDuplicateActive,
  kDuplicatePending,
};

ValidationResult validateTrajectory(const uav_msgs::Trajectory& trajectory,
                                    const std::vector<std::string>& supported_frames);

SampleResult sampleTrajectory(const uav_msgs::Trajectory& trajectory,
                              const ros::Time& now,
                              const ros::Time& previous_sample_time);

uav_msgs::SetpointPreview makePreviewMessage(const uav_msgs::Trajectory& trajectory,
                                             const SampleResult& sample,
                                             bool state_fresh);

geometry_msgs::Quaternion yawToQuaternion(double yaw);

double normalizeAngle(double angle);

double shortestAngleDelta(double from, double to);

bool isFinite(const uav_msgs::TrajectoryPoint& point);

bool isStateFresh(const uav_msgs::UavState& state,
                  const ros::Time& now,
                  const ros::Duration& timeout);

class TrajectoryCache {
 public:
  bool replaceIfValid(const uav_msgs::Trajectory& trajectory,
                      const std::vector<std::string>& supported_frames,
                      std::string* rejection_reason);

  bool queueOrReplaceIfValid(const uav_msgs::Trajectory& trajectory,
                             const std::vector<std::string>& supported_frames,
                             const ros::Time& now,
                             std::string* rejection_reason,
                             bool* queued_pending);

  TrajectoryUpdateAction queueOrReplaceIfValidDetailed(
      const uav_msgs::Trajectory& trajectory,
      const std::vector<std::string>& supported_frames,
      const ros::Time& now,
      std::string* rejection_reason);

  bool get(uav_msgs::Trajectory* trajectory) const;

  bool getActiveForTime(const ros::Time& now,
                        uav_msgs::Trajectory* trajectory,
                        bool* promoted_pending);

 private:
  mutable std::mutex mutex_;
  bool has_trajectory_ = false;
  bool has_pending_trajectory_ = false;
  uav_msgs::Trajectory trajectory_;
  uav_msgs::Trajectory pending_trajectory_;
};

}  // namespace uav_trajectory

#endif  // UAV_TRAJECTORY_TRAJECTORY_SAMPLER_HPP_
