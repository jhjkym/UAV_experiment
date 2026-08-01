#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <uav_trajectory/trajectory_sampler.hpp>

namespace {

constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;

uav_msgs::TrajectoryPoint makePoint(const double t,
                                    const double x,
                                    const double vx,
                                    const double yaw = 0.0,
                                    const double yaw_rate = 0.0) {
  uav_msgs::TrajectoryPoint point;
  point.time_from_start = ros::Duration(t);
  point.position.x = x;
  point.position.y = 2.0 * x;
  point.position.z = -x;
  point.velocity.x = vx;
  point.velocity.y = 2.0 * vx;
  point.velocity.z = -vx;
  point.acceleration.x = 0.0;
  point.acceleration.y = 0.0;
  point.acceleration.z = 0.0;
  point.yaw = yaw;
  point.yaw_rate = yaw_rate;
  return point;
}

uav_msgs::Trajectory makeTrajectory() {
  uav_msgs::Trajectory trajectory;
  trajectory.header.stamp = ros::Time(100.0);
  trajectory.header.frame_id = "map";
  trajectory.mode = uav_msgs::Trajectory::MODE_NOMINAL;
  trajectory.trajectory_id = 42;
  trajectory.points.push_back(makePoint(0.0, 0.0, 0.0));
  trajectory.points.push_back(makePoint(2.0, 2.0, 0.0));
  return trajectory;
}

std::vector<std::string> frames() {
  return {"map"};
}

}  // namespace

TEST(TrajectoryValidation, RejectsEmptyTrajectory) {
  uav_msgs::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  const auto result = uav_trajectory::validateTrajectory(trajectory, frames());
  EXPECT_FALSE(result.valid);
}

TEST(TrajectoryValidation, AcceptsSinglePointTrajectory) {
  uav_msgs::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points.push_back(makePoint(0.0, 1.0, 0.0));
  EXPECT_TRUE(uav_trajectory::validateTrajectory(trajectory, frames()).valid);
}

TEST(TrajectoryValidation, RejectsNonStrictTime) {
  auto trajectory = makeTrajectory();
  trajectory.points[1].time_from_start = ros::Duration(0.0);
  EXPECT_FALSE(uav_trajectory::validateTrajectory(trajectory, frames()).valid);
}

TEST(TrajectoryValidation, RejectsNanAndInfinity) {
  auto trajectory = makeTrajectory();
  trajectory.points[0].position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(uav_trajectory::validateTrajectory(trajectory, frames()).valid);
  trajectory = makeTrajectory();
  trajectory.points[0].velocity.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(uav_trajectory::validateTrajectory(trajectory, frames()).valid);
}

TEST(TrajectoryValidation, RejectsFrameMismatch) {
  auto trajectory = makeTrajectory();
  trajectory.header.frame_id = "odom";
  EXPECT_FALSE(uav_trajectory::validateTrajectory(trajectory, frames()).valid);
}

TEST(TrajectorySampler, StartBeforeBehaviorUsesFirstPoint) {
  const auto trajectory = makeTrajectory();
  const auto sample =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(99.5), ros::Time(0));
  ASSERT_TRUE(sample.has_trajectory);
  EXPECT_FALSE(sample.started);
  EXPECT_FALSE(sample.finished);
  EXPECT_NEAR(sample.point.position.x, 0.0, kTolerance);
}

TEST(TrajectorySampler, MiddleHermitePositionVelocityAcceleration) {
  const auto trajectory = makeTrajectory();
  const auto sample =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(101.0), ros::Time(0));
  ASSERT_TRUE(sample.has_trajectory);
  EXPECT_TRUE(sample.started);
  EXPECT_FALSE(sample.finished);
  EXPECT_NEAR(sample.point.position.x, 1.0, kTolerance);
  EXPECT_NEAR(sample.point.velocity.x, 1.5, kTolerance);
  EXPECT_NEAR(sample.point.acceleration.x, 0.0, kTolerance);
  EXPECT_NEAR(sample.point.velocity.y, 3.0, kTolerance);
  EXPECT_NEAR(sample.point.velocity.z, -1.5, kTolerance);
}

TEST(TrajectorySampler, EndpointsMatchPositionAndVelocity) {
  const auto trajectory = makeTrajectory();
  const auto start =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(100.0), ros::Time(0));
  const auto end =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(102.0), ros::Time(101.0));
  EXPECT_NEAR(start.point.position.x, trajectory.points.front().position.x, kTolerance);
  EXPECT_NEAR(start.point.velocity.x, trajectory.points.front().velocity.x, kTolerance);
  EXPECT_NEAR(end.point.position.x, trajectory.points.back().position.x, kTolerance);
  EXPECT_NEAR(end.point.velocity.x, trajectory.points.back().velocity.x, kTolerance);
  EXPECT_TRUE(end.finished);
}

TEST(TrajectorySampler, EndAfterBehaviorHoldsLastPoint) {
  const auto trajectory = makeTrajectory();
  const auto sample =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(103.0), ros::Time(102.0));
  EXPECT_TRUE(sample.started);
  EXPECT_TRUE(sample.finished);
  EXPECT_NEAR(sample.point.position.x, 2.0, kTolerance);
}

TEST(TrajectorySampler, SinglePointFinishesAtStart) {
  uav_msgs::Trajectory trajectory;
  trajectory.header.stamp = ros::Time(10.0);
  trajectory.header.frame_id = "map";
  trajectory.points.push_back(makePoint(0.0, 3.0, 0.0));
  const auto sample =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(10.1), ros::Time(0));
  EXPECT_TRUE(sample.started);
  EXPECT_TRUE(sample.finished);
  EXPECT_NEAR(sample.point.position.x, 3.0, kTolerance);
}

TEST(TrajectorySampler, YawCrossesPiBoundaryByShortestPath) {
  uav_msgs::Trajectory trajectory = makeTrajectory();
  trajectory.points[0].yaw = 170.0 * kPi / 180.0;
  trajectory.points[1].yaw = -170.0 * kPi / 180.0;
  const auto sample =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(101.0), ros::Time(0));
  EXPECT_NEAR(std::abs(sample.point.yaw), kPi, 1e-6);
  EXPECT_LT(std::abs(uav_trajectory::shortestAngleDelta(trajectory.points[0].yaw,
                                                        sample.point.yaw)),
            20.1 * kPi / 180.0);
}

TEST(TrajectorySampler, TimeBackwardsHoldsFirstPoint) {
  const auto trajectory = makeTrajectory();
  const auto sample =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(100.5), ros::Time(101.0));
  EXPECT_TRUE(sample.time_went_back);
  EXPECT_FALSE(sample.started);
  EXPECT_NEAR(sample.point.position.x, 0.0, kTolerance);
}

TEST(TrajectorySampler, QuaternionFromYawIsNormalized) {
  const auto q = uav_trajectory::yawToQuaternion(1.25);
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  EXPECT_NEAR(norm, 1.0, kTolerance);
}

TEST(TrajectoryCache, ReplacesWithValidTrajectoryAtomically) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  auto trajectory = makeTrajectory();
  EXPECT_TRUE(cache.replaceIfValid(trajectory, frames(), &reason));
  uav_msgs::Trajectory cached;
  ASSERT_TRUE(cache.get(&cached));
  EXPECT_EQ(cached.trajectory_id, 42u);

  trajectory.trajectory_id = 43;
  EXPECT_TRUE(cache.replaceIfValid(trajectory, frames(), &reason));
  ASSERT_TRUE(cache.get(&cached));
  EXPECT_EQ(cached.trajectory_id, 43u);
}

TEST(TrajectoryCache, InvalidNewTrajectoryDoesNotOverwriteCache) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  auto trajectory = makeTrajectory();
  ASSERT_TRUE(cache.replaceIfValid(trajectory, frames(), &reason));

  auto invalid = makeTrajectory();
  invalid.trajectory_id = 99;
  invalid.points.clear();
  EXPECT_FALSE(cache.replaceIfValid(invalid, frames(), &reason));

  uav_msgs::Trajectory cached;
  ASSERT_TRUE(cache.get(&cached));
  EXPECT_EQ(cached.trajectory_id, 42u);
}

TEST(TrajectoryCache, FutureTrajectoryIsPendingUntilStartTime) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  bool queued = false;
  auto active = makeTrajectory();
  ASSERT_TRUE(cache.queueOrReplaceIfValid(active, frames(), ros::Time(100.0),
                                          &reason, &queued));
  EXPECT_FALSE(queued);

  auto pending = makeTrajectory();
  pending.trajectory_id = 43;
  pending.header.stamp = ros::Time(110.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(pending, frames(), ros::Time(105.0),
                                          &reason, &queued));
  EXPECT_TRUE(queued);

  uav_msgs::Trajectory cached;
  bool promoted = false;
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(109.9), &cached, &promoted));
  EXPECT_FALSE(promoted);
  EXPECT_EQ(cached.trajectory_id, 42u);

  ASSERT_TRUE(cache.getActiveForTime(ros::Time(110.0), &cached, &promoted));
  EXPECT_TRUE(promoted);
  EXPECT_EQ(cached.trajectory_id, 43u);

  ASSERT_TRUE(cache.getActiveForTime(ros::Time(110.1), &cached, &promoted));
  EXPECT_FALSE(promoted);
  EXPECT_EQ(cached.trajectory_id, 43u);
}

TEST(TrajectoryCache, ImmediateTrajectoryReplacesPendingTrajectory) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  bool queued = false;
  ASSERT_TRUE(cache.queueOrReplaceIfValid(makeTrajectory(), frames(), ros::Time(100.0),
                                          &reason, &queued));
  auto pending = makeTrajectory();
  pending.trajectory_id = 43;
  pending.header.stamp = ros::Time(120.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(pending, frames(), ros::Time(101.0),
                                          &reason, &queued));
  EXPECT_TRUE(queued);

  auto immediate = makeTrajectory();
  immediate.trajectory_id = 44;
  immediate.header.stamp = ros::Time(101.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(immediate, frames(), ros::Time(101.0),
                                          &reason, &queued));
  EXPECT_FALSE(queued);

  uav_msgs::Trajectory cached;
  bool promoted = false;
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(120.0), &cached, &promoted));
  EXPECT_FALSE(promoted);
  EXPECT_EQ(cached.trajectory_id, 44u);
}

TEST(TrajectoryCache, NewPendingReplacesOldPending) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  bool queued = false;
  ASSERT_TRUE(cache.queueOrReplaceIfValid(makeTrajectory(), frames(), ros::Time(100.0),
                                          &reason, &queued));
  auto first_pending = makeTrajectory();
  first_pending.trajectory_id = 43;
  first_pending.header.stamp = ros::Time(120.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(first_pending, frames(), ros::Time(101.0),
                                          &reason, &queued));
  EXPECT_TRUE(queued);

  auto second_pending = makeTrajectory();
  second_pending.trajectory_id = 44;
  second_pending.header.stamp = ros::Time(115.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(second_pending, frames(), ros::Time(102.0),
                                          &reason, &queued));
  EXPECT_TRUE(queued);

  uav_msgs::Trajectory cached;
  bool promoted = false;
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(115.0), &cached, &promoted));
  EXPECT_TRUE(promoted);
  EXPECT_EQ(cached.trajectory_id, 44u);
}

TEST(TrajectoryCache, InvalidPendingDoesNotDestroyActiveOrPending) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  bool queued = false;
  ASSERT_TRUE(cache.queueOrReplaceIfValid(makeTrajectory(), frames(), ros::Time(100.0),
                                          &reason, &queued));
  auto pending = makeTrajectory();
  pending.trajectory_id = 43;
  pending.header.stamp = ros::Time(110.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(pending, frames(), ros::Time(101.0),
                                          &reason, &queued));
  EXPECT_TRUE(queued);

  auto invalid = makeTrajectory();
  invalid.trajectory_id = 99;
  invalid.header.stamp = ros::Time(105.0);
  invalid.points.clear();
  EXPECT_FALSE(cache.queueOrReplaceIfValid(invalid, frames(), ros::Time(102.0),
                                           &reason, &queued));

  uav_msgs::Trajectory cached;
  bool promoted = false;
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(109.0), &cached, &promoted));
  EXPECT_FALSE(promoted);
  EXPECT_EQ(cached.trajectory_id, 42u);
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(110.0), &cached, &promoted));
  EXPECT_TRUE(promoted);
  EXPECT_EQ(cached.trajectory_id, 43u);
}

TEST(TrajectoryCache, TimeBackwardsDoesNotPromoteFuturePending) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  bool queued = false;
  ASSERT_TRUE(cache.queueOrReplaceIfValid(makeTrajectory(), frames(), ros::Time(100.0),
                                          &reason, &queued));
  auto pending = makeTrajectory();
  pending.trajectory_id = 43;
  pending.header.stamp = ros::Time(110.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(pending, frames(), ros::Time(105.0),
                                          &reason, &queued));

  uav_msgs::Trajectory cached;
  bool promoted = false;
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(104.0), &cached, &promoted));
  EXPECT_FALSE(promoted);
  EXPECT_EQ(cached.trajectory_id, 42u);
}

TEST(TrajectoryCache, ActiveMayFinishWhilePendingWaits) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  bool queued = false;
  auto active = makeTrajectory();
  active.header.stamp = ros::Time(100.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(active, frames(), ros::Time(100.0),
                                          &reason, &queued));
  auto pending = makeTrajectory();
  pending.trajectory_id = 43;
  pending.header.stamp = ros::Time(110.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(pending, frames(), ros::Time(101.0),
                                          &reason, &queued));

  uav_msgs::Trajectory cached;
  bool promoted = false;
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(105.0), &cached, &promoted));
  EXPECT_FALSE(promoted);
  EXPECT_EQ(cached.trajectory_id, 42u);
  const auto sample = uav_trajectory::sampleTrajectory(cached, ros::Time(105.0),
                                                       ros::Time(104.0));
  EXPECT_TRUE(sample.finished);
}

TEST(TrajectoryCache, PendingFirstPointCanBeContinuousWithActiveOutput) {
  uav_trajectory::TrajectoryCache cache;
  std::string reason;
  bool queued = false;
  auto active = makeTrajectory();
  active.header.stamp = ros::Time(100.0);
  active.points.back() = active.points.front();
  active.points.back().time_from_start = ros::Duration(60.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(active, frames(), ros::Time(100.0),
                                          &reason, &queued));
  auto pending = makeTrajectory();
  pending.trajectory_id = 43;
  pending.header.stamp = ros::Time(105.0);
  pending.points.front() = active.points.front();
  pending.points.front().time_from_start = ros::Duration(0.0);
  ASSERT_TRUE(cache.queueOrReplaceIfValid(pending, frames(), ros::Time(104.0),
                                          &reason, &queued));

  uav_msgs::Trajectory cached;
  bool promoted = false;
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(104.9), &cached, &promoted));
  const auto before = uav_trajectory::sampleTrajectory(cached, ros::Time(104.9),
                                                       ros::Time(104.8));
  ASSERT_TRUE(cache.getActiveForTime(ros::Time(105.0), &cached, &promoted));
  const auto after = uav_trajectory::sampleTrajectory(cached, ros::Time(105.0),
                                                      ros::Time(0.0));
  EXPECT_TRUE(promoted);
  EXPECT_NEAR(after.point.position.x, before.point.position.x, kTolerance);
  EXPECT_NEAR(after.point.velocity.x, before.point.velocity.x, kTolerance);
  EXPECT_NEAR(after.point.acceleration.x, before.point.acceleration.x, kTolerance);
}

TEST(TrajectoryPreview, MakePreviewCarriesFrameIdAndStateFreshness) {
  const auto trajectory = makeTrajectory();
  const auto sample =
      uav_trajectory::sampleTrajectory(trajectory, ros::Time(101.0), ros::Time(0));
  const auto preview = uav_trajectory::makePreviewMessage(trajectory, sample, false);
  EXPECT_EQ(preview.header.frame_id, "map");
  EXPECT_EQ(preview.trajectory_id, 42u);
  EXPECT_TRUE(preview.trajectory_valid);
  EXPECT_TRUE(preview.started);
  EXPECT_FALSE(preview.finished);
  EXPECT_FALSE(preview.state_fresh);
}

TEST(StateFreshness, UsesValidityFlagsAndTimeout) {
  uav_msgs::UavState state;
  state.header.stamp = ros::Time(10.0);
  state.pose_valid = true;
  state.twist_valid = true;
  EXPECT_TRUE(uav_trajectory::isStateFresh(state, ros::Time(10.2), ros::Duration(0.5)));
  EXPECT_FALSE(uav_trajectory::isStateFresh(state, ros::Time(10.6), ros::Duration(0.5)));
  state.pose_valid = false;
  EXPECT_FALSE(uav_trajectory::isStateFresh(state, ros::Time(10.2), ros::Duration(0.5)));
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
