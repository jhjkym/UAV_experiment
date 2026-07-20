#include <cmath>

#include <gtest/gtest.h>

#include "uav_px4_bridge/frame_conversions.hpp"

namespace {

geometry_msgs::Vector3 vec(const double x, const double y, const double z) {
  geometry_msgs::Vector3 v;
  v.x = x;
  v.y = y;
  v.z = z;
  return v;
}

void expectVecNear(const geometry_msgs::Vector3& actual,
                   const geometry_msgs::Vector3& expected,
                   const double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

}  // namespace

TEST(FrameConversions, EnuToNedBasisVectors) {
  expectVecNear(uav_px4_bridge::enuToNed(vec(1.0, 0.0, 0.0)), vec(0.0, 1.0, 0.0));
  expectVecNear(uav_px4_bridge::enuToNed(vec(0.0, 1.0, 0.0)), vec(1.0, 0.0, 0.0));
  expectVecNear(uav_px4_bridge::enuToNed(vec(0.0, 0.0, 1.0)), vec(0.0, 0.0, -1.0));
}

TEST(FrameConversions, NedToEnuBasisVectors) {
  expectVecNear(uav_px4_bridge::nedToEnu(vec(1.0, 0.0, 0.0)), vec(0.0, 1.0, 0.0));
  expectVecNear(uav_px4_bridge::nedToEnu(vec(0.0, 1.0, 0.0)), vec(1.0, 0.0, 0.0));
  expectVecNear(uav_px4_bridge::nedToEnu(vec(0.0, 0.0, 1.0)), vec(0.0, 0.0, -1.0));
}

TEST(FrameConversions, FluToFrdBasisVectors) {
  expectVecNear(uav_px4_bridge::fluToFrd(vec(1.0, 0.0, 0.0)), vec(1.0, 0.0, 0.0));
  expectVecNear(uav_px4_bridge::fluToFrd(vec(0.0, 1.0, 0.0)), vec(0.0, -1.0, 0.0));
  expectVecNear(uav_px4_bridge::fluToFrd(vec(0.0, 0.0, 1.0)), vec(0.0, 0.0, -1.0));
}

TEST(FrameConversions, FrdToFluBasisVectors) {
  expectVecNear(uav_px4_bridge::frdToFlu(vec(1.0, 0.0, 0.0)), vec(1.0, 0.0, 0.0));
  expectVecNear(uav_px4_bridge::frdToFlu(vec(0.0, 1.0, 0.0)), vec(0.0, -1.0, 0.0));
  expectVecNear(uav_px4_bridge::frdToFlu(vec(0.0, 0.0, 1.0)), vec(0.0, 0.0, -1.0));
}

TEST(FrameConversions, RoundTripAndZeroVectors) {
  const geometry_msgs::Vector3 zero = vec(0.0, 0.0, 0.0);
  expectVecNear(uav_px4_bridge::nedToEnu(uav_px4_bridge::enuToNed(zero)), zero);
  expectVecNear(uav_px4_bridge::frdToFlu(uav_px4_bridge::fluToFrd(zero)), zero);

  const geometry_msgs::Vector3 arbitrary_world = vec(2.5, -3.0, 4.25);
  expectVecNear(
      uav_px4_bridge::nedToEnu(uav_px4_bridge::enuToNed(arbitrary_world)),
      arbitrary_world);

  const geometry_msgs::Vector3 arbitrary_body = vec(-1.0, 2.0, -3.0);
  expectVecNear(
      uav_px4_bridge::frdToFlu(uav_px4_bridge::fluToFrd(arbitrary_body)),
      arbitrary_body);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
