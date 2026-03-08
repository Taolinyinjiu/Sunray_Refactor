#include <gtest/gtest.h>

#include "utils/quintic_curve.hpp"

namespace {

double smoothstep5(double x) {
  const double x2 = x * x;
  const double x3 = x2 * x;
  const double x4 = x3 * x;
  const double x5 = x4 * x;
  return 6.0 * x5 - 15.0 * x4 + 10.0 * x3;
}

} // namespace

TEST(QuinticCurveTest, StartAtOriginIsValid) {
  Quintic_Curve curve;
  curve.set_start_position(Eigen::Vector3d(0.0, 0.0, 0.0));
  curve.set_end_position(Eigen::Vector3d(2.0, -1.0, 4.0));

  EXPECT_TRUE(curve.generate_by_z_height(2.0));
  const Eigen::Vector3d pos = curve.get_position();
  const Eigen::Vector3d vel = curve.get_velocity();
  const Eigen::Vector3d acc = curve.get_acceleration();

  EXPECT_NEAR(pos.x(), 1.0, 1e-9);
  EXPECT_NEAR(pos.y(), -0.5, 1e-9);
  EXPECT_NEAR(pos.z(), 2.0, 1e-9);
  EXPECT_GT(vel.norm(), 0.0);
  EXPECT_NEAR(acc.norm(), 0.0, 1e-9);
}

TEST(QuinticCurveTest, DescendCaseAlsoWorks) {
  Quintic_Curve curve;
  curve.set_start_position(Eigen::Vector3d(1.0, 2.0, 2.0));
  curve.set_end_position(Eigen::Vector3d(3.0, 6.0, -2.0));

  EXPECT_TRUE(curve.generate_by_z_height(0.0));
  const Eigen::Vector3d pos = curve.get_position();
  const double blend = smoothstep5(0.5);
  const Eigen::Vector3d expected =
      Eigen::Vector3d(1.0, 2.0, 2.0) +
      blend * Eigen::Vector3d(2.0, 4.0, -4.0);

  EXPECT_NEAR(pos.x(), expected.x(), 1e-9);
  EXPECT_NEAR(pos.y(), expected.y(), 1e-9);
  EXPECT_NEAR(pos.z(), expected.z(), 1e-9);
}

TEST(QuinticCurveTest, OutOfRangeReturnsFalseAndClampsOutput) {
  Quintic_Curve curve;
  curve.set_start_position(Eigen::Vector3d(0.0, 0.0, 1.0));
  curve.set_end_position(Eigen::Vector3d(2.0, 2.0, 3.0));

  EXPECT_FALSE(curve.generate_by_z_height(4.0));
  const Eigen::Vector3d pos = curve.get_position();
  const Eigen::Vector3d vel = curve.get_velocity();
  const Eigen::Vector3d acc = curve.get_acceleration();

  EXPECT_NEAR(pos.x(), 2.0, 1e-9);
  EXPECT_NEAR(pos.y(), 2.0, 1e-9);
  EXPECT_NEAR(pos.z(), 3.0, 1e-9);
  EXPECT_NEAR(vel.norm(), 0.0, 1e-9);
  EXPECT_NEAR(acc.norm(), 0.0, 1e-9);
}

TEST(QuinticCurveTest, ZeroZDistanceHandledSafely) {
  Quintic_Curve curve;
  curve.set_start_position(Eigen::Vector3d(1.0, 2.0, 3.0));
  curve.set_end_position(Eigen::Vector3d(4.0, 5.0, 3.0));

  EXPECT_TRUE(curve.generate_by_z_height(3.0));
  const Eigen::Vector3d pos = curve.get_position();
  EXPECT_NEAR(pos.x(), 1.0, 1e-9);
  EXPECT_NEAR(pos.y(), 2.0, 1e-9);
  EXPECT_NEAR(pos.z(), 3.0, 1e-9);

  EXPECT_FALSE(curve.generate_by_z_height(3.2));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
