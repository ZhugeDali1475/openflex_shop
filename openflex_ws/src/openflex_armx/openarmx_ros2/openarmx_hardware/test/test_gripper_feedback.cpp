#include <gtest/gtest.h>

#include <limits>
#include <utility>

#include "openarmx_hardware/gripper_feedback.hpp"

namespace openarmx_hardware {
namespace {

TEST(GripperFeedback, PreservesCalibratedFeedback) {
  for (const double raw : {0.0, 0.5, 1.0472}) {
    const auto result = validate_gripper_motor_feedback(raw);
    EXPECT_TRUE(result.valid);
    EXPECT_FALSE(result.clamped);
    EXPECT_DOUBLE_EQ(result.radians, raw);
  }
}

TEST(GripperFeedback, ClampsPlausibleMechanicalOvertravel) {
  for (const auto& [raw, expected] : {
           std::pair{-0.05, 0.0},
           std::pair{1.173277, 1.0472},
       }) {
    const auto result = validate_gripper_motor_feedback(raw);
    EXPECT_TRUE(result.valid) << "raw=" << raw;
    EXPECT_TRUE(result.clamped) << "raw=" << raw;
    EXPECT_DOUBLE_EQ(result.radians, expected);
  }
}

TEST(GripperFeedback, RejectsObservedErrorFrameMisparsedAsState) {
  const auto result =
      validate_gripper_motor_feedback(-12.545449257);

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.clamped);
  EXPECT_DOUBLE_EQ(result.radians, -12.545449257);
}

TEST(GripperFeedback, RejectsUnrelatedInvalidFeedback) {
  for (const double raw : {
           -12.57,
           -12.375,
           -11.5228,
           -10.0,
           -6.0,
           1.25,
           12.55,
           std::numeric_limits<double>::infinity(),
           -std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::quiet_NaN(),
       }) {
    const auto result = validate_gripper_motor_feedback(raw);
    EXPECT_FALSE(result.valid) << "raw=" << raw;
    EXPECT_FALSE(result.clamped) << "raw=" << raw;
  }
}

}  // namespace
}  // namespace openarmx_hardware
