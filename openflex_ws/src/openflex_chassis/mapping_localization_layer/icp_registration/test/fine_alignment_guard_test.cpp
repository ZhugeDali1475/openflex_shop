#include <gtest/gtest.h>

#include <limits>

#include "icp_registration/fine_alignment_guard.hpp"

namespace
{

using icp::fine_alignment_guard::HoldInput;
using icp::fine_alignment_guard::HoldLimits;
using icp::fine_alignment_guard::shouldDeferCompletedRelease;
using icp::fine_alignment_guard::shouldKeepLastTransform;
using icp::fine_alignment_guard::shouldRefreshCompletedLease;

HoldInput recordedDeliveryFrame(double p50 = 0.116)
{
  HoldInput input;
  input.guard_active = true;
  input.solver_converged = true;
  input.localization_healthy = true;
  input.wheel_recent = true;
  input.lio_recent = true;
  input.wheel_linear_speed = 0.038;
  input.wheel_travel_from_anchor = 0.19;
  input.lio_wheel_divergence_delta = 0.02;
  input.hold_age = 9.3;
  input.inlier_count = 1038;
  input.inlier_ratio = 0.692;
  input.inlier_rmse = 0.141;
  input.median_distance = p50;
  return input;
}

TEST(FineAlignmentGuard, KeepsLastTransformForRecordedDeliveryFailure)
{
  const HoldLimits limits;
  EXPECT_TRUE(shouldKeepLastTransform(recordedDeliveryFrame(0.110), limits));
  EXPECT_TRUE(shouldKeepLastTransform(recordedDeliveryFrame(0.116), limits));
  EXPECT_TRUE(shouldKeepLastTransform(recordedDeliveryFrame(0.107), limits));

  auto completed_approach = recordedDeliveryFrame(0.116);
  completed_approach.hold_age = 12.0;
  completed_approach.wheel_travel_from_anchor = 0.27;
  EXPECT_TRUE(shouldKeepLastTransform(completed_approach, limits));

  auto post_place_retreat = recordedDeliveryFrame(0.116);
  post_place_retreat.hold_age = 25.0;
  post_place_retreat.wheel_travel_from_anchor = 0.40;
  EXPECT_TRUE(shouldKeepLastTransform(post_place_retreat, limits));
}

TEST(FineAlignmentGuard, CompletedLeaseCanRestartAtTheAlignedPose)
{
  const HoldLimits limits;
  auto consumed_approach_lease = recordedDeliveryFrame(0.110);
  consumed_approach_lease.hold_age = limits.max_hold_duration + 0.01;
  EXPECT_FALSE(shouldKeepLastTransform(consumed_approach_lease, limits));

  auto renewed_completed_lease = consumed_approach_lease;
  renewed_completed_lease.hold_age = 0.01;
  renewed_completed_lease.wheel_travel_from_anchor = 0.0;
  renewed_completed_lease.wheel_linear_speed = 0.0;
  EXPECT_TRUE(shouldKeepLastTransform(renewed_completed_lease, limits));
}

TEST(FineAlignmentGuard, CompletedReleaseKeepsOnlyTheOriginalBoundedLease)
{
  EXPECT_TRUE(shouldDeferCompletedRelease(true, true));
  EXPECT_FALSE(shouldDeferCompletedRelease(true, false));
  EXPECT_FALSE(shouldDeferCompletedRelease(false, true));

  EXPECT_TRUE(shouldRefreshCompletedLease(true, true, false));
  EXPECT_FALSE(shouldRefreshCompletedLease(true, true, true));
  EXPECT_FALSE(shouldRefreshCompletedLease(true, false, false));
}

TEST(FineAlignmentGuard, RequiresExplicitHealthyActiveLease)
{
  const HoldLimits limits;
  auto input = recordedDeliveryFrame();
  input.guard_active = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.localization_healthy = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.solver_converged = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
}

TEST(FineAlignmentGuard, EnforcesMotionAndSensorBounds)
{
  const HoldLimits limits;
  auto input = recordedDeliveryFrame();
  input.wheel_linear_speed = limits.max_linear_speed + 0.001;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.wheel_travel_from_anchor = limits.max_wheel_travel + 0.001;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.hold_age = limits.max_hold_duration + 0.01;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.wheel_recent = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.lio_recent = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.lio_yaw_jump_detected = true;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.lio_wheel_divergence_delta =
    limits.max_lio_wheel_divergence_delta + 0.01;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
}

TEST(FineAlignmentGuard, RejectsUnsupportedOrGrosslyBadIcp)
{
  const HoldLimits limits;
  auto input = recordedDeliveryFrame();
  input.inlier_count = limits.min_inlier_count - 1;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.inlier_ratio = limits.min_inlier_ratio - 0.001;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.inlier_rmse = limits.max_inlier_rmse + 0.001;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.median_distance = limits.max_median_distance + 0.001;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedDeliveryFrame();
  input.median_distance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
}

}  // namespace
