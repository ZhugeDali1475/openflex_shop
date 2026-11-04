#include "icp_registration/pure_rotation_guard.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{

using icp::pure_rotation_guard::WeakQualityHoldInput;
using icp::pure_rotation_guard::WeakQualityHoldLimits;
using icp::pure_rotation_guard::PostRotationWeakQualityHoldInput;
using icp::pure_rotation_guard::PostRotationWeakQualityHoldLimits;
using icp::pure_rotation_guard::shouldKeepLastTransform;
using icp::pure_rotation_guard::shouldKeepLastTransformAfterRotation;

WeakQualityHoldInput recordedFailure(double rmse, double p50, double age)
{
  WeakQualityHoldInput input;
  input.solver_converged = true;
  input.localization_healthy = true;
  input.rotation_anchor_valid = true;
  input.wheel_recent = true;
  input.lio_recent = true;
  input.wheel_linear_speed = 0.0;
  input.wheel_angular_speed = 0.20;
  input.wheel_travel_from_anchor = 0.0;
  input.lio_wheel_divergence_delta = 0.02;
  input.quality_hold_age = age;
  input.inlier_ratio = 0.92;
  input.inlier_rmse = rmse;
  input.median_distance = p50;
  return input;
}

TEST(PureRotationGuard, KeepsLastTransformForRecordedBorderlineFailures)
{
  const WeakQualityHoldLimits limits;

  // nav_20260725_145740 at +64.7s, +65.7s and +66.7s.  These samples
  // narrowly crossed the normal RMSE/p50 limits during a verified pure spin.
  EXPECT_TRUE(
    shouldKeepLastTransform(
      recordedFailure(0.151, 0.098, 0.0), limits));
  EXPECT_TRUE(
    shouldKeepLastTransform(
      recordedFailure(0.152, 0.103, 1.0), limits));
  EXPECT_TRUE(
    shouldKeepLastTransform(
      recordedFailure(0.151, 0.085, 2.0), limits));
}

// The delivery station (nav_20260815_113124) failed on the lower-bound metric
// instead: unmapped structure inflated valid_count from 1589 to 1798 while
// inlier_count stayed flat, so inlier_ratio was diluted to 0.655..0.674 even
// though inlier_rmse (<=0.146) and p50 (<=0.104) stayed inside the normal
// limits.  Production loads margin 1.10 from icp.yaml for this case.
WeakQualityHoldLimits deliveryStationLimits()
{
  WeakQualityHoldLimits limits;
  limits.min_inlier_ratio = 0.68;
  limits.quality_margin_factor = 1.10;
  return limits;
}

TEST(PureRotationGuard, KeepsLastTransformForRecordedInlierRatioDip)
{
  const WeakQualityHoldLimits limits = deliveryStationLimits();

  // +87.07s, first rejection of the run.
  auto input = recordedFailure(0.132, 0.096, 0.0);
  input.inlier_ratio = 0.673;
  EXPECT_TRUE(shouldKeepLastTransform(input, limits));

  // +97.08s, lowest ratio observed in the whole 80-frame sequence.
  input = recordedFailure(0.139, 0.099, 5.0);
  input.inlier_ratio = 0.655;
  EXPECT_TRUE(shouldKeepLastTransform(input, limits));

  // A genuinely degraded ratio must still take the normal failure path.
  input = recordedFailure(0.141, 0.094, 2.0);
  input.inlier_ratio = 0.60;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
}

TEST(PureRotationGuard, WidensTheInlierRatioBoundOnlyByTheMargin)
{
  WeakQualityHoldLimits limits = deliveryStationLimits();
  const double margin = limits.quality_margin_factor;

  // Expressed as a quotient on purpose: the literal 0.6182 would multiply back
  // to 0.68002 and pass, while 0.618 gives 0.6798 and fails.
  auto input = recordedFailure(0.132, 0.096, 1.0);
  input.inlier_ratio = limits.min_inlier_ratio / margin;
  EXPECT_TRUE(shouldKeepLastTransform(input, limits));

  input.inlier_ratio = limits.min_inlier_ratio / margin - 0.01;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));

  // Without a margin the guard must fall back to the plain limit, proving the
  // tolerance comes from quality_margin_factor alone.
  limits.quality_margin_factor = 1.0;
  input.inlier_ratio = 0.679;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input.inlier_ratio = 0.68;
  EXPECT_TRUE(shouldKeepLastTransform(input, limits));
}

TEST(PureRotationGuard, InlierRatioDipStillRequiresEveryOtherGate)
{
  const WeakQualityHoldLimits limits = deliveryStationLimits();

  // Ratio is inside the widened bound, so each rejection below is caused by the
  // independent evidence gate under test.
  const auto diluted = [] {
      auto input = recordedFailure(0.139, 0.099, 1.0);
      input.inlier_ratio = 0.655;
      return input;
    };

  auto input = diluted();
  input.wheel_angular_speed = 0.09;   // rotation finished; guard must not apply
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.wheel_travel_from_anchor = 0.051;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.lio_wheel_divergence_delta = 0.26;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.wheel_recent = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.lio_recent = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.lio_yaw_jump_detected = true;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.quality_hold_age = 10.01;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));

  // Ratio inside the bound must not excuse a bad residual.
  input = diluted();
  input.inlier_rmse = 0.20;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.median_distance = 0.15;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));

  input = diluted();
  input.inlier_ratio = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = diluted();
  input.inlier_ratio = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
}

TEST(PureRotationGuard, DoesNotAcceptTheRelaxedIcpResult)
{
  // The helper deliberately returns only permission to retain the previous
  // transform.  Grossly bad ICP quality must still take the normal failure
  // path even when wheel motion looks like a pure rotation.
  const WeakQualityHoldLimits limits;
  EXPECT_FALSE(
    shouldKeepLastTransform(
      recordedFailure(0.20, 0.15, 1.0), limits));
  EXPECT_FALSE(
    shouldKeepLastTransform(
      recordedFailure(0.158, 0.103, 1.0), limits));
}

TEST(PureRotationGuard, RequiresIndependentMotionAndSensorGates)
{
  const WeakQualityHoldLimits limits;
  auto input = recordedFailure(0.152, 0.103, 1.0);

  input.solver_converged = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.localization_healthy = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.rotation_anchor_valid = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.wheel_linear_speed = 0.06;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.wheel_angular_speed = 0.09;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.wheel_travel_from_anchor = 0.051;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.wheel_recent = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.lio_recent = false;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.lio_yaw_jump_detected = true;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
  input = recordedFailure(0.152, 0.103, 1.0);
  input.lio_wheel_divergence_delta = 0.26;
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
}

TEST(PureRotationGuard, IsBoundedAndFailsClosedOnInvalidData)
{
  const WeakQualityHoldLimits limits;
  EXPECT_FALSE(
    shouldKeepLastTransform(
      recordedFailure(0.152, 0.103, 10.01), limits));

  auto input = recordedFailure(0.152, 0.103, 1.0);
  input.inlier_rmse = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shouldKeepLastTransform(input, limits));
}

PostRotationWeakQualityHoldInput recordedPostRotationFrame(
  double rmse, double p50, double rotation_age, double wheel_travel,
  double wheel_speed = 0.0)
{
  PostRotationWeakQualityHoldInput input;
  input.solver_converged = true;
  input.localization_healthy = true;
  input.rotation_anchor_valid = true;
  input.wheel_recent = true;
  input.lio_recent = true;
  input.wheel_linear_speed = wheel_speed;
  input.wheel_travel_from_anchor = wheel_travel;
  input.lio_wheel_divergence_delta = 0.02;
  input.rotation_age = rotation_age;
  input.inlier_ratio = 0.72;
  input.inlier_rmse = rmse;
  input.median_distance = p50;
  return input;
}

PostRotationWeakQualityHoldLimits competitionTransitionLimits()
{
  PostRotationWeakQualityHoldLimits limits;
  limits.max_linear_speed = 0.20;
  limits.max_wheel_travel = 0.20;
  limits.max_lio_wheel_divergence_delta = 0.25;
  limits.max_duration = 8.0;
  limits.min_inlier_ratio = 0.68;
  limits.max_inlier_rmse = 0.15;
  limits.max_median_distance = 0.10;
  limits.quality_margin_factor = 1.10;
  return limits;
}

TEST(PostRotationGuard, KeepsTrustedTransformForCompetitionFailureSequence)
{
  const auto limits = competitionTransitionLimits();

  // nav_20260815_145007 at 14:56:59.778, 14:57:00.768 and
  // 14:57:01.771. These three marginal residual failures revoked localization
  // even though support and wheel/LIO agreement remained healthy.
  EXPECT_TRUE(shouldKeepLastTransformAfterRotation(
    recordedPostRotationFrame(0.153, 0.098, 0.8, 0.00), limits));
  EXPECT_TRUE(shouldKeepLastTransformAfterRotation(
    recordedPostRotationFrame(0.1501, 0.086, 1.8, 0.00), limits));
  EXPECT_TRUE(shouldKeepLastTransformAfterRotation(
    recordedPostRotationFrame(0.153, 0.095, 2.8, 0.04, 0.123), limits));
}

TEST(PostRotationGuard, DoesNotRelaxTheNormalSupportRequirement)
{
  const auto limits = competitionTransitionLimits();
  auto input = recordedPostRotationFrame(0.153, 0.095, 2.0, 0.04);
  input.inlier_ratio = limits.min_inlier_ratio - 0.001;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));

  input.inlier_ratio = limits.min_inlier_ratio;
  EXPECT_TRUE(shouldKeepLastTransformAfterRotation(input, limits));
}

TEST(PostRotationGuard, RejectsExpiredOrUnboundedTransition)
{
  const auto limits = competitionTransitionLimits();
  const auto good = recordedPostRotationFrame(0.153, 0.095, 2.0, 0.04);
  ASSERT_TRUE(shouldKeepLastTransformAfterRotation(good, limits));

  auto input = good;
  input.rotation_age = limits.max_duration + 0.01;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.wheel_travel_from_anchor = limits.max_wheel_travel + 0.001;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.wheel_linear_speed = limits.max_linear_speed + 0.001;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.inlier_rmse = limits.max_inlier_rmse *
    limits.quality_margin_factor + 0.001;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.median_distance = limits.max_median_distance *
    limits.quality_margin_factor + 0.001;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
}

TEST(PostRotationGuard, RequiresFreshIndependentSensorAgreement)
{
  const auto limits = competitionTransitionLimits();
  const auto good = recordedPostRotationFrame(0.153, 0.095, 2.0, 0.04);

  auto input = good;
  input.rotation_anchor_valid = false;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.wheel_recent = false;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.lio_recent = false;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.lio_yaw_jump_detected = true;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = good;
  input.lio_wheel_divergence_delta =
    limits.max_lio_wheel_divergence_delta + 0.01;
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
}

TEST(PostRotationGuard, FailsClosedOnInvalidMetrics)
{
  const auto limits = competitionTransitionLimits();
  auto input = recordedPostRotationFrame(0.153, 0.095, 2.0, 0.04);
  input.inlier_rmse = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
  input = recordedPostRotationFrame(0.153, 0.095, 2.0, 0.04);
  input.rotation_age = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(shouldKeepLastTransformAfterRotation(input, limits));
}

}  // namespace
