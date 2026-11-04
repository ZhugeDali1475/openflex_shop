#include "icp_registration/low_support_guard.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{

using icp::low_support_guard::LowSupportInput;
using icp::low_support_guard::LowSupportLimits;
using icp::low_support_guard::shouldAcceptLowSupport;

// Production values from config/icp.yaml.
LowSupportLimits deliveryStationLimits()
{
  LowSupportLimits limits;
  limits.max_lio_wheel_divergence_delta = 0.25;
  limits.max_duration = 25.0;
  limits.min_inlier_ratio = 0.60;
  limits.min_inlier_count = 800;
  limits.residual_tighten = 1.0;
  limits.max_inlier_rmse = 0.15;
  limits.max_median_distance = 0.10;
  return limits;
}

LowSupportInput recordedFrame(
  std::size_t inlier_count, std::size_t valid_count, double rmse, double p50,
  double age)
{
  LowSupportInput input;
  input.solver_converged = true;
  input.localization_healthy = true;
  input.wheel_recent = true;
  input.lio_recent = true;
  input.lio_yaw_jump_detected = false;
  input.lio_wheel_divergence_delta = 0.0;
  input.low_support_age = age;
  input.inlier_count = inlier_count;
  input.inlier_ratio = static_cast<double>(inlier_count) /
    static_cast<double>(valid_count);
  input.inlier_rmse = rmse;
  input.median_distance = p50;
  return input;
}

// Every "Continuous ICP quality accepted=0" frame of the delivery-station
// failure in nav_20260815_113124, transcribed from /rosout.  Age is measured
// from the first rejected frame at +87.07s.
struct RecordedRejection
{
  double age;
  std::size_t inlier_count;
  std::size_t valid_count;
  double inlier_rmse;
  double median_distance;
};

const RecordedRejection kRecordedRejections[] = {
  {0.00, 1099, 1632, 0.132, 0.096},
  {1.00, 1101, 1634, 0.137, 0.100},
  {2.01, 1145, 1702, 0.141, 0.091},
  {3.00, 1145, 1712, 0.140, 0.091},
  {4.01, 1153, 1737, 0.140, 0.095},
  {5.01, 1144, 1741, 0.139, 0.098},
  {6.00, 1172, 1766, 0.142, 0.095},
  {7.00, 1156, 1729, 0.141, 0.089},
  {8.00, 1144, 1727, 0.138, 0.094},
  {9.01, 1170, 1759, 0.144, 0.099},
  {10.01, 1197, 1797, 0.146, 0.098},
  {11.02, 1178, 1798, 0.139, 0.099},
  {18.02, 1154, 1759, 0.143, 0.104},   // only frame over a normal limit (p50)
  {18.95, 1194, 1773, 0.141, 0.094},
  {20.00, 1140, 1700, 0.139, 0.090},
};

LowSupportInput toInput(const RecordedRejection & frame)
{
  return recordedFrame(
    frame.inlier_count, frame.valid_count, frame.inlier_rmse,
    frame.median_distance, frame.age);
}

// The whole point of the channel: these 15 frames latched the chassis off.
// Every one of them failed the normal gate on inlier_ratio alone (0.655~0.674
// against 0.680).  14 of the 15 satisfied the normal residual limits outright,
// so they must be accepted here; only +105.09s (p50=0.104 > 0.100) may not be.
// One isolated rejection is safe -- max_continuous_failures is 3.
TEST(LowSupportGuardTest, AcceptsRecordedRejectedSequence)
{
  const auto limits = deliveryStationLimits();
  std::size_t accepted = 0;
  for (const auto & frame : kRecordedRejections) {
    const bool over_normal_limit =
      frame.inlier_rmse > limits.max_inlier_rmse ||
      frame.median_distance > limits.max_median_distance;
    EXPECT_EQ(
      shouldAcceptLowSupport(toInput(frame), limits),
      !over_normal_limit) << "frame at age " << frame.age;
    accepted += over_normal_limit ? 0u : 1u;
  }
  EXPECT_EQ(accepted, 14u);
}

// Never three in a row, so the failure counter cannot reach
// max_continuous_failures and escalate to recovery.
TEST(LowSupportGuardTest, NeverRejectsThreeConsecutiveRecordedFrames)
{
  const auto limits = deliveryStationLimits();
  int consecutive = 0;
  int worst = 0;
  for (const auto & frame : kRecordedRejections) {
    consecutive = shouldAcceptLowSupport(toInput(frame), limits) ?
      0 : consecutive + 1;
    worst = std::max(worst, consecutive);
  }
  EXPECT_LT(worst, 3);
}

// +98.09s: the worst recorded ratio of the run (1178/1798 = 0.655).
TEST(LowSupportGuardTest, AcceptsWorstRecordedDilutedFrame)
{
  const auto limits = deliveryStationLimits();
  EXPECT_TRUE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, 0.139, 0.099, 11.02), limits));
}

// +106.02s and +107.07s: the chassis has already stopped, so the pure-rotation
// hold no longer applies, yet the ratio is still diluted.  These are the frames
// that would re-latch motion off during fine alignment.
TEST(LowSupportGuardTest, AcceptsStandstillFramesAfterRotationEnds)
{
  const auto limits = deliveryStationLimits();
  EXPECT_TRUE(
    shouldAcceptLowSupport(
      recordedFrame(1194, 1773, 0.141, 0.094, 18.95), limits));
  EXPECT_TRUE(
    shouldAcceptLowSupport(
      recordedFrame(1140, 1700, 0.139, 0.090, 20.00), limits));
}

// A diverging registration degrades residuals together with the ratio.  Same
// diluted ratio, residuals past the limits -> must be rejected.  This is what
// separates dilution from divergence; without it the channel would be a blanket
// ratio bypass.
TEST(LowSupportGuardTest, RejectsDilutedRatioWithLooseResiduals)
{
  const auto limits = deliveryStationLimits();
  EXPECT_FALSE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, limits.max_inlier_rmse + 0.001, 0.091, 1.0),
      limits));
  EXPECT_FALSE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, 0.134, limits.max_median_distance + 0.001, 1.0),
      limits));
}

// residual_tighten scales the residual bounds and is clamped to <= 1.0, so the
// channel can never accept a frame the normal gate rejected on residuals.  At
// the production value of 1.0 the bounds coincide with the normal limits; any
// smaller value must reject a frame that only just satisfies them.
TEST(LowSupportGuardTest, ResidualBoundsNeverExceedNormalLimits)
{
  const auto limits = deliveryStationLimits();
  auto at_normal_limit = recordedFrame(1178, 1798, 0.0, 0.0, 1.0);
  at_normal_limit.inlier_rmse = limits.max_inlier_rmse;
  at_normal_limit.median_distance = limits.max_median_distance;
  EXPECT_TRUE(shouldAcceptLowSupport(at_normal_limit, limits));

  auto tightened = limits;
  tightened.residual_tighten = 0.95;
  EXPECT_FALSE(shouldAcceptLowSupport(at_normal_limit, tightened));
  auto inside_tightened = at_normal_limit;
  inside_tightened.inlier_rmse =
    limits.max_inlier_rmse * tightened.residual_tighten;
  inside_tightened.median_distance =
    limits.max_median_distance * tightened.residual_tighten;
  EXPECT_TRUE(shouldAcceptLowSupport(inside_tightened, tightened));
}

// Support may shrink, but not without bound.
TEST(LowSupportGuardTest, RejectsGenuinelyWeakSupport)
{
  const auto limits = deliveryStationLimits();
  EXPECT_FALSE(
    shouldAcceptLowSupport(
      recordedFrame(900, 1800, 0.134, 0.091, 1.0), limits));

  auto few_inliers = recordedFrame(1178, 1798, 0.134, 0.091, 1.0);
  few_inliers.inlier_count = limits.min_inlier_count - 1;
  EXPECT_FALSE(shouldAcceptLowSupport(few_inliers, limits));

  auto enough_inliers = few_inliers;
  enough_inliers.inlier_count = limits.min_inlier_count;
  EXPECT_TRUE(shouldAcceptLowSupport(enough_inliers, limits));
}

TEST(LowSupportGuardTest, RequiresFreshIndependentObservations)
{
  const auto limits = deliveryStationLimits();
  const auto good = recordedFrame(1178, 1798, 0.134, 0.091, 1.0);
  ASSERT_TRUE(shouldAcceptLowSupport(good, limits));

  auto not_converged = good;
  not_converged.solver_converged = false;
  EXPECT_FALSE(shouldAcceptLowSupport(not_converged, limits));

  auto unhealthy = good;
  unhealthy.localization_healthy = false;
  EXPECT_FALSE(shouldAcceptLowSupport(unhealthy, limits));

  auto stale_wheel = good;
  stale_wheel.wheel_recent = false;
  EXPECT_FALSE(shouldAcceptLowSupport(stale_wheel, limits));

  auto stale_lio = good;
  stale_lio.lio_recent = false;
  EXPECT_FALSE(shouldAcceptLowSupport(stale_lio, limits));

  auto jumped = good;
  jumped.lio_yaw_jump_detected = true;
  EXPECT_FALSE(shouldAcceptLowSupport(jumped, limits));

  auto diverged = good;
  diverged.lio_wheel_divergence_delta =
    limits.max_lio_wheel_divergence_delta + 0.01;
  EXPECT_FALSE(shouldAcceptLowSupport(diverged, limits));

  auto diverged_negative = good;
  diverged_negative.lio_wheel_divergence_delta =
    -(limits.max_lio_wheel_divergence_delta + 0.01);
  EXPECT_FALSE(shouldAcceptLowSupport(diverged_negative, limits));
}

TEST(LowSupportGuardTest, ExpiresIndependentlyOfPureRotationHold)
{
  const auto limits = deliveryStationLimits();
  // The recorded rejection window is 20.0s end to end (+87.07s..+107.07s), well
  // past the 10 s pure-rotation hold cap.  max_duration must cover it with room
  // to spare rather than expire mid-window.
  EXPECT_GT(limits.max_duration, 20.0);
  EXPECT_TRUE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, 0.134, 0.091, 20.0), limits));
  EXPECT_TRUE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, 0.134, 0.091, limits.max_duration), limits));
  EXPECT_FALSE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, 0.134, 0.091, limits.max_duration + 0.01),
      limits));
  EXPECT_FALSE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, 0.134, 0.091, -0.01), limits));
}

TEST(LowSupportGuardTest, RejectsNonFiniteMetrics)
{
  const auto limits = deliveryStationLimits();
  const double inf = std::numeric_limits<double>::infinity();
  const double nan = std::numeric_limits<double>::quiet_NaN();

  auto infinite_rmse = recordedFrame(1178, 1798, inf, 0.091, 1.0);
  EXPECT_FALSE(shouldAcceptLowSupport(infinite_rmse, limits));

  auto nan_rmse = recordedFrame(1178, 1798, nan, 0.091, 1.0);
  EXPECT_FALSE(shouldAcceptLowSupport(nan_rmse, limits));

  auto nan_p50 = recordedFrame(1178, 1798, 0.134, nan, 1.0);
  EXPECT_FALSE(shouldAcceptLowSupport(nan_p50, limits));

  auto nan_ratio = recordedFrame(1178, 1798, 0.134, 0.091, 1.0);
  nan_ratio.inlier_ratio = nan;
  EXPECT_FALSE(shouldAcceptLowSupport(nan_ratio, limits));

  auto nan_divergence = recordedFrame(1178, 1798, 0.134, 0.091, 1.0);
  nan_divergence.lio_wheel_divergence_delta = nan;
  EXPECT_FALSE(shouldAcceptLowSupport(nan_divergence, limits));

  auto nan_age = recordedFrame(1178, 1798, 0.134, 0.091, nan);
  EXPECT_FALSE(shouldAcceptLowSupport(nan_age, limits));
}

// A mis-set residual_tighten above 1.0 must not widen the residual bounds past
// the normal continuous limits.
TEST(LowSupportGuardTest, ClampsResidualTightenToUnity)
{
  auto limits = deliveryStationLimits();
  limits.residual_tighten = 1.5;
  EXPECT_FALSE(
    shouldAcceptLowSupport(
      recordedFrame(1178, 1798, limits.max_inlier_rmse + 0.01, 0.091, 1.0),
      limits));
}

}  // namespace
