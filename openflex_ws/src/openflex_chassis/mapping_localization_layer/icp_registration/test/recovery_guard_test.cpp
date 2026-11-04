#include "icp_registration/recovery_guard.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

using icp::recovery_guard::GateResult;
using icp::recovery_guard::PlanarPose;
using icp::recovery_guard::StabilityWindow;
using icp::recovery_guard::acceptCandidateSample;
using icp::recovery_guard::evaluateCandidate;
using icp::recovery_guard::normalizeAngle;
using icp::recovery_guard::propagateFromWheel;
using icp::recovery_guard::recordFailureSample;

TEST(RecoveryGuard, PropagatesWheelMotionInTrustedMapFrame) {
  const PlanarPose trusted_map{2.0, 3.0, M_PI_2};
  const PlanarPose trusted_wheel{10.0, -4.0, -0.4};
  const PlanarPose current_wheel{
    trusted_wheel.x + std::cos(trusted_wheel.yaw),
    trusted_wheel.y + std::sin(trusted_wheel.yaw),
    trusted_wheel.yaw + 0.2};

  const PlanarPose predicted = propagateFromWheel(
    trusted_map, trusted_wheel, current_wheel);

  EXPECT_NEAR(predicted.x, 2.0, 1e-9);
  EXPECT_NEAR(predicted.y, 4.0, 1e-9);
  EXPECT_NEAR(predicted.yaw, M_PI_2 + 0.2, 1e-9);
}

TEST(RecoveryGuard, PureWheelRotationKeepsTrustedMapTranslation) {
  const PlanarPose trusted_map{-1.304, -1.613, 2.77};
  const PlanarPose trusted_wheel{-1.924, -0.224, -2.56};
  const PlanarPose current_wheel{
    trusted_wheel.x, trusted_wheel.y, trusted_wheel.yaw - 1.95};

  const PlanarPose predicted = propagateFromWheel(
    trusted_map, trusted_wheel, current_wheel);

  EXPECT_NEAR(predicted.x, trusted_map.x, 1e-9);
  EXPECT_NEAR(predicted.y, trusted_map.y, 1e-9);
  EXPECT_NEAR(
    normalizeAngle(predicted.yaw),
    normalizeAngle(trusted_map.yaw - 1.95), 1e-9);
}

TEST(RecoveryGuard, AcceptsCandidateNearTrustedWheelPrediction) {
  const PlanarPose trusted_map{-1.31, -0.36, -1.68};
  const PlanarPose trusted_wheel{-1.28, 1.34, -1.66};
  const PlanarPose current_wheel{-1.13, 1.27, -0.66};
  const PlanarPose predicted = propagateFromWheel(
    trusted_map, trusted_wheel, current_wheel);
  const PlanarPose candidate{
    predicted.x + 0.04, predicted.y - 0.03, predicted.yaw + 0.02};

  const GateResult result = evaluateCandidate(
    trusted_map, trusted_wheel, current_wheel, candidate, 0.20, 0.30);

  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.position_error, 0.05, 1e-9);
  EXPECT_NEAR(result.yaw_error, 0.02, 1e-9);
}

TEST(RecoveryGuard, RejectsRecordedWrongShelfCandidate) {
  // nav_20260723_085421: the trusted/wheel-propagated x was about -1.15 m,
  // while ambiguous global ICP staged a candidate near x=-0.22 m.
  const PlanarPose trusted_map{-1.31, -0.36, -1.68};
  const PlanarPose trusted_wheel{-1.28, 1.34, -1.66};
  const PlanarPose current_wheel{-1.05, 1.23, -0.60};
  const PlanarPose wrong_candidate{-0.22, -0.32, -0.93};

  const GateResult result = evaluateCandidate(
    trusted_map, trusted_wheel, current_wheel, wrong_candidate, 0.20, 0.30);

  EXPECT_FALSE(result.accepted);
  EXPECT_GT(result.position_error, 0.70);
}

TEST(RecoveryGuard, HandlesYawWrapAtPi) {
  const PlanarPose trusted_map{0.0, 0.0, M_PI - 0.02};
  const PlanarPose trusted_wheel{0.0, 0.0, 0.0};
  const PlanarPose current_wheel{0.0, 0.0, 0.03};
  const PlanarPose candidate{0.0, 0.0, -M_PI + 0.015};

  const GateResult result = evaluateCandidate(
    trusted_map, trusted_wheel, current_wheel, candidate, 0.20, 0.05);

  EXPECT_TRUE(result.accepted);
  EXPECT_NEAR(result.yaw_error, 0.005, 1e-9);
  EXPECT_NEAR(normalizeAngle(result.predicted.yaw), -M_PI + 0.01, 1e-9);
}

TEST(RecoveryGuard, StabilityWindowAllowsOneBorderlineFrame) {
  StabilityWindow window(5, 4);
  window.add(true);
  window.add(true);
  window.add(true);
  window.add(false);
  EXPECT_FALSE(window.ready());
  window.add(true);
  EXPECT_TRUE(window.ready());
  EXPECT_EQ(window.goodCount(), 4u);
  EXPECT_EQ(window.sampleCount(), 5u);
}

TEST(RecoveryGuard, StabilityWindowDoesNotAccumulateOldSuccessesForever) {
  StabilityWindow window(5, 4);
  for (int i = 0; i < 4; ++i) {
    window.add(true);
  }
  EXPECT_TRUE(window.ready());
  window.add(false);
  window.add(false);
  EXPECT_FALSE(window.ready());
  EXPECT_EQ(window.sampleCount(), 5u);
  EXPECT_EQ(window.goodCount(), 3u);
}

TEST(RecoveryGuard, UnhealthyFailureDoesNotEraseRecentConfirmations) {
  StabilityWindow window(5, 4);
  window.add(true);
  window.add(true);
  window.add(true);

  recordFailureSample(window, false);
  EXPECT_EQ(window.goodCount(), 3u);
  EXPECT_EQ(window.sampleCount(), 4u);
  EXPECT_FALSE(window.ready());

  window.add(true);
  EXPECT_TRUE(window.ready());
}

TEST(RecoveryGuard, AlreadyRecordedFailureIsNotCountedTwice) {
  StabilityWindow window(5, 4);
  window.add(true);
  window.add(false);

  recordFailureSample(window, false, true);
  EXPECT_EQ(window.goodCount(), 1u);
  EXPECT_EQ(window.sampleCount(), 2u);
}

TEST(RecoveryGuard, HealthyTrackingFailureStartsFreshRecoveryWindow) {
  StabilityWindow window(5, 4);
  window.add(true);
  window.add(true);

  recordFailureSample(window, true);
  EXPECT_EQ(window.goodCount(), 0u);
  EXPECT_EQ(window.sampleCount(), 0u);
}

TEST(RecoveryGuard, FirstFineFrameMayRefineCoarseCandidate) {
  EXPECT_TRUE(acceptCandidateSample(true, 0, true, false, true));
  EXPECT_FALSE(acceptCandidateSample(true, 1, true, false, true));
  EXPECT_TRUE(acceptCandidateSample(true, 1, true, true, true));
}

TEST(RecoveryGuard, FirstFineFrameStillRequiresIndependentSafetyGates) {
  EXPECT_FALSE(acceptCandidateSample(true, 0, false, true, true));
  EXPECT_FALSE(acceptCandidateSample(true, 0, true, true, false));
}

}  // namespace
