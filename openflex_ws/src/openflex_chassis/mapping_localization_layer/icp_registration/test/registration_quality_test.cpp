#include "icp_registration/registration_quality.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <vector>

namespace
{

using icp::registration_quality::passesThresholds;
using icp::registration_quality::summarizeSquaredDistances;

TEST(RegistrationQuality, SeparatesInliersFromAllPointMean)
{
  const auto quality = summarizeSquaredDistances(
    std::vector<double>{0.01, 0.04, 0.09, 1.00}, 0.5);

  EXPECT_EQ(quality.total_count, 4U);
  EXPECT_EQ(quality.valid_count, 4U);
  EXPECT_EQ(quality.inlier_count, 3U);
  EXPECT_NEAR(quality.all_mse, 0.285, 1e-12);
  EXPECT_NEAR(quality.inlier_ratio, 0.75, 1e-12);
  EXPECT_NEAR(quality.inlier_mse, 0.14 / 3.0, 1e-12);
  EXPECT_NEAR(quality.p50_distance, 0.25, 1e-12);
  EXPECT_NEAR(quality.p90_distance, 0.79, 1e-12);
}

TEST(RegistrationQuality, IgnoresInvalidNearestNeighborResults)
{
  const auto quality = summarizeSquaredDistances(
    std::vector<double>{0.01, -1.0, std::numeric_limits<double>::infinity(), 0.04},
    0.5);

  EXPECT_EQ(quality.total_count, 4U);
  EXPECT_EQ(quality.valid_count, 2U);
  EXPECT_EQ(quality.inlier_count, 2U);
  EXPECT_NEAR(quality.inlier_ratio, 1.0, 1e-12);
  EXPECT_NEAR(quality.inlier_rmse, std::sqrt(0.025), 1e-12);
}

TEST(RegistrationQuality, AppliesAllRobustThresholds)
{
  const auto recorded_good = summarizeSquaredDistances(
    std::vector<double>{
      0.0016, 0.0025, 0.0036, 0.0049, 0.0064, 0.0900, 0.8100}, 0.5);

  EXPECT_TRUE(passesThresholds(recorded_good, 0.80, 0.15, 0.10));
  EXPECT_FALSE(passesThresholds(recorded_good, 0.90, 0.15, 0.10));
  EXPECT_FALSE(passesThresholds(recorded_good, 0.80, 0.10, 0.10));
  EXPECT_FALSE(passesThresholds(recorded_good, 0.80, 0.15, 0.06));
}

}  // namespace
