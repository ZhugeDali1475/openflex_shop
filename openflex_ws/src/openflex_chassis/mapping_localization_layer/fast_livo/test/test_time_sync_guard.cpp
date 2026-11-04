#include <gtest/gtest.h>

#include <deque>

#include "time_sync_guard.h"

TEST(TimeSyncGuardTest, KeepsHealthyLidarWindow)
{
  const std::deque<double> times{10.0, 10.1, 10.2};
  EXPECT_EQ(fast_livo::lidarFramesToDrop(times, 3, 0.25), 0u);
}

TEST(TimeSyncGuardTest, DropsFramesBeyondCountLimit)
{
  const std::deque<double> times{10.0, 10.1, 10.2, 10.3, 10.4};
  EXPECT_EQ(fast_livo::lidarFramesToDrop(times, 3, 1.0), 2u);
}

TEST(TimeSyncGuardTest, DropsFramesBeyondTimeWindow)
{
  const std::deque<double> times{10.0, 10.1, 10.2, 10.3};
  EXPECT_EQ(fast_livo::lidarFramesToDrop(times, 10, 0.25), 1u);
}

TEST(TimeSyncGuardTest, RetainsAtLeastNewestFrame)
{
  const std::deque<double> times{10.0, 11.0};
  EXPECT_EQ(fast_livo::lidarFramesToDrop(times, 1, 0.0), 1u);
}

TEST(TimeSyncGuardTest, DetectsLatestDataSkew)
{
  EXPECT_FALSE(fast_livo::sensorStreamsOutOfSync(10.10, 10.0, 0.25));
  EXPECT_TRUE(fast_livo::sensorStreamsOutOfSync(10.30, 10.0, 0.25));
  EXPECT_TRUE(fast_livo::sensorStreamsOutOfSync(9.70, 10.0, 0.25));
}

TEST(TimeSyncGuardTest, SelectsNearestPoseWithinBound)
{
  const std::deque<double> times{10.0, 10.1, 10.2};
  double skew = 0.0;
  const auto index = fast_livo::nearestTimestampIndex(times, 10.12, 0.15, &skew);
  ASSERT_TRUE(index.has_value());
  EXPECT_EQ(*index, 1u);
  EXPECT_NEAR(skew, 0.02, 1e-9);
}

TEST(TimeSyncGuardTest, RejectsNearestPoseOutsideBound)
{
  const std::deque<double> times{5.0, 5.1, 5.2};
  double skew = 0.0;
  EXPECT_FALSE(
      fast_livo::nearestTimestampIndex(times, 10.0, 0.15, &skew).has_value());
  EXPECT_NEAR(skew, 4.8, 1e-9);
}
