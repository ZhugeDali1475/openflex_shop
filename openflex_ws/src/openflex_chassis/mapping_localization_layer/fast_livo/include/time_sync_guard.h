#ifndef FAST_LIVO_TIME_SYNC_GUARD_H
#define FAST_LIVO_TIME_SYNC_GUARD_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <optional>

namespace fast_livo
{

inline std::size_t lidarFramesToDrop(
    const std::deque<double> & frame_times,
    std::size_t max_frames,
    double max_backlog_sec)
{
  if (frame_times.size() <= 1) {
    return 0;
  }

  max_frames = std::max<std::size_t>(1, max_frames);
  max_backlog_sec = std::max(0.0, max_backlog_sec);

  std::size_t drop_count = 0;
  while (frame_times.size() - drop_count > 1) {
    const bool too_many = frame_times.size() - drop_count > max_frames;
    const bool too_old =
        frame_times.back() - frame_times[drop_count] > max_backlog_sec;
    if (!too_many && !too_old) {
      break;
    }
    ++drop_count;
  }
  return drop_count;
}

inline bool sensorStreamsOutOfSync(
    double newest_lidar_end_time,
    double newest_imu_time,
    double warn_threshold_sec)
{
  return std::abs(newest_lidar_end_time - newest_imu_time) >
         std::max(0.0, warn_threshold_sec);
}

inline std::optional<std::size_t> nearestTimestampIndex(
    const std::deque<double> & timestamps,
    double target,
    double max_skew_sec,
    double * matched_skew = nullptr)
{
  std::optional<std::size_t> best_index;
  double best_skew = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < timestamps.size(); ++i) {
    const double skew = std::abs(timestamps[i] - target);
    if (std::isfinite(skew) && skew < best_skew) {
      best_skew = skew;
      best_index = i;
    }
  }
  if (matched_skew != nullptr) {
    *matched_skew = best_skew;
  }
  if (!best_index.has_value() || best_skew > std::max(0.0, max_skew_sec)) {
    return std::nullopt;
  }
  return best_index;
}

}  // namespace fast_livo

#endif  // FAST_LIVO_TIME_SYNC_GUARD_H
