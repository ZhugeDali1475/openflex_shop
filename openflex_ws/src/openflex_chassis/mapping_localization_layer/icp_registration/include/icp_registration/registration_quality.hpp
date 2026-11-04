#ifndef ICP_REGISTRATION_REGISTRATION_QUALITY_HPP
#define ICP_REGISTRATION_REGISTRATION_QUALITY_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace icp::registration_quality
{

struct RegistrationQuality
{
  std::size_t total_count = 0;
  std::size_t valid_count = 0;
  std::size_t inlier_count = 0;
  double all_mse = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;
  double inlier_mse = std::numeric_limits<double>::infinity();
  double inlier_rmse = std::numeric_limits<double>::infinity();
  double p50_distance = std::numeric_limits<double>::infinity();
  double p90_distance = std::numeric_limits<double>::infinity();
};

inline double quantile(const std::vector<double> & sorted, double probability)
{
  if (sorted.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  const double clamped = std::clamp(probability, 0.0, 1.0);
  const double index = clamped * static_cast<double>(sorted.size() - 1);
  const std::size_t lower = static_cast<std::size_t>(std::floor(index));
  const std::size_t upper = static_cast<std::size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return sorted[lower] + fraction * (sorted[upper] - sorted[lower]);
}

// PCL nearestKSearch returns squared distances.  Accept squared distances here
// and square the configured correspondence distance explicitly so callers do
// not depend on getFitnessScore(max_range)'s easily-misread range semantics.
inline RegistrationQuality summarizeSquaredDistances(
  const std::vector<double> & squared_distances,
  double max_correspondence_distance)
{
  RegistrationQuality quality;
  quality.total_count = squared_distances.size();

  const double safe_max_distance = std::max(0.0, max_correspondence_distance);
  const double max_squared_distance = safe_max_distance * safe_max_distance;
  double all_squared_sum = 0.0;
  double inlier_squared_sum = 0.0;
  std::vector<double> valid_distances;
  valid_distances.reserve(squared_distances.size());

  for (const double squared_distance : squared_distances) {
    if (!std::isfinite(squared_distance) || squared_distance < 0.0) {
      continue;
    }
    ++quality.valid_count;
    all_squared_sum += squared_distance;
    valid_distances.push_back(std::sqrt(squared_distance));
    if (squared_distance <= max_squared_distance) {
      ++quality.inlier_count;
      inlier_squared_sum += squared_distance;
    }
  }

  if (quality.valid_count == 0) {
    return quality;
  }

  quality.all_mse = all_squared_sum / static_cast<double>(quality.valid_count);
  quality.inlier_ratio = static_cast<double>(quality.inlier_count) /
    static_cast<double>(quality.valid_count);
  std::sort(valid_distances.begin(), valid_distances.end());
  quality.p50_distance = quantile(valid_distances, 0.50);
  quality.p90_distance = quantile(valid_distances, 0.90);

  if (quality.inlier_count > 0) {
    quality.inlier_mse = inlier_squared_sum /
      static_cast<double>(quality.inlier_count);
    quality.inlier_rmse = std::sqrt(quality.inlier_mse);
  }
  return quality;
}

inline bool passesThresholds(
  const RegistrationQuality & quality,
  double min_inlier_ratio,
  double max_inlier_rmse,
  double max_p50_distance)
{
  return quality.valid_count > 0 && quality.inlier_count > 0 &&
         std::isfinite(quality.inlier_rmse) &&
         std::isfinite(quality.p50_distance) &&
         quality.inlier_ratio >= min_inlier_ratio &&
         quality.inlier_rmse <= max_inlier_rmse &&
         quality.p50_distance <= max_p50_distance;
}

}  // namespace icp::registration_quality

#endif  // ICP_REGISTRATION_REGISTRATION_QUALITY_HPP
