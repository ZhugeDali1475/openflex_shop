#ifndef ICP_REGISTRATION__FINE_ALIGNMENT_GUARD_HPP_
#define ICP_REGISTRATION__FINE_ALIGNMENT_GUARD_HPP_

#include <cmath>
#include <cstddef>

namespace icp::fine_alignment_guard
{

struct HoldInput
{
  bool guard_active = false;
  bool solver_converged = false;
  bool localization_healthy = false;
  bool wheel_recent = false;
  bool lio_recent = false;
  bool lio_yaw_jump_detected = false;
  double wheel_linear_speed = 0.0;
  double wheel_travel_from_anchor = 0.0;
  double lio_wheel_divergence_delta = 0.0;
  double hold_age = 0.0;
  std::size_t inlier_count = 0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  double median_distance = 0.0;
};

struct HoldLimits
{
  double max_linear_speed = 0.10;
  double max_wheel_travel = 0.45;
  double max_lio_wheel_divergence_delta = 0.25;
  double max_hold_duration = 30.0;
  std::size_t min_inlier_count = 800;
  double min_inlier_ratio = 0.65;
  double max_inlier_rmse = 0.165;
  double max_median_distance = 0.120;
};

inline bool shouldDeferCompletedRelease(bool guard_active, bool completed)
{
  return guard_active && completed;
}

inline bool shouldRefreshCompletedLease(
  bool guard_active, bool completed, bool release_pending)
{
  return guard_active && completed && !release_pending;
}

// During a bounded local fine-alignment operation, weak ICP results may be
// discarded while the last verified map->odom transform is retained. Local
// wheel/LIO odometry still propagates motion. This never authorizes publishing
// the weak ICP result itself.
inline bool shouldKeepLastTransform(
  const HoldInput & input, const HoldLimits & limits)
{
  const bool finite_metrics =
    std::isfinite(input.wheel_linear_speed) &&
    std::isfinite(input.wheel_travel_from_anchor) &&
    std::isfinite(input.lio_wheel_divergence_delta) &&
    std::isfinite(input.hold_age) &&
    std::isfinite(input.inlier_ratio) &&
    std::isfinite(input.inlier_rmse) &&
    std::isfinite(input.median_distance);

  return finite_metrics && input.guard_active && input.solver_converged &&
         input.localization_healthy && input.wheel_recent && input.lio_recent &&
         !input.lio_yaw_jump_detected &&
         std::abs(input.wheel_linear_speed) <= limits.max_linear_speed &&
         input.wheel_travel_from_anchor <= limits.max_wheel_travel &&
         std::abs(input.lio_wheel_divergence_delta) <=
         limits.max_lio_wheel_divergence_delta &&
         input.hold_age >= 0.0 &&
         input.hold_age <= limits.max_hold_duration &&
         input.inlier_count >= limits.min_inlier_count &&
         input.inlier_ratio >= limits.min_inlier_ratio &&
         input.inlier_rmse <= limits.max_inlier_rmse &&
         input.median_distance <= limits.max_median_distance;
}

}  // namespace icp::fine_alignment_guard

#endif  // ICP_REGISTRATION__FINE_ALIGNMENT_GUARD_HPP_
