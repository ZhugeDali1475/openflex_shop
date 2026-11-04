#ifndef ICP_REGISTRATION__PURE_ROTATION_GUARD_HPP_
#define ICP_REGISTRATION__PURE_ROTATION_GUARD_HPP_

#include <algorithm>
#include <cmath>

namespace icp::pure_rotation_guard
{

struct WeakQualityHoldInput
{
  bool solver_converged = false;
  bool localization_healthy = false;
  bool rotation_anchor_valid = false;
  bool wheel_recent = false;
  bool lio_recent = false;
  bool lio_yaw_jump_detected = false;
  double wheel_linear_speed = 0.0;
  double wheel_angular_speed = 0.0;
  double wheel_travel_from_anchor = 0.0;
  double lio_wheel_divergence_delta = 0.0;
  double quality_hold_age = 0.0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  double median_distance = 0.0;
};

struct WeakQualityHoldLimits
{
  double max_linear_speed = 0.05;
  double min_angular_speed = 0.10;
  double max_wheel_travel = 0.05;
  double max_lio_wheel_divergence_delta = 0.25;
  double max_hold_duration = 10.0;
  double min_inlier_ratio = 0.80;
  double max_inlier_rmse = 0.15;
  double max_median_distance = 0.10;
  double quality_margin_factor = 1.05;
};

struct PostRotationWeakQualityHoldInput
{
  bool solver_converged = false;
  bool localization_healthy = false;
  bool rotation_anchor_valid = false;
  bool wheel_recent = false;
  bool lio_recent = false;
  bool lio_yaw_jump_detected = false;
  double wheel_linear_speed = 0.0;
  double wheel_travel_from_anchor = 0.0;
  double lio_wheel_divergence_delta = 0.0;
  double rotation_age = 0.0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  double median_distance = 0.0;
};

struct PostRotationWeakQualityHoldLimits
{
  double max_linear_speed = 0.20;
  double max_wheel_travel = 0.20;
  double max_lio_wheel_divergence_delta = 0.25;
  double max_duration = 8.0;
  double min_inlier_ratio = 0.68;
  double max_inlier_rmse = 0.15;
  double max_median_distance = 0.10;
  double quality_margin_factor = 1.10;
};

// Continuous ICP is weakly observable at some headings during an in-place
// rotation.  A borderline quality failure may therefore keep the last
// verified map->odom transform, but only while independent wheel/LIO evidence
// proves that the chassis is still executing the same bounded pure rotation.
// No relaxed ICP result is accepted or published by this decision.
//
// quality_margin_factor widens all three quality metrics symmetrically.  The
// two upper bounds scale the limit up (limit * margin); inlier_ratio is a lower
// bound, so the measurement scales up instead (value * margin >= limit), which
// is the multiplicative equivalent of limit / margin without a divide.
inline bool shouldKeepLastTransform(
  const WeakQualityHoldInput & input,
  const WeakQualityHoldLimits & limits)
{
  const double margin = std::max(1.0, limits.quality_margin_factor);
  const bool finite_metrics =
    std::isfinite(input.wheel_linear_speed) &&
    std::isfinite(input.wheel_angular_speed) &&
    std::isfinite(input.wheel_travel_from_anchor) &&
    std::isfinite(input.lio_wheel_divergence_delta) &&
    std::isfinite(input.quality_hold_age) &&
    std::isfinite(input.inlier_ratio) &&
    std::isfinite(input.inlier_rmse) &&
    std::isfinite(input.median_distance);

  return finite_metrics && input.solver_converged &&
         input.localization_healthy && input.rotation_anchor_valid &&
         input.wheel_recent && input.lio_recent &&
         !input.lio_yaw_jump_detected &&
         input.wheel_linear_speed <= limits.max_linear_speed &&
         std::abs(input.wheel_angular_speed) >= limits.min_angular_speed &&
         input.wheel_travel_from_anchor <= limits.max_wheel_travel &&
         std::abs(input.lio_wheel_divergence_delta) <=
         limits.max_lio_wheel_divergence_delta &&
         input.quality_hold_age >= 0.0 &&
         input.quality_hold_age <= limits.max_hold_duration &&
         input.inlier_ratio * margin >= limits.min_inlier_ratio &&
         input.inlier_rmse <= limits.max_inlier_rmse * margin &&
         input.median_distance <= limits.max_median_distance * margin;
}

// The scan distribution remains unsettled briefly after a large in-place
// rotation, even after wheel angular speed drops below the pure-rotation gate.
// Keep the last verified map->odom for a bounded, low-speed exit from that
// geometry. The weak ICP result is discarded, and normal support is still
// required; only RMSE/p50 receive the narrow configured margin.
inline bool shouldKeepLastTransformAfterRotation(
  const PostRotationWeakQualityHoldInput & input,
  const PostRotationWeakQualityHoldLimits & limits)
{
  const double margin = std::max(1.0, limits.quality_margin_factor);
  const bool finite_metrics =
    std::isfinite(input.wheel_linear_speed) &&
    std::isfinite(input.wheel_travel_from_anchor) &&
    std::isfinite(input.lio_wheel_divergence_delta) &&
    std::isfinite(input.rotation_age) &&
    std::isfinite(input.inlier_ratio) &&
    std::isfinite(input.inlier_rmse) &&
    std::isfinite(input.median_distance);

  return finite_metrics && input.solver_converged &&
         input.localization_healthy && input.rotation_anchor_valid &&
         input.wheel_recent && input.lio_recent &&
         !input.lio_yaw_jump_detected &&
         std::abs(input.wheel_linear_speed) <= limits.max_linear_speed &&
         input.wheel_travel_from_anchor <= limits.max_wheel_travel &&
         std::abs(input.lio_wheel_divergence_delta) <=
         limits.max_lio_wheel_divergence_delta &&
         input.rotation_age >= 0.0 &&
         input.rotation_age <= limits.max_duration &&
         input.inlier_ratio >= limits.min_inlier_ratio &&
         input.inlier_rmse <= limits.max_inlier_rmse * margin &&
         input.median_distance <= limits.max_median_distance * margin;
}

}  // namespace icp::pure_rotation_guard

#endif  // ICP_REGISTRATION__PURE_ROTATION_GUARD_HPP_
