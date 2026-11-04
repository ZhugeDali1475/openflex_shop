#ifndef ICP_REGISTRATION__LOW_SUPPORT_GUARD_HPP_
#define ICP_REGISTRATION__LOW_SUPPORT_GUARD_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace icp::low_support_guard
{

struct LowSupportInput
{
  bool solver_converged = false;
  bool localization_healthy = false;
  bool wheel_recent = false;
  bool lio_recent = false;
  bool lio_yaw_jump_detected = false;
  double lio_wheel_divergence_delta = 0.0;
  double low_support_age = 0.0;
  std::size_t inlier_count = 0;
  double inlier_ratio = 0.0;
  double inlier_rmse = 0.0;
  double median_distance = 0.0;
};

struct LowSupportLimits
{
  double max_lio_wheel_divergence_delta = 0.25;
  double max_duration = 25.0;
  double min_inlier_ratio = 0.60;
  std::size_t min_inlier_count = 800;
  double residual_tighten = 1.0;
  double max_inlier_rmse = 0.15;
  double max_median_distance = 0.10;
};

// Unmapped structure near a station dilutes inlier_ratio without degrading the
// correspondences that do match: valid_count grows while inlier_count stays
// flat, so the ratio falls even though inlier_rmse and p50 stay well inside
// their normal limits.  A diverging registration behaves differently — its
// residuals degrade together with the ratio.  This channel accepts the first
// pattern and rejects the second.
//
// It is deliberately the opposite trade from pure_rotation_guard, and the two
// must not be confused:
//
//   pure_rotation_guard  DISCARDS the weak frame and republishes the last
//                        verified map->odom.  Correct while x/y are
//                        unobservable during an in-place spin, and it requires
//                        |omega| >= min_angular_speed, so it stops applying the
//                        moment the rotation ends.
//   low_support_guard    ACCEPTS the frame.  Correct when the pose is still
//                        observable and only the support count shrank, which is
//                        also true while standing still.  Because the result is
//                        published, the residual bounds are tightened below
//                        the normal limits rather than widened, and the channel
//                        carries its own independent duration cap.
//
// Only the support requirement is relaxed here.  residual_tighten scales the
// residual limits and is clamped to <= 1.0, so this channel can never accept a
// frame the normal continuous gate would have rejected on residuals.
//
// Measured basis (nav_20260815_113124, 15 consecutive rejected frames from
// +87.07s to +107.07s): every frame failed on inlier_ratio alone (0.655~0.673
// against 0.680), while 14 of the 15 satisfied the normal residual limits
// outright (inlier_rmse <= 0.146/0.150, p50 <= 0.099/0.100).  Only +105.09s
// exceeded, on p50=0.104.  residual_tighten therefore defaults to 1.0: pulling
// it to 0.95 would reject 7 of the 15 and still let max_continuous_failures
// escalate to recovery.  It stays configurable so a site with genuinely tighter
// residuals can harden the channel without a code change.
inline bool shouldAcceptLowSupport(
  const LowSupportInput & input,
  const LowSupportLimits & limits)
{
  const double tighten = std::clamp(limits.residual_tighten, 0.5, 1.0);
  const bool finite_metrics =
    std::isfinite(input.lio_wheel_divergence_delta) &&
    std::isfinite(input.low_support_age) &&
    std::isfinite(input.inlier_ratio) &&
    std::isfinite(input.inlier_rmse) &&
    std::isfinite(input.median_distance);

  return finite_metrics && input.solver_converged &&
         input.localization_healthy &&
         input.wheel_recent && input.lio_recent &&
         !input.lio_yaw_jump_detected &&
         std::abs(input.lio_wheel_divergence_delta) <=
         limits.max_lio_wheel_divergence_delta &&
         input.low_support_age >= 0.0 &&
         input.low_support_age <= limits.max_duration &&
         input.inlier_count >= limits.min_inlier_count &&
         input.inlier_ratio >= limits.min_inlier_ratio &&
         input.inlier_rmse <= limits.max_inlier_rmse * tighten &&
         input.median_distance <= limits.max_median_distance * tighten;
}

}  // namespace icp::low_support_guard

#endif  // ICP_REGISTRATION__LOW_SUPPORT_GUARD_HPP_
