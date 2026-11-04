#ifndef ICP_REGISTRATION_RECOVERY_GUARD_HPP
#define ICP_REGISTRATION_RECOVERY_GUARD_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>

namespace icp::recovery_guard
{

struct PlanarPose
{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct GateResult
{
  PlanarPose predicted;
  double position_error = 0.0;
  double yaw_error = 0.0;
  bool accepted = false;
};

class StabilityWindow
{
public:
  StabilityWindow(std::size_t window_size = 5, std::size_t required_good = 4)
  {
    configure(window_size, required_good);
  }

  void configure(std::size_t window_size, std::size_t required_good)
  {
    window_size_ = std::max<std::size_t>(1, window_size);
    required_good_ = std::max<std::size_t>(1, std::min(required_good, window_size_));
    reset();
  }

  void reset()
  {
    samples_.clear();
    good_count_ = 0;
  }

  void add(bool good)
  {
    samples_.push_back(good);
    if (good) {
      ++good_count_;
    }
    while (samples_.size() > window_size_) {
      if (samples_.front()) {
        --good_count_;
      }
      samples_.pop_front();
    }
  }

  std::size_t goodCount() const {return good_count_;}
  std::size_t sampleCount() const {return samples_.size();}
  bool ready() const {return good_count_ >= required_good_;}

private:
  std::size_t window_size_{5};
  std::size_t required_good_{4};
  std::size_t good_count_{0};
  std::deque<bool> samples_;
};

inline void recordFailureSample(
  StabilityWindow & window,
  bool localization_was_healthy,
  bool sample_already_recorded = false)
{
  if (localization_was_healthy) {
    window.reset();
  } else if (!sample_already_recorded) {
    window.add(false);
  }
}

inline bool acceptCandidateSample(
  bool candidate_exists,
  std::size_t verification_sample_count,
  bool quality_ok,
  bool candidate_consistent,
  bool wheel_gate_ok)
{
  if (!quality_ok || !wheel_gate_ok) {
    return false;
  }

  // The global/manual result is a coarse seed. The first independently
  // verified local ICP frame establishes the fine baseline; the wheel gate
  // still bounds that correction before later frames use the strict gate.
  if (!candidate_exists || verification_sample_count == 0) {
    return true;
  }
  return candidate_consistent;
}

inline double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

inline PlanarPose compose(const PlanarPose & lhs, const PlanarPose & rhs)
{
  const double c = std::cos(lhs.yaw);
  const double s = std::sin(lhs.yaw);
  return PlanarPose{
    lhs.x + c * rhs.x - s * rhs.y,
    lhs.y + s * rhs.x + c * rhs.y,
    normalizeAngle(lhs.yaw + rhs.yaw)};
}

inline PlanarPose inverse(const PlanarPose & pose)
{
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return PlanarPose{
    -(c * pose.x + s * pose.y),
    -(-s * pose.x + c * pose.y),
    normalizeAngle(-pose.yaw)};
}

inline PlanarPose propagateFromWheel(
  const PlanarPose & trusted_map_pose,
  const PlanarPose & trusted_wheel_pose,
  const PlanarPose & current_wheel_pose)
{
  return compose(
    trusted_map_pose,
    compose(inverse(trusted_wheel_pose), current_wheel_pose));
}

inline GateResult evaluateCandidate(
  const PlanarPose & trusted_map_pose,
  const PlanarPose & trusted_wheel_pose,
  const PlanarPose & current_wheel_pose,
  const PlanarPose & candidate_map_pose,
  double max_position_error,
  double max_yaw_error)
{
  GateResult result;
  result.predicted = propagateFromWheel(
    trusted_map_pose, trusted_wheel_pose, current_wheel_pose);
  result.position_error = std::hypot(
    candidate_map_pose.x - result.predicted.x,
    candidate_map_pose.y - result.predicted.y);
  result.yaw_error = std::abs(
    normalizeAngle(
      candidate_map_pose.yaw - result.predicted.yaw));
  result.accepted = result.position_error <= max_position_error &&
    result.yaw_error <= max_yaw_error;
  return result;
}

}  // namespace icp::recovery_guard

#endif  // ICP_REGISTRATION_RECOVERY_GUARD_HPP
