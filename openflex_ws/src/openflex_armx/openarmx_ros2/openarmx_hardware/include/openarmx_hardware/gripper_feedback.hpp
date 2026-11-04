#pragma once

#include <algorithm>
#include <cmath>

namespace openarmx_hardware {

struct GripperMotorFeedback {
  double radians;
  bool valid;
  bool clamped;
};

// The installed RS00 driver currently lets 0x15 error frames fall through to
// its state decoder. Never fold an out-of-range value back into the calibrated
// gripper range: doing so can hide a motor fault as plausible joint feedback.
inline GripperMotorFeedback validate_gripper_motor_feedback(
    double raw_radians) {
  constexpr double kOpenRadians = 1.0472;
  constexpr double kMechanicalToleranceRadians =
      (0.007 / 0.044) * kOpenRadians;

  if (!std::isfinite(raw_radians)) {
    return {raw_radians, false, false};
  }

  if (raw_radians >= -kMechanicalToleranceRadians &&
      raw_radians <= kOpenRadians + kMechanicalToleranceRadians) {
    const double bounded = std::clamp(raw_radians, 0.0, kOpenRadians);
    return {bounded, true, bounded != raw_radians};
  }

  return {raw_radians, false, false};
}

}  // namespace openarmx_hardware
