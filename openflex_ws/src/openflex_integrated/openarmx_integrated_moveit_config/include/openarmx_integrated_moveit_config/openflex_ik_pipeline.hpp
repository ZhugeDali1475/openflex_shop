#pragma once

#include <Eigen/Geometry>
#include <moveit/robot_state/robot_state.h>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

namespace openarmx_integrated_moveit_config
{
class OpenflexIkPipeline
{
public:
  enum class InitialRoute
  {
    STAGE_A,
    STAGE_B,
    STAGE_C,
  };

  struct Request
  {
    Eigen::Isometry3d target_tcp_odom = Eigen::Isometry3d::Identity();
    std::optional<double> desired_base_yaw;
    double yaw_range_rad = 0.0;
    int yaw_samples = 1;
    double front_distance_m = 0.70;
    double timeout_sec = 0.0;
  };

  struct Candidate
  {
    InitialRoute initial_route;
    InitialRoute selected_stage;
    std::vector<double> joint_positions;
    Eigen::Isometry3d tcp_odom = Eigen::Isometry3d::Identity();
  };

  using CandidateValidator = std::function<bool(const std::vector<double>&)>;

  // Whole-body instances are side-specific and own their child solvers. The
  // implementation is intentionally serialized because KinematicsBase child
  // plugins do not promise concurrent searchPositionIK safety.
  OpenflexIkPipeline() = default;
  OpenflexIkPipeline(const OpenflexIkPipeline&) = delete;
  OpenflexIkPipeline& operator=(const OpenflexIkPipeline&) = delete;
  static InitialRoute routeStageZero(const moveit::core::RobotState& state,
                                     const moveit::core::JointModelGroup& arm_group,
                                     const Eigen::Isometry3d& target,
                                     double workspace_fixed_lift_radius_m);
  static bool withinFrontWorkspace(const moveit::core::RobotState& state, const std::string& base_link,
                                   const std::string& tip_link, bool left_arm, double min_x, double max_x,
                                   double half_angle_rad);
  static Eigen::Vector2d frontBaseCandidateXY(const Eigen::Vector3d& target_position, double base_yaw,
                                               double front_distance_m, double lateral_offset_m);

private:
  mutable std::mutex solver_mutex_;
};
}  // namespace openarmx_integrated_moveit_config
