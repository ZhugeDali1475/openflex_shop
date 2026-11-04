#include "openarmx_integrated_moveit_config/openflex_ik_pipeline.hpp"

#include <cmath>

namespace openarmx_integrated_moveit_config
{
OpenflexIkPipeline::InitialRoute OpenflexIkPipeline::routeStageZero(
    const moveit::core::RobotState& state, const moveit::core::JointModelGroup& arm_group,
    const Eigen::Isometry3d& target, double workspace_fixed_lift_radius_m)
{
  if (arm_group.getActiveJointModels().empty() || workspace_fixed_lift_radius_m <= 0.0)
    return InitialRoute::STAGE_C;
  const auto* shoulder_joint = arm_group.getActiveJointModels().front();
  const auto* parent = shoulder_joint->getParentLinkModel();
  const auto* child = shoulder_joint->getChildLinkModel();
  if (!parent || !child)
    return InitialRoute::STAGE_C;
  const Eigen::Vector3d shoulder =
      (state.getGlobalLinkTransform(parent->getName()) * child->getJointOriginTransform()).translation();
  const Eigen::Vector3d delta = target.translation() - shoulder;
  if (!delta.allFinite())
    return InitialRoute::STAGE_C;
  if (delta.head<2>().norm() > workspace_fixed_lift_radius_m)
    return InitialRoute::STAGE_C;
  return delta.norm() <= workspace_fixed_lift_radius_m ? InitialRoute::STAGE_A : InitialRoute::STAGE_B;
}

bool OpenflexIkPipeline::withinFrontWorkspace(const moveit::core::RobotState& state, const std::string& base_link,
                                              const std::string& tip_link, bool left_arm, double min_x, double max_x,
                                              double half_angle_rad)
{
  const Eigen::Vector3d tip =
      (state.getGlobalLinkTransform(base_link).inverse() * state.getGlobalLinkTransform(tip_link)).translation();
  const double heading = std::atan2(tip.y(), tip.x());
  return tip.allFinite() && tip.x() >= min_x && tip.x() <= max_x &&
         (left_arm ? tip.y() >= 0.0 : tip.y() <= 0.0) && std::abs(heading) <= half_angle_rad;
}

Eigen::Vector2d OpenflexIkPipeline::frontBaseCandidateXY(const Eigen::Vector3d& target_position, double base_yaw,
                                                          double front_distance_m, double lateral_offset_m)
{
  return target_position.head<2>() - Eigen::Rotation2Dd(base_yaw).toRotationMatrix() *
                                      Eigen::Vector2d(front_distance_m, lateral_offset_m);
}
}  // namespace openarmx_integrated_moveit_config
