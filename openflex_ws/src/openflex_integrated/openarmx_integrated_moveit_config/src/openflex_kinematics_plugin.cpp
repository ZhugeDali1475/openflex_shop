// SPDX-License-Identifier: BSD-3-Clause

#include <openarmx_integrated_moveit_config/openflex_kinematics_plugin.hpp>
#include <openarmx_integrated_moveit_config/openflex_ik_pipeline.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <class_loader/class_loader.hpp>
#include <cstdint>
#include <functional>
#include <limits>
#include <moveit/robot_state/robot_state.h>
#include <random>

namespace openarmx_integrated_moveit_config
{
namespace
{
constexpr double kBoundEpsilon = 1e-8;
constexpr double kPi = 3.14159265358979323846;

rclcpp::Logger logger()
{
  return rclcpp::get_logger("openflex_kinematics");
}

double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double angularDistance(double from, double to)
{
  return std::abs(normalizeAngle(to - from));
}

}  // namespace

bool OpenflexKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr& node,
                                               const moveit::core::RobotModel& robot_model,
                                               const std::string& group_name, const std::string& base_frame,
                                               const std::vector<std::string>& tip_frames,
                                               double search_discretization)
{
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
  if (group_name == "lift_only")
    return initializeLiftOnly(node, robot_model, base_frame, tip_frames);
  if (group_name == "left_whole_body" || group_name == "right_whole_body")
    return initializeWholeBody(node, robot_model, group_name, search_discretization);

  if (group_name != "left_arm_with_lift" && group_name != "right_arm_with_lift")
  {
    RCLCPP_ERROR(logger(), "OpenflexKinematicsPlugin supports *_arm_with_lift and single-arm *_whole_body groups, got '%s'",
                 group_name.c_str());
    return false;
  }
  if (tip_frames.size() != 1 || !robot_model.hasLinkModel(base_frame) || !robot_model.hasLinkModel(tip_frames.front()))
  {
    RCLCPP_ERROR(logger(), "Group '%s' requires one valid base frame and one valid tip frame", group_name.c_str());
    return false;
  }

  robot_model_ptr_ = robot_model_;
  group_name_ = group_name;
  base_frame_ = base_frame;
  tip_frame_ = tip_frames.front();
  full_group_ = robot_model.getJointModelGroup(group_name_);
  arm_group_name_ = group_name_ == "left_arm_with_lift" ? "left_arm" : "right_arm";
  arm_group_ = robot_model.getJointModelGroup(arm_group_name_);
  if (!full_group_ || !arm_group_ || !arm_group_->hasLinkModel(tip_frame_))
  {
    RCLCPP_ERROR(logger(), "Cannot resolve groups '%s' and '%s' for tip '%s'", group_name_.c_str(),
                 arm_group_name_.c_str(), tip_frame_.c_str());
    return false;
  }

  full_variable_names_ = full_group_->getVariableNames();
  if (full_variable_names_.size() != 8)
  {
    RCLCPP_ERROR(logger(), "Group '%s' must expose lift plus seven arm variables, got %zu", group_name_.c_str(),
                 full_variable_names_.size());
    return false;
  }
  const auto lift = std::find(full_variable_names_.begin(), full_variable_names_.end(), "lift_joint");
  if (lift == full_variable_names_.end())
  {
    RCLCPP_ERROR(logger(), "Group '%s' has no lift_joint", group_name_.c_str());
    return false;
  }
  lift_index_ = static_cast<std::size_t>(std::distance(full_variable_names_.begin(), lift));

  lower_bounds_.clear();
  upper_bounds_.clear();
  lower_bounds_.reserve(full_variable_names_.size());
  upper_bounds_.reserve(full_variable_names_.size());
  for (const auto& variable_name : full_variable_names_)
  {
    const auto& bounds = robot_model.getVariableBounds(variable_name);
    if (!bounds.position_bounded_ || !std::isfinite(bounds.min_position_) || !std::isfinite(bounds.max_position_))
    {
      RCLCPP_ERROR(logger(), "Variable '%s' needs finite position limits", variable_name.c_str());
      return false;
    }
    lower_bounds_.push_back(bounds.min_position_);
    upper_bounds_.push_back(bounds.max_position_);
  }

  const auto& arm_joints = arm_group_->getJointModels();
  if (arm_joints.empty() || !arm_joints.front()->getParentLinkModel())
  {
    RCLCPP_ERROR(logger(), "Arm group '%s' has no chain root", arm_group_name_.c_str());
    return false;
  }
  arm_base_frame_ = arm_joints.front()->getParentLinkModel()->getName();
  try
  {
    arm_solver_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
        "moveit_core", "kinematics::KinematicsBase");
    arm_solver_ = arm_solver_loader_->createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
  }
  catch (const pluginlib::PluginlibException& error)
  {
    RCLCPP_ERROR(logger(), "Cannot load the Stage A KDL plugin: %s", error.what());
    return false;
  }
  if (!arm_solver_->initialize(node, robot_model, arm_group_name_, arm_base_frame_, { tip_frame_ }, search_discretization))
  {
    RCLCPP_ERROR(logger(), "Failed to initialize the Stage A KDL solver for '%s'", arm_group_name_.c_str());
    return false;
  }

  arm_full_indices_.clear();
  for (const auto& joint_name : arm_solver_->getJointNames())
  {
    const auto* joint = robot_model.getJointModel(joint_name);
    if (!joint || joint->getVariableNames().size() != 1)
    {
      RCLCPP_ERROR(logger(), "Stage A joint '%s' is not a 1-DOF MoveIt joint", joint_name.c_str());
      return false;
    }
    const auto full_index = std::find(full_variable_names_.begin(), full_variable_names_.end(),
                                      joint->getVariableNames().front());
    if (full_index == full_variable_names_.end())
    {
      RCLCPP_ERROR(logger(), "Stage A joint '%s' is not in group '%s'", joint_name.c_str(), group_name_.c_str());
      return false;
    }
    arm_full_indices_.push_back(static_cast<std::size_t>(std::distance(full_variable_names_.begin(), full_index)));
  }
  if (arm_full_indices_.size() != 7)
  {
    RCLCPP_ERROR(logger(), "Stage A chain must have seven joints, got %zu", arm_full_indices_.size());
    return false;
  }

  solver_joint_names_ = full_group_->getActiveJointModelNames();
  link_names_ = { tip_frame_ };
  try
  {
    full_solver_ = arm_solver_loader_->createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
  }
  catch (const pluginlib::PluginlibException& error)
  {
    RCLCPP_ERROR(logger(), "Cannot load the Stage B KDL plugin: %s", error.what());
    return false;
  }
  if (!full_solver_->initialize(node, robot_model, group_name_, base_frame_, { tip_frame_ }, search_discretization))
  {
    RCLCPP_ERROR(logger(), "Failed to initialize the Stage B KDL solver for '%s'", group_name_.c_str());
    return false;
  }
  full_solver_indices_.clear();
  for (const auto& joint_name : full_solver_->getJointNames())
  {
    const auto* joint = robot_model.getJointModel(joint_name);
    if (!joint || joint->getVariableNames().size() != 1)
    {
      RCLCPP_ERROR(logger(), "Stage B joint '%s' is not a 1-DOF MoveIt joint", joint_name.c_str());
      return false;
    }
    const auto variable = std::find(full_variable_names_.begin(), full_variable_names_.end(),
                                    joint->getVariableNames().front());
    if (variable == full_variable_names_.end())
    {
      RCLCPP_ERROR(logger(), "Stage B joint '%s' is not in group '%s'", joint_name.c_str(), group_name_.c_str());
      return false;
    }
    full_solver_indices_.push_back(static_cast<std::size_t>(std::distance(full_variable_names_.begin(), variable)));
  }
  if (full_solver_indices_.size() != full_variable_names_.size())
  {
    RCLCPP_ERROR(logger(), "Stage B KDL solver must expose all %zu group variables, got %zu", full_variable_names_.size(),
                 full_solver_indices_.size());
    return false;
  }
  if (!loadParameters(node, group_name_))
    return false;
  initialized_ = true;
  RCLCPP_INFO(logger(), "Openflex IK ready for '%s': fixed-seed 7-DOF KDL, then full 8-DOF KDL",
              group_name_.c_str());
  return true;
}

bool OpenflexKinematicsPlugin::initializeLiftOnly(const rclcpp::Node::SharedPtr& node,
                                                  const moveit::core::RobotModel& robot_model,
                                                  const std::string& base_frame,
                                                  const std::vector<std::string>& tip_frames)
{
  lift_only_mode_ = true;
  whole_body_mode_ = false;
  robot_model_ptr_ = robot_model_;
  group_name_ = "lift_only";
  base_frame_ = base_frame;
  if (tip_frames.size() != 1 || !robot_model.hasLinkModel(base_frame_) ||
      !robot_model.hasLinkModel(tip_frames.front()))
  {
    RCLCPP_ERROR(logger(), "Group 'lift_only' requires one valid base frame and one valid tip frame");
    return false;
  }
  tip_frame_ = tip_frames.front();
  full_group_ = robot_model.getJointModelGroup(group_name_);
  const auto* lift_joint = robot_model.getJointModel("lift_joint");
  if (!full_group_ || !lift_joint || lift_joint->getType() != moveit::core::JointModel::PRISMATIC ||
      !full_group_->hasLinkModel(tip_frame_))
  {
    RCLCPP_ERROR(logger(), "Cannot resolve the lift_only prismatic chain to tip '%s'", tip_frame_.c_str());
    return false;
  }

  full_variable_names_ = full_group_->getVariableNames();
  if (full_variable_names_.size() != 1 || full_variable_names_.front() != "lift_joint")
  {
    RCLCPP_ERROR(logger(), "Group 'lift_only' must expose only lift_joint");
    return false;
  }
  const auto& bounds = robot_model.getVariableBounds("lift_joint");
  if (!bounds.position_bounded_ || !std::isfinite(bounds.min_position_) || !std::isfinite(bounds.max_position_) ||
      bounds.max_position_ <= bounds.min_position_)
  {
    RCLCPP_ERROR(logger(), "lift_joint needs finite position limits");
    return false;
  }
  lower_bounds_ = { bounds.min_position_ };
  upper_bounds_ = { bounds.max_position_ };
  solver_joint_names_ = { "lift_joint" };
  link_names_ = { tip_frame_ };

  lift_reference_position_ = std::clamp(0.0, bounds.min_position_, bounds.max_position_);
  moveit::core::RobotState reference_state(robot_model_ptr_);
  reference_state.setToDefaultValues();
  reference_state.setJointGroupPositions(full_group_, std::vector<double>{ lift_reference_position_ });
  reference_state.update();
  const Eigen::Isometry3d base = reference_state.getGlobalLinkTransform(base_frame_);
  lift_reference_transform_ = base.inverse() * reference_state.getGlobalLinkTransform(tip_frame_);

  const double upward_range = bounds.max_position_ - lift_reference_position_;
  const double downward_range = lift_reference_position_ - bounds.min_position_;
  const double sample_delta = upward_range > 1e-6 ? std::min(0.01, upward_range) : -std::min(0.01, downward_range);
  if (std::abs(sample_delta) <= 1e-9)
  {
    RCLCPP_ERROR(logger(), "lift_joint has no usable range for analytic IK initialization");
    return false;
  }
  moveit::core::RobotState sample_state(reference_state);
  sample_state.setJointGroupPositions(full_group_,
                                      std::vector<double>{ lift_reference_position_ + sample_delta });
  sample_state.update();
  const Eigen::Isometry3d sample_transform =
      sample_state.getGlobalLinkTransform(base_frame_).inverse() * sample_state.getGlobalLinkTransform(tip_frame_);
  lift_axis_base_ = (sample_transform.translation() - lift_reference_transform_.translation()) / sample_delta;
  if (!lift_axis_base_.allFinite() || lift_axis_base_.norm() < 1e-9)
  {
    RCLCPP_ERROR(logger(), "Cannot derive the lift_joint translation axis");
    return false;
  }
  lift_axis_base_.normalize();

  if (!loadParameters(node, group_name_))
    return false;
  initialized_ = true;
  RCLCPP_INFO(logger(), "Openflex analytic lift IK ready for '%s' -> '%s'", base_frame_.c_str(), tip_frame_.c_str());
  return true;
}

bool OpenflexKinematicsPlugin::loadParameters(const rclcpp::Node::SharedPtr& node, const std::string& group_name)
{
  // MoveIt loads kinematics.yaml beneath robot_description_kinematics.
  const std::string prefix = "robot_description_kinematics." + group_name + ".openflex_ik.";
  const auto load_double = [&node, &prefix](const std::string& suffix, double& value) {
    const std::string name = prefix + suffix;
    if (!node->has_parameter(name))
      node->declare_parameter<double>(name, value);
    node->get_parameter(name, value);
  };
  const auto load_int = [&node, &prefix](const std::string& suffix, int& value) {
    const std::string name = prefix + suffix;
    if (!node->has_parameter(name))
      node->declare_parameter<int>(name, value);
    node->get_parameter(name, value);
  };
  load_double("workspace_fixed_lift_radius_m", parameters_.workspace_fixed_lift_radius_m);
  load_double("stage_a_timeout_sec", parameters_.stage_a_timeout_sec);
  load_double("stage_b_timeout_sec", parameters_.stage_b_timeout_sec);
  load_double("stage_c_timeout_per_seed_sec", parameters_.stage_c_timeout_per_seed_sec);
  load_double("stage_c_min_remaining_sec", parameters_.stage_c_min_remaining_sec);
  load_int("stage_c_yaw_count", parameters_.stage_c_yaw_count);
  load_int("stage_c_seeds_per_yaw", parameters_.stage_c_seeds_per_yaw);
  load_int("stage_c_random_seed", parameters_.stage_c_random_seed);
  load_double("stage_c_yaw_offset_rad", parameters_.stage_c_yaw_offset_rad);
  load_double("stage_c_yaw_range_rad", parameters_.stage_c_yaw_range_rad);
  load_double("stage_c_front_distance_m", parameters_.stage_c_front_distance_m);
  load_double("stage_c_left_lateral_offset_m", parameters_.stage_c_left_lateral_offset_m);
  load_double("stage_c_right_lateral_offset_m", parameters_.stage_c_right_lateral_offset_m);
  load_double("stage_c_orientation_tolerance_rad", parameters_.stage_c_orientation_tolerance_rad);
  load_double("front_min_x_m", parameters_.front_min_x_m);
  load_double("front_max_x_m", parameters_.front_max_x_m);
  load_double("front_half_angle_rad", parameters_.front_half_angle_rad);
  load_double("position_tolerance", parameters_.position_tolerance);
  load_double("rotation_tolerance_deg", parameters_.rotation_tolerance_deg);
  return parameters_.workspace_fixed_lift_radius_m > 0.0 && parameters_.stage_a_timeout_sec > 0.0 &&
         parameters_.stage_b_timeout_sec > 0.0 && parameters_.stage_c_timeout_per_seed_sec > 0.0 &&
         parameters_.stage_c_min_remaining_sec >= 0.0 && parameters_.stage_c_yaw_count > 0 &&
         parameters_.stage_c_seeds_per_yaw > 0 && parameters_.stage_c_yaw_range_rad >= 0.0 &&
         parameters_.stage_c_front_distance_m > 0.0 && parameters_.stage_c_orientation_tolerance_rad > 0.0 &&
         parameters_.front_min_x_m >= 0.0 && parameters_.front_max_x_m > parameters_.front_min_x_m &&
         parameters_.front_half_angle_rad > 0.0 && parameters_.front_half_angle_rad <= kPi &&
         parameters_.position_tolerance > 0.0 && parameters_.rotation_tolerance_deg > 0.0;
}

bool OpenflexKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                                  const std::vector<double>& ik_seed_state,
                                                  std::vector<double>& solution,
                                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                  const kinematics::KinematicsQueryOptions&) const
{
  return solve(ik_pose, ik_seed_state, 0.0, {}, IKCallbackFn(), false, solution, error_code);
}

bool OpenflexKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                                     const std::vector<double>& ik_seed_state, double timeout,
                                                     std::vector<double>& solution,
                                                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                     const kinematics::KinematicsQueryOptions&) const
{
  return solve(ik_pose, ik_seed_state, timeout, {}, IKCallbackFn(), true, solution, error_code);
}

bool OpenflexKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                                     const std::vector<double>& ik_seed_state, double timeout,
                                                     const std::vector<double>& consistency_limits,
                                                     std::vector<double>& solution,
                                                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                     const kinematics::KinematicsQueryOptions&) const
{
  return solve(ik_pose, ik_seed_state, timeout, consistency_limits, IKCallbackFn(), true, solution, error_code);
}

bool OpenflexKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                                     const std::vector<double>& ik_seed_state, double timeout,
                                                     std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                     const kinematics::KinematicsQueryOptions&) const
{
  return solve(ik_pose, ik_seed_state, timeout, {}, solution_callback, true, solution, error_code);
}

bool OpenflexKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                                     const std::vector<double>& ik_seed_state, double timeout,
                                                     const std::vector<double>& consistency_limits,
                                                     std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                     moveit_msgs::msg::MoveItErrorCodes& error_code,
                                                     const kinematics::KinematicsQueryOptions&) const
{
  return solve(ik_pose, ik_seed_state, timeout, consistency_limits, solution_callback, true, solution, error_code);
}

bool OpenflexKinematicsPlugin::solve(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& seed,
                                         double timeout, const std::vector<double>& consistency_limits,
                                         const IKCallbackFn& solution_callback, bool search, std::vector<double>& solution,
                                         moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (lift_only_mode_)
    return solveLiftOnly(ik_pose, seed, consistency_limits, solution_callback, solution, error_code);
  if (whole_body_mode_)
    return solveWholeBody(ik_pose, seed, timeout, consistency_limits, solution_callback, search, solution, error_code);

  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  if (!initialized_ || seed.size() != full_variable_names_.size())
    return false;
  std::vector<double> clamped_seed;
  if (!clampSeedToBounds(seed, clamped_seed))
    return false;
  if (!consistency_limits.empty() && consistency_limits.size() != seed.size())
    return false;

  const double stage_a_timeout = search ? parameters_.stage_a_timeout_sec : 0.0;
  if (solveStageA(ik_pose, clamped_seed, stage_a_timeout, consistency_limits, solution_callback, search, solution,
                  error_code))
    return true;
  return solveStageB(ik_pose, clamped_seed, consistency_limits, solution_callback, search,
                     search ? parameters_.stage_b_timeout_sec : 0.0, solution, error_code);
}

bool OpenflexKinematicsPlugin::solveLiftOnly(const geometry_msgs::msg::Pose& target_pose,
                                             const std::vector<double>& seed,
                                             const std::vector<double>& consistency_limits,
                                             const IKCallbackFn& solution_callback,
                                             std::vector<double>& solution,
                                             moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  if (!initialized_ || seed.size() != 1 || (!consistency_limits.empty() && consistency_limits.size() != 1))
    return false;

  std::vector<double> clamped_seed;
  if (!clampSeedToBounds(seed, clamped_seed))
    return false;

  const Eigen::Isometry3d target = poseToEigen(target_pose);
  const double displacement =
      lift_axis_base_.dot(target.translation() - lift_reference_transform_.translation());
  const std::vector<double> candidate{ lift_reference_position_ + displacement };
  if (!acceptCandidate(target_pose, clamped_seed, consistency_limits, solution_callback, candidate, error_code))
    return false;
  solution = candidate;
  return true;
}

bool OpenflexKinematicsPlugin::solveStageA(const geometry_msgs::msg::Pose& target,
                                               const std::vector<double>& seed, double timeout,
                                               const std::vector<double>& consistency_limits,
                                               const IKCallbackFn& solution_callback, bool search,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  const Eigen::Isometry3d target_in_arm = baseToArmBase(seed).inverse() * poseToEigen(target);
  const auto arm_target = eigenToPose(target_in_arm);
  const auto seed_arm = armSeed(seed);
  std::vector<double> arm_solution;
  moveit_msgs::msg::MoveItErrorCodes stage_error;

  if (!search)
  {
    if (!arm_solver_->getPositionIK(arm_target, seed_arm, arm_solution, stage_error))
      return false;
    const auto candidate = makeFullCandidate(seed[lift_index_], arm_solution);
    return acceptCandidate(target, seed, consistency_limits, solution_callback, candidate, error_code) &&
           ((solution = candidate), true);
  }

  std::vector<double> accepted;
  const IKCallbackFn stage_callback = [&](const geometry_msgs::msg::Pose&, const std::vector<double>& arm_candidate,
                                          moveit_msgs::msg::MoveItErrorCodes& callback_error) {
    const auto candidate = makeFullCandidate(seed[lift_index_], arm_candidate);
    if (acceptCandidate(target, seed, consistency_limits, solution_callback, candidate, callback_error))
      accepted = candidate;
  };
  if (!arm_solver_->searchPositionIK(arm_target, seed_arm, timeout, arm_solution, stage_callback, stage_error))
    return false;
  if (accepted.empty())
    return false;
  solution = accepted;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool OpenflexKinematicsPlugin::solveStageB(const geometry_msgs::msg::Pose& target,
                                               const std::vector<double>& seed,
                                               const std::vector<double>& consistency_limits,
                                               const IKCallbackFn& solution_callback, bool search,
                                               double timeout,
                                               std::vector<double>& solution,
                                               moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  const auto stage_seed = fullSolverSeed(seed);
  if (stage_seed.empty())
    return false;
  std::vector<double> stage_solution;
  moveit_msgs::msg::MoveItErrorCodes stage_error;
  if (!search)
  {
    if (!full_solver_->getPositionIK(target, stage_seed, stage_solution, stage_error))
      return false;
    const auto candidate = fullSolverCandidate(stage_solution);
    return acceptCandidate(target, seed, consistency_limits, solution_callback, candidate, error_code) &&
           ((solution = candidate), true);
  }

  std::vector<double> accepted;
  const IKCallbackFn stage_callback = [&](const geometry_msgs::msg::Pose&, const std::vector<double>& solver_candidate,
                                          moveit_msgs::msg::MoveItErrorCodes& callback_error) {
    const auto candidate = fullSolverCandidate(solver_candidate);
    if (acceptCandidate(target, seed, consistency_limits, solution_callback, candidate, callback_error))
      accepted = candidate;
  };
  if (!full_solver_->searchPositionIK(target, stage_seed, timeout, stage_solution, stage_callback, stage_error))
  {
    error_code = stage_error;
    return false;
  }
  if (accepted.empty())
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }
  solution = accepted;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool OpenflexKinematicsPlugin::initializeWholeBody(const rclcpp::Node::SharedPtr& node,
                                                       const moveit::core::RobotModel& robot_model,
                                                       const std::string& group_name,
                                                       double search_discretization)
{
  if (getTipFrame().empty() || !robot_model.hasLinkModel(getTipFrame()))
  {
    RCLCPP_ERROR(logger(), "Whole-body group '%s' requires one valid tip frame", group_name.c_str());
    return false;
  }

  robot_model_ptr_ = robot_model_;
  whole_body_mode_ = true;
  group_name_ = group_name;
  base_frame_ = getBaseFrame();
  tip_frame_ = getTipFrame();
  full_group_ = robot_model.getJointModelGroup(group_name_);
  lift_arm_group_name_ = group_name_ == "left_whole_body" ? "left_arm_with_lift" : "right_arm_with_lift";
  arm_group_name_ = group_name_ == "left_whole_body" ? "left_arm" : "right_arm";
  lift_arm_group_ = robot_model.getJointModelGroup(lift_arm_group_name_);
  arm_group_ = robot_model.getJointModelGroup(arm_group_name_);
  base_joint_ = robot_model.getJointModel("base_planar_joint");
  if (!full_group_ || !lift_arm_group_ || !arm_group_ || !base_joint_ ||
      base_joint_->getType() != moveit::core::JointModel::PLANAR ||
      !base_joint_->getChildLinkModel() || !robot_model.hasLinkModel(tip_frame_))
  {
    RCLCPP_ERROR(logger(), "Cannot resolve planar base and arm subgroups for '%s'", group_name_.c_str());
    return false;
  }
  full_variable_names_ = full_group_->getVariableNames();
  if (full_variable_names_.size() != 11)
  {
    RCLCPP_ERROR(logger(), "Whole-body group '%s' must expose 11 variables, got %zu", group_name_.c_str(),
                 full_variable_names_.size());
    return false;
  }
  const auto find_variable = [this](const std::string& name, std::size_t& index) {
    const auto iterator = std::find(full_variable_names_.begin(), full_variable_names_.end(), name);
    if (iterator == full_variable_names_.end())
      return false;
    index = static_cast<std::size_t>(std::distance(full_variable_names_.begin(), iterator));
    return true;
  };
  if (!find_variable("lift_joint", lift_index_) || !find_variable("base_planar_joint/x", base_x_index_) ||
      !find_variable("base_planar_joint/y", base_y_index_) ||
      !find_variable("base_planar_joint/theta", base_theta_index_))
  {
    RCLCPP_ERROR(logger(), "Whole-body group '%s' has incomplete planar base or lift variables", group_name_.c_str());
    return false;
  }

  lower_bounds_.clear();
  upper_bounds_.clear();
  for (const auto& variable_name : full_variable_names_)
  {
    const auto& bounds = robot_model.getVariableBounds(variable_name);
    lower_bounds_.push_back(bounds.position_bounded_ ? bounds.min_position_ : -std::numeric_limits<double>::infinity());
    upper_bounds_.push_back(bounds.position_bounded_ ? bounds.max_position_ : std::numeric_limits<double>::infinity());
  }

  const auto& arm_joints = arm_group_->getJointModels();
  const auto& lift_arm_joints = lift_arm_group_->getJointModels();
  if (arm_joints.empty() || lift_arm_joints.empty() || !arm_joints.front()->getParentLinkModel() ||
      !lift_arm_joints.front()->getParentLinkModel())
  {
    RCLCPP_ERROR(logger(), "Whole-body group '%s' has an invalid arm chain", group_name_.c_str());
    return false;
  }
  arm_base_frame_ = arm_joints.front()->getParentLinkModel()->getName();
  lift_arm_base_frame_ = lift_arm_joints.front()->getParentLinkModel()->getName();
  solver_tip_frame_ = arm_joints.back()->getChildLinkModel()->getName();
  if (!robot_model.hasLinkModel(solver_tip_frame_))
  {
    RCLCPP_ERROR(logger(), "Whole-body group '%s' has no arm-chain tip", group_name_.c_str());
    return false;
  }

  try
  {
    arm_solver_loader_ = std::make_shared<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
        "moveit_core", "kinematics::KinematicsBase");
    arm_solver_ = arm_solver_loader_->createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
    full_solver_ = arm_solver_loader_->createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
    pick_solver_ = arm_solver_loader_->createSharedInstance("pick_ik/PickIkPlugin");
  }
  catch (const pluginlib::PluginlibException& error)
  {
    RCLCPP_ERROR(logger(), "Cannot load official KDL child solvers: %s", error.what());
    return false;
  }
  if (!arm_solver_->initialize(node, robot_model, arm_group_name_, arm_base_frame_, { solver_tip_frame_ },
                               search_discretization) ||
      !full_solver_->initialize(node, robot_model, lift_arm_group_name_, lift_arm_base_frame_, { solver_tip_frame_ },
                                search_discretization) ||
      !pick_solver_->initialize(node, robot_model, arm_group_name_, arm_base_frame_, { tip_frame_ },
                                search_discretization))
  {
    RCLCPP_ERROR(logger(), "Cannot initialize KDL or PickIK child solvers for '%s'", group_name_.c_str());
    return false;
  }

  arm_full_indices_.clear();
  for (const auto& joint_name : arm_solver_->getJointNames())
  {
    const auto* joint = robot_model.getJointModel(joint_name);
    if (!joint || joint->getVariableNames().size() != 1 ||
        !find_variable(joint->getVariableNames().front(), arm_full_indices_.emplace_back()))
    {
      RCLCPP_ERROR(logger(), "Cannot map Stage A joint '%s' into '%s'", joint_name.c_str(), group_name_.c_str());
      return false;
    }
  }
  if (arm_full_indices_.size() != 7)
  {
    RCLCPP_ERROR(logger(), "Stage A solver for '%s' must expose seven joints", group_name_.c_str());
    return false;
  }

  full_solver_indices_.clear();
  for (const auto& joint_name : full_solver_->getJointNames())
  {
    const auto* joint = robot_model.getJointModel(joint_name);
    if (!joint || joint->getVariableNames().size() != 1 ||
        !find_variable(joint->getVariableNames().front(), full_solver_indices_.emplace_back()))
    {
      RCLCPP_ERROR(logger(), "Cannot map Stage B joint '%s' into '%s'", joint_name.c_str(), group_name_.c_str());
      return false;
    }
  }
  if (full_solver_indices_.size() != 8)
  {
    RCLCPP_ERROR(logger(), "Stage B solver for '%s' must expose lift plus seven joints", group_name_.c_str());
    return false;
  }

  pick_solver_indices_.clear();
  for (const auto& joint_name : pick_solver_->getJointNames())
  {
    const auto* joint = robot_model.getJointModel(joint_name);
    std::size_t full_index = 0;
    if (!joint || joint->getVariableNames().size() != 1 || !find_variable(joint->getVariableNames().front(), full_index))
    {
      RCLCPP_ERROR(logger(), "Cannot map Stage C PickIK joint '%s' into '%s'", joint_name.c_str(), group_name_.c_str());
      return false;
    }
    pick_solver_indices_.push_back(full_index);
  }
  if (pick_solver_indices_.size() != 7)
  {
    RCLCPP_ERROR(logger(), "Stage C PickIK solver for '%s' must expose seven joints", group_name_.c_str());
    return false;
  }

  solver_joint_names_ = full_group_->getActiveJointModelNames();
  link_names_ = { tip_frame_ };
  if (!loadParameters(node, group_name_))
  {
    RCLCPP_ERROR(logger(), "Whole-body group '%s' has invalid Openflex IK parameters", group_name_.c_str());
    return false;
  }

  moveit::core::RobotState reference_state(robot_model_ptr_);
  reference_state.setToDefaultValues();
  reference_state.update();
  tip_to_solver_tip_ = reference_state.getGlobalLinkTransform(tip_frame_).inverse() *
                       reference_state.getGlobalLinkTransform(solver_tip_frame_);

  initialized_ = true;
  RCLCPP_INFO(logger(), "Openflex whole-body IK ready for '%s': Stage 0 route, KDL Stage A/B, PickIK analytic Stage C",
              group_name_.c_str());
  return true;
}

std::vector<double> OpenflexKinematicsPlugin::groupFromSolverOrder(
    const std::vector<double>& solver_positions) const
{
  if (!whole_body_mode_ || solver_positions.size() != full_variable_names_.size())
    return solver_positions;
  // MoveIt creates this variable-level mapping only after initialize() returns.
  // It maps solver indices to JointModelGroup variable indices and therefore
  // must be read at call time for a group containing a planar virtual joint.
  const auto& solver_to_group = full_group_->getKinematicsSolverJointBijection();
  if (solver_to_group.size() != solver_positions.size())
    return solver_positions;
  std::vector<double> group_positions(solver_positions.size());
  for (std::size_t solver_index = 0; solver_index < solver_positions.size(); ++solver_index)
  {
    if (solver_to_group[solver_index] >= group_positions.size())
      return {};
    group_positions[solver_to_group[solver_index]] = solver_positions[solver_index];
  }
  return group_positions;
}

std::vector<double> OpenflexKinematicsPlugin::solverFromGroupOrder(
    const std::vector<double>& group_positions) const
{
  if (!whole_body_mode_ || group_positions.size() != full_variable_names_.size())
    return group_positions;
  const auto& solver_to_group = full_group_->getKinematicsSolverJointBijection();
  if (solver_to_group.size() != group_positions.size())
    return group_positions;
  std::vector<double> solver_positions(group_positions.size());
  for (std::size_t solver_index = 0; solver_index < solver_positions.size(); ++solver_index)
  {
    if (solver_to_group[solver_index] >= group_positions.size())
      return {};
    solver_positions[solver_index] = group_positions[solver_to_group[solver_index]];
  }
  return solver_positions;
}

Eigen::Isometry3d OpenflexKinematicsPlugin::targetInSolverBase(
    const Eigen::Isometry3d& target, const std::vector<double>& candidate,
    const std::string& solver_base_frame) const
{
  moveit::core::RobotState state(robot_model_ptr_);
  state.setToDefaultValues();
  state.setJointGroupPositions(full_group_, candidate);
  state.update();
  // odom is the virtual-joint parent frame, not a RobotModel link. MoveIt
  // supplies the target in this solver base frame, which is the RobotState
  // global coordinate system for the whole-body groups.
  const Eigen::Isometry3d requested_tip_global = target;
  const Eigen::Isometry3d solver_tip_global = requested_tip_global * tip_to_solver_tip_;
  return state.getGlobalLinkTransform(solver_base_frame).inverse() * solver_tip_global;
}

std::vector<double> OpenflexKinematicsPlugin::composeWholeBodyCandidate(
    const std::vector<double>& base_candidate, const std::vector<double>& lift_arm_solution) const
{
  if (base_candidate.size() != full_variable_names_.size() || lift_arm_solution.size() != full_solver_indices_.size())
    return {};
  auto candidate = base_candidate;
  for (std::size_t index = 0; index < lift_arm_solution.size(); ++index)
    candidate[full_solver_indices_[index]] = lift_arm_solution[index];
  return candidate;
}

double OpenflexKinematicsPlugin::stageTimeout(double requested_timeout, double configured_timeout,
                                                 double elapsed) const
{
  if (requested_timeout <= 0.0)
    return 0.0;
  const double remaining = requested_timeout - elapsed;
  return remaining > 0.0 ? std::min(configured_timeout, remaining) : 0.0;
}

Eigen::Vector3d OpenflexKinematicsPlugin::shoulderPosition(const std::vector<double>& full_positions) const
{
  if (full_positions.size() != full_variable_names_.size() || arm_group_->getActiveJointModels().empty())
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  const auto* shoulder_joint = arm_group_->getActiveJointModels().front();
  const auto* shoulder_parent = shoulder_joint->getParentLinkModel();
  const auto* shoulder_child = shoulder_joint->getChildLinkModel();
  if (!shoulder_parent || !shoulder_child)
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  moveit::core::RobotState state(robot_model_ptr_);
  state.setToDefaultValues();
  state.setJointGroupPositions(full_group_, full_positions);
  state.update();
  return (state.getGlobalLinkTransform(shoulder_parent->getName()) * shoulder_child->getJointOriginTransform()).translation();
}

OpenflexKinematicsPlugin::InitialRoute OpenflexKinematicsPlugin::routeStageZero(
    const Eigen::Isometry3d& target, const std::vector<double>& seed) const
{
  moveit::core::RobotState state(robot_model_ptr_);
  state.setToDefaultValues();
  state.setJointGroupPositions(full_group_, seed);
  state.update();
  switch (OpenflexIkPipeline::routeStageZero(state, *arm_group_, target, parameters_.workspace_fixed_lift_radius_m))
  {
    case OpenflexIkPipeline::InitialRoute::STAGE_A:
      return InitialRoute::STAGE_A;
    case OpenflexIkPipeline::InitialRoute::STAGE_B:
      return InitialRoute::STAGE_B;
    case OpenflexIkPipeline::InitialRoute::STAGE_C:
      return InitialRoute::STAGE_C;
  }
  return InitialRoute::SEQUENTIAL;
}

std::vector<double> OpenflexKinematicsPlugin::stageCArmSeed(
    const Eigen::Isometry3d& target, const std::vector<double>& full_seed, int yaw_index, int seed_index) const
{
  if (seed_index == 0)
  {
    std::vector<double> seed;
    seed.reserve(pick_solver_indices_.size());
    for (const auto index : pick_solver_indices_)
      seed.push_back(full_seed[index]);
    return seed;
  }

  std::size_t hash = static_cast<std::size_t>(parameters_.stage_c_random_seed);
  const auto mix = [&hash](double value) {
    hash ^= std::hash<double>{}(value) + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
  };
  for (const double value : full_seed)
    mix(value);
  for (const double value : target.translation())
    mix(value);
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      mix(target.rotation()(row, column));
  hash ^= static_cast<std::size_t>(yaw_index) * 0x85ebca6bU;
  hash ^= static_cast<std::size_t>(seed_index) * 0xc2b2ae35U;
  std::mt19937_64 generator(static_cast<std::uint64_t>(hash));
  std::vector<double> seed;
  seed.reserve(pick_solver_indices_.size());
  for (const auto full_index : pick_solver_indices_)
  {
    const auto& bounds = robot_model_ptr_->getVariableBounds(full_variable_names_[full_index]);
    if (!bounds.position_bounded_ || !std::isfinite(bounds.min_position_) || !std::isfinite(bounds.max_position_))
      return {};
    std::uniform_real_distribution<double> distribution(bounds.min_position_, bounds.max_position_);
    seed.push_back(distribution(generator));
  }
  return seed;
}

bool OpenflexKinematicsPlugin::solveWholeBodyStageA(
    const Eigen::Isometry3d& target, const std::vector<double>& original_seed,
    const std::vector<double>& stage_seed, double timeout, const std::vector<double>& consistency_limits,
    const IKCallbackFn& solution_callback, bool search, std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  const auto arm_target = eigenToPose(targetInSolverBase(target, stage_seed, arm_base_frame_));
  const auto seed_arm = armSeed(stage_seed);
  std::vector<double> arm_solution;
  moveit_msgs::msg::MoveItErrorCodes stage_error;
  const auto make_candidate = [this, &stage_seed](const std::vector<double>& arm_values) {
    auto candidate = stage_seed;
    if (arm_values.size() != arm_full_indices_.size())
      return std::vector<double>{};
    for (std::size_t index = 0; index < arm_values.size(); ++index)
      candidate[arm_full_indices_[index]] = arm_values[index];
    return candidate;
  };
  if (!search)
  {
    if (!arm_solver_->getPositionIK(arm_target, seed_arm, arm_solution, stage_error))
      return false;
    const auto candidate = make_candidate(arm_solution);
    return acceptCandidate(eigenToPose(target), original_seed, consistency_limits, solution_callback, candidate, error_code) &&
           ((solution = candidate), true);
  }
  std::vector<double> accepted;
  const IKCallbackFn callback = [&](const geometry_msgs::msg::Pose&, const std::vector<double>& arm_values,
                                    moveit_msgs::msg::MoveItErrorCodes& callback_error) {
    const auto candidate = make_candidate(arm_values);
    if (acceptCandidate(eigenToPose(target), original_seed, consistency_limits, solution_callback, candidate,
                        callback_error))
      accepted = candidate;
  };
  if (timeout <= 0.0 || !arm_solver_->searchPositionIK(arm_target, seed_arm, timeout, arm_solution, callback, stage_error) ||
      accepted.empty())
    return false;
  solution = accepted;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool OpenflexKinematicsPlugin::solveWholeBodyStageB(
    const Eigen::Isometry3d& target, const std::vector<double>& original_seed,
    const std::vector<double>& stage_seed, double timeout, const std::vector<double>& consistency_limits,
    const IKCallbackFn& solution_callback, bool search, std::vector<double>& solution,
    moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  std::vector<double> lift_arm_seed;
  lift_arm_seed.reserve(full_solver_indices_.size());
  for (const auto index : full_solver_indices_)
    lift_arm_seed.push_back(stage_seed[index]);
  const auto lift_arm_target = eigenToPose(targetInSolverBase(target, stage_seed, lift_arm_base_frame_));
  std::vector<double> lift_arm_solution;
  moveit_msgs::msg::MoveItErrorCodes stage_error;
  if (!search)
  {
    if (!full_solver_->getPositionIK(lift_arm_target, lift_arm_seed, lift_arm_solution, stage_error))
      return false;
    const auto candidate = composeWholeBodyCandidate(stage_seed, lift_arm_solution);
    return acceptCandidate(eigenToPose(target), original_seed, consistency_limits, solution_callback, candidate, error_code) &&
           ((solution = candidate), true);
  }
  std::vector<double> accepted;
  const IKCallbackFn callback = [&](const geometry_msgs::msg::Pose&, const std::vector<double>& solver_values,
                                    moveit_msgs::msg::MoveItErrorCodes& callback_error) {
    const auto candidate = composeWholeBodyCandidate(stage_seed, solver_values);
    if (acceptCandidate(eigenToPose(target), original_seed, consistency_limits, solution_callback, candidate,
                        callback_error))
      accepted = candidate;
  };
  if (timeout <= 0.0 ||
      !full_solver_->searchPositionIK(lift_arm_target, lift_arm_seed, timeout, lift_arm_solution, callback, stage_error) ||
      accepted.empty())
  {
    error_code = stage_error;
    return false;
  }
  solution = accepted;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool OpenflexKinematicsPlugin::solveWholeBodyStageC(
    const Eigen::Isometry3d& target, const std::vector<double>& original_seed,
    double timeout, const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback, bool search,
    std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if ((search && timeout <= 0.0) || !pick_solver_)
    return false;
  const auto start = std::chrono::steady_clock::now();
  const auto elapsed = [&start]() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  };
  // TCP yaw is a grasp-orientation constraint, not a base-heading constraint.
  // Generic MoveIt Pose goals have no task-level approach direction, so keep
  // the current RobotState heading as the stable Stage C sampling centre.
  const double target_yaw = original_seed[base_theta_index_];
  moveit_msgs::msg::MoveItErrorCodes last_error;
  last_error.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  std::vector<StageCCandidate> candidates;

  std::vector<double> yaw_samples;
  yaw_samples.reserve(static_cast<std::size_t>(parameters_.stage_c_yaw_count));
  yaw_samples.push_back(normalizeAngle(target_yaw + parameters_.stage_c_yaw_offset_rad));
  const int rings = std::max(1, (parameters_.stage_c_yaw_count - 1 + 1) / 2);
  for (int ring = 1; static_cast<int>(yaw_samples.size()) < parameters_.stage_c_yaw_count; ++ring)
  {
    const double offset = parameters_.stage_c_yaw_range_rad * static_cast<double>(ring) / rings;
    yaw_samples.push_back(normalizeAngle(target_yaw + parameters_.stage_c_yaw_offset_rad - offset));
    if (static_cast<int>(yaw_samples.size()) < parameters_.stage_c_yaw_count)
      yaw_samples.push_back(normalizeAngle(target_yaw + parameters_.stage_c_yaw_offset_rad + offset));
  }
  const double lateral_offset = group_name_ == "left_whole_body" ? parameters_.stage_c_left_lateral_offset_m
                                                                  : parameters_.stage_c_right_lateral_offset_m;

  for (std::size_t yaw_index = 0; yaw_index < yaw_samples.size(); ++yaw_index)
  {
    if (search && timeout - elapsed() < parameters_.stage_c_min_remaining_sec)
      break;
    const double yaw = yaw_samples[yaw_index];
    std::vector<double> orientation_reference = original_seed;
    const Eigen::Vector2d base_xy = OpenflexIkPipeline::frontBaseCandidateXY(
        target.translation(), yaw, parameters_.stage_c_front_distance_m, lateral_offset);
    orientation_reference[base_x_index_] = base_xy.x();
    orientation_reference[base_y_index_] = base_xy.y();
    orientation_reference[base_theta_index_] = yaw;
    moveit::core::RobotState reference_state(robot_model_ptr_);
    reference_state.setToDefaultValues();
    reference_state.setJointGroupPositions(full_group_, orientation_reference);
    reference_state.update();
    const geometry_msgs::msg::Pose arm_target = eigenToPose(
        reference_state.getGlobalLinkTransform(arm_base_frame_).inverse() * target);

    for (int seed_index = 0; seed_index < parameters_.stage_c_seeds_per_yaw; ++seed_index)
    {
      const double remaining = timeout - elapsed();
      if (search && remaining < parameters_.stage_c_min_remaining_sec)
        break;
      const auto arm_seed_values = stageCArmSeed(target, original_seed, static_cast<int>(yaw_index), seed_index);
      if (arm_seed_values.size() != pick_solver_indices_.size())
        continue;
      std::vector<double> arm_solution;
      moveit_msgs::msg::MoveItErrorCodes stage_error;
      bool solved = false;
      if (search)
      {
        const double per_seed_timeout = std::min(parameters_.stage_c_timeout_per_seed_sec, remaining);
        if (per_seed_timeout < parameters_.stage_c_min_remaining_sec)
          break;
        solved = pick_solver_->searchPositionIK(arm_target, arm_seed_values, per_seed_timeout, arm_solution, stage_error,
                                                kinematics::KinematicsQueryOptions());
      }
      else
      {
        solved = pick_solver_->getPositionIK(arm_target, arm_seed_values, arm_solution, stage_error);
      }
      last_error = stage_error;
      if (!solved || arm_solution.size() != pick_solver_indices_.size())
        continue;

      auto candidate = orientation_reference;
      for (std::size_t arm_index = 0; arm_index < arm_solution.size(); ++arm_index)
        candidate[pick_solver_indices_[arm_index]] = arm_solution[arm_index];
      moveit::core::RobotState state(robot_model_ptr_);
      state.setToDefaultValues();
      state.setJointGroupPositions(full_group_, candidate);
      state.update();
      const Eigen::Isometry3d tip_before_rebase = state.getGlobalLinkTransform(tip_frame_);
      if (Eigen::AngleAxisd(tip_before_rebase.rotation().transpose() * target.rotation()).angle() >
          parameters_.stage_c_orientation_tolerance_rad)
      {
        continue;
      }

      // Stage C belongs to PickIK only. Preserve the sampled heading, use the
      // PickIK arm FK, then analytically close XY and lift Z. Stage A/B remain
      // the only paths that invoke the official KDL child solvers.
      candidate[base_x_index_] += target.translation().x() - tip_before_rebase.translation().x();
      candidate[base_y_index_] += target.translation().y() - tip_before_rebase.translation().y();
      candidate[lift_index_] += target.translation().z() - tip_before_rebase.translation().z();
      if (!acceptCandidate(eigenToPose(target), original_seed, consistency_limits, solution_callback, candidate,
                           stage_error))
      {
        last_error = stage_error;
        continue;
      }

      double arm_displacement_squared = 0.0;
      for (const auto arm_index : pick_solver_indices_)
        arm_displacement_squared += std::pow(candidate[arm_index] - original_seed[arm_index], 2.0);
      candidates.push_back({ candidate,
                             static_cast<int>(yaw_index),
                             seed_index,
                             std::abs(candidate[lift_index_] - original_seed[lift_index_]),
                             std::sqrt(arm_displacement_squared),
                             std::hypot(candidate[base_x_index_] - original_seed[base_x_index_],
                                        candidate[base_y_index_] - original_seed[base_y_index_]),
                             angularDistance(original_seed[base_theta_index_], candidate[base_theta_index_]) });
    }
  }
  if (candidates.empty())
  {
    error_code = last_error;
    return false;
  }
  std::stable_sort(candidates.begin(), candidates.end(), [](const StageCCandidate& left, const StageCCandidate& right) {
    if (std::abs(left.lift_displacement - right.lift_displacement) > 1e-9)
      return left.lift_displacement < right.lift_displacement;
    if (std::abs(left.arm_displacement - right.arm_displacement) > 1e-9)
      return left.arm_displacement < right.arm_displacement;
    if (std::abs(left.base_displacement - right.base_displacement) > 1e-9)
      return left.base_displacement < right.base_displacement;
    if (std::abs(left.yaw_displacement - right.yaw_displacement) > 1e-9)
      return left.yaw_displacement < right.yaw_displacement;
    if (left.yaw_index != right.yaw_index)
      return left.yaw_index < right.yaw_index;
    return left.seed_index < right.seed_index;
  });
  solution = candidates.front().positions;
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool OpenflexKinematicsPlugin::solveWholeBody(const geometry_msgs::msg::Pose& target_pose,
                                                  const std::vector<double>& solver_seed, double timeout,
                                                  const std::vector<double>& solver_consistency_limits,
                                                  const IKCallbackFn& solution_callback, bool search,
                                                  std::vector<double>& solution,
                                                  moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
  const auto raw_seed = groupFromSolverOrder(solver_seed);
  const auto consistency_limits = groupFromSolverOrder(solver_consistency_limits);
  if (!initialized_ || raw_seed.size() != full_variable_names_.size() ||
      (!solver_consistency_limits.empty() && consistency_limits.size() != raw_seed.size()))
    return false;
  std::vector<double> seed;
  if (!clampSeedToBounds(raw_seed, seed))
    return false;

  const Eigen::Isometry3d target = poseToEigen(target_pose);
  InitialRoute route = routeStageZero(target, seed);

  const auto start = std::chrono::steady_clock::now();
  const auto elapsed = [&start] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  };
  std::vector<double> accepted;
  if (route == InitialRoute::STAGE_A || route == InitialRoute::SEQUENTIAL)
  {
    if (solveWholeBodyStageA(target, seed, seed, stageTimeout(timeout, parameters_.stage_a_timeout_sec, elapsed()),
                             consistency_limits, solution_callback, search, accepted, error_code))
    {
      solution = solverFromGroupOrder(accepted);
      return true;
    }
    route = InitialRoute::STAGE_B;
  }
  if (route == InitialRoute::STAGE_B || route == InitialRoute::SEQUENTIAL)
  {
    if (solveWholeBodyStageB(target, seed, seed, stageTimeout(timeout, parameters_.stage_b_timeout_sec, elapsed()),
                             consistency_limits, solution_callback, search, accepted, error_code))
    {
      solution = solverFromGroupOrder(accepted);
      return true;
    }
    route = InitialRoute::STAGE_C;
  }
  if (route == InitialRoute::STAGE_C &&
      solveWholeBodyStageC(target, seed, stageTimeout(timeout, timeout, elapsed()), consistency_limits,
                           solution_callback, search, accepted, error_code))
  {
    solution = solverFromGroupOrder(accepted);
    return true;
  }
  return false;
}

bool OpenflexKinematicsPlugin::acceptCandidate(const geometry_msgs::msg::Pose& target, const std::vector<double>& seed,
                                                   const std::vector<double>& consistency_limits,
                                                   const IKCallbackFn& solution_callback,
                                                   const std::vector<double>& candidate,
                                                   moveit_msgs::msg::MoveItErrorCodes& error_code) const
{
  if (!withinBounds(candidate) || !withinConsistency(seed, consistency_limits, candidate))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }
  if (whole_body_mode_ && !withinFrontWorkspace(candidate))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }
  const Eigen::Isometry3d actual = baseToTip(candidate);
  const Eigen::Isometry3d expected = poseToEigen(target);
  const double position_error = (actual.translation() - expected.translation()).norm();
  const double rotation_error = Eigen::AngleAxisd(actual.rotation().transpose() * expected.rotation()).angle();
  if (position_error > parameters_.position_tolerance || rotation_error > parameters_.rotation_tolerance_deg * kPi / 180.0)
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  if (solution_callback)
    solution_callback(target, whole_body_mode_ ? solverFromGroupOrder(candidate) : candidate, error_code);
  return error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

bool OpenflexKinematicsPlugin::clampSeedToBounds(const std::vector<double>& seed,
                                                 std::vector<double>& clamped_seed) const
{
  if (seed.size() != lower_bounds_.size() || seed.size() != upper_bounds_.size() ||
      seed.size() != full_variable_names_.size())
    return false;

  clamped_seed = seed;
  for (std::size_t index = 0; index < seed.size(); ++index)
  {
    const double value = seed[index];
    if (!std::isfinite(value))
      return false;

    double bounded_value = value;
    if (value < lower_bounds_[index])
    {
      bounded_value = lower_bounds_[index];
    }
    else if (value > upper_bounds_[index])
    {
      bounded_value = upper_bounds_[index];
    }

    if (bounded_value != value)
    {
      RCLCPP_WARN_ONCE(logger(),
                       "Clamping an out-of-bounds IK seed to its nearest joint bound (first occurrence: %s %.6f -> %.6f)",
                       full_variable_names_[index].c_str(), value, bounded_value);
      clamped_seed[index] = bounded_value;
    }
  }
  return true;
}

bool OpenflexKinematicsPlugin::withinBounds(const std::vector<double>& candidate) const
{
  if (whole_body_mode_)
  {
    if (candidate.size() != full_variable_names_.size() ||
        std::any_of(candidate.begin(), candidate.end(), [](const double value) { return !std::isfinite(value); }))
      return false;
    moveit::core::RobotState state(robot_model_ptr_);
    state.setToDefaultValues();
    state.setJointGroupPositions(full_group_, candidate);
    return state.satisfiesBounds(full_group_);
  }
  if (candidate.size() != lower_bounds_.size())
    return false;
  for (std::size_t index = 0; index < candidate.size(); ++index)
  {
    if (!std::isfinite(candidate[index]) || candidate[index] < lower_bounds_[index] - kBoundEpsilon ||
        candidate[index] > upper_bounds_[index] + kBoundEpsilon)
      return false;
  }
  return true;
}

bool OpenflexKinematicsPlugin::withinFrontWorkspace(const std::vector<double>& candidate) const
{
  if (!whole_body_mode_ || candidate.size() != full_variable_names_.size())
    return true;

  moveit::core::RobotState state(robot_model_ptr_);
  state.setToDefaultValues();
  state.setJointGroupPositions(full_group_, candidate);
  state.update();

  const auto* base_link = base_joint_->getChildLinkModel();
  if (!base_link)
    return false;
  return OpenflexIkPipeline::withinFrontWorkspace(
      state, base_link->getName(), tip_frame_, group_name_ == "left_whole_body", parameters_.front_min_x_m,
      parameters_.front_max_x_m, parameters_.front_half_angle_rad);
}

bool OpenflexKinematicsPlugin::withinConsistency(const std::vector<double>& seed,
                                                     const std::vector<double>& consistency_limits,
                                                     const std::vector<double>& candidate) const
{
  if (consistency_limits.empty())
    return true;
  if (seed.size() != candidate.size() || consistency_limits.size() != candidate.size())
    return false;
  for (std::size_t index = 0; index < candidate.size(); ++index)
  {
    const double difference = whole_body_mode_ && index == base_theta_index_
                                  ? std::atan2(std::sin(candidate[index] - seed[index]),
                                               std::cos(candidate[index] - seed[index]))
                                  : candidate[index] - seed[index];
    if (std::abs(difference) > consistency_limits[index])
      return false;
  }
  return true;
}

Eigen::Isometry3d OpenflexKinematicsPlugin::poseToEigen(const geometry_msgs::msg::Pose& pose) const
{
  Eigen::Quaterniond orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  if (orientation.norm() < 1e-12)
    orientation = Eigen::Quaterniond::Identity();
  else
    orientation.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = orientation.toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  return transform;
}

geometry_msgs::msg::Pose OpenflexKinematicsPlugin::eigenToPose(const Eigen::Isometry3d& transform) const
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Quaterniond orientation(transform.rotation());
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  pose.orientation.x = orientation.x();
  pose.orientation.y = orientation.y();
  pose.orientation.z = orientation.z();
  pose.orientation.w = orientation.w();
  return pose;
}

Eigen::Isometry3d OpenflexKinematicsPlugin::baseToArmBase(const std::vector<double>& full_positions) const
{
  moveit::core::RobotState state(robot_model_ptr_);
  state.setToDefaultValues();
  state.setJointGroupPositions(full_group_, full_positions);
  state.update();
  return state.getGlobalLinkTransform(base_frame_).inverse() * state.getGlobalLinkTransform(arm_base_frame_);
}

Eigen::Isometry3d OpenflexKinematicsPlugin::baseToTip(const std::vector<double>& full_positions) const
{
  moveit::core::RobotState state(robot_model_ptr_);
  state.setToDefaultValues();
  state.setJointGroupPositions(full_group_, full_positions);
  state.update();
  if (whole_body_mode_)
    return state.getGlobalLinkTransform(tip_frame_);
  return state.getGlobalLinkTransform(base_frame_).inverse() * state.getGlobalLinkTransform(tip_frame_);
}

std::vector<double> OpenflexKinematicsPlugin::armSeed(const std::vector<double>& full_seed) const
{
  std::vector<double> arm_seed;
  arm_seed.reserve(arm_full_indices_.size());
  for (const auto index : arm_full_indices_)
    arm_seed.push_back(full_seed[index]);
  return arm_seed;
}

std::vector<double> OpenflexKinematicsPlugin::fullSolverSeed(const std::vector<double>& full_seed) const
{
  if (full_seed.size() != full_variable_names_.size() || full_solver_indices_.empty())
    return {};
  std::vector<double> solver_seed;
  solver_seed.reserve(full_solver_indices_.size());
  for (const auto index : full_solver_indices_)
    solver_seed.push_back(full_seed[index]);
  return solver_seed;
}

std::vector<double> OpenflexKinematicsPlugin::fullSolverCandidate(const std::vector<double>& solver_candidate) const
{
  if (solver_candidate.size() != full_solver_indices_.size())
    return {};
  std::vector<double> candidate(full_variable_names_.size(), 0.0);
  for (std::size_t index = 0; index < solver_candidate.size(); ++index)
    candidate[full_solver_indices_[index]] = solver_candidate[index];
  return candidate;
}

std::vector<double> OpenflexKinematicsPlugin::makeFullCandidate(double lift_position,
                                                                     const std::vector<double>& arm_positions) const
{
  if (arm_positions.size() != arm_full_indices_.size())
    return {};
  std::vector<double> candidate(full_variable_names_.size(), 0.0);
  candidate[lift_index_] = lift_position;
  for (std::size_t index = 0; index < arm_positions.size(); ++index)
    candidate[arm_full_indices_[index]] = arm_positions[index];
  return candidate;
}

bool OpenflexKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                                 const std::vector<double>& joint_angles,
                                                 std::vector<geometry_msgs::msg::Pose>& poses) const
{
  poses.clear();
  const auto group_angles = groupFromSolverOrder(joint_angles);
  if (!initialized_ || group_angles.size() != full_variable_names_.size() || !withinBounds(group_angles))
    return false;
  moveit::core::RobotState state(robot_model_ptr_);
  state.setToDefaultValues();
  state.setJointGroupPositions(full_group_, group_angles);
  state.update();
  const Eigen::Isometry3d base = whole_body_mode_ ? Eigen::Isometry3d::Identity()
                                                   : state.getGlobalLinkTransform(base_frame_);
  for (const auto& link_name : link_names)
  {
    if (!robot_model_ptr_->hasLinkModel(link_name))
      return false;
    poses.push_back(eigenToPose(base.inverse() * state.getGlobalLinkTransform(link_name)));
  }
  return true;
}

const std::vector<std::string>& OpenflexKinematicsPlugin::getJointNames() const
{
  return solver_joint_names_;
}

const std::vector<std::string>& OpenflexKinematicsPlugin::getLinkNames() const
{
  return link_names_;
}
}  // namespace openarmx_integrated_moveit_config

CLASS_LOADER_REGISTER_CLASS(openarmx_integrated_moveit_config::OpenflexKinematicsPlugin,
                            kinematics::KinematicsBase)
