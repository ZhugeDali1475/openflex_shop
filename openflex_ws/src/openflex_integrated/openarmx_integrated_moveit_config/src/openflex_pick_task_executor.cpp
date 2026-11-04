// SPDX-License-Identifier: BSD-3-Clause

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <openarmx_integrated_moveit_msgs/action/plan_pregrasp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "openarmx_integrated_moveit_config/openflex_ik_pipeline.hpp"

namespace openarmx_integrated_moveit_config
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
  return std::remainder(angle, 2.0 * kPi);
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& orientation)
{
  const double sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return std::atan2(sin_yaw, cos_yaw);
}

geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(yaw * 0.5);
  orientation.w = std::cos(yaw * 0.5);
  return orientation;
}

bool finitePose(const geometry_msgs::msg::Pose& pose)
{
  const double norm = std::sqrt(pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
                                pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w);
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
         std::isfinite(pose.orientation.x) && std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w) && norm > 1e-6;
}

std::string normalizeAndDescribeJointBounds(moveit::core::RobotState& state,
                                            const moveit::core::JointModelGroup& group,
                                            std::vector<std::string>& normalized)
{
  // The desktop MoveIt stack lets FixStartStateBounds absorb small encoder
  // quantization at a hard limit. Keep a much tighter local cap than its
  // 0.1-rad default while accepting sub-milliradian feedback noise.
  constexpr double kNumericalBoundsTolerance = 1e-3;
  std::ostringstream message;
  message << std::fixed << std::setprecision(6);
  bool first = true;
  for (const auto& variable_name : group.getVariableNames())
  {
    const double value = state.getVariablePosition(variable_name);
    const auto& bounds = state.getRobotModel()->getVariableBounds(variable_name);
    if (!std::isfinite(value))
    {
      if (!first)
        message << "; ";
      first = false;
      message << variable_name << '=' << value << " is not finite";
      continue;
    }
    if (!bounds.position_bounded_ ||
        (value >= bounds.min_position_ && value <= bounds.max_position_))
      continue;

    const double violation = value < bounds.min_position_ ? bounds.min_position_ - value
                                                          : value - bounds.max_position_;
    if (violation <= kNumericalBoundsTolerance)
    {
      const double bounded = std::clamp(value, bounds.min_position_, bounds.max_position_);
      state.setVariablePosition(variable_name, bounded);
      std::ostringstream correction;
      correction << std::fixed << std::setprecision(9) << variable_name << ": " << value << " -> " << bounded;
      normalized.push_back(correction.str());
      continue;
    }
    if (!first)
      message << "; ";
    first = false;
    message << variable_name << '=' << value;
    message << " outside [" << bounds.min_position_ << ", " << bounds.max_position_ << ']';
  }
  if (!normalized.empty())
    state.update();
  return message.str();
}
}  // namespace

class OpenflexPickTaskExecutor : public rclcpp::Node
{
public:
  using PlanPregrasp = openarmx_integrated_moveit_msgs::action::PlanPregrasp;
  using GoalHandlePlanPregrasp = rclcpp_action::ServerGoalHandle<PlanPregrasp>;
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  OpenflexPickTaskExecutor() : Node("openflex_pick_task_executor")
  {
    odom_frame_ = "odom";
    left_tip_link_ = "openarmx_left_hand_tcp";
    right_tip_link_ = "openarmx_right_hand_tcp";
    left_group_name_ = "left_arm_with_lift";
    right_group_name_ = "right_arm_with_lift";

    // MoveItConfigsBuilder exposes kinematics.yaml below robot_description_kinematics.
    // This Action intentionally uses the left whole-body block as the shared
    // task-policy source. The mirrored right block is retained for RViz IK.
    const std::string prefix = "robot_description_kinematics.left_whole_body.openflex_ik.";
    const auto require_double = [this, &prefix](const std::string& suffix) {
      const std::string name = prefix + suffix;
      if (has_parameter(name))
      {
        double value = 0.0;
        if (get_parameter(name, value))
          return value;
      }
      try
      {
        return declare_parameter<double>(name);
      }
      catch (const rclcpp::exceptions::ParameterUninitializedException&)
      {
        throw std::runtime_error("Missing required kinematics.yaml parameter: " + name);
      }
    };
    const auto require_int = [this, &prefix](const std::string& suffix) -> int {
      const std::string name = prefix + suffix;
      if (has_parameter(name))
      {
        int value = 0;
        if (get_parameter(name, value))
          return value;
      }
      try
      {
        return static_cast<int>(declare_parameter<int>(name));
      }
      catch (const rclcpp::exceptions::ParameterUninitializedException&)
      {
        throw std::runtime_error("Missing required kinematics.yaml parameter: " + name);
      }
    };
    default_front_distance_m_ = require_double("stage_c_front_distance_m");
    workspace_fixed_lift_radius_m_ = require_double("workspace_fixed_lift_radius_m");
    left_lateral_offset_m_ = require_double("stage_c_left_lateral_offset_m");
    right_lateral_offset_m_ = require_double("stage_c_right_lateral_offset_m");
    front_min_x_m_ = require_double("front_min_x_m");
    front_max_x_m_ = require_double("front_max_x_m");
    front_half_angle_rad_ = require_double("front_half_angle_rad");
    default_yaw_range_rad_ = require_double("stage_c_yaw_range_rad");
    max_yaw_tolerance_rad_ = require_double("action_max_yaw_tolerance_rad");
    candidate_plan_timeout_sec_ = require_double("action_candidate_plan_timeout_sec");
    default_planning_timeout_sec_ = require_double("action_default_planning_timeout_sec");
    default_velocity_scale_ = require_double("action_default_velocity_scale");
    default_acceleration_scale_ = require_double("action_default_acceleration_scale");
    navigation_timeout_sec_ = require_double("action_navigation_timeout_sec");
    default_yaw_samples_ = require_int("stage_c_yaw_count");
    max_yaw_samples_ = require_int("action_max_yaw_samples");
    default_max_dof_ = require_int("action_default_max_dof");

    navigation_client_ = rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");
  }

  void initializeMoveGroups()
  {
    left_arm_planning_group_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "left_arm");
    right_arm_planning_group_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "right_arm");
    left_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), left_group_name_);
    right_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), right_group_name_);
    left_arm_planning_group_->setPoseReferenceFrame(odom_frame_);
    right_arm_planning_group_->setPoseReferenceFrame(odom_frame_);
    left_group_->setPoseReferenceFrame(odom_frame_);
    right_group_->setPoseReferenceFrame(odom_frame_);
    const auto model = left_group_->getRobotModel();
    left_arm_group_ = model->getJointModelGroup("left_arm");
    right_arm_group_ = model->getJointModelGroup("right_arm");
    left_whole_body_group_ = model->getJointModelGroup("left_whole_body");
    right_whole_body_group_ = model->getJointModelGroup("right_whole_body");
    if (!left_arm_planning_group_ || !right_arm_planning_group_ || !left_arm_group_ || !right_arm_group_ ||
        !left_whole_body_group_ || !right_whole_body_group_)
    {
      RCLCPP_ERROR(get_logger(), "Whole-body Pipeline groups are unavailable; task Action will not be advertised");
      return;
    }
    // Do not advertise the Action until both MoveIt clients are ready. This prevents
    // a launch-time request from being rejected while MoveGroupInterface is initializing.
    action_server_ = rclcpp_action::create_server<PlanPregrasp>(
        this, "/openflex/plan_pregrasp",
        std::bind(&OpenflexPickTaskExecutor::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&OpenflexPickTaskExecutor::handleCancel, this, std::placeholders::_1),
        std::bind(&OpenflexPickTaskExecutor::handleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Ready on /openflex/plan_pregrasp using %s and %s", left_group_name_.c_str(),
                right_group_name_.c_str());
  }

private:
  struct Candidate
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::RobotState start_state;
    geometry_msgs::msg::PoseStamped base_goal;
    sensor_msgs::msg::JointState arm_goal;
    double lift_goal = 0.0;
    double base_distance = 0.0;
    double base_yaw_distance = 0.0;
    uint8_t stage = PlanPregrasp::Result::STAGE_C;

    explicit Candidate(const moveit::core::RobotState& state) : start_state(state) {}
  };

  int resolvedMaxDof(const PlanPregrasp::Goal& goal) const
  {
    return goal.max_dof == PlanPregrasp::Goal::AUTO ? default_max_dof_ : static_cast<int>(goal.max_dof);
  }

  static bool isSupportedMaxDof(int max_dof)
  {
    return max_dof == PlanPregrasp::Goal::ARM_ONLY || max_dof == PlanPregrasp::Goal::ARM_WITH_LIFT ||
           max_dof == PlanPregrasp::Goal::WHOLE_BODY;
  }

  static int requiredDof(OpenflexIkPipeline::InitialRoute route)
  {
    switch (route)
    {
      case OpenflexIkPipeline::InitialRoute::STAGE_A:
        return PlanPregrasp::Goal::ARM_ONLY;
      case OpenflexIkPipeline::InitialRoute::STAGE_B:
        return PlanPregrasp::Goal::ARM_WITH_LIFT;
      case OpenflexIkPipeline::InitialRoute::STAGE_C:
        return PlanPregrasp::Goal::WHOLE_BODY;
    }
    return PlanPregrasp::Goal::WHOLE_BODY;
  }

  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&, std::shared_ptr<const PlanPregrasp::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (busy_ || !left_arm_planning_group_ || !right_arm_planning_group_ || !left_group_ || !right_group_ ||
        !finitePose(goal->tcp_pregrasp_pose.pose) ||
        goal->tcp_pregrasp_pose.header.frame_id != odom_frame_ ||
        (goal->arm != PlanPregrasp::Goal::LEFT && goal->arm != PlanPregrasp::Goal::RIGHT) ||
        !isSupportedMaxDof(resolvedMaxDof(*goal)) ||
        goal->base_yaw_samples > max_yaw_samples_ ||
        goal->base_yaw_tolerance < 0.0 || goal->base_yaw_tolerance > max_yaw_tolerance_rad_ || goal->front_distance < 0.0)
      return rclcpp_action::GoalResponse::REJECT;
    if (goal->base_yaw_mode == PlanPregrasp::Goal::EXPLICIT && !std::isfinite(goal->desired_base_yaw_odom))
      return rclcpp_action::GoalResponse::REJECT;
    else if (goal->base_yaw_mode != PlanPregrasp::Goal::CURRENT_STATE &&
             goal->base_yaw_mode != PlanPregrasp::Goal::EXPLICIT)
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePlanPregrasp>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandlePlanPregrasp> goal_handle)
  {
    std::thread([this, goal_handle] { execute(goal_handle); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlanPregrasp>& goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<PlanPregrasp::Result>();
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      busy_ = true;
    }
    const auto clear_busy = [this] {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      busy_ = false;
    };

    auto& lift_group = goal->arm == PlanPregrasp::Goal::LEFT ? *left_group_ : *right_group_;
    auto& arm_group_interface =
        goal->arm == PlanPregrasp::Goal::LEFT ? *left_arm_planning_group_ : *right_arm_planning_group_;
    const std::string& tip_link = goal->arm == PlanPregrasp::Goal::LEFT ? left_tip_link_ : right_tip_link_;
    const double lateral = goal->arm == PlanPregrasp::Goal::LEFT ? left_lateral_offset_m_ : right_lateral_offset_m_;
    const double front_distance = goal->front_distance > 0.0 ? goal->front_distance : default_front_distance_m_;
    const int max_dof = resolvedMaxDof(*goal);
    const double planning_timeout = goal->planning_timeout_sec > 0.0 ? goal->planning_timeout_sec
                                                                       : default_planning_timeout_sec_;
    const double velocity = clampScale(goal->velocity_scale, default_velocity_scale_);
    const double acceleration = clampScale(goal->acceleration_scale, default_acceleration_scale_);
    publishFeedback(goal_handle, "current_base_plan", 0, 0, 0);

    const auto current = lift_group.getCurrentState(2.0);
    if (!current)
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE;
      result->message = "No current RobotState";
      goal_handle->abort(result);
      clear_busy();
      return;
    }
    // MoveGroupInterface may return a state whose lazy FK cache has not yet been
    // evaluated. Work on an updated copy before querying base_link transforms.
    moveit::core::RobotState current_state(*current);
    current_state.update();
    const auto* arm_group = goal->arm == PlanPregrasp::Goal::LEFT ? left_arm_group_ : right_arm_group_;
    std::vector<std::string> normalized_bounds;
    const std::string bounds_violation =
        normalizeAndDescribeJointBounds(current_state, *arm_group, normalized_bounds);
    for (const auto& correction : normalized_bounds)
      RCLCPP_WARN(get_logger(), "Normalized numerical start-state bounds error: %s", correction.c_str());
    if (!bounds_violation.empty())
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      result->message = "Arm-only planning rejected: current arm state violates MoveIt bounds: " + bounds_violation;
      RCLCPP_ERROR(get_logger(), "%s", result->message.c_str());
      publishFeedback(goal_handle, "invalid_arm_start_state", 0, 0, 0);
      goal_handle->abort(result);
      clear_busy();
      return;
    }
    const auto route = OpenflexIkPipeline::routeStageZero(
        current_state, *arm_group, poseToEigen(goal->tcp_pregrasp_pose.pose), workspace_fixed_lift_radius_m_);
    result->initial_route = route == OpenflexIkPipeline::InitialRoute::STAGE_A
                                ? PlanPregrasp::Result::STAGE_A
                                : route == OpenflexIkPipeline::InitialRoute::STAGE_B ? PlanPregrasp::Result::STAGE_B
                                                                                       : PlanPregrasp::Result::STAGE_C;
    publishFeedback(goal_handle,
                    result->initial_route == PlanPregrasp::Result::STAGE_A
                        ? "stage_0_route_a"
                        : result->initial_route == PlanPregrasp::Result::STAGE_B ? "stage_0_route_b" : "stage_0_route_c",
                    0, 0, 0);

    if (max_dof < requiredDof(route))
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      result->message = "Stage 0 requires " + std::to_string(requiredDof(route)) +
                        " DOF, but max_dof=" + std::to_string(max_dof) + " forbids that stage";
      publishFeedback(goal_handle, "dof_limit_reached", 0, 0, 0);
      goal_handle->abort(result);
      clear_busy();
      return;
    }

    std::vector<Candidate> candidates;
    if (route == OpenflexIkPipeline::InitialRoute::STAGE_A)
    {
      publishFeedback(goal_handle, "stage_a", 0, 0, 0);
      if (auto direct = planAtBase(arm_group_interface, tip_link, current_state, currentBasePose(current_state), *goal,
                                   velocity, acceleration, candidate_plan_timeout_sec_, PlanPregrasp::Result::STAGE_A))
        candidates.push_back(std::move(*direct));

      if (candidates.empty() && max_dof >= PlanPregrasp::Goal::ARM_WITH_LIFT)
      {
        publishFeedback(goal_handle, "stage_b", 0, 0, 0);
        if (auto direct = planAtBase(lift_group, tip_link, current_state, currentBasePose(current_state), *goal,
                                     velocity, acceleration, candidate_plan_timeout_sec_, PlanPregrasp::Result::STAGE_B))
          candidates.push_back(std::move(*direct));
      }
    }
    else if (route == OpenflexIkPipeline::InitialRoute::STAGE_B)
    {
      publishFeedback(goal_handle, "stage_b", 0, 0, 0);
      if (auto direct = planAtBase(lift_group, tip_link, current_state, currentBasePose(current_state), *goal,
                                   velocity, acceleration, candidate_plan_timeout_sec_, PlanPregrasp::Result::STAGE_B))
        candidates.push_back(std::move(*direct));
    }

    if (candidates.empty() && max_dof < PlanPregrasp::Goal::WHOLE_BODY)
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
      result->message = max_dof == PlanPregrasp::Goal::ARM_ONLY
                            ? "Arm-only pregrasp planning failed; base and lift fallback are disabled"
                            : "No solution within max_dof=" + std::to_string(max_dof) +
                                  "; whole-body repositioning is disabled";
      publishFeedback(goal_handle, "dof_limit_reached", 0, 0, 0);
      goal_handle->abort(result);
      clear_busy();
      return;
    }

    if (candidates.empty())
    {
      publishFeedback(goal_handle, "stage_c", 0, 0, 0);
      publishFeedback(goal_handle, "front_candidate_search", 0, 0, 0);
      const double centre_yaw = goal->base_yaw_mode == PlanPregrasp::Goal::EXPLICIT
                                    ? goal->desired_base_yaw_odom
                                    : yawFromQuaternion(currentBasePose(current_state).pose.orientation);
      const auto yaws = symmetricYawSamples(
          centre_yaw, goal->base_yaw_tolerance > 0.0 ? goal->base_yaw_tolerance : default_yaw_range_rad_,
          goal->base_yaw_samples > 0 ? goal->base_yaw_samples : static_cast<uint16_t>(default_yaw_samples_));
      uint16_t generated = 0;
      uint16_t valid = 0;
      uint16_t planned = 0;
      for (const double yaw : yaws)
      {
        if (goal_handle->is_canceling())
        {
          result->message = "Canceled during candidate search";
          goal_handle->canceled(result);
          clear_busy();
          return;
        }
        auto base_goal = goal->tcp_pregrasp_pose;
        const Eigen::Vector2d base_xy = OpenflexIkPipeline::frontBaseCandidateXY(
            Eigen::Vector3d(goal->tcp_pregrasp_pose.pose.position.x, goal->tcp_pregrasp_pose.pose.position.y,
                            goal->tcp_pregrasp_pose.pose.position.z),
            yaw, front_distance, lateral);
        base_goal.pose.position.x = base_xy.x();
        base_goal.pose.position.y = base_xy.y();
        base_goal.pose.orientation = quaternionFromYaw(yaw);
        const auto* whole_group = goal->arm == PlanPregrasp::Goal::LEFT ? left_whole_body_group_ : right_whole_body_group_;
        for (int seed_index = 0; seed_index < 1; ++seed_index)
        {
          ++generated;
          if (auto candidate = planWholeBodyAtYaw(lift_group, whole_group, tip_link, current_state, base_goal, *goal,
                                                  velocity, acceleration, seed_index))
          {
            ++valid;
            ++planned;
            candidates.push_back(std::move(*candidate));
          }
          publishFeedback(goal_handle, "front_candidate_search", generated, valid, planned);
        }
      }
    }

    if (candidates.empty())
    {
      result->message = "No front-facing PickIK Stage C candidate could be planned";
      goal_handle->abort(result);
      clear_busy();
      return;
    }

    std::stable_sort(candidates.begin(), candidates.end(), [](const Candidate& left, const Candidate& right) {
      if (std::abs(left.base_distance - right.base_distance) > 1e-9)
        return left.base_distance < right.base_distance;
      return left.base_yaw_distance < right.base_yaw_distance;
    });
    Candidate selected = std::move(candidates.front());
    fillResult(*result, selected);
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

    if (!goal->execute)
    {
      result->message = "Plan-only request completed";
      goal_handle->succeed(result);
      clear_busy();
      return;
    }

    if (selected.stage != PlanPregrasp::Result::STAGE_C)
    {
      auto& execution_group = selected.stage == PlanPregrasp::Result::STAGE_A ? arm_group_interface : lift_group;
      publishFeedback(goal_handle, "arm_execute", 0, 0, 1);
      if (execution_group.execute(selected.plan) != moveit::core::MoveItErrorCode::SUCCESS)
      {
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
        result->message = "Arm trajectory execution failed";
        goal_handle->abort(result);
        clear_busy();
        return;
      }
      result->message = selected.stage == PlanPregrasp::Result::STAGE_A
                            ? "Stage A arm-only execution completed"
                            : "Stage B arm-and-lift execution completed";
      goal_handle->succeed(result);
      clear_busy();
      return;
    }

    if (!navigateTo(goal_handle, selected.base_goal))
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
      result->message = "Base navigation failed or was canceled";
      if (goal_handle->is_canceling())
        goal_handle->canceled(result);
      else
        goal_handle->abort(result);
      clear_busy();
      return;
    }

    publishFeedback(goal_handle, "arm_replan", 0, 0, 0);
    lift_group.setStartStateToCurrentState();
    lift_group.setPoseReferenceFrame(odom_frame_);
    lift_group.clearPoseTargets();
    lift_group.setPoseTarget(goal->tcp_pregrasp_pose.pose, tip_link);
    lift_group.setPlanningTime(planning_timeout);
    lift_group.setNumPlanningAttempts(1);
    lift_group.setMaxVelocityScalingFactor(velocity);
    lift_group.setMaxAccelerationScalingFactor(acceleration);
    moveit::planning_interface::MoveGroupInterface::Plan execution_plan;
    if (lift_group.plan(execution_plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      result->message = "Arm replan after base motion failed";
      goal_handle->abort(result);
      clear_busy();
      return;
    }
    publishFeedback(goal_handle, "arm_execute", 0, 0, 1);
    if (lift_group.execute(execution_plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
      result->message = "Arm trajectory execution failed";
      goal_handle->abort(result);
      clear_busy();
      return;
    }
    result->arm_trajectory = execution_plan.trajectory_;
    result->planning_time = execution_plan.planning_time_;
    result->message = "Base navigation and arm execution completed";
    goal_handle->succeed(result);
    clear_busy();
  }

  std::optional<Candidate> planWholeBodyAtYaw(
      moveit::planning_interface::MoveGroupInterface& group, const moveit::core::JointModelGroup* whole_group,
      const std::string& tip_link, const moveit::core::RobotState& current,
      geometry_msgs::msg::PoseStamped base_goal, const PlanPregrasp::Goal& goal, double velocity,
      double acceleration, int seed_index)
  {
    moveit::core::RobotState reference(current);
    reference.setVariablePosition("base_planar_joint/x", base_goal.pose.position.x);
    reference.setVariablePosition("base_planar_joint/y", base_goal.pose.position.y);
    reference.setVariablePosition("base_planar_joint/theta", yawFromQuaternion(base_goal.pose.orientation));
    reference.update();
    if (seed_index > 0)
      reference.setToRandomPositions(goal.arm == PlanPregrasp::Goal::LEFT ? left_arm_group_ : right_arm_group_);
    if (!reference.setFromIK(whole_group, goal.tcp_pregrasp_pose.pose, tip_link, goal.planning_timeout_sec > 0.0
                                                                                 ? goal.planning_timeout_sec
                                                                                 : default_planning_timeout_sec_))
      return std::nullopt;
    moveit::core::RobotState final_state(reference);
    final_state.update();
    if (!isFrontWorkspace(final_state, tip_link, goal.arm))
      return std::nullopt;
    const auto solved_base = currentBasePose(final_state);
    base_goal.pose.position.x = solved_base.pose.position.x;
    base_goal.pose.position.y = solved_base.pose.position.y;
    base_goal.pose.orientation = solved_base.pose.orientation;

    moveit::core::RobotState planning_start(current);
    planning_start.setVariablePosition("base_planar_joint/x", base_goal.pose.position.x);
    planning_start.setVariablePosition("base_planar_joint/y", base_goal.pose.position.y);
    planning_start.setVariablePosition("base_planar_joint/theta", yawFromQuaternion(base_goal.pose.orientation));
    planning_start.update();
    std::vector<double> joint_goal;
    final_state.copyJointGroupPositions(group.getRobotModel()->getJointModelGroup(group.getName()), joint_goal);
    group.setStartState(planning_start);
    group.clearPoseTargets();
    group.setJointValueTarget(joint_goal);
    group.setPlanningTime(candidate_plan_timeout_sec_);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(velocity);
    group.setMaxAccelerationScalingFactor(acceleration);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      return std::nullopt;

    Candidate candidate(planning_start);
    candidate.plan = std::move(plan);
    candidate.base_goal = base_goal;
    candidate.stage = PlanPregrasp::Result::STAGE_C;
    candidate.base_distance = std::hypot(base_goal.pose.position.x - currentBasePose(current).pose.position.x,
                                         base_goal.pose.position.y - currentBasePose(current).pose.position.y);
    candidate.base_yaw_distance = std::abs(normalizeAngle(yawFromQuaternion(base_goal.pose.orientation) -
                                                          yawFromQuaternion(currentBasePose(current).pose.orientation)));
    candidate.arm_goal.name = candidate.plan.trajectory_.joint_trajectory.joint_names;
    if (!candidate.plan.trajectory_.joint_trajectory.points.empty())
      candidate.arm_goal.position = candidate.plan.trajectory_.joint_trajectory.points.back().positions;
    for (std::size_t index = 0; index < candidate.arm_goal.name.size(); ++index)
      if (candidate.arm_goal.name[index] == "lift_joint")
        candidate.lift_goal = candidate.arm_goal.position[index];
    return candidate;
  }

  static Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose)
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    transform.linear() = Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                           .normalized()
                           .toRotationMatrix();
    return transform;
  }

  static geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    const Eigen::Quaterniond orientation(transform.rotation());
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
  }


  std::optional<Candidate> planAtBase(moveit::planning_interface::MoveGroupInterface& group, const std::string& tip_link,
                                      const moveit::core::RobotState& current,
                                      const geometry_msgs::msg::PoseStamped& base_goal,
                                      const PlanPregrasp::Goal& goal, double velocity, double acceleration, double timeout,
                                      uint8_t stage)
  {
    moveit::core::RobotState start(current);
    start.setVariablePosition("base_planar_joint/x", base_goal.pose.position.x);
    start.setVariablePosition("base_planar_joint/y", base_goal.pose.position.y);
    start.setVariablePosition("base_planar_joint/theta", yawFromQuaternion(base_goal.pose.orientation));
    start.update();
    group.setStartState(start);
    group.setPoseReferenceFrame(odom_frame_);
    group.clearPoseTargets();
    group.setPoseTarget(goal.tcp_pregrasp_pose.pose, tip_link);
    // The perception pose fixes the grasp center and approach direction, but
    // the 7-DOF arm may need a small wrist-roll/tilt adjustment at a fixed
    // base and lift. Keep this window bounded; it cannot introduce lift/base
    // motion and CartesianApproach performs the final path validation.
    // CartesianApproach enforces a 5 mm lateral corridor at the next stage.
    // Keep the entire pregrasp position error below that gate so a successful
    // pose plan cannot hand an invalid straight-line start to the next stage.
    group.setGoalPositionTolerance(0.003);
    // MoveIt applies this tolerance independently to the three orientation
    // constraint axes.  Keep each axis at 0.20 rad so their combined angular
    // error remains within CartesianApproach's 20 degree start gate.
    group.setGoalOrientationTolerance(0.20);
    group.setPlanningTime(timeout);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(velocity);
    group.setMaxAccelerationScalingFactor(acceleration);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS)
      return std::nullopt;

    moveit::core::RobotState final_state(start);
    if (!plan.trajectory_.joint_trajectory.points.empty())
    {
      const auto& point = plan.trajectory_.joint_trajectory.points.back();
      const auto& names = plan.trajectory_.joint_trajectory.joint_names;
      if (point.positions.size() != names.size())
        return std::nullopt;
      for (std::size_t index = 0; index < names.size(); ++index)
        final_state.setVariablePosition(names[index], point.positions[index]);
      final_state.update();
    }
    if (!isFrontWorkspace(final_state, tip_link, goal.arm))
      return std::nullopt;

    Candidate candidate(start);
    candidate.plan = std::move(plan);
    candidate.base_goal = base_goal;
    candidate.stage = stage;
    candidate.base_distance = std::hypot(base_goal.pose.position.x - currentBasePose(current).pose.position.x,
                                         base_goal.pose.position.y - currentBasePose(current).pose.position.y);
    candidate.base_yaw_distance = std::abs(normalizeAngle(yawFromQuaternion(base_goal.pose.orientation) -
                                                          yawFromQuaternion(currentBasePose(current).pose.orientation)));
    candidate.arm_goal.name = candidate.plan.trajectory_.joint_trajectory.joint_names;
    if (!candidate.plan.trajectory_.joint_trajectory.points.empty())
      candidate.arm_goal.position = candidate.plan.trajectory_.joint_trajectory.points.back().positions;
    for (std::size_t index = 0; index < candidate.arm_goal.name.size(); ++index)
      if (candidate.arm_goal.name[index] == "lift_joint")
        candidate.lift_goal = candidate.arm_goal.position[index];
    return candidate;
  }

  bool isFrontWorkspace(const moveit::core::RobotState& state, const std::string& tip_link, uint8_t arm) const
  {
    return OpenflexIkPipeline::withinFrontWorkspace(state, "base_link", tip_link,
                                                     arm == PlanPregrasp::Goal::LEFT, front_min_x_m_, front_max_x_m_,
                                                     front_half_angle_rad_);
  }

  geometry_msgs::msg::PoseStamped currentBasePose(const moveit::core::RobotState& state) const
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = odom_frame_;
    const Eigen::Isometry3d transform = state.getGlobalLinkTransform("base_link");
    const Eigen::Quaterniond orientation(transform.rotation());
    pose.pose.position.x = transform.translation().x();
    pose.pose.position.y = transform.translation().y();
    pose.pose.position.z = transform.translation().z();
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    return pose;
  }

  std::vector<double> symmetricYawSamples(double centre, double range, uint16_t count) const
  {
    std::vector<double> samples;
    samples.reserve(count);
    samples.push_back(normalizeAngle(centre));
    const int rings = std::max(1, static_cast<int>(count) / 2);
    for (int ring = 1; samples.size() < count; ++ring)
    {
      const double offset = range * static_cast<double>(ring) / rings;
      samples.push_back(normalizeAngle(centre - offset));
      if (samples.size() < count)
        samples.push_back(normalizeAngle(centre + offset));
    }
    return samples;
  }

  bool navigateTo(const std::shared_ptr<GoalHandlePlanPregrasp>& handle,
                  const geometry_msgs::msg::PoseStamped& base_goal)
  {
    publishFeedback(handle, "base_navigation", 0, 0, 0);
    if (!navigation_client_->wait_for_action_server(std::chrono::seconds(3)))
      return false;
    NavigateToPose::Goal goal;
    goal.pose = base_goal;
    auto goal_future = navigation_client_->async_send_goal(goal);
    if (goal_future.wait_for(std::chrono::seconds(3)) != std::future_status::ready)
      return false;
    const auto navigation_handle = goal_future.get();
    if (!navigation_handle)
      return false;
    auto result_future = navigation_client_->async_get_result(navigation_handle);
    const auto timeout = std::chrono::duration<double>(navigation_timeout_sec_);
    const auto started = std::chrono::steady_clock::now();
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready)
    {
      if (handle->is_canceling())
      {
        navigation_client_->async_cancel_goal(navigation_handle);
        return false;
      }
      if (std::chrono::steady_clock::now() - started > timeout)
      {
        navigation_client_->async_cancel_goal(navigation_handle);
        return false;
      }
    }
    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  void fillResult(PlanPregrasp::Result& result, const Candidate& candidate) const
  {
    result.selected_stage = candidate.stage;
    result.base_goal_odom = candidate.base_goal;
    result.lift_goal = candidate.lift_goal;
    result.arm_goal = candidate.arm_goal;
    result.arm_trajectory = candidate.plan.trajectory_;
    result.planning_time = candidate.plan.planning_time_;
  }

  void publishFeedback(const std::shared_ptr<GoalHandlePlanPregrasp>& handle, const std::string& phase,
                       uint16_t generated, uint16_t valid, uint16_t planned) const
  {
    auto feedback = std::make_shared<PlanPregrasp::Feedback>();
    feedback->phase = phase;
    feedback->candidates_generated = generated;
    feedback->candidates_valid = valid;
    feedback->candidates_planned = planned;
    handle->publish_feedback(feedback);
  }

  static double clampScale(double requested, double fallback)
  {
    const double value = requested > 0.0 ? requested : fallback;
    return std::clamp(value, 0.01, 1.0);
  }

  std::mutex goal_mutex_;
  bool busy_ = false;
  std::string odom_frame_;
  std::string left_tip_link_;
  std::string right_tip_link_;
  std::string left_group_name_;
  std::string right_group_name_;
  double left_lateral_offset_m_;
  double right_lateral_offset_m_;
  double front_min_x_m_;
  double front_max_x_m_;
  double front_half_angle_rad_;
  double workspace_fixed_lift_radius_m_;
  double default_yaw_range_rad_;
  double max_yaw_tolerance_rad_;
  int max_yaw_samples_;
  int default_yaw_samples_;
  int default_max_dof_;
  double default_front_distance_m_;
  double candidate_plan_timeout_sec_;
  double default_planning_timeout_sec_;
  double default_velocity_scale_;
  double default_acceleration_scale_;
  double navigation_timeout_sec_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_planning_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_planning_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> left_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> right_group_;
  const moveit::core::JointModelGroup* left_arm_group_ = nullptr;
  const moveit::core::JointModelGroup* right_arm_group_ = nullptr;
  const moveit::core::JointModelGroup* left_whole_body_group_ = nullptr;
  const moveit::core::JointModelGroup* right_whole_body_group_ = nullptr;
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigation_client_;
  rclcpp_action::Server<PlanPregrasp>::SharedPtr action_server_;
};
}  // namespace openarmx_integrated_moveit_config

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<openarmx_integrated_moveit_config::OpenflexPickTaskExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  std::thread spin_thread([&executor] { executor.spin(); });
  node->initializeMoveGroups();
  spin_thread.join();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
