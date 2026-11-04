#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <openarmx_integrated_moveit_msgs/action/execute_joint_posture.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace openarmx_integrated_moveit_config
{
class OpenflexJointPostureExecutor : public rclcpp::Node
{
public:
  using ExecuteJointPosture = openarmx_integrated_moveit_msgs::action::ExecuteJointPosture;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteJointPosture>;

  OpenflexJointPostureExecutor() : Node("openflex_joint_posture_executor")
  {
    action_name_ = declare_parameter<std::string>("action_name", "/openflex/execute_joint_posture");
    allowed_groups_ = declare_parameter<std::vector<std::string>>(
        "allowed_planning_groups",
        { "lift_only", "left_arm", "right_arm", "left_arm_with_lift", "right_arm_with_lift" });
    default_velocity_scale_ = declare_parameter<double>("default_velocity_scale", 0.20);
    default_acceleration_scale_ = declare_parameter<double>("default_acceleration_scale", 0.20);
    default_planning_timeout_sec_ = declare_parameter<double>("default_planning_timeout_sec", 5.0);
    current_state_timeout_sec_ = declare_parameter<double>("current_state_timeout_sec", 2.0);

    if (allowed_groups_.empty() || default_velocity_scale_ <= 0.0 || default_acceleration_scale_ <= 0.0 ||
        default_planning_timeout_sec_ <= 0.0 || current_state_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("joint posture executor parameters are invalid");
    }
    allowed_group_set_.insert(allowed_groups_.begin(), allowed_groups_.end());
  }

  void initializeMoveGroups()
  {
    for (const auto& group_name : allowed_groups_)
    {
      auto group =
          std::make_unique<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group_name);
      if (!group->startStateMonitor(0.1))
      {
        RCLCPP_WARN(get_logger(), "State monitor for '%s' is not complete yet", group_name.c_str());
      }
      groups_.emplace(group_name, std::move(group));
    }

    action_server_ = rclcpp_action::create_server<ExecuteJointPosture>(
        this, action_name_,
        std::bind(&OpenflexJointPostureExecutor::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&OpenflexJointPostureExecutor::handleCancel, this, std::placeholders::_1),
        std::bind(&OpenflexJointPostureExecutor::handleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Ready on %s for %zu whitelisted planning groups", action_name_.c_str(),
                allowed_groups_.size());
  }

private:
  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID&, const std::shared_ptr<const ExecuteJointPosture::Goal> goal)
  {
    const bool group_valid = allowed_group_set_.count(goal->planning_group) != 0;
    const bool target_valid = !goal->joint_goal.name.empty() &&
                              goal->joint_goal.name.size() == goal->joint_goal.position.size() &&
                              std::all_of(goal->joint_goal.position.begin(), goal->joint_goal.position.end(),
                                          [](double value) { return std::isfinite(value); });
    const bool request_valid = std::isfinite(goal->velocity_scale) &&
                               std::isfinite(goal->acceleration_scale) &&
                               std::isfinite(goal->planning_timeout_sec);
    if (!group_valid || !target_valid || !request_valid)
    {
      RCLCPP_WARN(get_logger(), "Rejecting invalid joint-posture goal for group '%s'",
                  goal->planning_group.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (busy_)
      return rclcpp_action::GoalResponse::REJECT;
    busy_ = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto group = groups_.find(goal_handle->get_goal()->planning_group);
    if (group != groups_.end())
      group->second->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto self = std::static_pointer_cast<OpenflexJointPostureExecutor>(shared_from_this());
    std::thread([self, goal_handle] { self->execute(goal_handle); }).detach();
  }

  static double scaleOrDefault(double requested, double fallback)
  {
    if (requested <= 0.0)
      requested = fallback;
    return std::clamp(requested, 0.01, 1.0);
  }

  void publishFeedback(const std::shared_ptr<GoalHandle>& goal_handle, const std::string& phase) const
  {
    auto feedback = std::make_shared<ExecuteJointPosture::Feedback>();
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
  }

  void finishBusy()
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    busy_ = false;
  }

  static sensor_msgs::msg::JointState targetMessage(
      const std::vector<std::string>& names, const moveit::core::RobotState& state)
  {
    sensor_msgs::msg::JointState message;
    message.name = names;
    message.position.reserve(names.size());
    for (const auto& name : names)
      message.position.push_back(state.getVariablePosition(name));
    return message;
  }

  void completeCanceled(
      const std::shared_ptr<GoalHandle>& goal_handle,
      const std::shared_ptr<ExecuteJointPosture::Result>& result,
      const std::string& message)
  {
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PREEMPTED;
    result->message = message;
    goal_handle->canceled(result);
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    try
    {
      executeImpl(goal_handle);
    }
    catch (const std::exception& exception)
    {
      RCLCPP_ERROR(get_logger(), "Unhandled joint-posture failure: %s", exception.what());
      auto result = std::make_shared<ExecuteJointPosture::Result>();
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      result->message = exception.what();
      if (goal_handle->is_canceling())
        goal_handle->canceled(result);
      else
        goal_handle->abort(result);
      finishBusy();
    }
  }

  void executeImpl(const std::shared_ptr<GoalHandle>& goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<ExecuteJointPosture::Result>();
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    auto& group = *groups_.at(goal->planning_group);

    publishFeedback(goal_handle, "current_state");
    const auto current = group.getCurrentState(current_state_timeout_sec_);
    if (goal_handle->is_canceling())
    {
      completeCanceled(goal_handle, result, "Canceled while waiting for current RobotState");
      finishBusy();
      return;
    }
    if (!current)
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE;
      result->message = "No current RobotState for joint-posture planning";
      goal_handle->abort(result);
      finishBusy();
      return;
    }

    const auto* joint_group = current->getJointModelGroup(goal->planning_group);
    if (!joint_group)
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GROUP_NAME;
      result->message = "MoveIt group is missing: " + goal->planning_group;
      goal_handle->abort(result);
      finishBusy();
      return;
    }

    const auto variable_names = joint_group->getVariableNames();
    const std::unordered_set<std::string> expected(variable_names.begin(), variable_names.end());
    const std::unordered_set<std::string> requested(
        goal->joint_goal.name.begin(), goal->joint_goal.name.end());
    if (requested.size() != goal->joint_goal.name.size() || requested != expected)
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      result->message = "Joint goal must contain every planning-group variable exactly once";
      goal_handle->abort(result);
      finishBusy();
      return;
    }

    moveit::core::RobotState target_state(*current);
    for (std::size_t index = 0; index < goal->joint_goal.name.size(); ++index)
      target_state.setVariablePosition(goal->joint_goal.name[index], goal->joint_goal.position[index]);
    target_state.update();
    if (!target_state.satisfiesBounds(joint_group))
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      result->message = "Joint-posture target violates MoveIt joint limits";
      goal_handle->abort(result);
      finishBusy();
      return;
    }

    result->resolved_joint_goal = targetMessage(variable_names, target_state);
    std::map<std::string, double> target_map;
    for (const auto& variable_name : variable_names)
      target_map[variable_name] = target_state.getVariablePosition(variable_name);

    group.setStartStateToCurrentState();
    group.clearPoseTargets();
    if (!group.setJointValueTarget(target_map))
    {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      result->message = "MoveIt rejected the joint-posture target";
      goal_handle->abort(result);
      finishBusy();
      return;
    }
    group.setPlanningTime(
        goal->planning_timeout_sec > 0.0 ? goal->planning_timeout_sec : default_planning_timeout_sec_);
    group.setNumPlanningAttempts(1);
    group.setMaxVelocityScalingFactor(scaleOrDefault(goal->velocity_scale, default_velocity_scale_));
    group.setMaxAccelerationScalingFactor(
        scaleOrDefault(goal->acceleration_scale, default_acceleration_scale_));

    if (goal_handle->is_canceling())
    {
      completeCanceled(goal_handle, result, "Canceled before joint-posture planning");
      finishBusy();
      return;
    }

    publishFeedback(goal_handle, "planning");
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_code = group.plan(plan);
    result->planning_time = plan.planning_time_;
    if (goal_handle->is_canceling())
    {
      completeCanceled(goal_handle, result, "Canceled during joint-posture planning");
      finishBusy();
      return;
    }
    if (plan_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result->error_code.val = plan_code.val;
      result->message = "MoveIt joint-posture planning failed";
      goal_handle->abort(result);
      finishBusy();
      return;
    }

    result->trajectory = plan.trajectory_;
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    if (!goal->execute)
    {
      result->message = "Joint-posture plan-only request completed";
      goal_handle->succeed(result);
      finishBusy();
      return;
    }

    if (goal_handle->is_canceling())
    {
      completeCanceled(goal_handle, result, "Canceled before joint-posture execution");
      finishBusy();
      return;
    }
    publishFeedback(goal_handle, "executing");
    const auto execute_code = group.execute(plan);
    if (goal_handle->is_canceling())
    {
      completeCanceled(goal_handle, result, "Joint-posture execution canceled");
      finishBusy();
      return;
    }
    if (execute_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      result->error_code.val = execute_code.val;
      result->message = "MoveIt joint-posture execution failed";
      goal_handle->abort(result);
      finishBusy();
      return;
    }

    result->message = "Joint-posture execution completed";
    goal_handle->succeed(result);
    finishBusy();
  }

  std::mutex goal_mutex_;
  bool busy_ = false;
  std::string action_name_;
  std::vector<std::string> allowed_groups_;
  std::unordered_set<std::string> allowed_group_set_;
  double default_velocity_scale_;
  double default_acceleration_scale_;
  double default_planning_timeout_sec_;
  double current_state_timeout_sec_;
  std::unordered_map<std::string, std::unique_ptr<moveit::planning_interface::MoveGroupInterface>> groups_;
  rclcpp_action::Server<ExecuteJointPosture>::SharedPtr action_server_;
};
}  // namespace openarmx_integrated_moveit_config

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<openarmx_integrated_moveit_config::OpenflexJointPostureExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  std::thread bootstrap_spin_thread([&executor] { executor.spin(); });
  node->initializeMoveGroups();
  executor.cancel();
  bootstrap_spin_thread.join();
  executor.spin();
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
