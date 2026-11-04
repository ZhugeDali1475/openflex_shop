#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <moveit/controller_manager/controller_manager.h>
#include <moveit_simple_controller_manager/follow_joint_trajectory_controller_handle.h>
#include <moveit_simple_controller_manager/gripper_controller_handle.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace openarmx_integrated_moveit_config
{
namespace
{
constexpr char BASE_CONTROLLER[] = "base_navigation_controller";
constexpr char BASE_JOINT[] = "base_planar_joint";
constexpr char ODOM_FRAME[] = "odom";

class NavigateToPoseControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  explicit NavigateToPoseControllerHandle(const rclcpp::Node::SharedPtr& node)
    : MoveItControllerHandle(BASE_CONTROLLER), node_(node), logger_(node->get_logger())
  {
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, "/navigate_to_pose");
  }

  bool sendTrajectory(const moveit_msgs::msg::RobotTrajectory& trajectory) override
  {
    const auto& multi_dof = trajectory.multi_dof_joint_trajectory;
    const auto joint_it = std::find(multi_dof.joint_names.begin(), multi_dof.joint_names.end(), BASE_JOINT);
    if (joint_it == multi_dof.joint_names.end() || multi_dof.points.empty())
    {
      RCLCPP_ERROR(logger_, "Base controller did not receive base_planar_joint trajectory data");
      return false;
    }
    const auto joint_index = static_cast<std::size_t>(std::distance(multi_dof.joint_names.begin(), joint_it));
    const auto& last_point = multi_dof.points.back();
    if (joint_index >= last_point.transforms.size())
    {
      RCLCPP_ERROR(logger_, "Base trajectory point has no transform for base_planar_joint");
      return false;
    }
    if (!client_->wait_for_action_server(std::chrono::seconds(2)))
    {
      RCLCPP_ERROR(logger_, "/navigate_to_pose action server is unavailable");
      return false;
    }

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header = multi_dof.header;
    if (goal.pose.header.frame_id.empty())
      goal.pose.header.frame_id = ODOM_FRAME;
    const auto& transform = last_point.transforms[joint_index];
    goal.pose.pose.position.x = transform.translation.x;
    goal.pose.pose.position.y = transform.translation.y;
    goal.pose.pose.position.z = transform.translation.z;
    goal.pose.pose.orientation = transform.rotation;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      done_ = false;
      goal_handle_.reset();
      status_ = moveit_controller_manager::ExecutionStatus::RUNNING;
    }
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this](const auto& goal_handle) {
      std::lock_guard<std::mutex> lock(mutex_);
      goal_handle_ = goal_handle;
      if (!goal_handle_)
      {
        status_ = moveit_controller_manager::ExecutionStatus::FAILED;
        done_ = true;
        done_condition_.notify_all();
      }
    };
    options.result_callback = [this](const auto& result) {
      std::lock_guard<std::mutex> lock(mutex_);
      switch (result.code)
      {
        case rclcpp_action::ResultCode::SUCCEEDED:
          status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
          break;
        case rclcpp_action::ResultCode::CANCELED:
          status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
          break;
        case rclcpp_action::ResultCode::ABORTED:
          status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
          break;
        default:
          status_ = moveit_controller_manager::ExecutionStatus::FAILED;
          break;
      }
      done_ = true;
      done_condition_.notify_all();
    };
    client_->async_send_goal(goal, options);
    return true;
  }

  bool cancelExecution() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (goal_handle_ && !done_)
      client_->async_cancel_goal(goal_handle_);
    return true;
  }

  bool waitForExecution(const rclcpp::Duration& timeout) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (timeout.nanoseconds() < 0)
    {
      done_condition_.wait(lock, [this] { return done_; });
      return true;
    }
    return done_condition_.wait_for(lock, std::chrono::nanoseconds(timeout.nanoseconds()), [this] { return done_; });
  }

  moveit_controller_manager::ExecutionStatus getLastExecutionStatus() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  std::mutex mutex_;
  std::condition_variable done_condition_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  moveit_controller_manager::ExecutionStatus status_{ moveit_controller_manager::ExecutionStatus::SUCCEEDED };
  bool done_{ true };
};

template <typename Handle>
std::shared_ptr<Handle> make_handle(const rclcpp::Node::SharedPtr& node, const std::string& name,
                                    const std::string& action_namespace, const std::vector<std::string>& joints)
{
  auto handle = std::make_shared<Handle>(node, name, action_namespace);
  for (const auto& joint : joints)
    handle->addJoint(joint);
  return handle;
}
}  // namespace

class OpenArmXControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node) override
  {
    controllers_.clear();
    joints_.clear();
    add_controller(
        "lift_joint_controller",
        make_handle<moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle>(
            node, "lift_joint_controller", "follow_joint_trajectory", { "lift_joint" }),
        { "lift_joint" });
    add_controller(
        "left_joint_trajectory_controller",
        make_handle<moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle>(
            node, "left_joint_trajectory_controller", "follow_joint_trajectory",
            { "openarmx_left_joint1", "openarmx_left_joint2", "openarmx_left_joint3", "openarmx_left_joint4",
              "openarmx_left_joint5", "openarmx_left_joint6", "openarmx_left_joint7" }),
        { "openarmx_left_joint1", "openarmx_left_joint2", "openarmx_left_joint3", "openarmx_left_joint4",
          "openarmx_left_joint5", "openarmx_left_joint6", "openarmx_left_joint7" });
    add_controller(
        "right_joint_trajectory_controller",
        make_handle<moveit_simple_controller_manager::FollowJointTrajectoryControllerHandle>(
            node, "right_joint_trajectory_controller", "follow_joint_trajectory",
            { "openarmx_right_joint1", "openarmx_right_joint2", "openarmx_right_joint3", "openarmx_right_joint4",
              "openarmx_right_joint5", "openarmx_right_joint6", "openarmx_right_joint7" }),
        { "openarmx_right_joint1", "openarmx_right_joint2", "openarmx_right_joint3", "openarmx_right_joint4",
          "openarmx_right_joint5", "openarmx_right_joint6", "openarmx_right_joint7" });
    add_controller(
        "left_gripper_controller",
        make_handle<moveit_simple_controller_manager::GripperControllerHandle>(
            node, "left_gripper_controller", "gripper_cmd", { "openarmx_left_finger_joint1" }),
        { "openarmx_left_finger_joint1" });
    add_controller(
        "right_gripper_controller",
        make_handle<moveit_simple_controller_manager::GripperControllerHandle>(
            node, "right_gripper_controller", "gripper_cmd", { "openarmx_right_finger_joint1" }),
        { "openarmx_right_finger_joint1" });
    add_controller(BASE_CONTROLLER, std::make_shared<NavigateToPoseControllerHandle>(node), { BASE_JOINT });
  }

  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string& name) override
  {
    const auto found = controllers_.find(name);
    return found == controllers_.end() ? nullptr : found->second;
  }

  void getControllersList(std::vector<std::string>& names) override
  {
    names.clear();
    for (const auto& [name, _] : controllers_)
      names.push_back(name);
  }

  void getActiveControllers(std::vector<std::string>& names) override
  {
    getControllersList(names);
  }

  void getControllerJoints(const std::string& name, std::vector<std::string>& joints) override
  {
    const auto found = joints_.find(name);
    joints = found == joints_.end() ? std::vector<std::string>{} : found->second;
  }

  ControllerState getControllerState(const std::string& name) override
  {
    ControllerState state;
    state.active_ = controllers_.count(name) != 0;
    state.default_ = state.active_;
    return state;
  }

  bool switchControllers(const std::vector<std::string>&, const std::vector<std::string>&) override
  {
    return true;
  }

private:
  void add_controller(const std::string& name, const moveit_controller_manager::MoveItControllerHandlePtr& handle,
                      std::vector<std::string> controller_joints)
  {
    controllers_[name] = handle;
    joints_[name] = std::move(controller_joints);
  }

  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> controllers_;
  std::map<std::string, std::vector<std::string>> joints_;
};

}  // namespace openarmx_integrated_moveit_config

PLUGINLIB_EXPORT_CLASS(openarmx_integrated_moveit_config::OpenArmXControllerManager,
                       moveit_controller_manager::MoveItControllerManager)
