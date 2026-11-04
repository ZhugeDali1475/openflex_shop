#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <openarmx_integrated_moveit_msgs/action/cartesian_approach.hpp>

namespace openflex_grasp_moveit
{

class CartesianApproachExecutor : public rclcpp::Node
{
public:
  using CartesianApproach = openarmx_integrated_moveit_msgs::action::CartesianApproach;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CartesianApproach>;

  CartesianApproachExecutor() : Node("cartesian_approach_executor")
  {
    action_name_ = declare_parameter<std::string>("action_name", "/openflex/cartesian_approach");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    left_tip_link_ = declare_parameter<std::string>("left_tip_link", "openarmx_left_hand_tcp");
    right_tip_link_ = declare_parameter<std::string>("right_tip_link", "openarmx_right_hand_tcp");
    default_eef_step_m_ = declare_parameter<double>("default_eef_step_m", 0.005);
    default_jump_threshold_ = declare_parameter<double>("default_jump_threshold", 0.0);
    default_min_fraction_ = declare_parameter<double>("default_min_fraction", 0.99);
    default_start_position_tolerance_m_ =
        declare_parameter<double>("default_start_position_tolerance_m", 0.015);
    default_start_orientation_tolerance_deg_ =
        declare_parameter<double>("default_start_orientation_tolerance_deg", 10.0);
    default_start_lateral_tolerance_m_ =
        declare_parameter<double>("default_start_lateral_tolerance_m", 0.005);
    default_orientation_step_deg_ =
        declare_parameter<double>("default_orientation_step_deg", 5.0);
    default_corridor_radius_m_ = declare_parameter<double>("default_corridor_radius_m", 0.06);
    default_corridor_endpoint_margin_m_ =
        declare_parameter<double>("default_corridor_endpoint_margin_m", 0.02);
    default_corridor_goal_position_tolerance_m_ =
        declare_parameter<double>("default_corridor_goal_position_tolerance_m", 0.005);
    default_corridor_goal_orientation_tolerance_deg_ =
        declare_parameter<double>("default_corridor_goal_orientation_tolerance_deg", 15.0);
    default_corridor_max_path_scale_ =
        declare_parameter<double>("default_corridor_max_path_scale", 2.5);
    default_corridor_max_extra_path_m_ =
        declare_parameter<double>("default_corridor_max_extra_path_m", 0.10);
    default_corridor_max_backtracking_m_ =
        declare_parameter<double>("default_corridor_max_backtracking_m", 0.03);
    default_corridor_planning_timeout_s_ =
        declare_parameter<double>("default_corridor_planning_timeout_s", 8.0);
    default_corridor_planning_attempts_ =
        declare_parameter<int>("default_corridor_planning_attempts", 3);
    corridor_validation_joint_step_rad_ =
        declare_parameter<double>("corridor_validation_joint_step_rad", 0.03);
    corridor_validation_tolerance_m_ =
        declare_parameter<double>("corridor_validation_tolerance_m", 0.003);
    execution_position_tolerance_m_ =
        declare_parameter<double>("execution_position_tolerance_m", 0.015);
    execution_orientation_tolerance_deg_ =
        declare_parameter<double>("execution_orientation_tolerance_deg", 10.0);
    default_velocity_scale_ = declare_parameter<double>("default_velocity_scale", 0.15);
    default_acceleration_scale_ = declare_parameter<double>("default_acceleration_scale", 0.15);
    if (!std::isfinite(execution_position_tolerance_m_) ||
        execution_position_tolerance_m_ <= 0.0 || execution_position_tolerance_m_ > 0.05 ||
        !std::isfinite(execution_orientation_tolerance_deg_) ||
        execution_orientation_tolerance_deg_ <= 0.0 ||
        execution_orientation_tolerance_deg_ > 30.0) {
      throw std::runtime_error("Invalid Cartesian execution endpoint tolerances");
    }
  }

  void initialize()
  {
    left_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "left_arm_with_lift");
    right_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "right_arm_with_lift");
    left_group_->setPoseReferenceFrame(odom_frame_);
    right_group_->setPoseReferenceFrame(odom_frame_);
    if (!left_group_->setEndEffectorLink(left_tip_link_))
      throw std::runtime_error("Failed to set left Cartesian end-effector link to " + left_tip_link_);
    if (!right_group_->setEndEffectorLink(right_tip_link_))
      throw std::runtime_error("Failed to set right Cartesian end-effector link to " + right_tip_link_);
    // Create subscriptions now, but do not block before rclcpp::spin() can process them.
    left_group_->startStateMonitor(0.0);
    right_group_->startStateMonitor(0.0);
    left_group_->allowReplanning(false);
    right_group_->allowReplanning(false);
    action_server_ = rclcpp_action::create_server<CartesianApproach>(
        this, action_name_,
        std::bind(&CartesianApproachExecutor::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CartesianApproachExecutor::handleCancel, this, std::placeholders::_1),
        std::bind(&CartesianApproachExecutor::handleAccepted, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Ready on %s (left_tip=%s, right_tip=%s)", action_name_.c_str(),
                left_group_->getEndEffectorLink().c_str(), right_group_->getEndEffectorLink().c_str());
  }

private:
  static bool finiteQuaternion(const geometry_msgs::msg::Quaternion& quaternion)
  {
    const double norm = std::sqrt(
        quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z +
        quaternion.w * quaternion.w);
    return std::isfinite(quaternion.x) && std::isfinite(quaternion.y) && std::isfinite(quaternion.z) &&
           std::isfinite(quaternion.w) && norm > 1e-6;
  }

  static bool finitePose(const geometry_msgs::msg::Pose& pose)
  {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) &&
           finiteQuaternion(pose.orientation);
  }

  static Eigen::Vector3d position(const geometry_msgs::msg::Pose& pose)
  {
    return {pose.position.x, pose.position.y, pose.position.z};
  }

  static Eigen::Quaterniond quaternion(const geometry_msgs::msg::Pose& pose)
  {
    return Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
        .normalized();
  }

  static double orientationDistance(const geometry_msgs::msg::Pose& first, const geometry_msgs::msg::Pose& second)
  {
    const double dot = std::clamp(std::abs(quaternion(first).dot(quaternion(second))), 0.0, 1.0);
    return 2.0 * std::acos(dot);
  }

  static geometry_msgs::msg::Pose poseFromTransform(const Eigen::Isometry3d& transform)
  {
    geometry_msgs::msg::Pose pose;
    const Eigen::Quaterniond rotation(transform.rotation());
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
  }

  static geometry_msgs::msg::Pose interpolate(
      const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end, double fraction)
  {
    geometry_msgs::msg::Pose pose;
    const Eigen::Vector3d point = position(start) + fraction * (position(end) - position(start));
    const Eigen::Quaterniond rotation = quaternion(start).slerp(fraction, quaternion(end)).normalized();
    pose.position.x = point.x();
    pose.position.y = point.y();
    pose.position.z = point.z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
  }

  struct CorridorMetrics
  {
    double max_lateral = 0.0;
    double path_length = 0.0;
    double backtracking = 0.0;
    double endpoint_position_error = 0.0;
    double endpoint_orientation_error = 0.0;
  };

  moveit_msgs::msg::Constraints makeCorridorConstraint(
      const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& end,
      const std::string& tip_link, double radius) const
  {
    const Eigen::Vector3d start_position = position(start);
    const Eigen::Vector3d delta = position(end) - start_position;
    const double distance = delta.norm();
    const Eigen::Quaterniond rotation =
        Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), delta / distance).normalized();

    shape_msgs::msg::SolidPrimitive corridor_box;
    corridor_box.type = shape_msgs::msg::SolidPrimitive::BOX;
    const double square_half_width = radius / std::sqrt(2.0);
    corridor_box.dimensions = {
        distance + 2.0 * default_corridor_endpoint_margin_m_,
        2.0 * square_half_width,
        2.0 * square_half_width,
    };

    geometry_msgs::msg::Pose corridor_pose;
    const Eigen::Vector3d center = start_position + 0.5 * delta;
    corridor_pose.position.x = center.x();
    corridor_pose.position.y = center.y();
    corridor_pose.position.z = center.z();
    corridor_pose.orientation.x = rotation.x();
    corridor_pose.orientation.y = rotation.y();
    corridor_pose.orientation.z = rotation.z();
    corridor_pose.orientation.w = rotation.w();

    moveit_msgs::msg::PositionConstraint position_constraint;
    position_constraint.header.frame_id = odom_frame_;
    position_constraint.link_name = tip_link;
    position_constraint.constraint_region.primitives.push_back(std::move(corridor_box));
    position_constraint.constraint_region.primitive_poses.push_back(std::move(corridor_pose));
    position_constraint.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.name = "bounded_tcp_corridor";
    constraints.position_constraints.push_back(std::move(position_constraint));
    return constraints;
  }

  bool validateCorridorTrajectory(
      const moveit::planning_interface::MoveGroupInterface& group,
      const moveit::core::RobotState& start_state,
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const geometry_msgs::msg::Pose& line_start,
      const geometry_msgs::msg::Pose& line_end,
      const std::string& tip_link,
      double radius,
      double max_path_length,
      double max_backtracking,
      CorridorMetrics& metrics,
      std::string& failure) const
  {
    const Eigen::Vector3d start_position = position(line_start);
    const Eigen::Vector3d delta = position(line_end) - start_position;
    const double direct_distance = delta.norm();
    if (direct_distance < 1e-6) {
      failure = "bounded corridor requires a non-zero translation";
      return false;
    }
    const Eigen::Vector3d direction = delta / direct_distance;
    const auto* joint_group = group.getRobotModel()->getJointModelGroup(group.getName());
    if (!joint_group) {
      failure = "planning group is missing from the robot model";
      return false;
    }

    robot_trajectory::RobotTrajectory candidate(group.getRobotModel(), group.getName());
    candidate.setRobotTrajectoryMsg(start_state, trajectory);
    if (candidate.getWayPointCount() == 0) {
      failure = "planner returned an empty trajectory";
      return false;
    }

    Eigen::Vector3d previous_tcp = start_position;
    double previous_progress = 0.0;
    geometry_msgs::msg::Pose final_pose = line_start;
    const double position_limit = radius + corridor_validation_tolerance_m_;
    const double longitudinal_limit =
        default_corridor_endpoint_margin_m_ + corridor_validation_tolerance_m_;
    const auto validate_pose = [&](const geometry_msgs::msg::Pose& pose) -> bool {
      const Eigen::Vector3d tcp = position(pose);
      const Eigen::Vector3d offset = tcp - start_position;
      const double progress = offset.dot(direction);
      const double lateral = (offset - progress * direction).norm();
      metrics.path_length += (tcp - previous_tcp).norm();
      metrics.max_lateral = std::max(metrics.max_lateral, lateral);
      if (progress < previous_progress)
        metrics.backtracking += previous_progress - progress;
      previous_progress = progress;
      previous_tcp = tcp;
      final_pose = pose;

      if (!std::isfinite(lateral) || lateral > position_limit) {
        failure = "TCP leaves corridor: lateral=" + std::to_string(lateral) +
                  " m, limit=" + std::to_string(radius) + " m";
        return false;
      }
      if (progress < -longitudinal_limit || progress > direct_distance + longitudinal_limit) {
        failure = "TCP leaves corridor segment longitudinally: progress=" +
                  std::to_string(progress) + " m";
        return false;
      }
      if (metrics.path_length > max_path_length + corridor_validation_tolerance_m_) {
        failure = "TCP path is too indirect: length=" + std::to_string(metrics.path_length) +
                  " m, limit=" + std::to_string(max_path_length) + " m";
        return false;
      }
      if (metrics.backtracking > max_backtracking + corridor_validation_tolerance_m_) {
        failure = "TCP path backtracks too far: backtracking=" +
                  std::to_string(metrics.backtracking) + " m, limit=" +
                  std::to_string(max_backtracking) + " m";
        return false;
      }
      return true;
    };

    moveit::core::RobotState previous_state(start_state);
    for (std::size_t index = 0; index < candidate.getWayPointCount(); ++index) {
      const auto& waypoint = candidate.getWayPoint(index);
      const double joint_distance = previous_state.distance(waypoint, joint_group);
      const std::size_t sample_count = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(joint_distance / corridor_validation_joint_step_rad_)));
      for (std::size_t sample_index = 1; sample_index <= sample_count; ++sample_index) {
        moveit::core::RobotState sampled(previous_state);
        previous_state.interpolate(
            waypoint,
            static_cast<double>(sample_index) / static_cast<double>(sample_count),
            sampled,
            joint_group);
        sampled.update();
        if (!validate_pose(poseFromTransform(sampled.getGlobalLinkTransform(tip_link))))
          return false;
      }
      previous_state = waypoint;
    }

    metrics.endpoint_position_error = (position(final_pose) - position(line_end)).norm();
    metrics.endpoint_orientation_error = orientationDistance(final_pose, line_end);
    if (metrics.endpoint_position_error > default_corridor_goal_position_tolerance_m_) {
      failure = "corridor endpoint position error=" +
                std::to_string(metrics.endpoint_position_error) + " m, limit=" +
                std::to_string(default_corridor_goal_position_tolerance_m_) + " m";
      return false;
    }
    const double orientation_limit = default_corridor_goal_orientation_tolerance_deg_ * M_PI / 180.0;
    if (metrics.endpoint_orientation_error > orientation_limit) {
      failure = "corridor endpoint orientation error=" +
                std::to_string(metrics.endpoint_orientation_error * 180.0 / M_PI) +
                " deg, limit=" +
                std::to_string(default_corridor_goal_orientation_tolerance_deg_) + " deg";
      return false;
    }
    return true;
  }

  bool validateMinimumTcpHeight(
      const moveit::planning_interface::MoveGroupInterface& group,
      const moveit::core::RobotState& start_state,
      const moveit_msgs::msg::RobotTrajectory& trajectory,
      const std::string& tip_link,
      double minimum_tcp_z,
      std::string& failure) const
  {
    const auto* joint_group = group.getRobotModel()->getJointModelGroup(group.getName());
    if (!joint_group) {
      failure = "planning group is missing from the robot model";
      return false;
    }
    const auto validate_state = [&](const moveit::core::RobotState& state) {
      const double tcp_z = state.getGlobalLinkTransform(tip_link).translation().z();
      if (!std::isfinite(tcp_z) || tcp_z + 1e-6 < minimum_tcp_z) {
        failure = "TCP height " + std::to_string(tcp_z) +
                  " m is below minimum " + std::to_string(minimum_tcp_z) + " m";
        return false;
      }
      return true;
    };

    moveit::core::RobotState checked_start(start_state);
    checked_start.update();
    if (!validate_state(checked_start))
      return false;

    robot_trajectory::RobotTrajectory candidate(group.getRobotModel(), group.getName());
    candidate.setRobotTrajectoryMsg(start_state, trajectory);
    if (candidate.getWayPointCount() == 0) {
      failure = "planner returned an empty trajectory";
      return false;
    }

    moveit::core::RobotState previous_state(start_state);
    for (std::size_t index = 0; index < candidate.getWayPointCount(); ++index) {
      const auto& waypoint = candidate.getWayPoint(index);
      const double joint_distance = previous_state.distance(waypoint, joint_group);
      const std::size_t sample_count = std::max<std::size_t>(
          1, static_cast<std::size_t>(
                 std::ceil(joint_distance / corridor_validation_joint_step_rad_)));
      for (std::size_t sample_index = 1; sample_index <= sample_count; ++sample_index) {
        moveit::core::RobotState sampled(previous_state);
        previous_state.interpolate(
            waypoint,
            static_cast<double>(sample_index) / static_cast<double>(sample_count),
            sampled,
            joint_group);
        sampled.update();
        if (!validate_state(sampled))
          return false;
      }
      previous_state = waypoint;
    }
    return true;
  }

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID&, std::shared_ptr<const CartesianApproach::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (busy_ || !left_group_ || !right_group_ ||
        (goal->arm != CartesianApproach::Goal::LEFT && goal->arm != CartesianApproach::Goal::RIGHT) ||
        goal->start_pose.header.frame_id != odom_frame_ || goal->end_pose.header.frame_id != odom_frame_ ||
        !finitePose(goal->start_pose.pose) || !finitePose(goal->end_pose.pose))
      return rclcpp_action::GoalResponse::REJECT;
    if (goal->eef_step < 0.0 || goal->jump_threshold < 0.0 || goal->min_fraction < 0.0 ||
        goal->min_fraction > 1.0 || goal->start_position_tolerance < 0.0 ||
        goal->start_orientation_tolerance_deg < 0.0 ||
        goal->orientation_mode > CartesianApproach::Goal::TRANSLATE_BEFORE_ROTATION ||
        goal->path_mode > CartesianApproach::Goal::UNCONSTRAINED_MOVEIT ||
        !std::isfinite(goal->corridor_radius) || !std::isfinite(goal->corridor_max_path_length) ||
        !std::isfinite(goal->corridor_max_backtracking) ||
        !std::isfinite(goal->corridor_planning_timeout) || goal->corridor_radius < 0.0 ||
        goal->corridor_max_path_length < 0.0 || goal->corridor_max_backtracking < 0.0 ||
        goal->corridor_planning_timeout < 0.0)
      return rclcpp_action::GoalResponse::REJECT;
    if (goal->use_minimum_tcp_z && !std::isfinite(goal->minimum_tcp_z))
      return rclcpp_action::GoalResponse::REJECT;
    if (!goal->end_joint_positions.empty() &&
        (goal->path_mode != CartesianApproach::Goal::UNCONSTRAINED_MOVEIT ||
         goal->end_joint_positions.size() != 8 ||
         std::any_of(
             goal->end_joint_positions.begin(), goal->end_joint_positions.end(),
             [](double position) { return !std::isfinite(position); })))
      return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread([this, goal_handle] { execute(goal_handle); }).detach();
  }

  void publishFeedback(const std::shared_ptr<GoalHandle>& handle, const std::string& phase, double fraction)
  {
    auto feedback = std::make_shared<CartesianApproach::Feedback>();
    feedback->phase = phase;
    feedback->path_fraction = fraction;
    handle->publish_feedback(feedback);
  }

  void execute(const std::shared_ptr<GoalHandle>& handle)
  {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<CartesianApproach::Result>();
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      busy_ = true;
    }
    const auto clear_busy = [this] {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      busy_ = false;
    };

    auto& group = goal->arm == CartesianApproach::Goal::LEFT ? *left_group_ : *right_group_;
    const std::string& tip_link = goal->arm == CartesianApproach::Goal::LEFT ? left_tip_link_ : right_tip_link_;
    const double eef_step = goal->eef_step > 0.0 ? goal->eef_step : default_eef_step_m_;
    const double min_fraction = goal->min_fraction > 0.0 ? goal->min_fraction : default_min_fraction_;
    const double position_tolerance = goal->start_position_tolerance > 0.0
                                          ? goal->start_position_tolerance
                                          : default_start_position_tolerance_m_;
    const double orientation_tolerance =
        (goal->start_orientation_tolerance_deg > 0.0 ? goal->start_orientation_tolerance_deg
                                                      : default_start_orientation_tolerance_deg_) *
        M_PI / 180.0;
    const double velocity = std::clamp(
        goal->velocity_scale > 0.0 ? goal->velocity_scale : default_velocity_scale_, 0.01, 1.0);
    const double acceleration = std::clamp(
        goal->acceleration_scale > 0.0 ? goal->acceleration_scale : default_acceleration_scale_, 0.01, 1.0);
    const double orientation_step_rad = default_orientation_step_deg_ * M_PI / 180.0;
    const bool unconstrained_moveit =
        goal->path_mode == CartesianApproach::Goal::UNCONSTRAINED_MOVEIT;
    const bool use_joint_target =
        unconstrained_moveit && !goal->end_joint_positions.empty();
    if (!unconstrained_moveit &&
        (!std::isfinite(default_orientation_step_deg_) || default_orientation_step_deg_ <= 0.0 ||
         default_orientation_step_deg_ > 45.0)) {
      result->message = "default_orientation_step_deg must be within (0, 45]";
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      handle->abort(result);
      clear_busy();
      return;
    }

    if (!unconstrained_moveit && (eef_step < 0.0005 || eef_step > 0.02)) {
      result->message = "eef_step must be between 0.0005 m and 0.02 m";
      handle->abort(result);
      clear_busy();
      return;
    }

    group.setPoseReferenceFrame(odom_frame_);
    auto start_state = group.getCurrentState(5.0);
    if (!start_state) {
      result->message = "No complete current RobotState for Cartesian segment";
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE;
      handle->abort(result);
      clear_busy();
      return;
    }
    if (goal->use_start_joint_state)
    {
      if (goal->execute)
      {
        result->message = "explicit start_joint_state is only valid for plan-only requests";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        handle->abort(result);
        clear_busy();
        return;
      }
      if (goal->start_joint_state.name.size() != goal->start_joint_state.position.size() ||
          goal->start_joint_state.name.empty())
      {
        result->message = "start_joint_state names/positions are empty or mismatched";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
        handle->abort(result);
        clear_busy();
        return;
      }
      const auto& model_variables = start_state->getRobotModel()->getVariableNames();
      for (std::size_t index = 0; index < goal->start_joint_state.name.size(); ++index)
      {
        if (std::find(model_variables.begin(), model_variables.end(),
                      goal->start_joint_state.name[index]) == model_variables.end())
        {
          result->message = "start_joint_state contains an unknown joint: " +
                            goal->start_joint_state.name[index];
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
          handle->abort(result);
          clear_busy();
          return;
        }
        if (!std::isfinite(goal->start_joint_state.position[index]))
        {
          result->message = "start_joint_state contains a non-finite value";
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
          handle->abort(result);
          clear_busy();
          return;
        }
        start_state->setVariablePosition(
            goal->start_joint_state.name[index], goal->start_joint_state.position[index]);
      }
    }
    start_state->update();
    group.setStartState(*start_state);
    const auto current_pose = poseFromTransform(start_state->getGlobalLinkTransform(tip_link));
    const double position_error = (position(current_pose) - position(goal->start_pose.pose)).norm();
    const double orientation_error = orientationDistance(current_pose, goal->start_pose.pose);
    if (!unconstrained_moveit &&
        (position_error > position_tolerance || orientation_error > orientation_tolerance)) {
      result->message = "current TCP is not at the requested Cartesian segment start: position error=" +
                        std::to_string(position_error) + " m, orientation error=" +
                        std::to_string(orientation_error * 180.0 / M_PI) + " deg";
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      handle->abort(result);
      clear_busy();
      return;
    }

    const auto& requested_start = unconstrained_moveit ? current_pose : goal->start_pose.pose;
    const Eigen::Vector3d requested_delta = position(goal->end_pose.pose) - position(requested_start);
    const double requested_distance = requested_delta.norm();
    const double requested_orientation_distance = orientationDistance(
        requested_start, goal->end_pose.pose);
    if (!unconstrained_moveit && requested_distance < 1e-6 && requested_orientation_distance < 1e-6) {
      result->message = "Cartesian segment has neither translation nor orientation change";
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      handle->abort(result);
      clear_busy();
      return;
    }
    if (goal->path_mode == CartesianApproach::Goal::STRICT_CARTESIAN && requested_distance >= 1e-6) {
      const Eigen::Vector3d requested_direction = requested_delta / requested_distance;
      const Eigen::Vector3d measured_offset = position(current_pose) - position(goal->start_pose.pose);
      const double measured_longitudinal = measured_offset.dot(requested_direction);
      const double measured_lateral =
          (measured_offset - measured_longitudinal * requested_direction).norm();
      if (measured_lateral > default_start_lateral_tolerance_m_) {
        result->message = "actual TCP is laterally displaced from the requested straight approach line: " +
                          std::to_string(measured_lateral) + " m";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        handle->abort(result);
        clear_busy();
        return;
      }
    }

    // Start from measured TCP feedback so tracking error does not create an
    // unplanned correction before the straight segment.
    const auto path_start = current_pose;
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = 0.0;
    CorridorMetrics corridor_metrics;
    if (goal->path_mode == CartesianApproach::Goal::BOUNDED_CORRIDOR) {
      const double direct_distance = (position(goal->end_pose.pose) - position(path_start)).norm();
      const double corridor_radius =
          goal->corridor_radius > 0.0 ? goal->corridor_radius : default_corridor_radius_m_;
      const double max_path_length = goal->corridor_max_path_length > 0.0
                                         ? goal->corridor_max_path_length
                                         : std::max(
                                               direct_distance * default_corridor_max_path_scale_,
                                               direct_distance + default_corridor_max_extra_path_m_);
      const double max_backtracking = goal->corridor_max_backtracking > 0.0
                                          ? goal->corridor_max_backtracking
                                          : default_corridor_max_backtracking_m_;
      const double planning_timeout = goal->corridor_planning_timeout > 0.0
                                          ? goal->corridor_planning_timeout
                                          : default_corridor_planning_timeout_s_;
      // A camera mounted close to the TCP can legitimately produce a
      // pregrasp only a few millimetres from the photo pose. Keep the
      // non-zero-translation guard numerical rather than imposing a 1 cm
      // motion requirement.
      if (direct_distance < 1e-4 || corridor_radius < 0.005 || corridor_radius > 0.20 ||
          max_path_length < direct_distance || max_path_length > 5.0 ||
          max_backtracking > max_path_length || planning_timeout < 0.5 || planning_timeout > 60.0 ||
          default_corridor_endpoint_margin_m_ < 0.0 ||
          default_corridor_goal_position_tolerance_m_ <= 0.0 ||
          default_corridor_goal_orientation_tolerance_deg_ <= 0.0 ||
          default_corridor_goal_orientation_tolerance_deg_ > 45.0 ||
          default_corridor_planning_attempts_ < 1 || default_corridor_planning_attempts_ > 20 ||
          corridor_validation_joint_step_rad_ <= 0.0 || corridor_validation_joint_step_rad_ > 0.2 ||
          corridor_validation_tolerance_m_ < 0.0 || corridor_validation_tolerance_m_ > 0.01) {
        result->message = "Invalid bounded-corridor planning limits: direct_distance=" +
                          std::to_string(direct_distance) + " m, radius=" +
                          std::to_string(corridor_radius) + " m, max_path=" +
                          std::to_string(max_path_length) + " m, max_backtracking=" +
                          std::to_string(max_backtracking) + " m, timeout=" +
                          std::to_string(planning_timeout) + " s";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        handle->abort(result);
        clear_busy();
        return;
      }

      group.clearPathConstraints();
      group.clearPoseTargets();
      group.setPlanningTime(planning_timeout);
      // Run attempts as separate requests so an invalid postprocessed OMPL
      // trajectory can be discarded before trying a fresh plan.
      group.setNumPlanningAttempts(1);
      group.setGoalPositionTolerance(default_corridor_goal_position_tolerance_m_);
      // MoveIt applies this value independently to roll, pitch and yaw. Scale
      // it so every accepted endpoint also satisfies our total-angle check.
      group.setGoalOrientationTolerance(
          default_corridor_goal_orientation_tolerance_deg_ * M_PI / (180.0 * std::sqrt(3.0)));
      group.setMaxVelocityScalingFactor(velocity);
      group.setMaxAccelerationScalingFactor(acceleration);
      group.setPathConstraints(makeCorridorConstraint(
          path_start, goal->end_pose.pose, tip_link, corridor_radius));
      if (!group.setPoseTarget(goal->end_pose, tip_link)) {
        group.clearPathConstraints();
        group.clearPoseTargets();
        result->message = "Failed to set bounded-corridor endpoint pose";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        handle->abort(result);
        clear_busy();
        return;
      }

      publishFeedback(handle, "corridor_plan", 0.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      moveit::core::MoveItErrorCode planning_code(
          moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
      std::string validation_failure;
      bool corridor_plan_valid = false;
      for (int attempt = 1; attempt <= default_corridor_planning_attempts_; ++attempt) {
        if (handle->is_canceling())
          break;

        group.setStartState(*start_state);
        plan = moveit::planning_interface::MoveGroupInterface::Plan();
        planning_code = group.plan(plan);
        if (planning_code != moveit::core::MoveItErrorCode::SUCCESS) {
          validation_failure.clear();
          RCLCPP_WARN(
              get_logger(), "Bounded-corridor planning attempt %d/%d failed with MoveIt code %d",
              attempt, default_corridor_planning_attempts_, planning_code.val);
          continue;
        }

        CorridorMetrics attempt_metrics;
        std::string attempt_failure;
        if (!validateCorridorTrajectory(
                group, *start_state, plan.trajectory_, path_start, goal->end_pose.pose,
                tip_link, corridor_radius, max_path_length, max_backtracking,
                attempt_metrics, attempt_failure)) {
          validation_failure = std::move(attempt_failure);
          planning_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
          RCLCPP_WARN(
              get_logger(), "Bounded-corridor planning attempt %d/%d rejected: %s",
              attempt, default_corridor_planning_attempts_, validation_failure.c_str());
          continue;
        }

        corridor_metrics = attempt_metrics;
        corridor_plan_valid = true;
        break;
      }
      group.clearPathConstraints();
      group.clearPoseTargets();
      if (handle->is_canceling()) {
        group.stop();
        result->message = "Bounded-corridor planning canceled";
        handle->canceled(result);
        clear_busy();
        return;
      }
      if (!corridor_plan_valid) {
        result->error_code.val = planning_code.val;
        result->message = validation_failure.empty()
                              ? "Bounded-corridor planning failed after " +
                                    std::to_string(default_corridor_planning_attempts_) +
                                    " attempts; last MoveIt code " + std::to_string(planning_code.val)
                              : "Bounded-corridor trajectory rejected after " +
                                    std::to_string(default_corridor_planning_attempts_) +
                                    " attempts: " + validation_failure;
        handle->abort(result);
        clear_busy();
        return;
      }
      trajectory = std::move(plan.trajectory_);
      fraction = 1.0;
      publishFeedback(handle, "corridor_plan", fraction);
      RCLCPP_INFO(
          get_logger(),
          "Validated corridor plan: radius=%.3f m, max_lateral=%.3f m, path=%.3f/%.3f m, backtrack=%.3f/%.3f m",
          corridor_radius, corridor_metrics.max_lateral, corridor_metrics.path_length,
          max_path_length, corridor_metrics.backtracking, max_backtracking);
    } else if (goal->path_mode == CartesianApproach::Goal::UNCONSTRAINED_MOVEIT) {
      const double planning_timeout = goal->corridor_planning_timeout > 0.0
                                          ? goal->corridor_planning_timeout
                                          : default_corridor_planning_timeout_s_;
      group.clearPathConstraints();
      group.clearPoseTargets();
      group.setStartState(*start_state);
      if (planning_timeout > 0.0)
        group.setPlanningTime(planning_timeout);
      group.setMaxVelocityScalingFactor(velocity);
      group.setMaxAccelerationScalingFactor(acceleration);
      if (use_joint_target) {
        const auto* joint_model_group = start_state->getJointModelGroup(group.getName());
        if (!joint_model_group ||
            joint_model_group->getVariableCount() != goal->end_joint_positions.size()) {
          result->message = "MoveIt right-arm joint target has the wrong size";
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
          handle->abort(result);
          clear_busy();
          return;
        }

        std::vector<double> requested_joint_target(
            goal->end_joint_positions.begin(), goal->end_joint_positions.end());
        moveit::core::RobotState bounded_target(*start_state);
        bounded_target.setJointGroupPositions(joint_model_group, requested_joint_target);
        bounded_target.enforceBounds(joint_model_group);
        std::vector<double> bounded_joint_target;
        bounded_target.copyJointGroupPositions(joint_model_group, bounded_joint_target);
        double max_bound_adjustment = 0.0;
        for (std::size_t index = 0; index < bounded_joint_target.size(); ++index) {
          max_bound_adjustment = std::max(
              max_bound_adjustment,
              std::abs(bounded_joint_target[index] - requested_joint_target[index]));
        }
        if (max_bound_adjustment > 1e-9) {
          RCLCPP_WARN(
              get_logger(),
              "Photo joint target was clamped to the right-arm model bounds (max adjustment %.6f rad)",
              max_bound_adjustment);
        }
        if (!group.setJointValueTarget(bounded_joint_target)) {
          result->message = "MoveIt failed to set the bounded right-arm photo joint target";
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
          handle->abort(result);
          clear_busy();
          return;
        }
      } else if (!group.setPoseTarget(goal->end_pose, tip_link)) {
        result->message = "MoveIt failed to set the unconstrained endpoint pose";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        handle->abort(result);
        clear_busy();
        return;
      }

      publishFeedback(handle, "moveit_plan", 0.0);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto planning_code = group.plan(plan);
      group.clearPathConstraints();
      group.clearPoseTargets();
      if (handle->is_canceling()) {
        group.stop();
        result->message = "Unconstrained MoveIt planning canceled";
        handle->canceled(result);
        clear_busy();
        return;
      }
      if (planning_code != moveit::core::MoveItErrorCode::SUCCESS) {
        result->error_code.val = planning_code.val;
        result->message = std::string("Unconstrained MoveIt ") +
                          (use_joint_target ? "joint-target" : "pose") +
                          " planning failed with code " +
                          std::to_string(planning_code.val);
        handle->abort(result);
        clear_busy();
        return;
      }
      trajectory = std::move(plan.trajectory_);
      fraction = 1.0;
      publishFeedback(handle, "moveit_plan", fraction);
      RCLCPP_INFO(
          get_logger(), "Standard unconstrained MoveIt %s plan completed",
          use_joint_target ? "joint-target" : "pose");
    } else {
    const double path_distance = (position(goal->end_pose.pose) - position(path_start)).norm();
    const double path_orientation_distance = orientationDistance(path_start, goal->end_pose.pose);
    const std::size_t translation_count = path_distance >= 1e-6
                                              ? static_cast<std::size_t>(std::ceil(path_distance / eef_step))
                                              : 0;
    const std::size_t rotation_count = path_orientation_distance >= 1e-6
                                           ? static_cast<std::size_t>(
                                               std::ceil(path_orientation_distance / orientation_step_rad))
                                           : 0;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.reserve(translation_count + rotation_count + 1);
    const auto add_rotation_waypoints = [&](const geometry_msgs::msg::Pose& rotation_start,
                                            const geometry_msgs::msg::Pose& rotation_end,
                                            const geometry_msgs::msg::Point& position_anchor) {
      for (std::size_t index = 1; index <= rotation_count; ++index) {
        auto waypoint = interpolate(
            rotation_start, rotation_end,
            static_cast<double>(index) / static_cast<double>(rotation_count));
        waypoint.position = position_anchor;
        waypoints.push_back(std::move(waypoint));
      }
    };
    const auto add_translation_waypoints = [&](const geometry_msgs::msg::Pose& translation_start,
                                               const geometry_msgs::msg::Pose& translation_end) {
      for (std::size_t index = 1; index <= translation_count; ++index) {
        auto waypoint = interpolate(
            translation_start, translation_end,
            static_cast<double>(index) / static_cast<double>(translation_count));
        waypoint.orientation = translation_end.orientation;
        waypoints.push_back(std::move(waypoint));
      }
    };

    if (goal->orientation_mode == CartesianApproach::Goal::ROTATE_BEFORE_TRANSLATION) {
      add_rotation_waypoints(path_start, goal->end_pose.pose, path_start.position);
      add_translation_waypoints(path_start, goal->end_pose.pose);
    } else if (goal->orientation_mode == CartesianApproach::Goal::TRANSLATE_BEFORE_ROTATION) {
      for (std::size_t index = 1; index <= translation_count; ++index) {
        auto waypoint = interpolate(
            path_start, goal->end_pose.pose,
            static_cast<double>(index) / static_cast<double>(translation_count));
        waypoint.orientation = path_start.orientation;
        waypoints.push_back(std::move(waypoint));
      }
      add_rotation_waypoints(
          path_start, goal->end_pose.pose, goal->end_pose.pose.position);
    } else {
      const std::size_t segment_count = std::max<std::size_t>(
          1, std::max(translation_count, rotation_count));
      waypoints.reserve(segment_count);
      for (std::size_t index = 1; index <= segment_count; ++index)
        waypoints.push_back(interpolate(path_start, goal->end_pose.pose,
                                        static_cast<double>(index) / static_cast<double>(segment_count)));
    }

    publishFeedback(handle, "cartesian_plan", 0.0);
    moveit_msgs::msg::MoveItErrorCodes planning_error;
    fraction = group.computeCartesianPath(
        waypoints, eef_step, goal->jump_threshold > 0.0 ? goal->jump_threshold : default_jump_threshold_,
        trajectory, true, &planning_error);
    publishFeedback(handle, "cartesian_plan", fraction);
    if (fraction < min_fraction || planning_error.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      result->error_code = planning_error;
      if (fraction < min_fraction || result->error_code.val == 0)
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      const char* mode = goal->orientation_mode == CartesianApproach::Goal::ROTATE_BEFORE_TRANSLATION
                             ? "rotate_before_translation"
                         : goal->orientation_mode == CartesianApproach::Goal::TRANSLATE_BEFORE_ROTATION
                             ? "translate_before_rotation"
                             : "interpolate";
      result->message = "Cartesian path incomplete: fraction=" + std::to_string(fraction) +
                        ", required=" + std::to_string(min_fraction) +
                        ", orientation_mode=" + mode +
                        ", planning_error=" + std::to_string(planning_error.val);
      handle->abort(result);
      clear_busy();
      return;
    }
    }

    if (goal->use_minimum_tcp_z) {
      std::string height_failure;
      if (!validateMinimumTcpHeight(
              group, *start_state, trajectory, tip_link,
              goal->minimum_tcp_z, height_failure)) {
        result->message = "TCP minimum-height constraint failed: " + height_failure;
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        handle->abort(result);
        clear_busy();
        return;
      }
    }

    result->path_fraction = fraction;
    if (unconstrained_moveit) {
      // Keep the trajectory and timing generated by standard MoveIt intact.
      result->trajectory = trajectory;
    } else {
      robot_trajectory::RobotTrajectory timed_trajectory(group.getRobotModel(), group.getName());
      auto timing_state = goal->use_start_joint_state ? start_state : group.getCurrentState(2.0);
      if (!timing_state) {
        result->message = "No current RobotState for Cartesian trajectory timing";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE;
        handle->abort(result);
        clear_busy();
        return;
      }
      timed_trajectory.setRobotTrajectoryMsg(*timing_state, trajectory);
      trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
      if (!time_parameterization.computeTimeStamps(timed_trajectory, velocity, acceleration)) {
        result->message = "Failed to time-parameterize Cartesian trajectory";
        result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
        handle->abort(result);
        clear_busy();
        return;
      }
      timed_trajectory.getRobotTrajectoryMsg(result->trajectory);
    }

    if (!goal->execute) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      if (goal->path_mode == CartesianApproach::Goal::UNCONSTRAINED_MOVEIT)
        result->message = "Standard unconstrained MoveIt plan-only request completed";
      else if (goal->path_mode == CartesianApproach::Goal::BOUNDED_CORRIDOR)
        result->message = "Validated bounded-corridor plan-only request completed";
      else
        result->message = "Cartesian plan-only request completed";
      handle->succeed(result);
      clear_busy();
      return;
    }

    if (handle->is_canceling()) {
      group.stop();
      result->message = "Cartesian execution canceled before start";
      handle->canceled(result);
      clear_busy();
      return;
    }
    publishFeedback(
        handle,
        goal->path_mode == CartesianApproach::Goal::UNCONSTRAINED_MOVEIT
            ? "moveit_execute"
        : goal->path_mode == CartesianApproach::Goal::BOUNDED_CORRIDOR
            ? "corridor_execute"
            : "cartesian_execute",
        fraction);
    const auto execution_code = group.execute(result->trajectory);
    if (execution_code != moveit::core::MoveItErrorCode::SUCCESS) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
      result->message = "Cartesian trajectory execution failed with MoveIt code " +
                        std::to_string(execution_code.val);
      const auto failed_state = group.getCurrentState(1.0);
      const auto& joint_trajectory = result->trajectory.joint_trajectory;
      if (failed_state && !joint_trajectory.points.empty() &&
          joint_trajectory.joint_names.size() ==
              joint_trajectory.points.back().positions.size()) {
        std::string joint_errors;
        for (std::size_t index = 0; index < joint_trajectory.joint_names.size(); ++index) {
          if (!joint_errors.empty())
            joint_errors += ", ";
          const auto& joint = joint_trajectory.joint_names[index];
          const double target = joint_trajectory.points.back().positions[index];
          const double actual = failed_state->getVariablePosition(joint);
          joint_errors += joint + "=" + std::to_string(actual - target);
        }
        result->message += "; endpoint joint errors [" + joint_errors + "]";
      }
      handle->abort(result);
      clear_busy();
      return;
    }

    // Standard MoveIt mode owns execution success. The endpoint check below is
    // specific to the custom Cartesian/corridor modes and must not turn a
    // successful MoveIt execution into a second, stricter failure condition.
    if (goal->path_mode == CartesianApproach::Goal::UNCONSTRAINED_MOVEIT) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      result->message = "Standard unconstrained MoveIt execution completed";
      handle->succeed(result);
      clear_busy();
      return;
    }

    const auto executed_state = group.getCurrentState(2.0);
    if (!executed_state) {
      result->message = "No current RobotState for Cartesian execution endpoint validation";
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::ROBOT_STATE_STALE;
      handle->abort(result);
      clear_busy();
      return;
    }
    executed_state->update();
    const auto executed_pose =
        poseFromTransform(executed_state->getGlobalLinkTransform(tip_link));
    const double executed_position_error =
        (position(executed_pose) - position(goal->end_pose.pose)).norm();
    const double executed_orientation_error =
        orientationDistance(executed_pose, goal->end_pose.pose) * 180.0 / M_PI;
    if (executed_position_error > execution_position_tolerance_m_ ||
        executed_orientation_error > execution_orientation_tolerance_deg_) {
      result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
      result->message =
          "Cartesian execution endpoint tracking failed: position error=" +
          std::to_string(executed_position_error) + " m (limit " +
          std::to_string(execution_position_tolerance_m_) +
          " m), orientation error=" + std::to_string(executed_orientation_error) +
          " deg (limit " + std::to_string(execution_orientation_tolerance_deg_) + " deg)";
      handle->abort(result);
      clear_busy();
      return;
    }
    RCLCPP_INFO(
        get_logger(),
        "Cartesian endpoint validated: position_error=%.4f m, orientation_error=%.2f deg",
        executed_position_error, executed_orientation_error);
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    if (goal->path_mode == CartesianApproach::Goal::UNCONSTRAINED_MOVEIT)
      result->message = "Standard unconstrained MoveIt execution completed";
    else if (goal->path_mode == CartesianApproach::Goal::BOUNDED_CORRIDOR)
      result->message = "Bounded-corridor execution completed";
    else
      result->message = "Cartesian segment execution completed";
    handle->succeed(result);
    clear_busy();
  }

  std::mutex goal_mutex_;
  bool busy_ = false;
  std::string action_name_;
  std::string odom_frame_;
  std::string left_tip_link_;
  std::string right_tip_link_;
  double default_eef_step_m_;
  double default_jump_threshold_;
  double default_min_fraction_;
  double default_start_position_tolerance_m_;
  double default_start_orientation_tolerance_deg_;
  double default_start_lateral_tolerance_m_;
  double default_orientation_step_deg_;
  double default_corridor_radius_m_;
  double default_corridor_endpoint_margin_m_;
  double default_corridor_goal_position_tolerance_m_;
  double default_corridor_goal_orientation_tolerance_deg_;
  double default_corridor_max_path_scale_;
  double default_corridor_max_extra_path_m_;
  double default_corridor_max_backtracking_m_;
  double default_corridor_planning_timeout_s_;
  int default_corridor_planning_attempts_;
  double corridor_validation_joint_step_rad_;
  double corridor_validation_tolerance_m_;
  double execution_position_tolerance_m_;
  double execution_orientation_tolerance_deg_;
  double default_velocity_scale_;
  double default_acceleration_scale_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> left_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> right_group_;
  rclcpp_action::Server<CartesianApproach>::SharedPtr action_server_;
};

}  // namespace openflex_grasp_moveit

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<openflex_grasp_moveit::CartesianApproachExecutor>();
  try {
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "Cartesian approach executor failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
