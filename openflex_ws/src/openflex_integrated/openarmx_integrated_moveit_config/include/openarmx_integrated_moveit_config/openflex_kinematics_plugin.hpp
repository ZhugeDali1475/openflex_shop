#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <pluginlib/class_loader.hpp>

namespace openarmx_integrated_moveit_config
{
class OpenflexKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  bool getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                     const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, std::vector<double>& solution, const IKCallbackFn& solution_callback,
                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                        double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                        const IKCallbackFn& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                     std::vector<geometry_msgs::msg::Pose>& poses) const override;

  bool initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                  const std::string& group_name, const std::string& base_frame,
                  const std::vector<std::string>& tip_frames, double search_discretization) override;

  const std::vector<std::string>& getJointNames() const override;
  const std::vector<std::string>& getLinkNames() const override;

private:
  struct Parameters
  {
    // Stage 0 is a conservative routing aid. A failure always falls through
    // to the later stages, so the radius never rejects an otherwise valid IK.
    double workspace_fixed_lift_radius_m = 1.10;
    double stage_a_timeout_sec = 0.03;
    double stage_b_timeout_sec = 0.03;
    double stage_c_timeout_per_seed_sec = 0.02;
    double stage_c_min_remaining_sec = 0.002;
    int stage_c_yaw_count = 8;
    int stage_c_seeds_per_yaw = 4;
    int stage_c_random_seed = 20260727;
    double stage_c_yaw_offset_rad = 0.0;
    double stage_c_yaw_range_rad = 0.7853981633974483;
    double stage_c_front_distance_m = 0.70;
    double stage_c_left_lateral_offset_m = 0.20;
    double stage_c_right_lateral_offset_m = -0.20;
    double stage_c_orientation_tolerance_rad = 0.005;
    double front_min_x_m = 0.05;
    double front_max_x_m = 1.20;
    double front_half_angle_rad = 1.3962634015954636;
    double position_tolerance = 0.002;
    double rotation_tolerance_deg = 2.0;
  };

  enum class InitialRoute
  {
    STAGE_A,
    STAGE_B,
    STAGE_C,
    SEQUENTIAL,
  };

  struct StageCCandidate
  {
    std::vector<double> positions;
    int yaw_index = 0;
    int seed_index = 0;
    double lift_displacement = 0.0;
    double arm_displacement = 0.0;
    double base_displacement = 0.0;
    double yaw_displacement = 0.0;
  };

  bool solve(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& seed, double timeout,
             const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback, bool search,
             std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code) const;
  bool solveStageA(const geometry_msgs::msg::Pose& target, const std::vector<double>& seed, double timeout,
                   const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback, bool search,
                   std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code) const;
  bool solveStageB(const geometry_msgs::msg::Pose& target, const std::vector<double>& seed,
                   const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback, bool search,
                   double timeout, std::vector<double>& solution,
                   moveit_msgs::msg::MoveItErrorCodes& error_code) const;

  bool initializeLiftOnly(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                          const std::string& base_frame, const std::vector<std::string>& tip_frames);
  bool solveLiftOnly(const geometry_msgs::msg::Pose& target, const std::vector<double>& seed,
                     const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback,
                     std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code) const;

  bool initializeWholeBody(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                           const std::string& group_name, double search_discretization);
  bool solveWholeBody(const geometry_msgs::msg::Pose& target, const std::vector<double>& seed,
                      double timeout, const std::vector<double>& consistency_limits,
                      const IKCallbackFn& solution_callback, bool search, std::vector<double>& solution,
                      moveit_msgs::msg::MoveItErrorCodes& error_code) const;
  InitialRoute routeStageZero(const Eigen::Isometry3d& target, const std::vector<double>& seed) const;
  bool solveWholeBodyStageA(const Eigen::Isometry3d& target, const std::vector<double>& original_seed,
                            const std::vector<double>& stage_seed, double timeout,
                            const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback,
                            bool search, std::vector<double>& solution,
                            moveit_msgs::msg::MoveItErrorCodes& error_code) const;
  bool solveWholeBodyStageB(const Eigen::Isometry3d& target, const std::vector<double>& original_seed,
                            const std::vector<double>& stage_seed, double timeout,
                            const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback,
                            bool search, std::vector<double>& solution,
                            moveit_msgs::msg::MoveItErrorCodes& error_code) const;
  bool solveWholeBodyStageC(const Eigen::Isometry3d& target, const std::vector<double>& original_seed,
                            double timeout, const std::vector<double>& consistency_limits,
                            const IKCallbackFn& solution_callback, bool search, std::vector<double>& solution,
                            moveit_msgs::msg::MoveItErrorCodes& error_code) const;
  std::vector<double> stageCArmSeed(const Eigen::Isometry3d& target, const std::vector<double>& full_seed,
                                    int yaw_index, int seed_index) const;
  Eigen::Vector3d shoulderPosition(const std::vector<double>& full_positions) const;
  Eigen::Isometry3d targetInSolverBase(const Eigen::Isometry3d& target,
                                       const std::vector<double>& candidate,
                                       const std::string& solver_base_frame) const;
  std::vector<double> groupFromSolverOrder(const std::vector<double>& solver_positions) const;
  std::vector<double> solverFromGroupOrder(const std::vector<double>& group_positions) const;
  std::vector<double> composeWholeBodyCandidate(const std::vector<double>& base_candidate,
                                                const std::vector<double>& lift_arm_solution) const;
  double stageTimeout(double requested_timeout, double configured_timeout, double elapsed) const;

  bool acceptCandidate(const geometry_msgs::msg::Pose& target, const std::vector<double>& seed,
                       const std::vector<double>& consistency_limits, const IKCallbackFn& solution_callback,
                       const std::vector<double>& candidate, moveit_msgs::msg::MoveItErrorCodes& error_code) const;
  bool clampSeedToBounds(const std::vector<double>& seed, std::vector<double>& clamped_seed) const;
  bool withinBounds(const std::vector<double>& candidate) const;
  bool withinFrontWorkspace(const std::vector<double>& candidate) const;
  bool withinConsistency(const std::vector<double>& seed, const std::vector<double>& consistency_limits,
                         const std::vector<double>& candidate) const;
  Eigen::Isometry3d poseToEigen(const geometry_msgs::msg::Pose& pose) const;
  geometry_msgs::msg::Pose eigenToPose(const Eigen::Isometry3d& transform) const;
  Eigen::Isometry3d baseToArmBase(const std::vector<double>& full_positions) const;
  Eigen::Isometry3d baseToTip(const std::vector<double>& full_positions) const;
  std::vector<double> armSeed(const std::vector<double>& full_seed) const;
  std::vector<double> fullSolverSeed(const std::vector<double>& full_seed) const;
  std::vector<double> fullSolverCandidate(const std::vector<double>& solver_candidate) const;
  std::vector<double> makeFullCandidate(double lift_position, const std::vector<double>& arm_positions) const;
  bool loadParameters(const rclcpp::Node::SharedPtr& node, const std::string& group_name);

  bool initialized_ = false;
  bool lift_only_mode_ = false;
  bool whole_body_mode_ = false;
  moveit::core::RobotModelConstPtr robot_model_ptr_;
  const moveit::core::JointModelGroup* full_group_ = nullptr;
  const moveit::core::JointModelGroup* lift_arm_group_ = nullptr;
  const moveit::core::JointModelGroup* arm_group_ = nullptr;
  const moveit::core::JointModel* base_joint_ = nullptr;
  std::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> arm_solver_loader_;
  std::shared_ptr<kinematics::KinematicsBase> arm_solver_;
  std::shared_ptr<kinematics::KinematicsBase> full_solver_;
  std::shared_ptr<kinematics::KinematicsBase> pick_solver_;

  std::string group_name_;
  std::string lift_arm_group_name_;
  std::string arm_group_name_;
  std::string base_frame_;
  std::string lift_arm_base_frame_;
  std::string arm_base_frame_;
  std::string tip_frame_;
  std::string solver_tip_frame_;
  Eigen::Isometry3d tip_to_solver_tip_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d lift_reference_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Vector3d lift_axis_base_ = Eigen::Vector3d::UnitZ();
  double lift_reference_position_ = 0.0;
  std::vector<std::string> full_variable_names_;
  std::vector<std::string> solver_joint_names_;
  std::vector<std::string> link_names_;
  std::vector<std::size_t> arm_full_indices_;
  std::vector<std::size_t> full_solver_indices_;
  std::vector<std::size_t> pick_solver_indices_;
  std::vector<double> lower_bounds_;
  std::vector<double> upper_bounds_;
  std::size_t lift_index_ = 0;
  std::size_t base_x_index_ = 0;
  std::size_t base_y_index_ = 0;
  std::size_t base_theta_index_ = 0;
  Parameters parameters_;
};
}  // namespace openarmx_integrated_moveit_config
