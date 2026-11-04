#include "icp_registration/icp_registration.hpp"
#include "icp_registration/fine_alignment_guard.hpp"
#include "icp_registration/low_support_guard.hpp"
#include "icp_registration/pure_rotation_guard.hpp"
#include "icp_registration/recovery_guard.hpp"
#include "icp_registration/registration_quality.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <iostream>
#include <fstream>
#include <utility>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/registration/transformation_estimation_2D.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/create_timer_ros.h>

namespace icp {

namespace {

constexpr std::size_t kMinAlignmentPoints = 15;
constexpr std::uint8_t kFineAlignmentGuardInactive = 0;
constexpr std::uint8_t kFineAlignmentGuardActive = 1;
constexpr std::uint8_t kFineAlignmentGuardCompleted = 2;

void normalizePoint(pcl::PointXYZI &) {}

bool isFinitePoint(const pcl::PointXYZI &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

PointCloudXYZI::Ptr projectCloudToPlane(const PointCloudXYZI::Ptr &cloud) {
  PointCloudXYZI::Ptr planar(new PointCloudXYZI);
  planar->header = cloud->header;
  planar->reserve(cloud->size());
  for (const auto &point : cloud->points) {
    PointType projected = point;
    projected.z = 0.0f;
    planar->push_back(projected);
  }
  planar->width = planar->size();
  planar->height = 1;
  planar->is_dense = true;
  return planar;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr sanitizeCloud(
    const typename pcl::PointCloud<PointT>::Ptr &cloud,
    std::size_t *removed_count = nullptr) {
  typename pcl::PointCloud<PointT>::Ptr filtered(
      new pcl::PointCloud<PointT>);
  filtered->header = cloud->header;
  filtered->reserve(cloud->size());

  std::size_t removed = 0;
  for (const auto &point : cloud->points) {
    PointT normalized_point = point;
    normalizePoint(normalized_point);
    if (isFinitePoint(normalized_point)) {
      filtered->push_back(normalized_point);
    } else {
      ++removed;
    }
  }

  filtered->width = filtered->size();
  filtered->height = 1;
  filtered->is_dense = true;
  if (removed_count != nullptr) {
    *removed_count = removed;
  }
  return filtered;
}

registration_quality::RegistrationQuality calculateRegistrationQuality(
    const PointCloudXYZI &aligned_source,
    const PointCloudXYZI::ConstPtr &target,
    double max_correspondence_distance) {
  std::vector<double> squared_distances;
  squared_distances.reserve(aligned_source.size());
  if (!target || target->empty()) {
    squared_distances.resize(
        aligned_source.size(), std::numeric_limits<double>::infinity());
    return registration_quality::summarizeSquaredDistances(
        squared_distances, max_correspondence_distance);
  }

  pcl::KdTreeFLANN<PointType> tree;
  tree.setInputCloud(target);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  for (const auto &point : aligned_source.points) {
    if (tree.nearestKSearch(
            point, 1, nearest_index, nearest_squared_distance) == 1) {
      squared_distances.push_back(nearest_squared_distance.front());
    } else {
      squared_distances.push_back(std::numeric_limits<double>::infinity());
    }
  }
  return registration_quality::summarizeSquaredDistances(
      squared_distances, max_correspondence_distance);
}

}  // namespace

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(30), refine_iter_(20),
      sc_loaded_(false), submap_half_range_(5),
      ndt_resolution_(1.0), ndt_step_size_(0.1),
      ndt_max_iterations_(30), ndt_epsilon_(0.01), ndt_score_thresh_(1.0),
      first_scan_(true), auto_realign_interval_sec_(1.0),
      initial_alignment_delay_sec_(0.0),
      min_points_for_alignment_(200),
      alignment_in_progress_(false) {
  is_ready_ = false;
  tf_smoothing_active_ = false;
  initial_localization_done_ = false;
  // A staged candidate keeps the last verified TF live.  At startup there is
  // no verified pose yet, so publish a valid identity transform rather than a
  // zero quaternion while localization remains held.
  map_to_odom_.transform.rotation.w = 1.0;
  map_to_odom_current_.transform.rotation.w = 1.0;
  map_to_odom_target_.transform.rotation.w = 1.0;
  last_auto_realign_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  last_continuous_realign_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  node_start_time_ = this->now();
  cloud_stamp_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  cloud_in_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }
  // Read the pcd file
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  reader.read(pcd_path_, *cloud);
  std::size_t removed_points = 0;
  cloud = sanitizeCloud<pcl::PointXYZI>(cloud, &removed_points);
  if (removed_points > 0) {
    RCLCPP_WARN(this->get_logger(),
                "Dropped %zu non-finite map points from %s",
                removed_points, pcd_path_.c_str());
  }
  if (cloud->empty()) {
    throw std::runtime_error("PCD map became empty after filtering invalid points");
  }
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);
  cloud = sanitizeCloud<pcl::PointXYZI>(cloud, &removed_points);
  if (cloud->empty()) {
    throw std::runtime_error(
        "PCD map became empty after voxel filtering invalid points");
  }

  refine_map_ = cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*refine_map_, *point_rough);
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  rough_map_ = sanitizeCloud<pcl::PointXYZI>(filterd_point_rough);

  rough_iter_ = this->declare_parameter("rough_iter", 30);
  refine_iter_ = this->declare_parameter("refine_iter", 20);

  icp_rough_.setMaximumIterations(rough_iter_);

  icp_refine_.setMaximumIterations(refine_iter_);
  global_icp_max_correspondence_distance_ = std::max(
      0.1, this->declare_parameter("icp_max_correspondence_distance", 1.0));
  continuous_icp_max_correspondence_distance_ = std::max(
      0.1, this->declare_parameter(
          "continuous_icp_max_correspondence_distance", 0.5));
  continuous_min_inlier_ratio_ = std::clamp(
      this->declare_parameter("continuous_min_inlier_ratio", 0.80),
      0.05, 1.0);
  continuous_max_inlier_rmse_ = std::max(
      0.01, this->declare_parameter("continuous_max_inlier_rmse", 0.15));
  continuous_max_median_distance_ = std::max(
      0.01, this->declare_parameter(
          "continuous_max_median_distance", 0.10));
  icp_rough_.setMaxCorrespondenceDistance(
      global_icp_max_correspondence_distance_);
  icp_refine_.setMaxCorrespondenceDistance(
      global_icp_max_correspondence_distance_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // Parameters
  map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  base_frame_id_ =
      this->declare_parameter("base_frame_id", std::string("base_link"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("laser"));
  thresh_ = this->declare_parameter("thresh", 0.15);
  continuous_thresh_ =
      this->declare_parameter("continuous_thresh", thresh_);
  xy_offset_ = this->declare_parameter("xy_offset", 0.2);
  yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
  if (yaw_resolution_ <= 0.0) {
    throw std::runtime_error(
        "yaw_resolution must be > 0 degrees, got " +
        std::to_string(yaw_resolution_ * 180.0 / M_PI));
  }
  auto_realign_interval_sec_ =
      this->declare_parameter("auto_realign_interval_sec", 1.0);
  initial_alignment_delay_sec_ =
      this->declare_parameter("initial_alignment_delay_sec", 0.0);
  continuous_realign_interval_sec_ =
      this->declare_parameter("continuous_realign_interval_sec", 0.0);
  localization_stable_samples_ =
      this->declare_parameter("localization_stable_samples", 4);
  localization_stability_window_size_ = this->declare_parameter(
      "localization_stability_window_size", 5);
  max_continuous_failures_ =
      this->declare_parameter("max_continuous_failures", 3);
  localization_stable_pos_thresh_ =
      this->declare_parameter("localization_stable_pos_thresh", 0.06);
  localization_stable_yaw_thresh_ =
      this->declare_parameter("localization_stable_yaw_thresh", 0.04);
  localization_stable_score_thresh_ =
      this->declare_parameter("localization_stable_score_thresh", continuous_thresh_);
  max_base_translation_speed_ =
      this->declare_parameter("max_base_translation_speed", 0.85);
  translation_jump_margin_ =
      this->declare_parameter("translation_jump_margin", 0.15);
  recovery_wheel_position_gate_ = this->declare_parameter(
      "recovery_wheel_position_gate", 0.20);
  recovery_wheel_yaw_gate_ = this->declare_parameter(
      "recovery_wheel_yaw_gate", 0.30);
  trusted_recovery_anchor_timeout_sec_ = this->declare_parameter(
      "trusted_recovery_anchor_timeout_sec", 30.0);
  local_recovery_timeout_sec_ = this->declare_parameter(
      "local_recovery_timeout_sec", 12.0);
  localization_stable_samples_ = std::max(1, localization_stable_samples_);
  localization_stability_window_size_ = std::max(
      localization_stable_samples_, localization_stability_window_size_);
  recovery_stability_window_.configure(
      static_cast<std::size_t>(localization_stability_window_size_),
      static_cast<std::size_t>(localization_stable_samples_));
  max_continuous_failures_ = std::max(1, max_continuous_failures_);
  localization_stable_pos_thresh_ =
      std::max(0.01, localization_stable_pos_thresh_);
  localization_stable_yaw_thresh_ =
      std::max(0.01, localization_stable_yaw_thresh_);
  localization_stable_score_thresh_ = std::max(
      0.001, std::min(localization_stable_score_thresh_, continuous_thresh_));
  max_base_translation_speed_ = std::max(0.1, max_base_translation_speed_);
  translation_jump_margin_ = std::max(0.02, translation_jump_margin_);
  recovery_wheel_position_gate_ = std::max(0.05, recovery_wheel_position_gate_);
  recovery_wheel_yaw_gate_ = std::max(0.10, recovery_wheel_yaw_gate_);
  trusted_recovery_anchor_timeout_sec_ = std::max(
      1.0, trusted_recovery_anchor_timeout_sec_);
  local_recovery_timeout_sec_ = std::max(1.0, local_recovery_timeout_sec_);
  local_recovery_start_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  min_points_for_alignment_ = static_cast<std::size_t>(
      this->declare_parameter("min_points_for_alignment", 200));
  map_crop_radius_ = this->declare_parameter("map_crop_radius", 50.0);
  tf_smooth_duration_ = this->declare_parameter("tf_smooth_duration", 0.5);
  constrain_to_2d_ = this->declare_parameter("constrain_to_2d", true);
  preserve_3d_structure_for_planar_icp_ = this->declare_parameter(
      "preserve_3d_structure_for_planar_icp", true);
  tf_smooth_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  initial_pose_.orientation.w = 1.0;
  try {
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
             initial_pose_vec.at(5));
    initial_pose_.orientation = tf2::toMsg(q);
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }
  initial_pose_ = projectPoseToPlane(initial_pose_);
  publish_identity_until_localized_ = this->declare_parameter(
      "publish_identity_until_localized", true);

  if (constrain_to_2d_) {
    RCLCPP_INFO(this->get_logger(),
                "ICP map->odom is constrained to planar x/y/yaw motion; "
                "3D correspondence structure=%s",
                preserve_3d_structure_for_planar_icp_ ? "preserved" : "flattened");
  }

  // NDT parameters
  ndt_resolution_ = this->declare_parameter("ndt_resolution", 1.0);
  ndt_step_size_ = this->declare_parameter("ndt_step_size", 0.1);
  ndt_max_iterations_ = this->declare_parameter("ndt_max_iterations", 30);
  ndt_epsilon_ = this->declare_parameter("ndt_epsilon", 0.01);
  ndt_score_thresh_ = this->declare_parameter("ndt_score_thresh", 1.0);
  sc_icp_thresh_ = this->declare_parameter("sc_icp_thresh", 0.25);
  submap_half_range_ = this->declare_parameter("submap_half_range", 5);
  this->declare_parameter("sc_ambiguity_distance_margin", 0.03);
  this->declare_parameter("sc_candidate_separation", 1.0);
  this->declare_parameter("grid_half_cells", 2);
  this->declare_parameter("grid_search_timeout_sec", 30.0);
  this->declare_parameter("allow_sc_override_initial_pose", false);
  allow_ambiguous_sc_grid_fallback_ = this->declare_parameter(
      "allow_ambiguous_sc_grid_fallback", true);
  global_grid_score_thresh_ = this->declare_parameter(
      "global_grid_score_thresh", 0.04);
  global_grid_score_thresh_ = std::max(0.001, global_grid_score_thresh_);
  global_grid_score_margin_ = this->declare_parameter(
      "global_grid_score_margin", 0.015);
  global_grid_score_margin_ = std::max(0.0, global_grid_score_margin_);
  global_grid_candidate_separation_ = this->declare_parameter(
      "global_grid_candidate_separation", 1.0);
  global_grid_candidate_separation_ = std::max(
      0.1, global_grid_candidate_separation_);
  reject_ambiguous_global_candidates_ = this->declare_parameter(
      "reject_ambiguous_global_candidates", true);
  this->declare_parameter("manual_pose_xy_search_radius", 0.8);
  this->declare_parameter("manual_pose_yaw_search_range", 0.35);

  // SC body frame: the frame used during mapping (mid360_link by default)
  sc_body_frame_ = this->declare_parameter("sc_body_frame", std::string("mid360_link"));

  // Initialize NDT
  ndt_.setResolution(ndt_resolution_);
  ndt_.setStepSize(ndt_step_size_);
  ndt_.setMaximumIterations(ndt_max_iterations_);
  ndt_.setTransformationEpsilon(ndt_epsilon_);

  RCLCPP_INFO(this->get_logger(),
              "ICP config: thresh=%.2f, continuous_thresh=%.2f, xy_offset=%.1f, rough_iter=%d, refine_iter=%d, initial_delay=%.1fs",
              thresh_, continuous_thresh_, xy_offset_, rough_iter_, refine_iter_,
              initial_alignment_delay_sec_);
  RCLCPP_INFO(
      this->get_logger(),
      "Continuous ICP robust quality: inlier_distance=%.2fm "
      "min_ratio=%.2f max_inlier_rmse=%.2fm max_p50=%.2fm "
      "(legacy all-point threshold %.2f is diagnostic only)",
      continuous_icp_max_correspondence_distance_,
      continuous_min_inlier_ratio_, continuous_max_inlier_rmse_,
      continuous_max_median_distance_, continuous_thresh_);

  // Derive map_dir from pcd_path and try to load SC database
  map_dir_ = pcd_path_.parent_path();
  double sc_dist_thresh = this->declare_parameter("sc_dist_thresh", 0.2);
  sc_manager_.setSCdistThres(sc_dist_thresh);
  if (loadScanContextDatabase() && loadKeyframePoses()) {
    RCLCPP_INFO(this->get_logger(),
                "ScanContext database loaded: %zu keyframes, SC relocalization enabled (sc_dist_thresh=%.2f)",
                keyframe_poses_.size(), sc_dist_thresh);
  } else {
    RCLCPP_INFO(this->get_logger(),
                "ScanContext database not available, using grid-search ICP only");
  }

  // This deployment deliberately has no absolute GPS prior.  Global
  // candidates are accepted from a unique Scan Context or an operator prior;
  // when SC is ambiguous, the configured guarded grid fallback may still
  // propose a candidate, but its strict score and multi-frame stability gates
  // remain mandatory.
  RCLCPP_INFO(this->get_logger(),
              "GPS prior disabled; global localization uses SC uniqueness, "
              "/initialpose, or guarded grid fallback");

  // LIO yaw jump detection parameters
  lio_jump_divergence_thresh_ = this->declare_parameter("lio_jump_divergence_thresh", 0.5);
  lio_jump_icp_score_factor_ = this->declare_parameter("lio_jump_icp_score_factor", 0.1);
  lio_jump_detection_timeout_ = this->declare_parameter("lio_jump_detection_timeout", 10.0);
  icp_wheel_abs_yaw_thresh_ = this->declare_parameter("icp_wheel_abs_yaw_thresh", 1.5);
  icp_wheel_recovery_yaw_gate_thresh_ =
      this->declare_parameter("icp_wheel_recovery_yaw_gate_thresh", 0.52);
  wheel_odom_pose_timeout_ = this->declare_parameter("wheel_odom_pose_timeout", 0.5);
  pure_rotation_angular_speed_thresh_ = this->declare_parameter(
      "pure_rotation_angular_speed_thresh", 0.10);
  pure_rotation_linear_speed_thresh_ = this->declare_parameter(
      "pure_rotation_linear_speed_thresh", 0.05);
  pure_rotation_translation_freeze_settle_time_ = this->declare_parameter(
      "pure_rotation_translation_freeze_settle_time", 1.25);
  pure_rotation_anchor_release_distance_ = this->declare_parameter(
      "pure_rotation_anchor_release_distance", 0.05);
  pure_rotation_weak_quality_margin_factor_ = this->declare_parameter(
      "pure_rotation_weak_quality_margin_factor", 1.05);
  pure_rotation_weak_quality_max_duration_ = this->declare_parameter(
      "pure_rotation_weak_quality_max_duration", 10.0);
  post_rotation_weak_quality_max_duration_ = this->declare_parameter(
      "post_rotation_weak_quality_max_duration", 8.0);
  post_rotation_weak_quality_max_travel_ = this->declare_parameter(
      "post_rotation_weak_quality_max_travel", 0.20);
  post_rotation_weak_quality_max_linear_speed_ = this->declare_parameter(
      "post_rotation_weak_quality_max_linear_speed", 0.20);
  fine_alignment_guard_max_duration_ = this->declare_parameter(
      "fine_alignment_guard_max_duration", 30.0);
  fine_alignment_guard_max_travel_ = this->declare_parameter(
      "fine_alignment_guard_max_travel", 0.45);
  fine_alignment_guard_max_linear_speed_ = this->declare_parameter(
      "fine_alignment_guard_max_linear_speed", 0.10);
  fine_alignment_guard_max_divergence_ = this->declare_parameter(
      "fine_alignment_guard_max_lio_wheel_divergence", 0.25);
  fine_alignment_guard_min_inlier_count_ = static_cast<std::size_t>(
      this->declare_parameter("fine_alignment_guard_min_inlier_count", 800));
  fine_alignment_guard_min_inlier_ratio_ = this->declare_parameter(
      "fine_alignment_guard_min_inlier_ratio", 0.65);
  fine_alignment_guard_max_inlier_rmse_ = this->declare_parameter(
      "fine_alignment_guard_max_inlier_rmse", 0.165);
  fine_alignment_guard_max_median_distance_ = this->declare_parameter(
      "fine_alignment_guard_max_median_distance", 0.120);
  continuous_translation_correction_limit_ = this->declare_parameter(
      "continuous_translation_correction_limit", 0.12);
  continuous_low_support_min_inlier_ratio_ = this->declare_parameter(
      "continuous_low_support_min_inlier_ratio", 0.60);
  continuous_low_support_min_inlier_count_ = this->declare_parameter(
      "continuous_low_support_min_inlier_count", 800);
  continuous_low_support_residual_tighten_ = this->declare_parameter(
      "continuous_low_support_residual_tighten", 1.0);
  continuous_low_support_max_duration_ = this->declare_parameter(
      "continuous_low_support_max_duration", 25.0);
  pure_rotation_angular_speed_thresh_ =
      std::max(0.01, pure_rotation_angular_speed_thresh_);
  pure_rotation_linear_speed_thresh_ =
      std::max(0.0, pure_rotation_linear_speed_thresh_);
  pure_rotation_translation_freeze_settle_time_ =
      std::max(0.0, pure_rotation_translation_freeze_settle_time_);
  pure_rotation_anchor_release_distance_ =
      std::max(0.01, pure_rotation_anchor_release_distance_);
  pure_rotation_weak_quality_margin_factor_ = std::clamp(
      pure_rotation_weak_quality_margin_factor_, 1.0, 1.25);
  pure_rotation_weak_quality_max_duration_ = std::clamp(
      pure_rotation_weak_quality_max_duration_, 0.0, 30.0);
  post_rotation_weak_quality_max_duration_ = std::clamp(
      post_rotation_weak_quality_max_duration_, 0.0, 30.0);
  post_rotation_weak_quality_max_travel_ = std::clamp(
      post_rotation_weak_quality_max_travel_,
      pure_rotation_anchor_release_distance_, 0.50);
  post_rotation_weak_quality_max_linear_speed_ = std::clamp(
      post_rotation_weak_quality_max_linear_speed_,
      pure_rotation_linear_speed_thresh_, 0.50);
  fine_alignment_guard_max_duration_ = std::clamp(
      fine_alignment_guard_max_duration_, 1.0, 60.0);
  fine_alignment_guard_max_travel_ = std::clamp(
      fine_alignment_guard_max_travel_, 0.05, 0.75);
  fine_alignment_guard_max_linear_speed_ = std::clamp(
      fine_alignment_guard_max_linear_speed_, 0.01, 0.30);
  fine_alignment_guard_max_divergence_ = std::clamp(
      fine_alignment_guard_max_divergence_, 0.05, 0.50);
  fine_alignment_guard_min_inlier_count_ = std::max<std::size_t>(
      200, fine_alignment_guard_min_inlier_count_);
  fine_alignment_guard_min_inlier_ratio_ = std::clamp(
      fine_alignment_guard_min_inlier_ratio_, 0.30,
      continuous_min_inlier_ratio_);
  fine_alignment_guard_max_inlier_rmse_ = std::clamp(
      fine_alignment_guard_max_inlier_rmse_,
      continuous_max_inlier_rmse_, 0.25);
  fine_alignment_guard_max_median_distance_ = std::clamp(
      fine_alignment_guard_max_median_distance_,
      continuous_max_median_distance_, 0.20);
  continuous_translation_correction_limit_ =
      std::max(0.02, continuous_translation_correction_limit_);
  continuous_low_support_min_inlier_ratio_ = std::clamp(
      continuous_low_support_min_inlier_ratio_, 0.30,
      continuous_min_inlier_ratio_);
  continuous_low_support_min_inlier_count_ =
      std::max(200, continuous_low_support_min_inlier_count_);
  continuous_low_support_residual_tighten_ = std::clamp(
      continuous_low_support_residual_tighten_, 0.5, 1.0);
  continuous_low_support_max_duration_ = std::clamp(
      continuous_low_support_max_duration_, 0.0, 60.0);
  lio_jump_detect_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  pure_rotation_weak_quality_hold_start_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  continuous_low_support_start_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  fine_alignment_guard_start_time_ =
      rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

  std::string localization_hold_topic =
      this->declare_parameter("localization_hold_topic", std::string("/localization_hold"));
  localization_hold_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      localization_hold_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  std::string localization_state_topic = this->declare_parameter(
      "localization_state_topic", std::string("/localization_ok"));
  auto localization_state_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  localization_state_qos.reliable();
  localization_state_qos.transient_local();
  localization_state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      localization_state_topic, localization_state_qos);
  publishLocalizationState(false, "startup", true);

  // Wheel odometry subscription for LIO yaw jump detection
  std::string wheel_odom_topic = this->declare_parameter("wheel_odom_topic", std::string("/odom"));
  auto wheel_odom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  wheel_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic, wheel_odom_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_odom_x_ = msg->pose.pose.position.x;
        wheel_odom_y_ = msg->pose.pose.position.y;
        wheel_odom_yaw_ = yaw;
        wheel_odom_linear_speed_ = std::hypot(
            msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        wheel_odom_angular_speed_ = msg->twist.twist.angular.z;
        wheel_odom_stamp_ = msg->header.stamp;
        wheel_odom_valid_ = true;
        if (std::abs(wheel_odom_angular_speed_) >=
            pure_rotation_angular_speed_thresh_) {
            last_wheel_rotation_stamp_ = msg->header.stamp;
        }
      });
  const std::string lio_odom_topic = this->declare_parameter(
      "lio_odom_topic", std::string("/fastlio2/lio_odom"));
  lio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      lio_odom_topic, wheel_odom_qos,
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        lio_odom_yaw_ = yaw;
        lio_odom_stamp_ = msg->header.stamp;
        lio_odom_valid_ = true;
      });
  const std::string fine_alignment_guard_topic = this->declare_parameter(
      "fine_alignment_guard_topic",
      std::string("/shelf_fine_align/localization_guard_state"));
  auto fine_alignment_guard_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  fine_alignment_guard_qos.reliable().transient_local();
  fine_alignment_guard_state_sub_ =
      create_subscription<std_msgs::msg::UInt8>(
          fine_alignment_guard_topic, fine_alignment_guard_qos,
          std::bind(
              &IcpNode::fineAlignmentGuardStateCallback, this,
              std::placeholders::_1));
  RCLCPP_INFO(
      this->get_logger(),
      "Fine-alignment localization guard: topic=%s duration=%.1fs "
      "travel=%.2fm p50<=%.3fm",
      fine_alignment_guard_topic.c_str(),
      fine_alignment_guard_max_duration_, fine_alignment_guard_max_travel_,
      fine_alignment_guard_max_median_distance_);
  RCLCPP_INFO(this->get_logger(),
              "LIO yaw jump detection: raw_lio=%s wheel_odom=%s, divergence_thresh=%.2frad, score_factor=%.2f",
              lio_odom_topic.c_str(), wheel_odom_topic.c_str(),
              lio_jump_divergence_thresh_, lio_jump_icp_score_factor_);

  // Set up the pointcloud subscriber
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::SensorDataQoS();
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));

  // Timer-based continuous realignment: decouples ICP frequency from
  // pointcloud arrival rate.  Fires at the configured interval and uses
  // the latest cached cloud.
  if (continuous_realign_interval_sec_ > 0.0) {
    continuous_realign_timer_ = create_wall_timer(
        std::chrono::duration<double>(continuous_realign_interval_sec_),
        std::bind(&IcpNode::continuousRealignTimerCallback, this));
    RCLCPP_INFO(this->get_logger(),
                "Continuous realign timer created: interval=%.1fs",
                continuous_realign_interval_sec_);
  }

  // The project GUI offers TRANSIENT_LOCAL so a startup pose survives a late
  // subscriber, while RViz's 2D Pose Estimate offers VOLATILE.  A transient
  // reader cannot receive the RViz publisher, so keep one reader for each
  // durability.  The shared callback deduplicates messages delivered to both.
  auto initial_pose_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  initial_pose_qos.reliable().transient_local();
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", initial_pose_qos,
          std::bind(
              &IcpNode::initialPoseRequestCallback, this,
              std::placeholders::_1));
  auto initial_pose_volatile_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  initial_pose_volatile_qos.reliable().durability_volatile();
  initial_pose_volatile_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", initial_pose_volatile_qos,
          std::bind(
              &IcpNode::initialPoseRequestCallback, this,
              std::placeholders::_1));

  // A global SC/grid attempt can take seconds.  Do not make an operator wait
  // for that worker to finish, and never drop a strong /initialpose request.
  // The callback below only records the newest request and asks the running
  // automatic search to stop; this short timer starts the manual worker once
  // the previous thread has actually joined.
  manual_pose_request_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&IcpNode::processPendingInitialPoseRequest, this));

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok() && !stop_tf_thread_.load()) {
      {
        std::lock_guard lock(mutex_);
        if (is_ready_ || publish_identity_until_localized_) {
          geometry_msgs::msg::TransformStamped tf_to_publish;

          if (tf_smoothing_active_ && tf_smooth_duration_ > 0.0) {
            double elapsed = (now() - tf_smooth_start_time_).seconds();
            double alpha = std::clamp(elapsed / tf_smooth_duration_, 0.0, 1.0);

            // LERP translation
            auto &cur_t = map_to_odom_current_.transform.translation;
            auto &tgt_t = map_to_odom_target_.transform.translation;
            tf_to_publish.transform.translation.x = cur_t.x + alpha * (tgt_t.x - cur_t.x);
            tf_to_publish.transform.translation.y = cur_t.y + alpha * (tgt_t.y - cur_t.y);
            tf_to_publish.transform.translation.z = cur_t.z + alpha * (tgt_t.z - cur_t.z);

            // SLERP rotation
            tf2::Quaternion q_cur, q_tgt;
            tf2::fromMsg(map_to_odom_current_.transform.rotation, q_cur);
            tf2::fromMsg(map_to_odom_target_.transform.rotation, q_tgt);
            tf2::Quaternion q_interp = q_cur.slerp(q_tgt, alpha);
            tf_to_publish.transform.rotation = tf2::toMsg(q_interp);

            if (alpha >= 1.0) {
              tf_smoothing_active_ = false;
              map_to_odom_.transform = map_to_odom_target_.transform;
              RCLCPP_DEBUG(this->get_logger(), "TF smoothing complete");
            }
          } else {
            tf_to_publish.transform = map_to_odom_.transform;
          }

          tf_to_publish.header.stamp = now();
          tf_to_publish.header.frame_id = map_frame_id_;
          tf_to_publish.child_frame_id = odom_frame_id_;
          tf_broadcaster_->sendTransform(tf_to_publish);
        }
      }
      rate.sleep();
    }
  });

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {
  stop_tf_thread_.store(true);
  if (tf_publisher_thread_ && tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();
  }
  {
    std::lock_guard<std::mutex> lock(align_thread_mutex_);
    if (align_thread_.joinable()) {
      align_thread_.join();
    }
  }
}

void IcpNode::initialPoseRequestCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (!msg) {
    return;
  }
  const std::int64_t message_stamp_ns =
      static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(msg->header.stamp.nanosec);
  const rclcpp::Time receive_time = now();
  bool alignment_was_busy = false;
  {
    std::lock_guard<std::mutex> lock(initial_pose_request_mutex_);
    const auto &pose = msg->pose.pose;
    const auto &last_pose = last_initial_pose_request_;
    const bool same_pose = last_initial_pose_request_valid_ &&
        std::abs(pose.position.x - last_pose.position.x) < 1.0e-9 &&
        std::abs(pose.position.y - last_pose.position.y) < 1.0e-9 &&
        std::abs(pose.position.z - last_pose.position.z) < 1.0e-9 &&
        std::abs(pose.orientation.x - last_pose.orientation.x) < 1.0e-9 &&
        std::abs(pose.orientation.y - last_pose.orientation.y) < 1.0e-9 &&
        std::abs(pose.orientation.z - last_pose.orientation.z) < 1.0e-9 &&
        std::abs(pose.orientation.w - last_pose.orientation.w) < 1.0e-9;
    const bool same_nonzero_stamp = message_stamp_ns != 0 &&
        message_stamp_ns == last_initial_pose_request_stamp_ns_;
    const bool recently_received = last_initial_pose_request_valid_ &&
        std::abs(
            (receive_time - last_initial_pose_request_receive_time_).seconds()) <
        0.5;
    if (same_pose && (same_nonzero_stamp || recently_received)) {
      RCLCPP_DEBUG(
          this->get_logger(),
          "Ignoring duplicate /initialpose delivered by dual QoS readers");
      return;
    }
    last_initial_pose_request_valid_ = true;
    last_initial_pose_request_stamp_ns_ = message_stamp_ns;
    last_initial_pose_request_receive_time_ = receive_time;
    last_initial_pose_request_ = pose;
    // Keep the newest operator request.  It is intentionally queued even if
    // an automatic worker is already running; dropping it leaves the robot
    // held forever in repeated-shelf scenes.
    pending_initial_pose_request_ = msg;
    alignment_was_busy = alignment_in_progress_.load();
    if (alignment_was_busy) {
      manual_alignment_cancel_requested_.store(true);
    }
  }

  if (alignment_was_busy) {
    RCLCPP_WARN(
        this->get_logger(),
        "Queued /initialpose and requested cancellation of the active "
        "automatic alignment");
  }

  processPendingInitialPoseRequest();
}

void IcpNode::processPendingInitialPoseRequest() {
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(initial_pose_request_mutex_);
    if (!pending_initial_pose_request_) {
      return;
    }

    bool expected = false;
    if (!alignment_in_progress_.compare_exchange_strong(expected, true)) {
      // The automatic worker will observe this flag in its candidate loop and
      // stop before it can stage a stale global pose.  Leave the newest pose
      // queued for the next timer tick.
      manual_alignment_cancel_requested_.store(true);
      return;
    }

    msg = pending_initial_pose_request_;
    pending_initial_pose_request_.reset();
    // A new manual worker owns the cancellation state.  This store is made
    // while holding the queue mutex so a concurrent callback cannot be lost
    // between dequeuing and starting the thread.
    manual_alignment_cancel_requested_.store(false);
  }

  RCLCPP_INFO(this->get_logger(),
              "Starting queued manual /initialpose alignment");
  manual_initial_pose_pending_.store(true);
  has_sc_fallback_pose_ = false;
  // A manual pose is a fresh absolute prior.  Never compare it with
  // wheel/ICP baselines calibrated from the previous map hypothesis.
  last_accepted_base_valid_ = false;
  last_accepted_wheel_valid_ = false;
  pure_rotation_anchor_valid_ = false;
  pure_rotation_anchor_lio_wheel_divergence_valid_ = false;
  pure_rotation_weak_quality_hold_active_ = false;
  local_recovery_active_ = false;
  global_recovery_active_ = false;
  localization_loss_time_valid_ = false;
  icp_wheel_yaw_offset_valid_ = false;
  prev_divergence_valid_ = false;
  lio_yaw_jump_detected_ = false;
  consecutive_reject_count_ = 0;
  resetLocalizationStability("manual_initial_pose");
  std::lock_guard<std::mutex> lock(align_thread_mutex_);
  if (align_thread_.joinable()) {
    align_thread_.join();
  }
  align_thread_ = std::thread([this, msg]() {
    try {
      initialPoseCallback(msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(),
                   "initialPoseCallback exception: %s", e.what());
    }
    alignment_in_progress_.store(false);
  });
}

Eigen::Matrix4d IcpNode::projectTransformToPlane(
    const Eigen::Matrix4d &transform) const {
  if (!constrain_to_2d_) {
    return transform;
  }

  Eigen::Matrix4d planar = Eigen::Matrix4d::Identity();
  planar(0, 3) = transform(0, 3);
  planar(1, 3) = transform(1, 3);

  const Eigen::Matrix3d rotation = transform.block<3, 3>(0, 0);
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  planar.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return planar;
}

geometry_msgs::msg::Pose IcpNode::projectPoseToPlane(
    const geometry_msgs::msg::Pose &pose) const {
  if (!constrain_to_2d_) {
    return pose;
  }

  geometry_msgs::msg::Pose planar_pose = pose;
  planar_pose.position.z = 0.0;

  const Eigen::Quaterniond q(
      pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z);
  const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const Eigen::Quaterniond planar_q(
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
  planar_pose.orientation.w = planar_q.w();
  planar_pose.orientation.x = planar_q.x();
  planar_pose.orientation.y = planar_q.y();
  planar_pose.orientation.z = planar_q.z();
  return planar_pose;
}

void IcpNode::updateMapToOdom(const Eigen::Matrix4d &new_transform, bool force_snap) {
  std::lock_guard lock(mutex_);

  const Eigen::Matrix4d constrained_transform =
      projectTransformToPlane(new_transform);

  // Build the new transform message
  Eigen::Quaterniond q_new(constrained_transform.block<3, 3>(0, 0));

  if (force_snap || !initial_localization_done_ || tf_smooth_duration_ <= 0.0) {
    // Snap immediately: first localization, SC reloc, or smoothing disabled
    map_to_odom_.transform.translation.x = constrained_transform(0, 3);
    map_to_odom_.transform.translation.y = constrained_transform(1, 3);
    map_to_odom_.transform.translation.z = constrained_transform(2, 3);
    map_to_odom_.transform.rotation.w = q_new.w();
    map_to_odom_.transform.rotation.x = q_new.x();
    map_to_odom_.transform.rotation.y = q_new.y();
    map_to_odom_.transform.rotation.z = q_new.z();
    initial_localization_done_ = true;
    tf_smoothing_active_ = false;
  } else {
    // Start smooth transition
    if (tf_smoothing_active_) {
      // Already smoothing: compute current interpolated position as new start
      double elapsed = (now() - tf_smooth_start_time_).seconds();
      double alpha = std::clamp(elapsed / tf_smooth_duration_, 0.0, 1.0);

      auto &cur_t = map_to_odom_current_.transform.translation;
      auto &tgt_t = map_to_odom_target_.transform.translation;
      map_to_odom_current_.transform.translation.x = cur_t.x + alpha * (tgt_t.x - cur_t.x);
      map_to_odom_current_.transform.translation.y = cur_t.y + alpha * (tgt_t.y - cur_t.y);
      map_to_odom_current_.transform.translation.z = cur_t.z + alpha * (tgt_t.z - cur_t.z);

      tf2::Quaternion q_c, q_t;
      tf2::fromMsg(map_to_odom_current_.transform.rotation, q_c);
      tf2::fromMsg(map_to_odom_target_.transform.rotation, q_t);
      tf2::Quaternion q_interp = q_c.slerp(q_t, alpha);
      map_to_odom_current_.transform.rotation = tf2::toMsg(q_interp);
    } else {
      // Start fresh: current position is what we've been publishing
      map_to_odom_current_.transform = map_to_odom_.transform;
    }

    map_to_odom_target_.transform.translation.x = constrained_transform(0, 3);
    map_to_odom_target_.transform.translation.y = constrained_transform(1, 3);
    map_to_odom_target_.transform.translation.z = constrained_transform(2, 3);
    map_to_odom_target_.transform.rotation.w = q_new.w();
    map_to_odom_target_.transform.rotation.x = q_new.x();
    map_to_odom_target_.transform.rotation.y = q_new.y();
    map_to_odom_target_.transform.rotation.z = q_new.z();
    tf_smooth_start_time_ = now();
    tf_smoothing_active_ = true;

    RCLCPP_DEBUG(this->get_logger(),
        "TF smoothing: (%.3f,%.3f) -> (%.3f,%.3f) over %.2fs",
        map_to_odom_current_.transform.translation.x,
        map_to_odom_current_.transform.translation.y,
        map_to_odom_target_.transform.translation.x,
        map_to_odom_target_.transform.translation.y,
        tf_smooth_duration_);
  }

  is_ready_ = true;
}

PointCloudXYZI::Ptr IcpNode::cropMapByPosition(
    const PointCloudXYZI::Ptr &full_map,
    const Eigen::Vector3d &position, double radius) const {
  if (radius <= 0.0 || !full_map || full_map->empty()) {
    return full_map;
  }

  PointCloudXYZI::Ptr cropped(new PointCloudXYZI);
  pcl::CropBox<PointType> crop_filter;
  crop_filter.setInputCloud(full_map);

  Eigen::Vector4f min_pt(
      static_cast<float>(position.x() - radius),
      static_cast<float>(position.y() - radius),
      -std::numeric_limits<float>::max(), 1.0f);
  Eigen::Vector4f max_pt(
      static_cast<float>(position.x() + radius),
      static_cast<float>(position.y() + radius),
      std::numeric_limits<float>::max(), 1.0f);

  crop_filter.setMin(min_pt);
  crop_filter.setMax(max_pt);
  crop_filter.filter(*cropped);

  cropped->header = full_map->header;
  return cropped;
}

bool IcpNode::mapPoseFromTransform(const Eigen::Matrix4d &map_to_odom,
                                   const std::string &target_frame,
                                   const rclcpp::Time &stamp,
                                   Eigen::Matrix4d &map_to_target) {
  try {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          odom_frame_id_, target_frame, stamp,
          rclcpp::Duration::from_seconds(0.5));
    } catch (tf2::ExtrapolationException &) {
      transform = tf_buffer_->lookupTransform(
          odom_frame_id_, target_frame, tf2::TimePointZero);
    }

    Eigen::Matrix4d odom_to_target = Eigen::Matrix4d::Identity();
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q(transform.transform.rotation.w,
                         transform.transform.rotation.x,
                         transform.transform.rotation.y,
                         transform.transform.rotation.z);
    odom_to_target.block<3, 3>(0, 0) = q.toRotationMatrix();
    odom_to_target.block<3, 1>(0, 3) = t;
    map_to_target = projectTransformToPlane(map_to_odom * odom_to_target);
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),
                "Failed to compute map->%s seed pose: %s",
                target_frame.c_str(), ex.what());
    return false;
  }
}

void IcpNode::seedLastAcceptedBasePose(const Eigen::Matrix4d &map_to_odom,
                                       const rclcpp::Time &stamp,
                                       const std::string &source) {
  Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
  if (!mapPoseFromTransform(map_to_odom, base_frame_id_, stamp, map_to_base)) {
    last_accepted_base_valid_ = false;
    return;
  }

  last_accepted_base_x_ = map_to_base(0, 3);
  last_accepted_base_y_ = map_to_base(1, 3);
  last_accepted_base_yaw_ = std::atan2(map_to_base(1, 0), map_to_base(0, 0));
  last_accepted_base_stamp_ = now();
  last_accepted_base_valid_ = true;
  {
    std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
    last_accepted_wheel_x_ = wheel_odom_x_;
    last_accepted_wheel_y_ = wheel_odom_y_;
    last_accepted_wheel_yaw_ = wheel_odom_yaw_;
    last_accepted_wheel_valid_ = wheel_odom_valid_ &&
        ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
  }
  consecutive_reject_count_ = 0;
  icp_wheel_yaw_offset_valid_ = false;

  RCLCPP_INFO(this->get_logger(),
              "%s seeded map->%s pose: x=%.2f y=%.2f yaw=%.1f deg",
              source.c_str(), base_frame_id_.c_str(),
              last_accepted_base_x_, last_accepted_base_y_,
              last_accepted_base_yaw_ * 180.0 / M_PI);
}

bool IcpNode::loadScanContextDatabase() {
  std::filesystem::path sc_dir = map_dir_ / "sc_data";
  if (!std::filesystem::exists(sc_dir)) {
    RCLCPP_INFO(this->get_logger(), "No sc_data directory found at %s",
                sc_dir.string().c_str());
    sc_loaded_ = false;
    return false;
  }

  // Read metadata
  std::filesystem::path meta_path = sc_dir / "metadata.txt";
  if (!std::filesystem::exists(meta_path)) {
    sc_loaded_ = false;
    return false;
  }

  size_t N;
  int rows, cols;
  {
    std::ifstream f(meta_path);
    f >> N >> rows >> cols;
    f.close();
  }

  if (N == 0) {
    sc_loaded_ = false;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loading SC database: N=%zu, rows=%d, cols=%d",
              N, rows, cols);

  // Load polarcontexts
  {
    std::ifstream f(sc_dir / "polarcontexts.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontexts_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontexts_[i].resize(rows, cols);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontexts_[i].data()),
             rows * cols * sizeof(double));
    }
    f.close();
  }

  // Load invkeys
  {
    std::ifstream f(sc_dir / "invkeys.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontext_invkeys_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontext_invkeys_[i].resize(rows, 1);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontext_invkeys_[i].data()),
             rows * sizeof(double));
    }
    f.close();
  }

  // Load vkeys
  {
    std::ifstream f(sc_dir / "vkeys.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontext_vkeys_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontext_vkeys_[i].resize(1, cols);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontext_vkeys_[i].data()),
             cols * sizeof(double));
    }
    f.close();
  }

  // Load invkeys_mat
  {
    std::ifstream f(sc_dir / "invkeys_mat.bin", std::ios::binary);
    if (!f.is_open()) { sc_loaded_ = false; return false; }
    sc_manager_.polarcontext_invkeys_mat_.resize(N);
    for (size_t i = 0; i < N; i++) {
      sc_manager_.polarcontext_invkeys_mat_[i].resize(rows);
      f.read(reinterpret_cast<char *>(sc_manager_.polarcontext_invkeys_mat_[i].data()),
             rows * sizeof(float));
    }
    f.close();
  }

  // Rebuild KD-tree
  sc_manager_.rebuildTree();
  sc_loaded_ = true;
  return true;
}

bool IcpNode::loadKeyframePoses() {
  std::filesystem::path poses_path = map_dir_ / "poses.txt";
  if (!std::filesystem::exists(poses_path)) {
    RCLCPP_WARN(this->get_logger(), "No poses.txt found at %s",
                poses_path.string().c_str());
    return false;
  }

  std::ifstream f(poses_path);
  std::string line;
  keyframe_poses_.clear();
  int line_num = 0;
  while (std::getline(f, line)) {
    ++line_num;
    if (line.empty()) continue;
    std::istringstream iss(line);
    KeyframePose kp;
    if (!(iss >> kp.filename >> kp.x >> kp.y >> kp.z >> kp.qw >> kp.qx >> kp.qy >> kp.qz)) {
      RCLCPP_WARN(this->get_logger(),
                  "Skipping malformed line %d in %s: \"%s\"",
                  line_num, poses_path.string().c_str(), line.c_str());
      continue;
    }
    double norm = std::sqrt(kp.qw * kp.qw + kp.qx * kp.qx + kp.qy * kp.qy + kp.qz * kp.qz);
    if (norm < 1e-9) {
      RCLCPP_WARN(this->get_logger(),
                  "Skipping line %d with zero quaternion in %s",
                  line_num, poses_path.string().c_str());
      continue;
    }
    keyframe_poses_.push_back(kp);
  }
  f.close();

  RCLCPP_INFO(this->get_logger(), "Loaded %zu keyframe poses from %s",
              keyframe_poses_.size(), poses_path.string().c_str());
  return !keyframe_poses_.empty();
}

PointCloudXYZI::Ptr IcpNode::buildSubmap(int center_idx, int half_range) {
  PointCloudXYZI::Ptr submap(new PointCloudXYZI);

  std::filesystem::path patches_dir = map_dir_ / "patches";
  if (!std::filesystem::exists(patches_dir)) {
    RCLCPP_WARN(this->get_logger(), "No patches directory found");
    return submap;
  }

  int min_idx = std::max(0, center_idx - half_range);
  int max_idx = std::min(static_cast<int>(keyframe_poses_.size()) - 1, center_idx + half_range);

  pcl::PCDReader reader;
  for (int i = min_idx; i <= max_idx; i++) {
    std::filesystem::path patch_path = patches_dir / keyframe_poses_[i].filename;
    if (!std::filesystem::exists(patch_path))
      continue;

    PointCloudXYZI::Ptr patch(new PointCloudXYZI);
    reader.read(patch_path.string(), *patch);

    // Transform patch to global coordinates
    Eigen::Quaterniond q(keyframe_poses_[i].qw, keyframe_poses_[i].qx,
                         keyframe_poses_[i].qy, keyframe_poses_[i].qz);
    Eigen::Vector3d t(keyframe_poses_[i].x, keyframe_poses_[i].y, keyframe_poses_[i].z);

    PointCloudXYZI::Ptr transformed(new PointCloudXYZI);
    pcl::transformPointCloud(*patch, *transformed, t.cast<float>(),
                             Eigen::Quaternionf(q.cast<float>()));
    *submap += *transformed;
  }

  // Voxel filter the submap
  if (!submap->empty()) {
    pcl::VoxelGrid<PointType> voxel;
    double leaf = 0.1;  // use refine resolution
    voxel.setLeafSize(leaf, leaf, leaf);
    voxel.setInputCloud(submap);
    voxel.filter(*submap);
  }

  return submap;
}

std::vector<int> IcpNode::filterKeyframesByGps(double gps_x, double gps_y, double radius) const {
  std::vector<int> candidates;
  double r2 = radius * radius;
  for (int i = 0; i < static_cast<int>(keyframe_poses_.size()); i++) {
    double dx = keyframe_poses_[i].x - gps_x;
    double dy = keyframe_poses_[i].y - gps_y;
    if (dx * dx + dy * dy <= r2) {
      candidates.push_back(i);
    }
  }
  return candidates;
}

bool IcpNode::scRelocalize(PointCloudXYZI::Ptr cloud, Eigen::Matrix4d &result_transform) {
  sc_match_ambiguous_ = false;
  if (!sc_loaded_ || keyframe_poses_.empty()) {
    return false;
  }

  // Evaluate all descriptors so acceptance is based on the best and the
  // second-best spatially distinct places.  A nearest-neighbour winner alone
  // is unsafe in repeated corridors: adjacent keyframes from one place can
  // fill the retrieval set while a nearly identical remote place is hidden.
  struct ScCandidate {
    int index = -1;
    double distance = std::numeric_limits<double>::infinity();
    int alignment = 0;
  };

  Eigen::MatrixXd current_descriptor = sc_manager_.makeScancontext(*cloud);
  std::vector<ScCandidate> sc_candidates;
  const std::size_t descriptor_count = std::min(
      sc_manager_.polarcontexts_.size(), keyframe_poses_.size());
  sc_candidates.reserve(descriptor_count);
  for (std::size_t i = 0; i < descriptor_count; ++i) {
    Eigen::MatrixXd candidate_descriptor = sc_manager_.polarcontexts_[i];
    const auto distance_result = sc_manager_.distanceBtnScanContext(
        current_descriptor, candidate_descriptor);
    sc_candidates.push_back(ScCandidate{
        static_cast<int>(i), distance_result.first, distance_result.second});
  }
  std::sort(sc_candidates.begin(), sc_candidates.end(),
            [](const ScCandidate &lhs, const ScCandidate &rhs) {
              return lhs.distance < rhs.distance;
            });

  if (sc_candidates.empty() ||
      sc_candidates.front().distance >= sc_manager_.SC_DIST_THRES) {
    sc_manager_.last_match_dist = sc_candidates.empty()
        ? std::numeric_limits<double>::infinity()
        : sc_candidates.front().distance;
    RCLCPP_INFO(this->get_logger(),
                "SC found no candidate below threshold (best=%.4f, thresh=%.4f)",
                sc_manager_.last_match_dist, sc_manager_.SC_DIST_THRES);
    return false;
  }

  const ScCandidate &best_sc = sc_candidates.front();
  const double place_separation = std::max(
      0.1, get_parameter("sc_candidate_separation").as_double());
  const double ambiguity_margin = std::max(
      0.0, get_parameter("sc_ambiguity_distance_margin").as_double());
  double second_place_distance = std::numeric_limits<double>::infinity();
  int second_place_index = -1;
  const KeyframePose &best_pose = keyframe_poses_[best_sc.index];
  for (std::size_t i = 1; i < sc_candidates.size(); ++i) {
    const ScCandidate &candidate = sc_candidates[i];
    const KeyframePose &candidate_pose = keyframe_poses_[candidate.index];
    if (std::hypot(candidate_pose.x - best_pose.x,
                   candidate_pose.y - best_pose.y) >= place_separation) {
      second_place_distance = candidate.distance;
      second_place_index = candidate.index;
      break;
    }
  }

  if (!std::isfinite(second_place_distance)) {
    sc_match_ambiguous_ = true;
    RCLCPP_WARN(
        this->get_logger(),
        "SC cannot establish global uniqueness: best keyframe=%d dist=%.4f, "
        "but no spatially independent second candidate exists; require /initialpose",
        best_sc.index, best_sc.distance);
    return false;
  }

  if (std::isfinite(second_place_distance) &&
      second_place_distance - best_sc.distance < ambiguity_margin) {
    sc_match_ambiguous_ = true;
    RCLCPP_WARN(
        this->get_logger(),
        "SC localization ambiguous: best keyframe=%d dist=%.4f, "
        "distinct keyframe=%d dist=%.4f, margin=%.4f < %.4f; keeping motion held",
        best_sc.index, best_sc.distance, second_place_index,
        second_place_distance, second_place_distance - best_sc.distance,
        ambiguity_margin);
    return false;
  }

  const int match_idx = best_sc.index;
  const float yaw_diff = static_cast<float>(
      best_sc.alignment * sc_manager_.PC_UNIT_SECTORANGLE * M_PI / 180.0);
  sc_manager_.last_match_dist = best_sc.distance;

  RCLCPP_INFO(
      this->get_logger(),
      "SC unique match keyframe=%d yaw_diff=%.1fdeg dist=%.4f "
      "second_distinct=%.4f threshold=%.4f",
      match_idx, yaw_diff * 180.0 / M_PI, sc_manager_.last_match_dist,
      second_place_distance, sc_manager_.SC_DIST_THRES);

  // Build submap around matched keyframe
  PointCloudXYZI::Ptr submap = buildSubmap(match_idx, submap_half_range_);
  if (submap->empty()) {
    RCLCPP_WARN(this->get_logger(), "Built empty submap for keyframe %d", match_idx);
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Built submap with %zu points around keyframe %d",
              submap->size(), match_idx);

  // Construct initial guess from matched keyframe pose + SC yaw difference.
  // distanceBtnScanContext(current, keyframe) returns the column shift that
  // rotates the keyframe descriptor into the current descriptor, so:
  //   current_yaw = keyframe_yaw - yaw_diff
  const KeyframePose &kp = keyframe_poses_[match_idx];
  Eigen::Quaterniond q_kf(kp.qw, kp.qx, kp.qy, kp.qz);
  Eigen::Vector3d t_kf(kp.x, kp.y, kp.z);

  // Apply SC yaw correction
  Eigen::AngleAxisd yaw_correction(-static_cast<double>(yaw_diff), Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d rot_init = yaw_correction.toRotationMatrix() * q_kf.toRotationMatrix();

  Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
  init_guess.block<3, 3>(0, 0) = rot_init;
  init_guess.block<3, 1>(0, 3) = t_kf;

  // Diagnostic: print source cloud bounding box and init guess
  {
    Eigen::Vector4f src_min, src_max, tgt_min, tgt_max;
    pcl::getMinMax3D(*cloud, src_min, src_max);
    pcl::getMinMax3D(*submap, tgt_min, tgt_max);
    RCLCPP_INFO(this->get_logger(),
                "SC diag: source(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f], "
                "submap(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f]",
                cloud->size(),
                src_min[0], src_min[1], src_min[2], src_max[0], src_max[1], src_max[2],
                submap->size(),
                tgt_min[0], tgt_min[1], tgt_min[2], tgt_max[0], tgt_max[1], tgt_max[2]);
    RCLCPP_INFO(this->get_logger(),
                "SC diag: init_guess pos=(%.2f,%.2f,%.2f), kf_pos=(%.2f,%.2f,%.2f)",
                init_guess(0,3), init_guess(1,3), init_guess(2,3), kp.x, kp.y, kp.z);
  }

  // Downsample source cloud
  PointCloudXYZI::Ptr source_rough(new PointCloudXYZI);
  PointCloudXYZI::Ptr source_refine(new PointCloudXYZI);
  voxel_rough_filter_.setInputCloud(cloud);
  voxel_rough_filter_.filter(*source_rough);
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*source_refine);

  source_rough = sanitizeCloud<pcl::PointXYZI>(source_rough);
  source_refine = sanitizeCloud<pcl::PointXYZI>(source_refine);

  // Clip source cloud height to match submap z-range to avoid score inflation
  {
    Eigen::Vector4f tgt_min, tgt_max;
    pcl::getMinMax3D(*submap, tgt_min, tgt_max);
    float z_margin = 0.5f;
    float z_lo = tgt_min[2] - z_margin;
    float z_hi = tgt_max[2] + z_margin;
    auto height_filter = [z_lo, z_hi](const PointCloudXYZI::Ptr &c) {
      PointCloudXYZI::Ptr filtered(new PointCloudXYZI);
      filtered->reserve(c->size());
      for (const auto &p : c->points) {
        if (p.z >= z_lo && p.z <= z_hi) filtered->push_back(p);
      }
      return filtered;
    };
    source_rough = height_filter(source_rough);
    source_refine = height_filter(source_refine);
    RCLCPP_INFO(this->get_logger(),
                "SC height clip [%.1f, %.1f]: rough=%zu, refine=%zu pts",
                z_lo, z_hi, source_rough->size(), source_refine->size());
  }

  if (source_rough->size() < kMinAlignmentPoints ||
      source_refine->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(), "SC relocalize: insufficient points after voxel filter");
    return false;
  }

  if (constrain_to_2d_) {
    if (!preserve_3d_structure_for_planar_icp_) {
      source_rough = projectCloudToPlane(source_rough);
      source_refine = projectCloudToPlane(source_refine);
      submap = projectCloudToPlane(submap);
    }
    init_guess = projectTransformToPlane(init_guess);
  }

  // ICP fine alignment helper (reused for both NDT→ICP and direct ICP paths)
  // Use relaxed thresholds for SC relocalization (initial alignment is rough)
  const double sc_icp_thresh = std::max(0.05, sc_icp_thresh_);
  pcl::IterativeClosestPoint<PointType, PointType> icp_fine;
  icp_fine.setMaximumIterations(50);
  icp_fine.setMaxCorrespondenceDistance(3.0);
  icp_fine.setTransformationEpsilon(1e-6);
  icp_fine.setEuclideanFitnessEpsilon(1e-6);
  icp_fine.setInputSource(source_refine);
  icp_fine.setInputTarget(submap);
  if (constrain_to_2d_) {
    icp_fine.setTransformationEstimation(
        std::make_shared<pcl::registration::TransformationEstimation2D<
            PointType, PointType, float>>());
  }

  // Stage 1: NDT coarse alignment against submap
  PointCloudXYZI::Ptr ndt_aligned(new PointCloudXYZI);
  ndt_.setInputSource(source_rough);
  ndt_.setInputTarget(submap);
  ndt_.align(*ndt_aligned, init_guess.cast<float>());

  bool ndt_ok = ndt_.hasConverged() && ndt_.getFitnessScore() <= ndt_score_thresh_;
  if (ndt_ok) {
    RCLCPP_INFO(this->get_logger(), "NDT converged with score=%.4f", ndt_.getFitnessScore());

    // Stage 2a: ICP fine alignment using NDT result
    Eigen::Matrix4f ndt_result = ndt_.getFinalTransformation();
    if (constrain_to_2d_) {
      ndt_result = projectTransformToPlane(
          ndt_result.cast<double>()).cast<float>();
    }
    PointCloudXYZI::Ptr icp_aligned(new PointCloudXYZI);
    icp_fine.align(*icp_aligned, ndt_result);

    if (icp_fine.hasConverged() && icp_fine.getFitnessScore() <= sc_icp_thresh) {
      result_transform = icp_fine.getFinalTransformation().cast<double>();
      RCLCPP_INFO(this->get_logger(),
                  "SC+NDT+ICP relocalization success! icp_score=%.4f",
                  icp_fine.getFitnessScore());
      return true;
    }
    RCLCPP_INFO(this->get_logger(),
                "SC+NDT+ICP failed at ICP stage: converged=%d, score=%.3f (thresh=%.3f)",
                icp_fine.hasConverged(), icp_fine.getFitnessScore(), sc_icp_thresh);
  } else {
    RCLCPP_INFO(this->get_logger(), "SC+NDT failed: converged=%d, score=%.3f",
                ndt_.hasConverged(), ndt_.getFitnessScore());
  }

  // Stage 2b: NDT failed or NDT+ICP failed — try ICP directly with SC initial guess
  // SC provides a reasonable position + yaw, often good enough for ICP without NDT
  RCLCPP_INFO(this->get_logger(), "Trying SC+ICP directly (bypassing NDT)...");
  PointCloudXYZI::Ptr icp_direct_aligned(new PointCloudXYZI);
  icp_fine.align(*icp_direct_aligned, init_guess.cast<float>());

  if (icp_fine.hasConverged() && icp_fine.getFitnessScore() <= sc_icp_thresh) {
    result_transform = icp_fine.getFinalTransformation().cast<double>();
    RCLCPP_INFO(this->get_logger(),
                "SC+ICP direct relocalization success! icp_score=%.4f",
                icp_fine.getFitnessScore());
    return true;
  }

  RCLCPP_INFO(this->get_logger(),
              "SC+ICP direct also failed: converged=%d, score=%.3f (thresh=%.3f)",
              icp_fine.hasConverged(), icp_fine.getFitnessScore(), sc_icp_thresh);

  // Store SC keyframe pose so grid-search fallback uses it instead of (0,0,0)
  sc_fallback_pose_.position.x = kp.x;
  sc_fallback_pose_.position.y = kp.y;
  sc_fallback_pose_.position.z = kp.z;
  Eigen::Quaterniond q_init(rot_init);
  sc_fallback_pose_.orientation.w = q_init.w();
  sc_fallback_pose_.orientation.x = q_init.x();
  sc_fallback_pose_.orientation.y = q_init.y();
  sc_fallback_pose_.orientation.z = q_init.z();
  has_sc_fallback_pose_ = true;

  return false;
}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr incoming_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *incoming_cloud);

  // Transform to laser_frame_id_ if the incoming cloud is in a different frame
  std::string source_frame = msg->header.frame_id;
  if (!source_frame.empty() && source_frame != laser_frame_id_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
          laser_frame_id_, source_frame,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(0.1));
      Eigen::Vector3f t(
          tf_stamped.transform.translation.x,
          tf_stamped.transform.translation.y,
          tf_stamped.transform.translation.z);
      Eigen::Quaternionf q(
          tf_stamped.transform.rotation.w,
          tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y,
          tf_stamped.transform.rotation.z);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(
          new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*incoming_cloud, *transformed, t, q);
      incoming_cloud = transformed;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Cannot transform cloud from '%s' to '%s': %s",
          source_frame.c_str(), laser_frame_id_.c_str(), ex.what());
      return;
    }
  }

  std::size_t removed_points = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr clean =
      sanitizeCloud<pcl::PointXYZI>(incoming_cloud, &removed_points);
  if (removed_points > 0) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropped %zu non-finite scan points before ICP", removed_points);
  }

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *cloud_in_ = *clean;
    cloud_stamp_ = (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
        ? now()
        : rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
  }

  if (clean->empty()) {
    return;
  }

  if (clean->size() < min_points_for_alignment_) {
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for denser scan before ICP: %zu / %zu points",
        clean->size(), min_points_for_alignment_);
    return;
  }

  const rclcpp::Time current_time =
      msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
          ? now()
          : rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());

  // Already localized — continuous realignment is handled by the timer
  {
    std::lock_guard lock(mutex_);
    if (is_ready_) {
      return;
    }
  }

  if (initial_alignment_delay_sec_ > 0.0) {
    const double startup_age = (now() - node_start_time_).seconds();
    if (startup_age < initial_alignment_delay_sec_) {
      RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Waiting %.1fs before initial ICP alignment so FAST-LIVO can stabilize (%.1f / %.1f)",
          initial_alignment_delay_sec_ - startup_age,
          startup_age,
          initial_alignment_delay_sec_);
      return;
    }
  }

  // Initial localization: retry periodically
  bool should_retry =
      first_scan_ ||
      (current_time - last_auto_realign_time_).seconds() >=
          auto_realign_interval_sec_;
  if (!should_retry) {
    return;
  }

  bool expected = false;
  if (!alignment_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  last_auto_realign_time_ = current_time;
  first_scan_ = false;

  // Build the initial guess pose for ICP
  auto pose_msg =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  manual_initial_pose_pending_.store(false);
  pose_msg->header = msg->header;
  pose_msg->pose.pose = initial_pose_;
  RCLCPP_INFO(this->get_logger(),
              "Attempting ICP auto-alignment with %zu points",
              clean->size());

  {
    std::lock_guard<std::mutex> lock(align_thread_mutex_);
    if (align_thread_.joinable()) {
      align_thread_.join();
    }
    align_thread_ = std::thread([this, pose_msg]() {
      try {
        initialPoseCallback(pose_msg, false);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "initialPoseCallback exception: %s", e.what());
      }
      alignment_in_progress_.store(false);
    });
  }
}

IcpResult IcpNode::runIcpRefine(const PointCloudXYZI::Ptr &source,
                               const Eigen::Matrix4d &initial_guess) {
  IcpResult result;

  std::size_t removed_points = 0;
  PointCloudXYZI::Ptr refine_source(new PointCloudXYZI);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);
  refine_source = sanitizeCloud<pcl::PointXYZI>(refine_source, &removed_points);

  if (refine_source->size() < min_points_for_alignment_) {
    RCLCPP_WARN(this->get_logger(),
                "runIcpRefine: insufficient points (%zu)", refine_source->size());
    return result;
  }

  icp_refine_.setInputSource(refine_source);

  Eigen::Vector3d crop_center(initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3));
  PointCloudXYZI::Ptr local_map = cropMapByPosition(refine_map_, crop_center, map_crop_radius_);
  if (local_map->size() < min_points_for_alignment_) {
    local_map = refine_map_;
  }

  icp_refine_.setInputTarget(local_map);
  if (constrain_to_2d_) {
    if (!preserve_3d_structure_for_planar_icp_) {
      refine_source = projectCloudToPlane(refine_source);
      local_map = projectCloudToPlane(local_map);
    }
    icp_refine_.setInputSource(refine_source);
    icp_refine_.setInputTarget(local_map);
    icp_refine_.setTransformationEstimation(
        std::make_shared<pcl::registration::TransformationEstimation2D<
            PointType, PointType, float>>());
  }
  pcl::PointCloud<pcl::PointXYZI> aligned;
  const Eigen::Matrix4d icp_initial_guess = constrain_to_2d_
      ? projectTransformToPlane(initial_guess)
      : initial_guess;
  icp_refine_.setMaxCorrespondenceDistance(
      continuous_icp_max_correspondence_distance_);
  icp_refine_.align(aligned, icp_initial_guess.cast<float>());

  result.score = icp_refine_.getFitnessScore();
  result.converged = icp_refine_.hasConverged() && result.score < continuous_thresh_;
  result.map_to_laser = icp_refine_.getFinalTransformation().cast<double>();
  return result;
}

void IcpNode::publishLocalizationHold(const std::string &reason) {
  if (localization_hold_pub_ == nullptr) {
    return;
  }

  std_msgs::msg::Empty msg;
  localization_hold_pub_->publish(msg);
  RCLCPP_WARN(this->get_logger(),
              "Localization hold requested: %s", reason.c_str());
}

void IcpNode::publishLocalizationState(bool healthy, const std::string &reason,
                                       bool force) {
  const bool previous = localization_healthy_.exchange(healthy);
  if (!force && previous == healthy && last_localization_state_reason_ == reason) {
    return;
  }

  last_localization_state_reason_ = reason;
  if (localization_state_pub_ != nullptr) {
    std_msgs::msg::Bool msg;
    msg.data = healthy;
    localization_state_pub_->publish(msg);
  }

  if (previous != healthy || force) {
    if (healthy) {
      RCLCPP_INFO(this->get_logger(),
                  "Localization HEALTHY: %s", reason.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Localization NOT READY: %s", reason.c_str());
    }
  }
}

void IcpNode::fineAlignmentGuardStateCallback(
    const std_msgs::msg::UInt8::SharedPtr msg) {
  const std::uint8_t state = msg->data;
  if (state == kFineAlignmentGuardInactive) {
    bool was_active = false;
    bool deferred_release = false;
    {
      std::lock_guard<std::mutex> lock(fine_alignment_guard_mutex_);
      was_active = fine_alignment_guard_active_;
      deferred_release = fine_alignment_guard::shouldDeferCompletedRelease(
          fine_alignment_guard_active_, fine_alignment_guard_completed_);
      if (deferred_release) {
        fine_alignment_guard_release_pending_ = true;
      } else {
        fine_alignment_guard_active_ = false;
        fine_alignment_guard_completed_ = false;
        fine_alignment_guard_release_pending_ = false;
      }
    }
    if (deferred_release) {
      RCLCPP_INFO(
          this->get_logger(),
          "Fine-alignment guard release requested; retaining the completed "
          "bounded lease until its original time/travel limits are consumed");
    } else if (was_active) {
      RCLCPP_INFO(
          this->get_logger(),
          "Fine-alignment localization guard cancelled/released");
    }
    return;
  }

  if (state == kFineAlignmentGuardCompleted) {
    const rclcpp::Time current_time = now();
    bool wheel_recent = false;
    bool lio_recent = false;
    double wheel_x = 0.0;
    double wheel_y = 0.0;
    double divergence = 0.0;
    {
      std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
      wheel_recent = wheel_odom_valid_ &&
          ((current_time - wheel_odom_stamp_).seconds() <=
           wheel_odom_pose_timeout_);
      lio_recent = lio_odom_valid_ &&
          ((current_time - lio_odom_stamp_).seconds() <=
           wheel_odom_pose_timeout_);
      wheel_x = wheel_odom_x_;
      wheel_y = wheel_odom_y_;
      divergence = normalizeAngle(lio_odom_yaw_ - wheel_odom_yaw_);
    }

    bool active = false;
    bool renewed = false;
    {
      std::lock_guard<std::mutex> lock(fine_alignment_guard_mutex_);
      active = fine_alignment_guard_active_;
      if (active) {
        fine_alignment_guard_completed_ = true;
        fine_alignment_guard_release_pending_ = false;
        if (localization_healthy_.load() && wheel_recent && lio_recent &&
            !lio_yaw_jump_detected_) {
          fine_alignment_guard_anchor_wheel_x_ = wheel_x;
          fine_alignment_guard_anchor_wheel_y_ = wheel_y;
          fine_alignment_guard_anchor_divergence_ = divergence;
          fine_alignment_guard_start_time_ = current_time;
          renewed = true;
        }
      }
    }
    if (renewed) {
      RCLCPP_INFO(
          this->get_logger(),
          "Fine-alignment localization guard entered completed near-field "
          "lease: timer and wheel/LIO anchor renewed");
    } else if (active) {
      RCLCPP_WARN(
          this->get_logger(),
          "Fine-alignment localization guard completed without lease renewal: "
          "localization=%d wheel_recent=%d lio_recent=%d lio_jump=%d; "
          "existing bounded lease retained",
          localization_healthy_.load() ? 1 : 0, wheel_recent ? 1 : 0,
          lio_recent ? 1 : 0, lio_yaw_jump_detected_ ? 1 : 0);
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Ignoring completed fine-alignment guard without an active lease");
    }
    return;
  }

  if (state != kFineAlignmentGuardActive) {
    RCLCPP_WARN(
        this->get_logger(), "Ignoring unknown fine-alignment guard state: %u",
        static_cast<unsigned int>(state));
    return;
  }

  const rclcpp::Time current_time = now();
  bool wheel_recent = false;
  bool lio_recent = false;
  double wheel_x = 0.0;
  double wheel_y = 0.0;
  double divergence = 0.0;
  {
    std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
    wheel_recent = wheel_odom_valid_ &&
        ((current_time - wheel_odom_stamp_).seconds() <=
         wheel_odom_pose_timeout_);
    lio_recent = lio_odom_valid_ &&
        ((current_time - lio_odom_stamp_).seconds() <=
         wheel_odom_pose_timeout_);
    wheel_x = wheel_odom_x_;
    wheel_y = wheel_odom_y_;
    divergence = normalizeAngle(lio_odom_yaw_ - wheel_odom_yaw_);
  }

  if (!localization_healthy_.load() || !wheel_recent || !lio_recent ||
      lio_yaw_jump_detected_) {
    std::lock_guard<std::mutex> lock(fine_alignment_guard_mutex_);
    fine_alignment_guard_active_ = false;
    fine_alignment_guard_completed_ = false;
    fine_alignment_guard_release_pending_ = false;
    RCLCPP_WARN(
        this->get_logger(),
        "Rejected fine-alignment localization guard: localization=%d "
        "wheel_recent=%d lio_recent=%d lio_jump=%d",
        localization_healthy_.load() ? 1 : 0, wheel_recent ? 1 : 0,
        lio_recent ? 1 : 0, lio_yaw_jump_detected_ ? 1 : 0);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(fine_alignment_guard_mutex_);
    fine_alignment_guard_active_ = true;
    fine_alignment_guard_completed_ = false;
    fine_alignment_guard_release_pending_ = false;
    fine_alignment_guard_anchor_wheel_x_ = wheel_x;
    fine_alignment_guard_anchor_wheel_y_ = wheel_y;
    fine_alignment_guard_anchor_divergence_ = divergence;
    fine_alignment_guard_start_time_ = current_time;
  }
  RCLCPP_INFO(
      this->get_logger(),
      "Fine-alignment localization guard active: trusted wheel/LIO anchor captured");
}

void IcpNode::refreshFineAlignmentCompletedLease() {
  {
    std::lock_guard<std::mutex> lock(fine_alignment_guard_mutex_);
    if (!fine_alignment_guard::shouldRefreshCompletedLease(
            fine_alignment_guard_active_, fine_alignment_guard_completed_,
            fine_alignment_guard_release_pending_)) {
      return;
    }
  }

  const rclcpp::Time current_time = now();
  bool wheel_recent = false;
  bool lio_recent = false;
  double wheel_x = 0.0;
  double wheel_y = 0.0;
  double divergence = 0.0;
  {
    std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
    wheel_recent = wheel_odom_valid_ &&
        ((current_time - wheel_odom_stamp_).seconds() <=
         wheel_odom_pose_timeout_);
    lio_recent = lio_odom_valid_ &&
        ((current_time - lio_odom_stamp_).seconds() <=
         wheel_odom_pose_timeout_);
    wheel_x = wheel_odom_x_;
    wheel_y = wheel_odom_y_;
    divergence = normalizeAngle(lio_odom_yaw_ - wheel_odom_yaw_);
  }
  if (!localization_healthy_.load() || !wheel_recent || !lio_recent ||
      lio_yaw_jump_detected_) {
    return;
  }

  std::lock_guard<std::mutex> lock(fine_alignment_guard_mutex_);
  if (!fine_alignment_guard::shouldRefreshCompletedLease(
          fine_alignment_guard_active_, fine_alignment_guard_completed_,
          fine_alignment_guard_release_pending_)) {
    return;
  }
  fine_alignment_guard_anchor_wheel_x_ = wheel_x;
  fine_alignment_guard_anchor_wheel_y_ = wheel_y;
  fine_alignment_guard_anchor_divergence_ = divergence;
  fine_alignment_guard_start_time_ = current_time;
}

void IcpNode::resetLocalizationStability(const std::string &reason) {
  localization_stable_count_ = 0;
  recovery_stability_window_.reset();
  continuous_failure_count_ = 0;
  recovery_candidate_valid_ = false;
  publishLocalizationState(false, reason);
  publishLocalizationHold(reason);
}

void IcpNode::registerAlignmentFailure(const std::string &reason,
                                       bool immediately_unhealthy,
                                       bool stability_sample_recorded) {
  ++continuous_failure_count_;
  recovery_guard::recordFailureSample(
      recovery_stability_window_, localization_healthy_.load(),
      stability_sample_recorded);
  localization_stable_count_ = static_cast<int>(
      recovery_stability_window_.goodCount());

  if (immediately_unhealthy ||
      continuous_failure_count_ >= max_continuous_failures_) {
    publishLocalizationState(false, reason);
    publishLocalizationHold(reason);
  }

  if (continuous_failure_count_ < max_continuous_failures_) {
    return;
  }

  bool current_wheel_recent = false;
  {
    std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
    current_wheel_recent = wheel_odom_valid_ &&
        ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
  }
  const bool trusted_local_anchor_available =
      last_accepted_base_valid_ && last_accepted_wheel_valid_ &&
      current_wheel_recent;

  // A few weak scans do not erase the known trajectory.  Hold motion, keep
  // the last verified TF, and continue tight ICP from the wheel-propagated
  // trusted pose.  Global SC/grid is deliberately deferred because repeated
  // shelves may be globally ambiguous even though local motion is known.
  if (!global_recovery_active_ && trusted_local_anchor_available) {
    if (!local_recovery_active_) {
      local_recovery_active_ = true;
      local_recovery_start_time_ = now();
      recovery_candidate_valid_ = false;
      recovery_stability_window_.reset();
      if (!localization_loss_time_valid_) {
        localization_loss_time_ = local_recovery_start_time_;
        localization_loss_time_valid_ = true;
      }
      RCLCPP_ERROR(
          this->get_logger(),
          "Localization DEGRADED after repeated failures (%s); motion held, "
          "starting trusted LOCAL_RECOVERY for up to %.1fs",
          reason.c_str(), local_recovery_timeout_sec_);
    }

    const double local_recovery_age =
        (now() - local_recovery_start_time_).seconds();
    if (local_recovery_age >= 0.0 &&
        local_recovery_age < local_recovery_timeout_sec_) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        is_ready_ = true;
      }
      continuous_failure_count_ = 0;
      RCLCPP_WARN(
          this->get_logger(),
          "Trusted LOCAL_RECOVERY continuing: age=%.2fs/%.2fs; "
          "global SC/grid remains disabled",
          local_recovery_age, local_recovery_timeout_sec_);
      return;
    }

    RCLCPP_ERROR(
        this->get_logger(),
        "Trusted LOCAL_RECOVERY timed out after %.2fs; escalating to guarded "
        "GLOBAL_RECOVERY",
        local_recovery_age);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Stop publishing a transform whose quality can no longer be verified.
    // The pointcloud callback will now run SC/NDT global localization again.
    is_ready_ = false;
  }
  local_recovery_active_ = false;
  global_recovery_active_ = true;
  pure_rotation_anchor_valid_ = false;
  pure_rotation_anchor_lio_wheel_divergence_valid_ = false;
  pure_rotation_weak_quality_hold_active_ = false;
  first_scan_ = true;
  recovery_candidate_valid_ = false;
  recovery_stability_window_.reset();
  // Keep the last verified map pose and its wheel-odometry anchor.  Automatic
  // recovery must remain tied to this trusted trajectory; discarding it here
  // lets a repeated shelf minimum redefine its own wheel baseline and pass the
  // subsequent multi-frame gate.  An explicit /initialpose callback is the
  // only path that intentionally invalidates this anchor.
  prev_divergence_valid_ = false;
  icp_wheel_yaw_offset_valid_ = false;
  lio_yaw_jump_detected_ = false;
  consecutive_reject_count_ = 0;
  continuous_failure_count_ = 0;
  if (!localization_loss_time_valid_) {
    localization_loss_time_ = now();
    localization_loss_time_valid_ = true;
  }
  RCLCPP_ERROR(
      this->get_logger(),
      "Localization LOST after repeated failures (%s); restarting global "
      "SC/NDT with trusted_pose=%d trusted_wheel=%d",
      reason.c_str(), last_accepted_base_valid_ ? 1 : 0,
      last_accepted_wheel_valid_ ? 1 : 0);
}

void IcpNode::checkLioYawJump(double lio_yaw) {
  // Expire stale jump flag
  if (lio_yaw_jump_detected_) {
    double elapsed = (now() - lio_jump_detect_time_).seconds();
    if (elapsed > lio_jump_detection_timeout_) {
      RCLCPP_WARN(this->get_logger(),
                  "LIO yaw jump flag expired after %.1fs", elapsed);
      lio_yaw_jump_detected_ = false;
      consecutive_reject_count_ = 0;
      icp_wheel_yaw_offset_valid_ = false;  // recalibrate after recovery
    }
  }

  // Read wheel odom yaw
  double wheel_yaw;
  bool wheel_valid;
  {
    std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
    wheel_yaw = wheel_odom_yaw_;
    wheel_valid = wheel_odom_valid_ &&
        ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
  }

  if (!wheel_valid) {
    return;
  }

  // Compute current divergence = lio_yaw - wheel_yaw (normalized to [-pi, pi])
  double divergence = lio_yaw - wheel_yaw;
  if (divergence > M_PI) divergence -= 2.0 * M_PI;
  if (divergence < -M_PI) divergence += 2.0 * M_PI;

  if (!prev_divergence_valid_) {
    prev_lio_wheel_divergence_ = divergence;
    prev_divergence_valid_ = true;
    RCLCPP_INFO(this->get_logger(),
                "LIO-wheel divergence initialized: %.1f°",
                divergence * 180.0 / M_PI);
    return;
  }

  // How much did the divergence change since last sample?
  double divergence_delta = divergence - prev_lio_wheel_divergence_;
  if (divergence_delta > M_PI) divergence_delta -= 2.0 * M_PI;
  if (divergence_delta < -M_PI) divergence_delta += 2.0 * M_PI;

  RCLCPP_DEBUG(this->get_logger(),
      "LIO-wheel divergence: cur=%.1f° prev=%.1f° delta=%.1f° (thresh=%.0f°)",
      divergence * 180.0 / M_PI,
      prev_lio_wheel_divergence_ * 180.0 / M_PI,
      divergence_delta * 180.0 / M_PI,
      lio_jump_divergence_thresh_ * 180.0 / M_PI);

  // Update previous divergence
  prev_lio_wheel_divergence_ = divergence;

  // Detect: divergence changed more than threshold → LIO jumped but wheels didn't
  if (std::abs(divergence_delta) > lio_jump_divergence_thresh_) {
    lio_yaw_jump_detected_ = true;
    lio_jump_detect_time_ = now();
    RCLCPP_WARN(this->get_logger(),
        "LIO yaw jump DETECTED: divergence_delta=%.1f° (thresh=%.0f°) "
        "— LIO drifted from wheel odom, will allow high-confidence ICP through",
        std::abs(divergence_delta) * 180.0 / M_PI,
        lio_jump_divergence_thresh_ * 180.0 / M_PI);
  }
}

void IcpNode::continuousRealignTimerCallback() {
  // Only fire after initial localization is done
  {
    std::lock_guard lock(mutex_);
    if (!is_ready_) {
      return;
    }
  }

  // Check if alignment is already running
  bool expected = false;
  if (!alignment_in_progress_.compare_exchange_strong(expected, true)) {
    return;
  }

  // Check the recovery deadline on every timer pass.  Previously it was
  // evaluated mostly from failure handling, so alternating accepted/rejected
  // frames could keep LOCAL_RECOVERY alive indefinitely.
  if (local_recovery_active_) {
    const double local_recovery_age =
        (now() - local_recovery_start_time_).seconds();
    if (local_recovery_age >= local_recovery_timeout_sec_) {
      registerAlignmentFailure("local_recovery_timeout", true);
      alignment_in_progress_.store(false);
      return;
    }
  }

  // Grab the latest cached cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_snapshot(
      new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Time scan_stamp;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (cloud_in_->empty()) {
      registerAlignmentFailure("continuous_cloud_empty", true);
      alignment_in_progress_.store(false);
      return;
    }
    *cloud_snapshot = *cloud_in_;
    scan_stamp = cloud_stamp_;
  }

  // Check cloud freshness — reject if too old
  const double cloud_age = (now() - scan_stamp).seconds();
  if (cloud_age > continuous_realign_interval_sec_ * 5.0) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Cached cloud too old (%.1fs), skipping continuous realign",
                          cloud_age);
    registerAlignmentFailure("continuous_cloud_stale", true);
    alignment_in_progress_.store(false);
    return;
  }

  if (cloud_snapshot->size() < min_points_for_alignment_) {
    registerAlignmentFailure("continuous_cloud_too_sparse", true);
    alignment_in_progress_.store(false);
    return;
  }

  // Build initial guess from current map->odom * odom->base
  auto pose_msg =
      std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  pose_msg->header.stamp = scan_stamp;
  pose_msg->header.frame_id = map_frame_id_;

  Eigen::Matrix4d odom_to_base = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d map_to_odom_mat = Eigen::Matrix4d::Identity();

  try {
    auto tf_odom_base = tf_buffer_->lookupTransform(
        odom_frame_id_, base_frame_id_,
        scan_stamp,
        rclcpp::Duration::from_seconds(0.1));
    Eigen::Quaterniond q_ob(
        tf_odom_base.transform.rotation.w,
        tf_odom_base.transform.rotation.x,
        tf_odom_base.transform.rotation.y,
        tf_odom_base.transform.rotation.z);
    odom_to_base.block<3, 3>(0, 0) = q_ob.toRotationMatrix();
    odom_to_base(0, 3) = tf_odom_base.transform.translation.x;
    odom_to_base(1, 3) = tf_odom_base.transform.translation.y;
    odom_to_base(2, 3) = tf_odom_base.transform.translation.z;

    // Check the raw FAST-LIO yaw against wheel odometry.  The odom->base TF
    // above is published by safe_odom_mux and may already have switched to
    // wheel mode, so it cannot identify an LIO jump reliably.
    {
      double raw_lio_yaw = 0.0;
      bool raw_lio_recent = false;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        raw_lio_yaw = lio_odom_yaw_;
        raw_lio_recent = lio_odom_valid_ &&
            ((now() - lio_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      }
      if (raw_lio_recent) {
        checkLioYawJump(raw_lio_yaw);
      } else {
        prev_divergence_valid_ = false;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Raw LIO odometry is stale; skipping LIO-wheel jump decision");
      }
    }

    {
      std::lock_guard lock(mutex_);
      Eigen::Quaterniond q_mo(
          map_to_odom_.transform.rotation.w,
          map_to_odom_.transform.rotation.x,
          map_to_odom_.transform.rotation.y,
          map_to_odom_.transform.rotation.z);
      map_to_odom_mat.block<3, 3>(0, 0) = q_mo.toRotationMatrix();
      map_to_odom_mat(0, 3) = map_to_odom_.transform.translation.x;
      map_to_odom_mat(1, 3) = map_to_odom_.transform.translation.y;
      map_to_odom_mat(2, 3) = map_to_odom_.transform.translation.z;
    }

    Eigen::Matrix4d map_to_base = map_to_odom_mat * odom_to_base;
    if (!localization_healthy_.load() && local_recovery_active_ &&
        !recovery_candidate_valid_) {
      double current_wheel_x = 0.0;
      double current_wheel_y = 0.0;
      double current_wheel_yaw = 0.0;
      double trusted_wheel_x = 0.0;
      double trusted_wheel_y = 0.0;
      double trusted_wheel_yaw = 0.0;
      bool wheel_seed_valid = false;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        current_wheel_x = wheel_odom_x_;
        current_wheel_y = wheel_odom_y_;
        current_wheel_yaw = wheel_odom_yaw_;
        trusted_wheel_x = last_accepted_wheel_x_;
        trusted_wheel_y = last_accepted_wheel_y_;
        trusted_wheel_yaw = last_accepted_wheel_yaw_;
        wheel_seed_valid = wheel_odom_valid_ && last_accepted_wheel_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      }

      if (!wheel_seed_valid || !last_accepted_base_valid_) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Trusted LOCAL_RECOVERY seed unavailable; keeping last-good TF");
        registerAlignmentFailure(
            "local_recovery_wheel_seed_unavailable", true);
        alignment_in_progress_.store(false);
        return;
      }

      const auto predicted = recovery_guard::propagateFromWheel(
          recovery_guard::PlanarPose{
              last_accepted_base_x_, last_accepted_base_y_,
              last_accepted_base_yaw_},
          recovery_guard::PlanarPose{
              trusted_wheel_x, trusted_wheel_y, trusted_wheel_yaw},
          recovery_guard::PlanarPose{
              current_wheel_x, current_wheel_y, current_wheel_yaw});
      map_to_base.setIdentity();
      const double predicted_cos = std::cos(predicted.yaw);
      const double predicted_sin = std::sin(predicted.yaw);
      map_to_base(0, 0) = predicted_cos;
      map_to_base(0, 1) = -predicted_sin;
      map_to_base(1, 0) = predicted_sin;
      map_to_base(1, 1) = predicted_cos;
      map_to_base(0, 3) = predicted.x;
      map_to_base(1, 3) = predicted.y;
      RCLCPP_INFO(
          this->get_logger(),
          "Trusted LOCAL_RECOVERY seed: map_base=(%.3f,%.3f,%.1fdeg)",
          predicted.x, predicted.y, predicted.yaw * 180.0 / M_PI);
    }
    if (!localization_healthy_.load() && recovery_candidate_valid_) {
      // During verification the published TF is intentionally kept at the
      // last verified value.  Seed ICP from the staged candidate instead of
      // that old TF, otherwise every verification pass would optimize around
      // the wrong pose.
      map_to_base.setIdentity();
      const double c = std::cos(recovery_candidate_base_yaw_);
      const double s = std::sin(recovery_candidate_base_yaw_);
      map_to_base(0, 0) = c;
      map_to_base(0, 1) = -s;
      map_to_base(1, 0) = s;
      map_to_base(1, 1) = c;
      map_to_base(0, 3) = recovery_candidate_base_x_;
      map_to_base(1, 3) = recovery_candidate_base_y_;
    }
    Eigen::Quaterniond q_mb(map_to_base.block<3, 3>(0, 0));
    pose_msg->pose.pose.position.x = map_to_base(0, 3);
    pose_msg->pose.pose.position.y = map_to_base(1, 3);
    pose_msg->pose.pose.position.z = map_to_base(2, 3);
    pose_msg->pose.pose.orientation.w = q_mb.w();
    pose_msg->pose.pose.orientation.x = q_mb.x();
    pose_msg->pose.pose.orientation.y = q_mb.y();
    pose_msg->pose.pose.orientation.z = q_mb.z();
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                          "Continuous realign timer: TF lookup failed: %s", ex.what());
    registerAlignmentFailure("continuous_tf_lookup_failed", true);
    alignment_in_progress_.store(false);
    return;
  }

  RCLCPP_INFO(this->get_logger(),
              "Continuous re-alignment with %zu points (cloud age: %.1fs)",
              cloud_snapshot->size(), cloud_age);

  // --- Dual-ICP recovery for LIO yaw jump (Mode B) ---
  // When LIO yaw flip is detected, the normal ICP path uses a flipped
  // initial_guess and may converge to the wrong (mirrored) solution.
  // Instead, run ICP from TWO initial guesses: normal + 180°-flipped,
  // and pick the one with the best score.
  if (lio_yaw_jump_detected_) {
    RCLCPP_WARN(this->get_logger(),
        "Dual-ICP recovery: LIO yaw jump active, running two-guess ICP");

    // Compute base_to_laser (identity when laser_frame == base_frame)
    Eigen::Matrix4d base_to_laser = Eigen::Matrix4d::Identity();
    try {
      if (base_frame_id_ != laser_frame_id_) {
        auto tf_bl = tf_buffer_->lookupTransform(
            base_frame_id_, laser_frame_id_,
            rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(0.5));
        Eigen::Quaterniond q_bl(
            tf_bl.transform.rotation.w, tf_bl.transform.rotation.x,
            tf_bl.transform.rotation.y, tf_bl.transform.rotation.z);
        base_to_laser.block<3, 3>(0, 0) = q_bl.toRotationMatrix();
        base_to_laser(0, 3) = tf_bl.transform.translation.x;
        base_to_laser(1, 3) = tf_bl.transform.translation.y;
        base_to_laser(2, 3) = tf_bl.transform.translation.z;
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
          "Dual-ICP: TF lookup failed: %s, falling back to normal path", ex.what());
      goto normal_path;
    }

    {
      // guess_A: normal initial guess = map→odom × odom→base × base→laser
      Eigen::Matrix4d guess_A = map_to_odom_mat * odom_to_base * base_to_laser;

      // guess_B: same position, yaw rotated 180°
      Eigen::Matrix4d yaw_flip = Eigen::Matrix4d::Identity();
      yaw_flip(0, 0) = -1.0;  // cos(180°)
      yaw_flip(1, 1) = -1.0;  // cos(180°)
      // yaw_flip(0,1)=0, yaw_flip(1,0)=0 (sin(180°)=0)
      Eigen::Matrix4d guess_B = guess_A;
      guess_B.block<3, 3>(0, 0) = yaw_flip.block<3, 3>(0, 0) * guess_A.block<3, 3>(0, 0);

      RCLCPP_INFO(this->get_logger(),
          "Dual-ICP: guess_A yaw=%.1f°, guess_B yaw=%.1f°",
          std::atan2(guess_A(1, 0), guess_A(0, 0)) * 180.0 / M_PI,
          std::atan2(guess_B(1, 0), guess_B(0, 0)) * 180.0 / M_PI);

      IcpResult result_A = runIcpRefine(cloud_snapshot, guess_A);
      IcpResult result_B = runIcpRefine(cloud_snapshot, guess_B);

      RCLCPP_INFO(this->get_logger(),
          "Dual-ICP results: A(conv=%d, score=%.4f) B(conv=%d, score=%.4f)",
          result_A.converged ? 1 : 0, result_A.score,
          result_B.converged ? 1 : 0, result_B.score);

      double wheel_yaw_now = 0.0;
      bool wheel_ready = false;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_yaw_now = wheel_odom_yaw_;
        wheel_ready = wheel_odom_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      }

      if (!wheel_ready || !icp_wheel_yaw_offset_valid_) {
        RCLCPP_WARN(this->get_logger(),
            "Dual-ICP: wheel yaw baseline unavailable, keeping last good pose");
        publishLocalizationHold("dual_icp_no_wheel_baseline");
        registerAlignmentFailure("dual_icp_no_wheel_baseline", true);
        alignment_in_progress_.store(false);
        return;
      }

      const double expected_base_yaw =
          normalizeAngle(wheel_yaw_now + icp_wheel_yaw_offset_);
      const double score_limit = thresh_ * lio_jump_icp_score_factor_;

      const auto candidate_yaw_error = [&](const IcpResult &candidate) {
        if (!candidate.converged || candidate.score > score_limit) {
          return std::numeric_limits<double>::infinity();
        }
        const double candidate_yaw = std::atan2(candidate.map_to_laser(1, 0),
                                                candidate.map_to_laser(0, 0));
        return std::abs(normalizeAngle(candidate_yaw - expected_base_yaw));
      };

      const double yaw_error_A = candidate_yaw_error(result_A);
      const double yaw_error_B = candidate_yaw_error(result_B);
      RCLCPP_INFO(this->get_logger(),
          "Dual-ICP wheel gate: expected_base_yaw=%.1f°, A_err=%.1f°, B_err=%.1f°",
          expected_base_yaw * 180.0 / M_PI,
          std::isfinite(yaw_error_A) ? yaw_error_A * 180.0 / M_PI : -1.0,
          std::isfinite(yaw_error_B) ? yaw_error_B * 180.0 / M_PI : -1.0);

      IcpResult *best = nullptr;
      if (yaw_error_A <= icp_wheel_recovery_yaw_gate_thresh_ &&
          yaw_error_B <= icp_wheel_recovery_yaw_gate_thresh_) {
        best = (result_A.score <= result_B.score) ? &result_A : &result_B;
      } else if (yaw_error_A <= icp_wheel_recovery_yaw_gate_thresh_) {
        best = &result_A;
      } else if (yaw_error_B <= icp_wheel_recovery_yaw_gate_thresh_) {
        best = &result_B;
      }

      if (!best) {
        RCLCPP_WARN(this->get_logger(),
            "Dual-ICP: both candidates violate wheel yaw gate (expected=%.1f°), keeping last good pose",
            expected_base_yaw * 180.0 / M_PI);
        publishLocalizationHold("dual_icp_wheel_gate_reject");
        registerAlignmentFailure("dual_icp_wheel_gate_reject", true);
        alignment_in_progress_.store(false);
        return;
      }

      // Stage the recovery result.  Even the dual-ICP path is not allowed to
      // force-snap map->odom before the normal multi-scan verification gate;
      // wheel yaw only selects between the two hypotheses.
      const Eigen::Matrix4d best_map_to_base =
          best->map_to_laser * base_to_laser.inverse();
      const double new_base_x = best_map_to_base(0, 3);
      const double new_base_y = best_map_to_base(1, 3);
      const double new_base_yaw = std::atan2(best_map_to_base(1, 0),
                                              best_map_to_base(0, 0));

      RCLCPP_WARN(this->get_logger(),
          "Dual-ICP RECOVERY: accepting best result (score=%.4f, "
          "base=(%.2f,%.2f,%.1f°), expected_yaw=%.1f°) — force-snapping map->odom",
          best->score, new_base_x, new_base_y,
          new_base_yaw * 180.0 / M_PI,
          expected_base_yaw * 180.0 / M_PI);

      recovery_candidate_base_x_ = new_base_x;
      recovery_candidate_base_y_ = new_base_y;
      recovery_candidate_base_yaw_ = new_base_yaw;
      recovery_candidate_valid_ = true;
      localization_stable_count_ = 0;
      recovery_stability_window_.reset();
      lio_yaw_jump_detected_ = false;
      consecutive_reject_count_ = 0;
      icp_wheel_yaw_offset_valid_ = false;  // recalibrate after recovery
      publishLocalizationState(false, "dual_icp_recovery_candidate_staged");
      publishLocalizationHold("dual_icp_recovery_candidate_staged");
      RCLCPP_WARN(this->get_logger(),
          "Dual-ICP recovery candidate staged at (%.2f, %.2f, %.1fdeg); "
          "waiting for independent verification",
          new_base_x, new_base_y, new_base_yaw * 180.0 / M_PI);
    }
    alignment_in_progress_.store(false);
    return;
  }
  normal_path:

  {
    std::lock_guard<std::mutex> lock(align_thread_mutex_);
    if (align_thread_.joinable()) {
      align_thread_.join();
    }
    align_thread_ = std::thread([this, pose_msg]() {
      try {
        initialPoseCallback(pose_msg, /*skip_sc=*/true);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(),
                     "continuousRealignTimer exception: %s", e.what());
      }
      alignment_in_progress_.store(false);
    });
  }
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg,
    bool skip_sc) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_snapshot(
      new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Time scan_stamp;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *cloud_snapshot = *cloud_in_;
    scan_stamp = cloud_stamp_;
  }

  std::size_t removed_points = 0;
  cloud_snapshot = sanitizeCloud<pcl::PointXYZI>(cloud_snapshot, &removed_points);
  if (removed_points > 0) {
    RCLCPP_WARN(this->get_logger(),
                "Dropped %zu non-finite points from alignment snapshot",
                removed_points);
  }

  if (cloud_snapshot->empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Skipping ICP alignment because the input cloud is empty");
    return;
  }

  if (cloud_snapshot->size() < min_points_for_alignment_) {
    RCLCPP_INFO(this->get_logger(),
                "Skipping ICP alignment: only %zu points available",
                cloud_snapshot->size());
    return;
  }

  initial_pose_ = projectPoseToPlane(msg->pose.pose);

  // A non-zero covariance is how RViz and other operators identify an
  // explicit /initialpose message.  Do not let an unverified SC match
  // override that absolute operator prior unless explicitly requested.
  bool explicit_initial_pose = manual_initial_pose_pending_.exchange(false);
  for (const std::size_t index : {std::size_t(0), std::size_t(7),
                                  std::size_t(35)}) {
    explicit_initial_pose = explicit_initial_pose ||
        (std::isfinite(msg->pose.covariance[index]) &&
         msg->pose.covariance[index] > 1e-6);
  }
  const bool allow_sc_override =
      get_parameter("allow_sc_override_initial_pose").as_bool();

  // An operator request may have arrived just after an automatic worker took
  // its snapshot.  Bail out before any SC/grid result can affect recovery;
  // the queued manual worker will be started by the request timer.
  if (!explicit_initial_pose && manual_alignment_cancel_requested_.load()) {
    RCLCPP_INFO(this->get_logger(),
                "Automatic alignment cancelled for queued /initialpose");
    return;
  }

  // Keep the candidate separate from the transform currently published to
  // Nav2.  This lambda also establishes a relative wheel baseline without
  // marking the candidate as an accepted map pose.
  const auto stageGlobalCandidate = [&](const Eigen::Matrix4d &candidate_map_to_odom,
                                        const rclcpp::Time &candidate_stamp,
                                        const std::string &reason) {
    if (!explicit_initial_pose && manual_alignment_cancel_requested_.load()) {
      RCLCPP_INFO(this->get_logger(),
                  "Discarding automatic candidate because /initialpose is queued");
      return false;
    }
    Eigen::Matrix4d candidate_map_to_base = Eigen::Matrix4d::Identity();
    if (!mapPoseFromTransform(candidate_map_to_odom, base_frame_id_,
                              candidate_stamp, candidate_map_to_base)) {
      RCLCPP_WARN(this->get_logger(),
                  "Cannot stage global candidate: map->%s lookup failed",
                  base_frame_id_.c_str());
      registerAlignmentFailure("global_candidate_pose_lookup_failed", true);
      return false;
    }

    const double candidate_x = candidate_map_to_base(0, 3);
    const double candidate_y = candidate_map_to_base(1, 3);
    const double candidate_yaw = std::atan2(candidate_map_to_base(1, 0),
                                            candidate_map_to_base(0, 0));

    // During automatic recovery, a low ICP score is not enough to replace the
    // absolute map hypothesis.  Propagate the last verified map pose with raw
    // wheel odometry and veto candidates that jump to a different shelf or
    // aisle.  Manual /initialpose explicitly clears the trusted anchor before
    // entering this callback and therefore remains authoritative.
    if (!explicit_initial_pose && last_accepted_base_valid_) {
      double current_wheel_x = 0.0;
      double current_wheel_y = 0.0;
      double current_wheel_yaw = 0.0;
      bool current_wheel_valid = false;
      bool trusted_wheel_valid = false;
      double trusted_wheel_x = 0.0;
      double trusted_wheel_y = 0.0;
      double trusted_wheel_yaw = 0.0;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        current_wheel_x = wheel_odom_x_;
        current_wheel_y = wheel_odom_y_;
        current_wheel_yaw = wheel_odom_yaw_;
        current_wheel_valid = wheel_odom_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
        trusted_wheel_x = last_accepted_wheel_x_;
        trusted_wheel_y = last_accepted_wheel_y_;
        trusted_wheel_yaw = last_accepted_wheel_yaw_;
        trusted_wheel_valid = last_accepted_wheel_valid_;
      }

      const double recovery_age = localization_loss_time_valid_
          ? (now() - localization_loss_time_).seconds()
          : 0.0;
      if (!current_wheel_valid || !trusted_wheel_valid || recovery_age < 0.0 ||
          recovery_age > trusted_recovery_anchor_timeout_sec_) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Automatic global candidate rejected: trusted recovery anchor "
            "unavailable/stale (current_wheel=%d trusted_wheel=%d recovery_age=%.2fs/%.2fs); "
            "an explicit /initialpose is required",
            current_wheel_valid ? 1 : 0, trusted_wheel_valid ? 1 : 0,
            recovery_age, trusted_recovery_anchor_timeout_sec_);
        registerAlignmentFailure(
            "global_candidate_trusted_anchor_unavailable", true);
        return false;
      }

      const recovery_guard::GateResult gate = recovery_guard::evaluateCandidate(
          recovery_guard::PlanarPose{
              last_accepted_base_x_, last_accepted_base_y_,
              last_accepted_base_yaw_},
          recovery_guard::PlanarPose{
              trusted_wheel_x, trusted_wheel_y, trusted_wheel_yaw},
          recovery_guard::PlanarPose{
              current_wheel_x, current_wheel_y, current_wheel_yaw},
          recovery_guard::PlanarPose{
              candidate_x, candidate_y, candidate_yaw},
          recovery_wheel_position_gate_, recovery_wheel_yaw_gate_);

      RCLCPP_INFO(
          this->get_logger(),
          "Automatic global candidate trusted-anchor gate: "
          "candidate=(%.2f,%.2f,%.1fdeg) predicted=(%.2f,%.2f,%.1fdeg) "
          "error=(%.3fm,%.2fdeg) limit=(%.3fm,%.2fdeg)",
          candidate_x, candidate_y, candidate_yaw * 180.0 / M_PI,
          gate.predicted.x, gate.predicted.y,
          gate.predicted.yaw * 180.0 / M_PI,
          gate.position_error, gate.yaw_error * 180.0 / M_PI,
          recovery_wheel_position_gate_,
          recovery_wheel_yaw_gate_ * 180.0 / M_PI);

      if (!gate.accepted) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Automatic global candidate rejected by trusted wheel anchor; "
            "keeping last verified map->odom and motion held");
        registerAlignmentFailure(
            "global_candidate_trusted_anchor_reject", true);
        return false;
      }
    }

    recovery_candidate_base_x_ = candidate_x;
    recovery_candidate_base_y_ = candidate_y;
    recovery_candidate_base_yaw_ = candidate_yaw;
    recovery_candidate_valid_ = true;
    local_recovery_active_ = false;
    global_recovery_active_ = true;
    pure_rotation_anchor_valid_ = false;
    pure_rotation_anchor_lio_wheel_divergence_valid_ = false;
    pure_rotation_weak_quality_hold_active_ = false;
    localization_stable_count_ = 0;
    continuous_failure_count_ = 0;

    if (!last_accepted_base_valid_) {
      // Startup/manual localization has no trusted trajectory yet.  Use the
      // unambiguous/operator-constrained global candidate only as a provisional
      // relative-motion anchor until the independent scan sequence commits.
      last_accepted_base_x_ = candidate_x;
      last_accepted_base_y_ = candidate_y;
      last_accepted_base_yaw_ = candidate_yaw;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        last_accepted_wheel_x_ = wheel_odom_x_;
        last_accepted_wheel_y_ = wheel_odom_y_;
        last_accepted_wheel_yaw_ = wheel_odom_yaw_;
        last_accepted_wheel_valid_ = wheel_odom_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      }
      RCLCPP_INFO(
          this->get_logger(),
          "Global candidate established a provisional startup/manual wheel anchor");
    } else {
      RCLCPP_INFO(
          this->get_logger(),
          "Global candidate staged without replacing the last trusted map/wheel anchor");
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Keep publishing the previous (or identity at startup) transform while
      // the chassis is held.  The candidate is supplied to the next ICP pass
      // through recovery_candidate_* and is committed atomically later.
      initial_localization_done_ = true;
      is_ready_ = true;
    }
    publishLocalizationState(false, reason);
    publishLocalizationHold(reason);
    RCLCPP_WARN(this->get_logger(),
                "Staged global candidate (x=%.2f y=%.2f yaw=%.1fdeg); "
                "map->odom remains unchanged until verification",
                candidate_x, candidate_y, candidate_yaw * 180.0 / M_PI);
    return true;
  };

  // Try SC-based relocalization first if available (skip during continuous re-alignment)
  if (sc_loaded_ && !skip_sc && (!explicit_initial_pose || allow_sc_override)) {
    RCLCPP_INFO(this->get_logger(), "Attempting SC-based relocalization...");

    // SC database was built with body_frame=mid360_link (mapping mode).
    // In navigation mode, body_cloud is in base_link frame.
    // Transform the source cloud from laser_frame (base_link) to mid360_link
    // so that ICP source and submap target are in the same coordinate system.
    PointCloudXYZI::Ptr sc_cloud = cloud_snapshot;
    Eigen::Matrix4d laser_to_sc_body = Eigen::Matrix4d::Identity();
    if (laser_frame_id_ != sc_body_frame_) {
      try {
        // lookupTransform(target, source) returns source→target transform
        // We want laser_frame → sc_body_frame, so target=sc_body, source=laser
        auto tf_l2s = tf_buffer_->lookupTransform(
            sc_body_frame_, laser_frame_id_,
            rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
            rclcpp::Duration::from_seconds(1.0));
        Eigen::Vector3d t_l2s(tf_l2s.transform.translation.x,
                              tf_l2s.transform.translation.y,
                              tf_l2s.transform.translation.z);
        Eigen::Quaterniond q_l2s(
            tf_l2s.transform.rotation.w, tf_l2s.transform.rotation.x,
            tf_l2s.transform.rotation.y, tf_l2s.transform.rotation.z);
        laser_to_sc_body.block<3, 3>(0, 0) = q_l2s.toRotationMatrix();
        laser_to_sc_body.block<3, 1>(0, 3) = t_l2s;

        sc_cloud.reset(new PointCloudXYZI);
        pcl::transformPointCloud(*cloud_snapshot, *sc_cloud,
                                 laser_to_sc_body.cast<float>());
        RCLCPP_INFO(this->get_logger(),
                    "Transformed scan from %s to %s for SC matching",
                    laser_frame_id_.c_str(), sc_body_frame_.c_str());
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "Cannot transform to SC body frame: %s, using raw cloud", ex.what());
      }
    }

    Eigen::Matrix4d sc_result;  // this will be map → sc_body (mid360_link)
    if (scRelocalize(sc_cloud, sc_result)) {
      // SC relocalization succeeded
      // sc_result is map → mid360_link
      // We need map → odom = (map → mid360_link) × (mid360_link → odom)
      // sc_result = T_map^mid360 (transforms points from mid360_link to map)
      // We need T_map^odom = T_map^mid360 × T_mid360^odom
      Eigen::Matrix4d sc_body_from_odom = Eigen::Matrix4d::Identity();
      try {
        // lookupTransform(target, source): returns T_target^source
        // We want T_mid360^odom, so target=mid360_link, source=odom
        geometry_msgs::msg::TransformStamped transform;
        try {
          transform = tf_buffer_->lookupTransform(
              sc_body_frame_, odom_frame_id_,
              scan_stamp,
              rclcpp::Duration::from_seconds(1.0));
        } catch (tf2::ExtrapolationException &) {
          RCLCPP_WARN(this->get_logger(),
                      "SC: Scan timestamp too old for TF, using latest transform");
          transform = tf_buffer_->lookupTransform(
              sc_body_frame_, odom_frame_id_,
              tf2::TimePointZero);
        }
        Eigen::Vector3d t(transform.transform.translation.x,
                          transform.transform.translation.y,
                          transform.transform.translation.z);
        Eigen::Quaterniond q(
            transform.transform.rotation.w, transform.transform.rotation.x,
            transform.transform.rotation.y, transform.transform.rotation.z);
        sc_body_from_odom.block<3, 3>(0, 0) = q.toRotationMatrix();
        sc_body_from_odom.block<3, 1>(0, 3) = t;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(),
                    "TF lookup failed after SC relocalization: %s", ex.what());
        // Fall through to grid-search fallback
        goto fallback;
      }

      {
        // T_map^odom = T_map^mid360 × T_mid360^odom
        Eigen::Matrix4d result = sc_result * sc_body_from_odom;
        const double sc_x = sc_result(0, 3);
        const double sc_y = sc_result(1, 3);
        const double sc_yaw = std::atan2(sc_result(1, 0), sc_result(0, 0));
        if (!stageGlobalCandidate(result, scan_stamp,
                                  "global_sc_candidate_stabilizing")) {
          return;
        }
        RCLCPP_INFO(this->get_logger(),
                    "SC accepted map->%s pose: x=%.2f y=%.2f yaw=%.1f deg",
                    sc_body_frame_.c_str(), sc_x, sc_y, sc_yaw * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(),
                    "SC relocalization candidate staged; TF remains held until verification");
      }
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "SC relocalization failed, falling back to grid-search ICP");
    if (sc_match_ambiguous_ && !explicit_initial_pose &&
        !allow_ambiguous_sc_grid_fallback_) {
      // Deployments that require an operator prior can retain the old strict
      // behaviour.  The default WS configuration enables the guarded global
      // fallback below so a stationary robot can self-localize without a
      // clicked RViz pose.
      RCLCPP_WARN(this->get_logger(),
                  "SC reported globally ambiguous geometry; global grid "
                  "fallback is disabled by configuration");
      registerAlignmentFailure("sc_global_ambiguity", true);
      return;
    }
    if (sc_match_ambiguous_ && !explicit_initial_pose) {
      RCLCPP_WARN(this->get_logger(),
                  "SC reported globally ambiguous geometry; trying guarded "
                  "global grid ICP (score must be <= %.3f and pass %d-frame "
                  "stability verification)",
                  global_grid_score_thresh_, localization_stable_samples_);
    }
  }

  if (explicit_initial_pose && !allow_sc_override && sc_loaded_ && !skip_sc) {
    RCLCPP_INFO(this->get_logger(),
                "Explicit /initialpose is authoritative; skipping SC override");
  }

fallback:
  // If SC matched a keyframe but NDT/ICP failed, use that keyframe pose
  // as the grid-search center instead of the (possibly zero) initial_pose_.
  geometry_msgs::msg::Pose fallback_pose = msg->pose.pose;
  if (has_sc_fallback_pose_) {
    RCLCPP_INFO(this->get_logger(),
                "Using SC keyframe pose as grid-search center: (%.2f, %.2f)",
                sc_fallback_pose_.position.x, sc_fallback_pose_.position.y);
    fallback_pose = sc_fallback_pose_;
    has_sc_fallback_pose_ = false;
  }
  fallback_pose = projectPoseToPlane(fallback_pose);

  // Original grid-search ICP path
  // Treat /initialpose as the robot base pose, then convert it to the
  // current laser-frame pose using the fixed base->laser extrinsic.
  Eigen::Vector3d pos(fallback_pose.position.x, fallback_pose.position.y,
                      fallback_pose.position.z);
  Eigen::Quaterniond q(
      fallback_pose.orientation.w, fallback_pose.orientation.x,
      fallback_pose.orientation.y, fallback_pose.orientation.z);
  Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
  map_to_base.block<3, 3>(0, 0) = q.toRotationMatrix();
  map_to_base.block<3, 1>(0, 3) = pos;

  Eigen::Matrix4d base_to_laser = Eigen::Matrix4d::Identity();
  if (base_frame_id_ != laser_frame_id_) {
    try {
      auto transform = tf_buffer_->lookupTransform(
          base_frame_id_, laser_frame_id_,
          rclcpp::Time(0, 0, this->get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(1.0));
      Eigen::Vector3d t(transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z);
      Eigen::Quaterniond q_fixed(
          transform.transform.rotation.w, transform.transform.rotation.x,
          transform.transform.rotation.y, transform.transform.rotation.z);
      base_to_laser.block<3, 3>(0, 0) = q_fixed.toRotationMatrix();
      base_to_laser.block<3, 1>(0, 3) = t;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(),
                  "Waiting for fixed TF %s -> %s before ICP: %s",
                  base_frame_id_.c_str(), laser_frame_id_.c_str(), ex.what());
      return;
    }
  }

  Eigen::Matrix4d initial_guess = map_to_base * base_to_laser;

  Eigen::Matrix4d map_to_laser;
  if (skip_sc) {
    // Continuous re-alignment: skip grid-search, directly refine from current pose
    RCLCPP_INFO(this->get_logger(), "Continuous ICP refine from current pose");

    std::size_t removed_points = 0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
        new pcl::PointCloud<pcl::PointXYZI>);
    voxel_refine_filter_.setInputCloud(cloud_snapshot);
    voxel_refine_filter_.filter(*refine_source);
    refine_source = sanitizeCloud<pcl::PointXYZI>(refine_source, &removed_points);

    if (refine_source->size() < min_points_for_alignment_) {
      RCLCPP_WARN(this->get_logger(),
                  "Continuous ICP: insufficient points (%zu)", refine_source->size());
      registerAlignmentFailure("continuous_filtered_cloud_too_sparse", true);
      return;
    }

    icp_refine_.setInputSource(refine_source);

    // Crop map around current estimated position
    Eigen::Vector3d crop_center(initial_guess(0, 3), initial_guess(1, 3), initial_guess(2, 3));
    PointCloudXYZI::Ptr local_refine_map = cropMapByPosition(refine_map_, crop_center, map_crop_radius_);
    if (local_refine_map->size() < min_points_for_alignment_) {
      RCLCPP_WARN(this->get_logger(),
                  "Continuous ICP: cropped map too small (%zu pts), using full map",
                  local_refine_map->size());
      local_refine_map = refine_map_;
    }
    RCLCPP_DEBUG(this->get_logger(),
        "Continuous ICP map crop: %zu -> %zu pts (r=%.1f)",
        refine_map_->size(), local_refine_map->size(), map_crop_radius_);

    icp_refine_.setInputTarget(local_refine_map);
    if (constrain_to_2d_) {
      if (!preserve_3d_structure_for_planar_icp_) {
        refine_source = projectCloudToPlane(refine_source);
        local_refine_map = projectCloudToPlane(local_refine_map);
      }
      icp_refine_.setInputSource(refine_source);
      icp_refine_.setInputTarget(local_refine_map);
      icp_refine_.setTransformationEstimation(
          std::make_shared<pcl::registration::TransformationEstimation2D<
              PointType, PointType, float>>());
    }
    pcl::PointCloud<pcl::PointXYZI> aligned;
    const Eigen::Matrix4d icp_initial_guess = constrain_to_2d_
        ? projectTransformToPlane(initial_guess)
        : initial_guess;
    icp_refine_.setMaxCorrespondenceDistance(
        continuous_icp_max_correspondence_distance_);
    icp_refine_.align(aligned, icp_initial_guess.cast<float>());

    continuous_quality_ = calculateRegistrationQuality(
        aligned, local_refine_map,
        continuous_icp_max_correspondence_distance_);
    continuous_quality_ok_ = registration_quality::passesThresholds(
        continuous_quality_, continuous_min_inlier_ratio_,
        continuous_max_inlier_rmse_, continuous_max_median_distance_);
    score_ = continuous_quality_.all_mse;
    success_ = icp_refine_.hasConverged() && continuous_quality_ok_;

    RCLCPP_INFO(
        this->get_logger(),
        "Continuous ICP quality: solver=%d accepted=%d all_mse=%.4f "
        "inliers=%zu/%zu (%.3f/%.3f) inlier_rmse=%.3fm/%.3fm "
        "p50=%.3fm/%.3fm p90=%.3fm",
        icp_refine_.hasConverged() ? 1 : 0, success_ ? 1 : 0,
        continuous_quality_.all_mse, continuous_quality_.inlier_count,
        continuous_quality_.valid_count, continuous_quality_.inlier_ratio,
        continuous_min_inlier_ratio_, continuous_quality_.inlier_rmse,
        continuous_max_inlier_rmse_, continuous_quality_.p50_distance,
        continuous_max_median_distance_, continuous_quality_.p90_distance);

    bool low_support_accepted = false;
    if (!success_) {
      // At some headings an in-place spin makes the shelf cloud weakly
      // observable and robust ICP metrics can cross the normal limits by only
      // a few millimetres.  Do not let those borderline frames erase an
      // already verified pose.  This exception is intentionally narrow and
      // fail-closed: independent wheel and raw-LIO observations must both be
      // fresh and agree with the divergence captured at the pure-rotation
      // anchor.  The weak ICP result itself is never published; map->odom
      // remains at the last accepted transform while local odometry propagates
      // the bounded rotation.
      double wheel_linear_speed = 0.0;
      double wheel_angular_speed = 0.0;
      double wheel_x = 0.0;
      double wheel_y = 0.0;
      double wheel_yaw = 0.0;
      double lio_yaw = 0.0;
      double rotation_age = std::numeric_limits<double>::infinity();
      bool wheel_recent = false;
      bool lio_recent = false;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_linear_speed = wheel_odom_linear_speed_;
        wheel_angular_speed = wheel_odom_angular_speed_;
        wheel_x = wheel_odom_x_;
        wheel_y = wheel_odom_y_;
        wheel_yaw = wheel_odom_yaw_;
        lio_yaw = lio_odom_yaw_;
        wheel_recent = wheel_odom_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <=
             wheel_odom_pose_timeout_);
        lio_recent = lio_odom_valid_ &&
            ((now() - lio_odom_stamp_).seconds() <=
             wheel_odom_pose_timeout_);
        if (last_wheel_rotation_stamp_.nanoseconds() > 0) {
          rotation_age = (now() - last_wheel_rotation_stamp_).seconds();
        }
      }

      const double wheel_travel_from_anchor = pure_rotation_anchor_valid_
          ? std::hypot(wheel_x - pure_rotation_anchor_wheel_x_,
                       wheel_y - pure_rotation_anchor_wheel_y_)
          : std::numeric_limits<double>::infinity();
      const double current_divergence = normalizeAngle(lio_yaw - wheel_yaw);
      const double divergence_delta =
          pure_rotation_anchor_lio_wheel_divergence_valid_
          ? normalizeAngle(
              current_divergence -
              pure_rotation_anchor_lio_wheel_divergence_)
          : std::numeric_limits<double>::infinity();

      bool fine_guard_active = false;
      bool fine_guard_completed = false;
      double fine_guard_anchor_x = 0.0;
      double fine_guard_anchor_y = 0.0;
      double fine_guard_anchor_divergence = 0.0;
      rclcpp::Time fine_guard_start_time(
          0, 0, this->get_clock()->get_clock_type());
      {
        std::lock_guard<std::mutex> lock(fine_alignment_guard_mutex_);
        fine_guard_active = fine_alignment_guard_active_;
        fine_guard_completed = fine_alignment_guard_completed_;
        fine_guard_anchor_x = fine_alignment_guard_anchor_wheel_x_;
        fine_guard_anchor_y = fine_alignment_guard_anchor_wheel_y_;
        fine_guard_anchor_divergence =
            fine_alignment_guard_anchor_divergence_;
        fine_guard_start_time = fine_alignment_guard_start_time_;
      }
      const double fine_guard_travel = fine_guard_active
          ? std::hypot(
              wheel_x - fine_guard_anchor_x, wheel_y - fine_guard_anchor_y)
          : std::numeric_limits<double>::infinity();
      const double fine_guard_divergence_delta = fine_guard_active
          ? normalizeAngle(current_divergence - fine_guard_anchor_divergence)
          : std::numeric_limits<double>::infinity();
      const double fine_guard_age = fine_guard_active
          ? (now() - fine_guard_start_time).seconds()
          : std::numeric_limits<double>::infinity();
      const fine_alignment_guard::HoldInput fine_guard_input{
          fine_guard_active, icp_refine_.hasConverged(),
          localization_healthy_.load(), wheel_recent, lio_recent,
          lio_yaw_jump_detected_, wheel_linear_speed, fine_guard_travel,
          fine_guard_divergence_delta, fine_guard_age,
          continuous_quality_.inlier_count, continuous_quality_.inlier_ratio,
          continuous_quality_.inlier_rmse,
          continuous_quality_.p50_distance};
      const fine_alignment_guard::HoldLimits fine_guard_limits{
          fine_alignment_guard_max_linear_speed_,
          fine_alignment_guard_max_travel_,
          fine_alignment_guard_max_divergence_,
          fine_alignment_guard_max_duration_,
          fine_alignment_guard_min_inlier_count_,
          fine_alignment_guard_min_inlier_ratio_,
          fine_alignment_guard_max_inlier_rmse_,
          fine_alignment_guard_max_median_distance_};
      if (fine_alignment_guard::shouldKeepLastTransform(
              fine_guard_input, fine_guard_limits)) {
        continuous_low_support_active_ = false;
        localization_stable_count_ = 0;
        continuous_failure_count_ = 0;
        RCLCPP_WARN(
            this->get_logger(),
            "Fine-alignment weak ICP: retaining last verified map->odom "
            "(lease=%s age=%.2fs/%.2fs, wheel_v=%.3f, "
            "wheel_travel=%.3fm/%.3fm, lio_wheel_delta=%.2fdeg, "
            "inliers=%zu ratio=%.3f, rmse=%.3f, p50=%.3f); "
            "weak ICP result discarded",
            fine_guard_completed ? "completed" : "active", fine_guard_age,
            fine_alignment_guard_max_duration_, wheel_linear_speed,
            fine_guard_travel, fine_alignment_guard_max_travel_,
            fine_guard_divergence_delta * 180.0 / M_PI,
            continuous_quality_.inlier_count,
            continuous_quality_.inlier_ratio,
            continuous_quality_.inlier_rmse,
            continuous_quality_.p50_distance);
        return;
      }

      if (!pure_rotation_weak_quality_hold_active_) {
        pure_rotation_weak_quality_hold_start_time_ = now();
      }
      const double weak_quality_hold_age =
          (now() - pure_rotation_weak_quality_hold_start_time_).seconds();
      const pure_rotation_guard::WeakQualityHoldInput hold_input{
          icp_refine_.hasConverged(), localization_healthy_.load(),
          pure_rotation_anchor_valid_ &&
              pure_rotation_anchor_lio_wheel_divergence_valid_,
          wheel_recent, lio_recent, lio_yaw_jump_detected_,
          wheel_linear_speed, wheel_angular_speed,
          wheel_travel_from_anchor, divergence_delta,
          weak_quality_hold_age, continuous_quality_.inlier_ratio,
          continuous_quality_.inlier_rmse,
          continuous_quality_.p50_distance};
      const pure_rotation_guard::WeakQualityHoldLimits hold_limits{
          pure_rotation_linear_speed_thresh_,
          pure_rotation_angular_speed_thresh_,
          pure_rotation_anchor_release_distance_,
          lio_jump_divergence_thresh_,
          pure_rotation_weak_quality_max_duration_,
          continuous_min_inlier_ratio_, continuous_max_inlier_rmse_,
          continuous_max_median_distance_,
          pure_rotation_weak_quality_margin_factor_};

      if (pure_rotation_guard::shouldKeepLastTransform(
              hold_input, hold_limits)) {
        pure_rotation_weak_quality_hold_active_ = true;
        localization_stable_count_ = 0;
        RCLCPP_WARN(
            this->get_logger(),
            "Pure-rotation borderline ICP quality: retaining last verified "
            "map->odom (age=%.2fs/%.2fs, wheel_v=%.3f, wheel_w=%.3f, "
            "wheel_travel=%.3f, lio_wheel_delta=%.2fdeg, "
            "inliers=%.3f, rmse=%.3f, p50=%.3f); weak ICP result discarded",
            weak_quality_hold_age,
            pure_rotation_weak_quality_max_duration_, wheel_linear_speed,
            wheel_angular_speed, wheel_travel_from_anchor,
            divergence_delta * 180.0 / M_PI,
            continuous_quality_.inlier_ratio,
            continuous_quality_.inlier_rmse,
            continuous_quality_.p50_distance);
        return;
      }

      pure_rotation_weak_quality_hold_active_ = false;

      const pure_rotation_guard::PostRotationWeakQualityHoldInput
          post_rotation_input{
              icp_refine_.hasConverged(), localization_healthy_.load(),
              pure_rotation_anchor_valid_ &&
                  pure_rotation_anchor_lio_wheel_divergence_valid_,
              wheel_recent, lio_recent, lio_yaw_jump_detected_,
              wheel_linear_speed, wheel_travel_from_anchor, divergence_delta,
              rotation_age, continuous_quality_.inlier_ratio,
              continuous_quality_.inlier_rmse,
              continuous_quality_.p50_distance};
      const pure_rotation_guard::PostRotationWeakQualityHoldLimits
          post_rotation_limits{
              post_rotation_weak_quality_max_linear_speed_,
              post_rotation_weak_quality_max_travel_,
              lio_jump_divergence_thresh_,
              post_rotation_weak_quality_max_duration_,
              continuous_min_inlier_ratio_, continuous_max_inlier_rmse_,
              continuous_max_median_distance_,
              pure_rotation_weak_quality_margin_factor_};

      if (pure_rotation_guard::shouldKeepLastTransformAfterRotation(
              post_rotation_input, post_rotation_limits)) {
        continuous_low_support_active_ = false;
        localization_stable_count_ = 0;
        RCLCPP_WARN(
            this->get_logger(),
            "Post-rotation borderline ICP quality: retaining last verified "
            "map->odom (rotation_age=%.2fs/%.2fs, wheel_v=%.3f, "
            "wheel_travel=%.3fm/%.3fm, lio_wheel_delta=%.2fdeg, "
            "inliers=%.3f, rmse=%.3f, p50=%.3f); weak ICP result discarded",
            rotation_age, post_rotation_weak_quality_max_duration_,
            wheel_linear_speed, wheel_travel_from_anchor,
            post_rotation_weak_quality_max_travel_,
            divergence_delta * 180.0 / M_PI,
            continuous_quality_.inlier_ratio,
            continuous_quality_.inlier_rmse,
            continuous_quality_.p50_distance);
        return;
      }

      // The hold above requires |omega| >= min_angular_speed, so it stops
      // applying the instant the final rotation ends -- while the diluted
      // inlier_ratio persists, because it is caused by unmapped structure at
      // this pose rather than by the rotation.  Recheck as a low-support frame:
      // if the matched points still fit tightly, publish the result instead of
      // latching motion off.  Unlike the hold, the anchor is not required (it is
      // released once the chassis stops), so when no anchor divergence baseline
      // exists the latched lio_yaw_jump_detected_ flag and the per-sample
      // divergence detector are what cover LIO drift.
      if (!continuous_low_support_active_) {
        continuous_low_support_start_time_ = now();
      }
      const double low_support_age =
          (now() - continuous_low_support_start_time_).seconds();
      const low_support_guard::LowSupportInput low_support_input{
          icp_refine_.hasConverged(), localization_healthy_.load(),
          wheel_recent, lio_recent, lio_yaw_jump_detected_,
          std::isfinite(divergence_delta) ? divergence_delta : 0.0,
          low_support_age, continuous_quality_.inlier_count,
          continuous_quality_.inlier_ratio,
          continuous_quality_.inlier_rmse,
          continuous_quality_.p50_distance};
      const low_support_guard::LowSupportLimits low_support_limits{
          lio_jump_divergence_thresh_,
          continuous_low_support_max_duration_,
          continuous_low_support_min_inlier_ratio_,
          static_cast<std::size_t>(
              std::max(0, continuous_low_support_min_inlier_count_)),
          continuous_low_support_residual_tighten_,
          continuous_max_inlier_rmse_, continuous_max_median_distance_};

      if (low_support_guard::shouldAcceptLowSupport(
              low_support_input, low_support_limits)) {
        continuous_low_support_active_ = true;
        low_support_accepted = true;
        success_ = true;
        continuous_quality_ok_ = true;
        RCLCPP_WARN(
            this->get_logger(),
            "Low-support ICP accepted: unmapped structure diluted the inlier "
            "ratio but residuals stayed tight (age=%.2fs/%.2fs, "
            "inliers=%zu/%zu (%.3f>=%.3f), rmse=%.3f<=%.3f, p50=%.3f<=%.3f)",
            low_support_age, continuous_low_support_max_duration_,
            continuous_quality_.inlier_count, continuous_quality_.valid_count,
            continuous_quality_.inlier_ratio,
            continuous_low_support_min_inlier_ratio_,
            continuous_quality_.inlier_rmse,
            continuous_max_inlier_rmse_ *
                continuous_low_support_residual_tighten_,
            continuous_quality_.p50_distance,
            continuous_max_median_distance_ *
                continuous_low_support_residual_tighten_);
      } else {
        continuous_low_support_active_ = false;
        // Don't clear is_ready_ on continuous failure — keep last good pose
        RCLCPP_WARN(this->get_logger(),
                    "Continuous ICP robust quality rejected the alignment, "
                    "keeping last good pose");
        registerAlignmentFailure(
            icp_refine_.hasConverged() ?
            "continuous_icp_quality_reject" :
            "continuous_icp_solver_not_converged");
        return;
      }
    }
    if (!low_support_accepted) {
      continuous_low_support_active_ = false;
      refreshFineAlignmentCompletedLease();
    }
    pure_rotation_weak_quality_hold_active_ = false;
    map_to_laser = icp_refine_.getFinalTransformation().cast<double>();
  } else {
    // Initial alignment: full grid-search
    RCLCPP_INFO(this->get_logger(), "Aligning the pointcloud");
    map_to_laser = multiAlignSync(
        cloud_snapshot, initial_guess, explicit_initial_pose);
    if (!success_) {
      std::lock_guard lock(mutex_);
      is_ready_ = false;
      RCLCPP_INFO(this->get_logger(),
                  "ICP did not converge yet, will retry from the latest scan");
      return;
    }
  }

  if (!explicit_initial_pose && manual_alignment_cancel_requested_.load()) {
    RCLCPP_INFO(this->get_logger(),
                "Discarding completed automatic alignment because "
                "/initialpose is queued");
    return;
  }

  Eigen::Matrix4d laser_to_odom = Eigen::Matrix4d::Identity();
  try {
    // Get odom to laser transform — try scan timestamp first,
    // fall back to latest TF if scan is too old (e.g. after long grid-search)
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          laser_frame_id_, odom_frame_id_,
          scan_stamp,
          rclcpp::Duration::from_seconds(1.0));
    } catch (tf2::ExtrapolationException &) {
      RCLCPP_WARN(this->get_logger(),
                  "Scan timestamp too old for TF, using latest transform");
      transform = tf_buffer_->lookupTransform(
          laser_frame_id_, odom_frame_id_,
          tf2::TimePointZero);
    }
    Eigen::Vector3d t(transform.transform.translation.x,
                      transform.transform.translation.y,
                      transform.transform.translation.z);
    Eigen::Quaterniond q_odom(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);
    laser_to_odom.block<3, 3>(0, 0) = q_odom.toRotationMatrix();
    laser_to_odom.block<3, 1>(0, 3) = t;
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    registerAlignmentFailure("alignment_result_tf_failed", true);
    return;
  }
  Eigen::Matrix4d result = map_to_laser * laser_to_odom;
  const Eigen::Matrix4d odom_to_laser = laser_to_odom.inverse();

  // All downstream navigation consumers receive the planar map->odom that
  // updateMapToOdom() publishes.  Apply that same projection before checking
  // or constraining the candidate pose.  Previously the translation freeze
  // was solved with the full 3D ICP rotation and roll/pitch were discarded
  // only while publishing.  The published planar transform therefore moved
  // map->base_link by a small amount on every continuous alignment even when
  // the 3D calculation claimed that translation was frozen.
  if (constrain_to_2d_) {
    result = projectTransformToPlane(result);
    map_to_laser = result * odom_to_laser;
  }

  // Continuous ICP may become weakly observable after rotation. Compare its
  // x/y correction with the pose propagated by the local odom transform. This
  // rejects the delayed 30-50 cm corrections seen after a spin while still
  // allowing global relocalization (skip_sc=false) to move the pose freely.
  if (skip_sc && initial_localization_done_ && localization_healthy_.load()) {
    bool freeze_translation = false;
    bool wheel_recent = false;
    bool rotating_now = false;
    bool rotation_recent = false;
    bool lio_recent = false;
    double wheel_linear_speed = 0.0;
    double wheel_angular_speed = 0.0;
    double wheel_x = 0.0;
    double wheel_y = 0.0;
    double wheel_yaw = 0.0;
    double lio_yaw = 0.0;
    double rotation_age = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
      wheel_recent = wheel_odom_valid_ &&
          ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      wheel_linear_speed = wheel_odom_linear_speed_;
      wheel_angular_speed = wheel_odom_angular_speed_;
      wheel_x = wheel_odom_x_;
      wheel_y = wheel_odom_y_;
      wheel_yaw = wheel_odom_yaw_;
      lio_yaw = lio_odom_yaw_;
      lio_recent = lio_odom_valid_ &&
          ((now() - lio_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      if (last_wheel_rotation_stamp_.nanoseconds() > 0) {
        rotation_age = (now() - last_wheel_rotation_stamp_).seconds();
      }
      rotating_now =
          std::abs(wheel_angular_speed) >= pure_rotation_angular_speed_thresh_;
      rotation_recent = rotation_age >= 0.0 &&
          rotation_age <= pure_rotation_translation_freeze_settle_time_;
    }

    const bool rotation_trigger = wheel_recent &&
        wheel_linear_speed <= pure_rotation_linear_speed_thresh_ &&
        (rotating_now || rotation_recent);
    double anchor_wheel_travel = std::numeric_limits<double>::infinity();
    if (pure_rotation_anchor_valid_ && wheel_recent) {
      anchor_wheel_travel = std::hypot(
          wheel_x - pure_rotation_anchor_wheel_x_,
          wheel_y - pure_rotation_anchor_wheel_y_);
    }

    const bool post_rotation_anchor_needed =
        rotation_age >= 0.0 &&
        rotation_age <= post_rotation_weak_quality_max_duration_ &&
        anchor_wheel_travel <= post_rotation_weak_quality_max_travel_;
    const bool legacy_anchor_needed =
        anchor_wheel_travel <= pure_rotation_anchor_release_distance_;
    if (pure_rotation_anchor_valid_ &&
        (!wheel_recent ||
         (!rotating_now && !rotation_recent && !legacy_anchor_needed &&
          !post_rotation_anchor_needed))) {
      RCLCPP_INFO(
          this->get_logger(),
          "Pure-rotation wheel anchor released: wheel_recent=%d "
          "wheel_travel=%.3fm (freeze=%.3fm transition=%.3fm) "
          "rotation_age=%.2fs/%.2fs",
          wheel_recent ? 1 : 0, anchor_wheel_travel,
          pure_rotation_anchor_release_distance_,
          post_rotation_weak_quality_max_travel_, rotation_age,
          post_rotation_weak_quality_max_duration_);
      pure_rotation_anchor_valid_ = false;
      pure_rotation_anchor_lio_wheel_divergence_valid_ = false;
      pure_rotation_weak_quality_hold_active_ = false;
    }

    if (!pure_rotation_anchor_valid_ && rotation_trigger &&
        last_accepted_base_valid_) {
      pure_rotation_anchor_map_x_ = last_accepted_base_x_;
      pure_rotation_anchor_map_y_ = last_accepted_base_y_;
      pure_rotation_anchor_map_yaw_ = last_accepted_base_yaw_;
      pure_rotation_anchor_wheel_x_ = wheel_x;
      pure_rotation_anchor_wheel_y_ = wheel_y;
      pure_rotation_anchor_wheel_yaw_ = wheel_yaw;
      pure_rotation_anchor_lio_wheel_divergence_ =
          normalizeAngle(lio_yaw - wheel_yaw);
      pure_rotation_anchor_lio_wheel_divergence_valid_ = lio_recent;
      pure_rotation_anchor_valid_ = true;
      pure_rotation_weak_quality_hold_active_ = false;
      anchor_wheel_travel = 0.0;
      RCLCPP_INFO(
          this->get_logger(),
          "Pure-rotation wheel anchor captured: map=(%.3f,%.3f,%.1fdeg) "
          "wheel=(%.3f,%.3f,%.1fdeg)",
          pure_rotation_anchor_map_x_, pure_rotation_anchor_map_y_,
          pure_rotation_anchor_map_yaw_ * 180.0 / M_PI,
          pure_rotation_anchor_wheel_x_, pure_rotation_anchor_wheel_y_,
          pure_rotation_anchor_wheel_yaw_ * 180.0 / M_PI);
    }

    freeze_translation = pure_rotation_anchor_valid_ && wheel_recent &&
        (rotation_trigger ||
         anchor_wheel_travel <= pure_rotation_anchor_release_distance_);

    Eigen::Matrix4d previous_map_to_odom = Eigen::Matrix4d::Identity();
    {
      std::lock_guard lock(mutex_);
      const auto &previous_tf = map_to_odom_.transform;
      const Eigen::Quaterniond previous_q(
          previous_tf.rotation.w, previous_tf.rotation.x,
          previous_tf.rotation.y, previous_tf.rotation.z);
      previous_map_to_odom.block<3, 3>(0, 0) =
          previous_q.normalized().toRotationMatrix();
      previous_map_to_odom(0, 3) = previous_tf.translation.x;
      previous_map_to_odom(1, 3) = previous_tf.translation.y;
      previous_map_to_odom(2, 3) = previous_tf.translation.z;
    }

    const double raw_base_x = map_to_laser(0, 3);
    const double raw_base_y = map_to_laser(1, 3);
    const Eigen::Matrix4d odom_predicted_map_to_laser =
        previous_map_to_odom * odom_to_laser;
    Eigen::Matrix4d translation_reference_map_to_laser =
        odom_predicted_map_to_laser;
    if (freeze_translation) {
      const auto predicted_map_base = recovery_guard::propagateFromWheel(
          recovery_guard::PlanarPose{
              pure_rotation_anchor_map_x_, pure_rotation_anchor_map_y_,
              pure_rotation_anchor_map_yaw_},
          recovery_guard::PlanarPose{
              pure_rotation_anchor_wheel_x_, pure_rotation_anchor_wheel_y_,
              pure_rotation_anchor_wheel_yaw_},
          recovery_guard::PlanarPose{wheel_x, wheel_y, wheel_yaw});
      Eigen::Matrix4d predicted_map_to_base = Eigen::Matrix4d::Identity();
      const double predicted_cos = std::cos(predicted_map_base.yaw);
      const double predicted_sin = std::sin(predicted_map_base.yaw);
      predicted_map_to_base(0, 0) = predicted_cos;
      predicted_map_to_base(0, 1) = -predicted_sin;
      predicted_map_to_base(1, 0) = predicted_sin;
      predicted_map_to_base(1, 1) = predicted_cos;
      predicted_map_to_base(0, 3) = predicted_map_base.x;
      predicted_map_to_base(1, 3) = predicted_map_base.y;
      translation_reference_map_to_laser =
          predicted_map_to_base * base_to_laser;
    }
    const double translation_correction = std::hypot(
        raw_base_x - translation_reference_map_to_laser(0, 3),
        raw_base_y - translation_reference_map_to_laser(1, 3));
    const bool excessive_translation_correction =
        translation_correction > continuous_translation_correction_limit_;

    // A large correction while not rotating is evidence that the published
    // map->odom no longer agrees with the static map.  Do not silently replace
    // the ICP translation with the odom prediction and then report the result
    // as accepted: that leaves localization_ok=true while the live cloud drifts
    // farther away from the 2D map.  Latch motion off immediately.  On the next
    // continuous alignment localization_healthy_ is false, so the verified raw
    // ICP correction is allowed through while the chassis is held at zero; the
    // existing stabilization gate requires multiple stable samples before
    // motion is enabled again.
    if (excessive_translation_correction && !freeze_translation &&
        localization_healthy_.load()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Continuous ICP translation correction exceeded safety limit: "
          "%.3f m > %.3f m, raw_base=(%.3f,%.3f) "
          "odom_predicted=(%.3f,%.3f); holding motion for controlled realignment",
          translation_correction, continuous_translation_correction_limit_,
          raw_base_x, raw_base_y,
          odom_predicted_map_to_laser(0, 3),
          odom_predicted_map_to_laser(1, 3));
      registerAlignmentFailure(
          "continuous_icp_translation_correction_exceeded", true);
      return;
    }

    if (freeze_translation) {
      // Keep ICP yaw, but choose map->odom translation so the resulting
      // map->laser x/y equals the local-odom prediction. Merely retaining the
      // old map->odom translation would rotate the non-zero odom position
      // around the odom origin and create another apparent x/y jump.
      const Eigen::Matrix4d icp_yaw_map_to_laser = result * odom_to_laser;
      result(0, 3) += translation_reference_map_to_laser(0, 3) -
          icp_yaw_map_to_laser(0, 3);
      result(1, 3) += translation_reference_map_to_laser(1, 3) -
          icp_yaw_map_to_laser(1, 3);
      map_to_laser = result * odom_to_laser;
    }

    if (freeze_translation) {
      RCLCPP_INFO(
          this->get_logger(),
          "Pure-rotation ICP translation frozen: wheel_v=%.3f m/s "
          "wheel_w=%.3f rad/s rotation_age=%.2fs wheel_travel=%.3fm "
          "correction=%.3fm, "
          "raw_base=(%.3f,%.3f) constrained_base=(%.3f,%.3f)",
          wheel_linear_speed, wheel_angular_speed, rotation_age,
          anchor_wheel_travel, translation_correction, raw_base_x, raw_base_y,
          map_to_laser(0, 3), map_to_laser(1, 3));
    }
  }

  // Sanity check: reject continuous ICP results that produce unreasonably
  // large corrections.  Compare the NEW map→base_link pose against the
  // PREVIOUS one.  This catches both ICP errors AND odom jumps.
  if (skip_sc && initial_localization_done_) {
    // Compute new map→base_link = result(map→odom) × odom→base_link
    // We already have map_to_laser (=map→base_link since laser=base_link)
    // and result = map_to_laser * laser_to_odom (=map→odom).
    // But map_to_laser IS the new map→base estimate from ICP.
    const double new_base_x = map_to_laser(0, 3);
    const double new_base_y = map_to_laser(1, 3);
    const double new_base_yaw = std::atan2(map_to_laser(1, 0), map_to_laser(0, 0));
    double accepted_pos_delta = std::numeric_limits<double>::infinity();
    double accepted_yaw_delta = std::numeric_limits<double>::infinity();

    if (last_accepted_base_valid_) {
      const double d_x = new_base_x - last_accepted_base_x_;
      const double d_y = new_base_y - last_accepted_base_y_;
      const double pos_delta = std::hypot(d_x, d_y);
      double yaw_delta = std::abs(new_base_yaw - last_accepted_base_yaw_);
      if (yaw_delta > M_PI) yaw_delta = 2.0 * M_PI - yaw_delta;
      accepted_pos_delta = pos_delta;
      accepted_yaw_delta = yaw_delta;

      // Dynamic yaw threshold: omega_max * dt + margin.
      // At omega_max=1.0 rad/s and 1.5s ICP interval, max expected
      // yaw change ≈ 1.5 rad (~86°).  A fixed 35° or 55° threshold
      // falsely rejects normal turns.  Instead, compute from elapsed
      // time since last accepted result.
      constexpr double kOmegaMax = 1.2;        // slightly above controller's omega_max
      constexpr double kMinYawDelta = 0.61;     // floor: ~35°, always catch 180° flips
      constexpr double kYawMargin = 0.35;       // ~20° margin for LIO drift
      const double dt_since_last =
          (now() - last_accepted_base_stamp_).seconds();
      const double kMaxYawDelta = std::max(
          kMinYawDelta,
          kOmegaMax * dt_since_last + kYawMargin);
      const double kMaxPositionDelta =
          max_base_translation_speed_ * std::max(0.0, dt_since_last) +
          translation_jump_margin_;

      RCLCPP_DEBUG(this->get_logger(),
          "ICP sanity: base (%.2f,%.2f,%.2f°) -> (%.2f,%.2f,%.2f°), "
          "pos_delta=%.3f, yaw_delta=%.3f",
          last_accepted_base_x_, last_accepted_base_y_,
          last_accepted_base_yaw_ * 180.0 / M_PI,
          new_base_x, new_base_y, new_base_yaw * 180.0 / M_PI,
          pos_delta, yaw_delta);

      if (localization_healthy_.load() && pos_delta > kMaxPositionDelta) {
        RCLCPP_WARN(this->get_logger(),
            "Continuous ICP rejected: base translation jump %.3f m exceeds %.3f m "
            "(dt=%.2fs)",
            pos_delta, kMaxPositionDelta, dt_since_last);
        registerAlignmentFailure("continuous_icp_translation_jump", true);
        return;
      }

      if (localization_healthy_.load() && yaw_delta > kMaxYawDelta) {
        // Check if this is a known LIO yaw jump with high-confidence ICP
        const double score_limit = thresh_ * lio_jump_icp_score_factor_;
        if (lio_yaw_jump_detected_ && score_ < score_limit) {
          double wheel_yaw_now;
          bool wheel_ready_now;
          {
            std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
            wheel_yaw_now = wheel_odom_yaw_;
            wheel_ready_now = wheel_odom_valid_ &&
                ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
          }

          if (!wheel_ready_now || !icp_wheel_yaw_offset_valid_) {
            RCLCPP_WARN(this->get_logger(),
                "LIO yaw jump RECOVERY rejected: wheel yaw baseline unavailable");
            publishLocalizationHold("jump_recovery_no_wheel_baseline");
            registerAlignmentFailure("jump_recovery_no_wheel_baseline", true);
            ++consecutive_reject_count_;
            return;
          }

          const double expected_base_yaw =
              normalizeAngle(wheel_yaw_now + icp_wheel_yaw_offset_);
          const double recovery_yaw_error =
              std::abs(normalizeAngle(new_base_yaw - expected_base_yaw));
          if (recovery_yaw_error > icp_wheel_recovery_yaw_gate_thresh_) {
            RCLCPP_WARN(this->get_logger(),
                "LIO yaw jump RECOVERY rejected: wheel yaw gate failed "
                "(candidate=%.1f°, expected=%.1f°, err=%.1f°, thresh=%.1f°)",
                new_base_yaw * 180.0 / M_PI,
                expected_base_yaw * 180.0 / M_PI,
                recovery_yaw_error * 180.0 / M_PI,
                icp_wheel_recovery_yaw_gate_thresh_ * 180.0 / M_PI);
            publishLocalizationHold("jump_recovery_wheel_gate_reject");
            registerAlignmentFailure("jump_recovery_wheel_gate_reject", true);
            ++consecutive_reject_count_;
            return;
          }

          // Dual gate passed: wheel odom confirmed LIO jump + ICP score excellent
          RCLCPP_WARN(this->get_logger(),
              "LIO yaw jump RECOVERY: accepting ICP result (yaw_delta=%.1f°, "
              "score=%.4f < %.4f, wheel_err=%.1f°) — force-snapping map->odom",
              yaw_delta * 180.0 / M_PI, score_, score_limit,
              recovery_yaw_error * 180.0 / M_PI);
          lio_yaw_jump_detected_ = false;
          consecutive_reject_count_ = 0;
          icp_wheel_yaw_offset_valid_ = false;  // recalibrate after recovery
          // Update accepted base pose and force-snap TF
          last_accepted_base_x_ = new_base_x;
          last_accepted_base_y_ = new_base_y;
          last_accepted_base_yaw_ = new_base_yaw;
          last_accepted_base_stamp_ = now();
          last_accepted_base_valid_ = true;
          updateMapToOdom(result, /*force_snap=*/true);
          resetLocalizationStability("yaw_jump_recovery_stabilizing");
          RCLCPP_INFO(this->get_logger(),
                      "ICP ready (jump recovery), publishing TF %s -> %s",
                      map_frame_id_.c_str(), odom_frame_id_.c_str());
          return;
        }

        RCLCPP_WARN(this->get_logger(),
            "Continuous ICP rejected: base yaw_delta=%.1f° (max %.1f°, dt=%.2fs), "
            "pos_delta=%.3f m — ICP likely converged to mirror pose "
            "(lio_jump=%d, score=%.4f, score_limit=%.4f)",
            yaw_delta * 180.0 / M_PI, kMaxYawDelta * 180.0 / M_PI, dt_since_last,
            pos_delta,
            lio_yaw_jump_detected_ ? 1 : 0, score_, score_limit);
        publishLocalizationHold("continuous_icp_mirror_pose_reject");
        registerAlignmentFailure("continuous_icp_mirror_pose_reject", true);

        // Consecutive reject detection: if sanity check rejects multiple times
        // in a row, LIO is likely drifting gradually (not a sharp jump).
        // Trigger dual-ICP recovery on the next timer callback.
        ++consecutive_reject_count_;
        if (consecutive_reject_count_ >= kMaxConsecutiveRejects && !lio_yaw_jump_detected_) {
          lio_yaw_jump_detected_ = true;
          lio_jump_detect_time_ = now();
          RCLCPP_WARN(this->get_logger(),
              "Consecutive sanity rejects (%d) → triggering dual-ICP recovery",
              consecutive_reject_count_);
        }
        return;
      }
    }

    // Accept: reset consecutive reject counter and update last known good base pose
    consecutive_reject_count_ = 0;

    // Absolute yaw divergence check: compare accepted ICP yaw against wheel odom.
    // This catches gradual LIO drift where per-frame yaw_delta is small but
    // cumulative drift is large (the sanity check above can't catch this).
    {
      double wheel_yaw_now;
      bool wheel_valid_now;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_yaw_now = wheel_odom_yaw_;
        wheel_valid_now = wheel_odom_valid_;
      }

      if (wheel_valid_now) {
        // current offset = accepted_base_yaw - wheel_yaw (normalized to [-pi, pi])
        double cur_offset = new_base_yaw - wheel_yaw_now;
        if (cur_offset > M_PI) cur_offset -= 2.0 * M_PI;
        if (cur_offset < -M_PI) cur_offset += 2.0 * M_PI;

        if (!icp_wheel_yaw_offset_valid_ && localization_healthy_.load()) {
          icp_wheel_yaw_offset_ = cur_offset;
          icp_wheel_yaw_offset_valid_ = true;
          RCLCPP_INFO(this->get_logger(),
              "ICP-wheel yaw offset calibrated: %.1f°",
              cur_offset * 180.0 / M_PI);
        } else if (icp_wheel_yaw_offset_valid_) {
          // How far has the offset drifted from calibration?
          double offset_drift = cur_offset - icp_wheel_yaw_offset_;
          if (offset_drift > M_PI) offset_drift -= 2.0 * M_PI;
          if (offset_drift < -M_PI) offset_drift += 2.0 * M_PI;

          if (std::abs(offset_drift) > icp_wheel_abs_yaw_thresh_) {
            // ICP result has drifted far from wheel odom baseline — reject and trigger recovery
            RCLCPP_WARN(this->get_logger(),
                "Absolute yaw divergence DETECTED: ICP-wheel offset drift = %.1f° "
                "(calib=%.1f°, cur=%.1f°, thresh=%.0f°) — triggering dual-ICP recovery",
                offset_drift * 180.0 / M_PI,
                icp_wheel_yaw_offset_ * 180.0 / M_PI,
                cur_offset * 180.0 / M_PI,
                icp_wheel_abs_yaw_thresh_ * 180.0 / M_PI);
            lio_yaw_jump_detected_ = true;
            lio_jump_detect_time_ = now();
            publishLocalizationHold("absolute_yaw_divergence_detected");
            registerAlignmentFailure("absolute_yaw_divergence_detected", true);
            // Do NOT update last_accepted_base — this result is suspect
            return;
          }

          // Do not let an unverified recovery candidate move the wheel-yaw
          // baseline.  It becomes eligible only after the staged candidate
          // sequence has been committed below.
          if (localization_healthy_.load()) {
            icp_wheel_yaw_offset_ += icp_wheel_yaw_offset_alpha_ * offset_drift;
            // Normalize
            if (icp_wheel_yaw_offset_ > M_PI) icp_wheel_yaw_offset_ -= 2.0 * M_PI;
            if (icp_wheel_yaw_offset_ < -M_PI) icp_wheel_yaw_offset_ += 2.0 * M_PI;
          }
        }
      }
    }

    // When localization is unhealthy, never publish each raw ICP candidate.
    // The safety gate has already stopped the chassis, so keep the last-good
    // map->odom transform fixed and compare candidates with one another.  This
    // prevents a low-score but wrong local minimum from moving the Nav2 pose
    // every second and then becoming the seed for the following ICP pass.
    if (!localization_healthy_.load()) {
      const bool had_recovery_candidate = recovery_candidate_valid_;
      double staged_pos_delta = accepted_pos_delta;
      double staged_yaw_delta = accepted_yaw_delta;
      if (had_recovery_candidate) {
        staged_pos_delta = std::hypot(
            new_base_x - recovery_candidate_base_x_,
            new_base_y - recovery_candidate_base_y_);
        staged_yaw_delta = std::abs(normalizeAngle(
            new_base_yaw - recovery_candidate_base_yaw_));
      }

      const bool quality_ok = continuous_quality_ok_;
      const bool candidate_consistent = quality_ok &&
          staged_pos_delta <= localization_stable_pos_thresh_ &&
          staged_yaw_delta <= localization_stable_yaw_thresh_;

      // A repeatable ICP local minimum is not necessarily the correct map
      // pose.  Compare it with wheel odometry propagated from the last
      // accepted map pose.  This is an innovation gate: wheel odometry is not
      // allowed to move map->odom, but it can veto the 0.3-0.6 m lateral ICP
      // shifts observed in nav_20260721_100003 while preserving real turns.
      bool wheel_gate_valid = false;
      double wheel_gate_pos_delta = std::numeric_limits<double>::infinity();
      double wheel_gate_yaw_delta = std::numeric_limits<double>::infinity();
      double wheel_x = 0.0;
      double wheel_y = 0.0;
      double wheel_yaw = 0.0;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_x = wheel_odom_x_;
        wheel_y = wheel_odom_y_;
        wheel_yaw = wheel_odom_yaw_;
        wheel_gate_valid = wheel_odom_valid_ && last_accepted_wheel_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      }
      if (wheel_gate_valid) {
        const auto planar_pose = [](double x, double y, double yaw) {
          Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
          const double c = std::cos(yaw);
          const double s = std::sin(yaw);
          pose(0, 0) = c;
          pose(0, 1) = -s;
          pose(1, 0) = s;
          pose(1, 1) = c;
          pose(0, 3) = x;
          pose(1, 3) = y;
          return pose;
        };
        const Eigen::Matrix4d predicted =
            planar_pose(last_accepted_base_x_, last_accepted_base_y_,
                        last_accepted_base_yaw_) *
            planar_pose(last_accepted_wheel_x_, last_accepted_wheel_y_,
                        last_accepted_wheel_yaw_).inverse() *
            planar_pose(wheel_x, wheel_y, wheel_yaw);
        wheel_gate_pos_delta = std::hypot(
            new_base_x - predicted(0, 3), new_base_y - predicted(1, 3));
        const double predicted_yaw = std::atan2(predicted(1, 0), predicted(0, 0));
        wheel_gate_yaw_delta = std::abs(
            normalizeAngle(new_base_yaw - predicted_yaw));
      }
      const bool wheel_gate_ok = wheel_gate_valid &&
          wheel_gate_pos_delta <= recovery_wheel_position_gate_ &&
          wheel_gate_yaw_delta <= recovery_wheel_yaw_gate_;
      const bool candidate_ok = recovery_guard::acceptCandidateSample(
          had_recovery_candidate, recovery_stability_window_.sampleCount(),
          quality_ok, candidate_consistent, wheel_gate_ok);
      if (!had_recovery_candidate) {
        recovery_stability_window_.reset();
      }
      recovery_stability_window_.add(candidate_ok);
      localization_stable_count_ = static_cast<int>(
          recovery_stability_window_.goodCount());
      if (candidate_ok) {
        continuous_failure_count_ = 0;
        // Keep the most recent verified pose as the consistency baseline.
        // A weak p50 frame must not drag the baseline away from the stable
        // cluster used by the following recovery frame.
        recovery_candidate_base_x_ = new_base_x;
        recovery_candidate_base_y_ = new_base_y;
        recovery_candidate_base_yaw_ = new_base_yaw;
        recovery_candidate_valid_ = true;
      }

      RCLCPP_INFO(this->get_logger(),
          "Localization staged candidate: %d/%d good in %zu/%d samples "
          "(delta=%.3fm, %.2fdeg, inliers=%.3f, rmse=%.3fm, "
          "p50=%.3fm, all_mse=%.4f); keeping last-good TF",
          localization_stable_count_, localization_stable_samples_,
          recovery_stability_window_.sampleCount(),
          localization_stability_window_size_,
          staged_pos_delta, staged_yaw_delta * 180.0 / M_PI,
          continuous_quality_.inlier_ratio,
          continuous_quality_.inlier_rmse,
          continuous_quality_.p50_distance, score_);
      RCLCPP_INFO(this->get_logger(),
          "Recovery wheel gate: valid=%d pos_delta=%.3fm/%.3f "
          "yaw_delta=%.2fdeg/%.2fdeg",
          wheel_gate_valid ? 1 : 0, wheel_gate_pos_delta,
          recovery_wheel_position_gate_, wheel_gate_yaw_delta * 180.0 / M_PI,
          recovery_wheel_yaw_gate_ * 180.0 / M_PI);

      if (!candidate_ok) {
        registerAlignmentFailure(
            "recovery_candidate_gate_reject", false, true);
        return;
      }

      if (!recovery_stability_window_.ready()) {
        return;
      }

      // The robot is held at zero and the candidate has been independently
      // reproduced for several scans.  Commit it before reopening the safety
      // gate so Nav2 never controls against a partially interpolated pose.
      last_accepted_base_x_ = new_base_x;
      last_accepted_base_y_ = new_base_y;
      last_accepted_base_yaw_ = new_base_yaw;
      last_accepted_base_stamp_ = now();
      last_accepted_base_valid_ = true;
      recovery_candidate_valid_ = false;
      recovery_stability_window_.reset();
      consecutive_reject_count_ = 0;

      double wheel_yaw_now = 0.0;
      bool wheel_valid_now = false;
      {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        wheel_yaw_now = wheel_odom_yaw_;
        wheel_valid_now = wheel_odom_valid_ &&
            ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
      }
      if (wheel_valid_now) {
        icp_wheel_yaw_offset_ = normalizeAngle(new_base_yaw - wheel_yaw_now);
        icp_wheel_yaw_offset_valid_ = true;
      }
      if (wheel_gate_valid) {
        std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
        last_accepted_wheel_x_ = wheel_odom_x_;
        last_accepted_wheel_y_ = wheel_odom_y_;
        last_accepted_wheel_yaw_ = wheel_odom_yaw_;
        last_accepted_wheel_valid_ = true;
      }

      updateMapToOdom(result, /*force_snap=*/true);
      local_recovery_active_ = false;
      global_recovery_active_ = false;
      pure_rotation_anchor_valid_ = false;
      pure_rotation_anchor_lio_wheel_divergence_valid_ = false;
      pure_rotation_weak_quality_hold_active_ = false;
      localization_loss_time_valid_ = false;
      publishLocalizationState(true, "continuous_icp_staged_candidate_committed");
      RCLCPP_INFO(this->get_logger(),
          "Committed verified ICP recovery pose: x=%.2f y=%.2f yaw=%.1f deg",
          new_base_x, new_base_y, new_base_yaw * 180.0 / M_PI);
      return;
    }

    last_accepted_base_x_ = new_base_x;
    last_accepted_base_y_ = new_base_y;
    last_accepted_base_yaw_ = new_base_yaw;
    last_accepted_base_stamp_ = now();
    last_accepted_base_valid_ = true;
    {
      std::lock_guard<std::mutex> lock(wheel_odom_mutex_);
      last_accepted_wheel_x_ = wheel_odom_x_;
      last_accepted_wheel_y_ = wheel_odom_y_;
      last_accepted_wheel_yaw_ = wheel_odom_yaw_;
      last_accepted_wheel_valid_ = wheel_odom_valid_ &&
          ((now() - wheel_odom_stamp_).seconds() <= wheel_odom_pose_timeout_);
    }

    continuous_failure_count_ = 0;
  }

  if (!skip_sc) {
    // Any global search started without an explicit operator prior is allowed
    // to stage a candidate only when its refined ICP score is substantially
    // better than normal navigation scores.  This prevents a repeated
    // corridor/local minimum from being accepted merely because ICP
    // converged.  The existing continuous four-sample + wheel-motion gates
    // remain authoritative before /localization_ok becomes true.

    // A stable low-score minimum is not necessarily globally correct in
    // repeated shelves.  If a spatially independent second minimum is tied
    // with the winner, keep motion held and require a better observation or an
    // explicit operator prior.
    if (!explicit_initial_pose && reject_ambiguous_global_candidates_ &&
        global_grid_ambiguous_) {
      RCLCPP_WARN(this->get_logger(),
                  "Guarded global grid ICP rejected unresolved candidate "
                  "ambiguity (best/second spatially independent minima are "
                  "within %.4f)",
                  global_grid_score_margin_);
      registerAlignmentFailure("global_grid_candidate_ambiguity", true);
      return;
    }

    if (!explicit_initial_pose && score_ > global_grid_score_thresh_) {
      RCLCPP_WARN(this->get_logger(),
                  "Guarded global grid ICP rejected unverified result%s: "
                  "score=%.4f > %.4f",
                  sc_match_ambiguous_ ? " (SC ambiguous)" : "",
                  score_, global_grid_score_thresh_);
      registerAlignmentFailure(
          sc_match_ambiguous_ ? "sc_ambiguous_grid_score" :
                                "global_grid_score_reject",
          true);
      return;
    }
    (void)stageGlobalCandidate(result, scan_stamp,
                               "grid_icp_candidate_stabilizing");
    return;
  }

  updateMapToOdom(result);
  const double accepted_base_x = map_to_laser(0, 3);
  const double accepted_base_y = map_to_laser(1, 3);
  const double accepted_base_yaw =
      std::atan2(map_to_laser(1, 0), map_to_laser(0, 0));
  RCLCPP_INFO(this->get_logger(),
              "ICP accepted map->%s pose: x=%.2f y=%.2f yaw=%.1f deg score=%.4f",
              laser_frame_id_.c_str(), accepted_base_x, accepted_base_y,
              accepted_base_yaw * 180.0 / M_PI, score_);
  RCLCPP_INFO(this->get_logger(),
              "ICP ready, publishing TF %s -> %s",
              map_frame_id_.c_str(), odom_frame_id_.c_str());
}

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d &init_guess,
                                        bool use_tight_prior) {
  static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = std::asin(std::clamp(-rot(2, 0), -1.0, 1.0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;
  global_grid_ambiguous_ = false;
  std::size_t removed_points = 0;
  source = sanitizeCloud<pcl::PointXYZI>(source, &removed_points);
  if (source->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(),
                "Skipping ICP: only %zu valid source points available",
                source->size());
    return Eigen::Matrix4d::Zero();
  }

  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
  Eigen::Vector3d rpy = rotate2rpy(rotation);
  if (constrain_to_2d_) {
    xyz.z() = 0.0;
    rpy.x() = 0.0;
    rpy.y() = 0.0;
  }
  Eigen::AngleAxisf rollAngle(rpy(0), Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(rpy(1), Eigen::Vector3f::UnitY());
  std::vector<Eigen::Matrix4f> candidates;
  Eigen::Matrix4f temp_pose;

  RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

  const double manual_xy_search_radius = std::max(
      0.0, get_parameter("manual_pose_xy_search_radius").as_double());
  const double manual_yaw_search_range = std::max(
      0.0, get_parameter("manual_pose_yaw_search_range").as_double());
  const int grid_half_cells = use_tight_prior
      ? std::max(0, static_cast<int>(std::ceil(
          manual_xy_search_radius / std::max(0.01, xy_offset_))))
      : std::max<int>(
          0, static_cast<int>(get_parameter("grid_half_cells").as_int()));
  const double candidate_xy_step = use_tight_prior && grid_half_cells > 0
      ? manual_xy_search_radius / grid_half_cells
      : xy_offset_;
  const double candidate_yaw_offset = use_tight_prior
      ? manual_yaw_search_range
      : yaw_offset_;
  const double candidate_yaw_resolution = use_tight_prior
      ? std::max(0.05, std::min(yaw_resolution_, manual_yaw_search_range))
      : yaw_resolution_;
  RCLCPP_INFO(this->get_logger(),
              "Grid prior mode=%s xy_radius=%.2fm yaw_range=%.1fdeg",
              use_tight_prior ? "manual_tight" : "global",
              grid_half_cells * candidate_xy_step,
              candidate_yaw_offset * 180.0 / M_PI);
  for (int i = -grid_half_cells; i <= grid_half_cells; i++) {
    for (int j = -grid_half_cells; j <= grid_half_cells; j++) {
      for (double yaw_delta = -candidate_yaw_offset;
           yaw_delta <= candidate_yaw_offset + 1e-6;
           yaw_delta += candidate_yaw_resolution) {
        Eigen::Vector3f pos(xyz(0) + i * candidate_xy_step,
                            xyz(1) + j * candidate_xy_step,
                            xyz(2));
        Eigen::AngleAxisf yawAngle(rpy(2) + yaw_delta,
                                   Eigen::Vector3f::UnitZ());
        temp_pose.setIdentity();
        temp_pose.block<3, 3>(0, 0) =
            (rollAngle * pitchAngle * yawAngle).toRotationMatrix();
        temp_pose.block<3, 1>(0, 3) = pos;
        candidates.push_back(temp_pose);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
      new pcl::PointCloud<pcl::PointXYZI>);

  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);

  rough_source = sanitizeCloud<pcl::PointXYZI>(rough_source, &removed_points);
  refine_source = sanitizeCloud<pcl::PointXYZI>(refine_source, &removed_points);

  // Crop maps around initial guess position (+ grid search extent)
  double effective_crop_radius = map_crop_radius_;
  if (effective_crop_radius > 0.0) {
    effective_crop_radius += grid_half_cells * candidate_xy_step;
  }
  PointCloudXYZI::Ptr local_rough_map = cropMapByPosition(rough_map_, xyz, effective_crop_radius);
  PointCloudXYZI::Ptr local_refine_map = cropMapByPosition(refine_map_, xyz, effective_crop_radius);
  if (local_rough_map->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(),
        "Cropped rough map too small (%zu pts within %.1fm), using full map",
        local_rough_map->size(), effective_crop_radius);
    local_rough_map = rough_map_;
    local_refine_map = refine_map_;
  }
  RCLCPP_INFO(this->get_logger(),
      "Map crop: rough %zu->%zu, refine %zu->%zu (radius=%.1fm around [%.1f,%.1f])",
      rough_map_->size(), local_rough_map->size(),
      refine_map_->size(), local_refine_map->size(),
      effective_crop_radius, xyz.x(), xyz.y());

  // Clip source cloud height to match target map z-range
  {
    Eigen::Vector4f tgt_min, tgt_max;
    pcl::getMinMax3D(*local_rough_map, tgt_min, tgt_max);
    float z_margin = 0.5f;
    float z_lo = tgt_min[2] - z_margin;
    float z_hi = tgt_max[2] + z_margin;
    auto height_filter = [z_lo, z_hi](const PointCloudXYZI::Ptr &c) {
      PointCloudXYZI::Ptr filtered(new PointCloudXYZI);
      filtered->reserve(c->size());
      for (const auto &p : c->points) {
        if (p.z >= z_lo && p.z <= z_hi) filtered->push_back(p);
      }
      return filtered;
    };
    rough_source = height_filter(rough_source);
    refine_source = height_filter(refine_source);
  }

  if (constrain_to_2d_) {
    if (!preserve_3d_structure_for_planar_icp_) {
      rough_source = projectCloudToPlane(rough_source);
      refine_source = projectCloudToPlane(refine_source);
      local_rough_map = projectCloudToPlane(local_rough_map);
      local_refine_map = projectCloudToPlane(local_refine_map);
    }
    icp_rough_.setTransformationEstimation(
        std::make_shared<pcl::registration::TransformationEstimation2D<
            PointType, PointType, float>>());
    icp_refine_.setTransformationEstimation(
        std::make_shared<pcl::registration::TransformationEstimation2D<
            PointType, PointType, float>>());
  }

  if (rough_source->size() < kMinAlignmentPoints ||
      refine_source->size() < kMinAlignmentPoints) {
    RCLCPP_WARN(this->get_logger(),
                "Skipping ICP: insufficient valid voxelized points (rough=%zu, refine=%zu)",
                rough_source->size(), refine_source->size());
    return Eigen::Matrix4d::Zero();
  }

  PointCloudXYZI::Ptr align_point(new PointCloudXYZI);

  // Diagnostic: source/target stats for grid search
  {
    Eigen::Vector4f src_min, src_max, tgt_min, tgt_max;
    pcl::getMinMax3D(*rough_source, src_min, src_max);
    pcl::getMinMax3D(*local_rough_map, tgt_min, tgt_max);
    RCLCPP_INFO(this->get_logger(),
                "Grid-search diag: source(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f], "
                "target(%zu pts) bbox=[%.1f,%.1f,%.1f]-[%.1f,%.1f,%.1f]",
                rough_source->size(),
                src_min[0], src_min[1], src_min[2], src_max[0], src_max[1], src_max[2],
                local_rough_map->size(),
                tgt_min[0], tgt_min[1], tgt_min[2], tgt_max[0], tgt_max[1], tgt_max[2]);
  }

  Eigen::Matrix4f best_rough_transform;
  double best_rough_score = 10.0;
  bool rough_converge = false;
  int total_converged = 0;
  int total_evaluated = 0;
  double worst_score = 0.0;
  std::vector<std::pair<double, Eigen::Matrix4f>> rough_results;
  rough_results.reserve(candidates.size());
  const double grid_search_timeout_sec = std::max(
      0.1, get_parameter("grid_search_timeout_sec").as_double());
  bool search_timed_out = false;
  auto tic = std::chrono::system_clock::now();
  icp_rough_.setMaxCorrespondenceDistance(
      global_icp_max_correspondence_distance_);
  icp_refine_.setMaxCorrespondenceDistance(
      global_icp_max_correspondence_distance_);
  icp_rough_.setInputTarget(local_rough_map);
  for (Eigen::Matrix4f &init_pose : candidates) {
    if (manual_alignment_cancel_requested_.load()) {
      RCLCPP_INFO(this->get_logger(),
                  "Cancelling automatic grid search for queued /initialpose "
                  "(%d/%zu candidates evaluated)",
                  total_evaluated, candidates.size());
      return Eigen::Matrix4d::Zero();
    }
    // Timeout: abort grid-search if it takes too long
    auto elapsed = std::chrono::duration<double>(
        std::chrono::system_clock::now() - tic).count();
    if (elapsed > grid_search_timeout_sec) {
      RCLCPP_WARN(this->get_logger(),
                  "Grid-search timeout after %.1fs (%d/%zu candidates evaluated)",
                  elapsed, total_evaluated, candidates.size());
      search_timed_out = true;
      break;
    }
    total_evaluated++;
    icp_rough_.setInputSource(rough_source);
    icp_rough_.align(*align_point, init_pose);
    if (!icp_rough_.hasConverged())
      continue;
    double rough_score = icp_rough_.getFitnessScore();
    total_converged++;
    if (rough_score > worst_score)
      worst_score = rough_score;
    if (rough_score > 2 * thresh_)
      continue;
    rough_results.emplace_back(
        rough_score, icp_rough_.getFinalTransformation());
    if (rough_score < best_rough_score) {
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Grid-search done: %zu candidates, %d converged, best_rough=%.4f, worst=%.4f, thresh=%.2f",
              candidates.size(), total_converged, best_rough_score, worst_score, thresh_);

  // A partial search has no known optimum and must never be treated as a
  // successful global localization.  The caller will retry with a fresh scan.
  if (search_timed_out || !rough_converge)
    return Eigen::Matrix4d::Zero();

  // A low ICP score alone is not sufficient in repeated shelves/corridors.
  // Keep the best rough minimum, then compare it with the best spatially
  // independent minimum.  If both are effectively tied, an automatic global
  // pose is unobservable and must remain held; an explicit /initialpose is
  // still allowed to use its tight prior.
  if (!use_tight_prior && rough_results.size() > 1) {
    std::sort(rough_results.begin(), rough_results.end(),
              [](const auto &lhs, const auto &rhs) {
                return lhs.first < rhs.first;
              });
    const Eigen::Matrix4f &best_transform = rough_results.front().second;
    double second_distinct_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = 1; i < rough_results.size(); ++i) {
      const Eigen::Matrix4f &candidate_transform = rough_results[i].second;
      const double xy_distance = std::hypot(
          static_cast<double>(candidate_transform(0, 3) - best_transform(0, 3)),
          static_cast<double>(candidate_transform(1, 3) - best_transform(1, 3)));
      if (xy_distance >= global_grid_candidate_separation_) {
        second_distinct_score = rough_results[i].first;
        break;
      }
    }
    if (std::isfinite(second_distinct_score) &&
        second_distinct_score - rough_results.front().first <
            global_grid_score_margin_) {
      global_grid_ambiguous_ = true;
      RCLCPP_WARN(this->get_logger(),
                  "Global grid minima remain ambiguous: best=%.4f, "
                  "second_distinct=%.4f, margin=%.4f < %.4f",
                  rough_results.front().first, second_distinct_score,
                  second_distinct_score - rough_results.front().first,
                  global_grid_score_margin_);
    }
  }

  icp_refine_.setInputSource(refine_source);
  icp_refine_.setInputTarget(local_refine_map);
  icp_refine_.align(*align_point, best_rough_transform);
  score_ = icp_refine_.getFitnessScore();

  RCLCPP_INFO(this->get_logger(),
              "Grid-search refine: converged=%d, score=%.4f (thresh=%.2f)",
              icp_refine_.hasConverged(), score_, thresh_);

  if (!icp_refine_.hasConverged())
    return Eigen::Matrix4d::Zero();
  if (score_ > thresh_)
    return Eigen::Matrix4d::Zero();
  success_ = true;
  auto toc = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = toc - tic;
  RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);
  RCLCPP_INFO(this->get_logger(), "score: %f", score_);

  return icp_refine_.getFinalTransformation().cast<double>();
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
