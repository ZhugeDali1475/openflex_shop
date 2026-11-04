// Generate collision-free FK targets for a selectable MoveIt joint group.

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
std::vector<std::string> splitCsv(const std::string& value)
{
  std::vector<std::string> items;
  std::stringstream stream(value);
  std::string item;
  while (std::getline(stream, item, ','))
  {
    item.erase(std::remove_if(item.begin(), item.end(), [](const unsigned char character) { return std::isspace(character); }),
               item.end());
    if (!item.empty())
      items.push_back(item);
  }
  return items;
}

bool parseStepOverrides(const std::string& value, std::unordered_map<std::string, double>& overrides,
                        std::string& error)
{
  for (const auto& item : splitCsv(value))
  {
    const auto separator = item.find('=');
    if (separator == std::string::npos || separator == 0 || separator == item.size() - 1 ||
        item.find('=', separator + 1) != std::string::npos)
    {
      error = "expected name=step, got '" + item + "'";
      return false;
    }
    const auto name = item.substr(0, separator);
    try
    {
      std::size_t parsed_characters = 0;
      const double step = std::stod(item.substr(separator + 1), &parsed_characters);
      if (parsed_characters != item.size() - separator - 1 || !std::isfinite(step) || step <= 0.0)
      {
        error = "step for '" + name + "' must be finite and > 0";
        return false;
      }
      if (!overrides.emplace(name, step).second)
      {
        error = "duplicate step override for '" + name + "'";
        return false;
      }
    }
    catch (const std::exception&)
    {
      error = "invalid step for '" + name + "'";
      return false;
    }
  }
  return true;
}

bool parseFixedPositions(const std::string& value, std::unordered_map<std::string, double>& positions,
                         std::string& error)
{
  for (const auto& item : splitCsv(value))
  {
    const auto separator = item.find('=');
    if (separator == std::string::npos || separator == 0 || separator == item.size() - 1 ||
        item.find('=', separator + 1) != std::string::npos)
    {
      error = "expected name=position, got '" + item + "'";
      return false;
    }
    const auto name = item.substr(0, separator);
    try
    {
      std::size_t parsed_characters = 0;
      const double position = std::stod(item.substr(separator + 1), &parsed_characters);
      if (parsed_characters != item.size() - separator - 1 || !std::isfinite(position))
      {
        error = "position for '" + name + "' must be finite";
        return false;
      }
      if (!positions.emplace(name, position).second)
      {
        error = "duplicate fixed position for '" + name + "'";
        return false;
      }
    }
    catch (const std::exception&)
    {
      error = "invalid position for '" + name + "'";
      return false;
    }
  }
  return true;
}

bool parseSamplingRanges(const std::string& value,
                         std::unordered_map<std::string, std::pair<double, double>>& ranges,
                         std::string& error)
{
  for (const auto& item : splitCsv(value))
  {
    const auto separator = item.find('=');
    const auto colon = item.find(':', separator == std::string::npos ? 0 : separator + 1);
    if (separator == std::string::npos || colon == std::string::npos || separator == 0 || colon == separator + 1 ||
        colon == item.size() - 1 || item.find('=', separator + 1) != std::string::npos ||
        item.find(':', colon + 1) != std::string::npos)
    {
      error = "expected name=min:max, got '" + item + "'";
      return false;
    }
    try
    {
      std::size_t lower_characters = 0;
      std::size_t upper_characters = 0;
      const double lower = std::stod(item.substr(separator + 1, colon - separator - 1), &lower_characters);
      const double upper = std::stod(item.substr(colon + 1), &upper_characters);
      if (lower_characters != colon - separator - 1 || upper_characters != item.size() - colon - 1 ||
          !std::isfinite(lower) || !std::isfinite(upper) || lower >= upper)
      {
        error = "range for '" + item.substr(0, separator) + "' must satisfy finite min < max";
        return false;
      }
      if (!ranges.emplace(item.substr(0, separator), std::make_pair(lower, upper)).second)
      {
        error = "duplicate range for '" + item.substr(0, separator) + "'";
        return false;
      }
    }
    catch (const std::exception&)
    {
      error = "invalid range for '" + item.substr(0, separator) + "'";
      return false;
    }
  }
  return true;
}

void writeStringArray(std::ostream& output, const std::vector<std::string>& values)
{
  output << "[";
  for (std::size_t index = 0; index < values.size(); ++index)
    output << (index == 0 ? "" : ",") << "\"" << values[index] << "\"";
  output << "]";
}

class PoseDatasetGenerator : public rclcpp::Node
{
public:
  PoseDatasetGenerator() : Node("generate_pose_dataset")
  {
    declare_parameter<std::string>("output_path", "/tmp/openarmx_left_arm_poses.jsonl");
    declare_parameter<std::string>("group_name", "left_arm");
    declare_parameter<std::string>("tip_link", "openarmx_left_hand");
    declare_parameter<std::string>("frame_id", "odom");
    declare_parameter<std::string>("sampling_joint_names", "");
    declare_parameter<double>("joint_step_rad", M_PI / 3.0);
    declare_parameter<std::string>("sampling_joint_step_overrides", "");
    declare_parameter<std::string>("sampling_joint_extra_values", "");
    declare_parameter<std::string>("sampling_mode", "grid");
    declare_parameter<int>("target_count", 0);
    declare_parameter<int>("random_seed", 20260720);
    declare_parameter<int>("max_random_attempts", 200000);
    declare_parameter<std::string>("sampling_joint_position_ranges", "");
    declare_parameter<std::string>("fixed_joint_positions", "");
    declare_parameter<double>("joint_limit_margin", 0.02);
    declare_parameter<int>("max_combinations", 200000);
    declare_parameter<bool>("report_first_collision", false);
  }

  int run()
  {
    const auto output_path = get_parameter("output_path").as_string();
    group_name_ = get_parameter("group_name").as_string();
    tip_link_ = get_parameter("tip_link").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    const auto requested_sampling_variables = splitCsv(get_parameter("sampling_joint_names").as_string());
    const auto sampling_mode = get_parameter("sampling_mode").as_string();
    const int target_count = get_parameter("target_count").as_int();
    const int random_seed = get_parameter("random_seed").as_int();
    const int max_random_attempts = get_parameter("max_random_attempts").as_int();
    const double joint_step = get_parameter("joint_step_rad").as_double();
    std::unordered_map<std::string, double> step_overrides;
    std::string override_error;
    if (!parseStepOverrides(get_parameter("sampling_joint_step_overrides").as_string(), step_overrides, override_error))
    {
      RCLCPP_ERROR(get_logger(), "Invalid sampling_joint_step_overrides: %s", override_error.c_str());
      return 2;
    }
    std::unordered_map<std::string, double> extra_sampling_values;
    std::string extra_value_error;
    if (!parseFixedPositions(get_parameter("sampling_joint_extra_values").as_string(), extra_sampling_values,
                             extra_value_error))
    {
      RCLCPP_ERROR(get_logger(), "Invalid sampling_joint_extra_values: %s", extra_value_error.c_str());
      return 2;
    }
    std::unordered_map<std::string, std::pair<double, double>> range_overrides;
    std::string range_error;
    if (!parseSamplingRanges(get_parameter("sampling_joint_position_ranges").as_string(), range_overrides,
                             range_error))
    {
      RCLCPP_ERROR(get_logger(), "Invalid sampling_joint_position_ranges: %s", range_error.c_str());
      return 2;
    }
    std::string position_error;
    if (!parseFixedPositions(get_parameter("fixed_joint_positions").as_string(), fixed_joint_positions_, position_error))
    {
      RCLCPP_ERROR(get_logger(), "Invalid fixed_joint_positions: %s", position_error.c_str());
      return 2;
    }
    const double limit_margin = get_parameter("joint_limit_margin").as_double();
    const int max_combinations = get_parameter("max_combinations").as_int();
    report_first_collision_ = get_parameter("report_first_collision").as_bool();
    if (frame_id_ != "odom" || joint_step <= 0.0 || limit_margin < 0.0 || limit_margin >= 0.5 || max_combinations <= 0 ||
        (sampling_mode != "grid" && sampling_mode != "random") ||
        (sampling_mode == "random" && (target_count <= 0 || max_random_attempts < target_count)))
    {
      RCLCPP_ERROR(get_logger(), "Invalid sampling configuration");
      return 2;
    }

    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const auto model = loader.getModel();
    if (!model)
    {
      RCLCPP_ERROR(get_logger(), "Failed to load RobotModel from robot_description and robot_description_semantic");
      return 2;
    }
    const auto* group = model->getJointModelGroup(group_name_);
    if (!group || !model->hasLinkModel(tip_link_))
    {
      RCLCPP_ERROR(get_logger(), "Unknown group '%s' or tip link '%s'", group_name_.c_str(), tip_link_.c_str());
      return 2;
    }

    group_variable_names_ = group->getVariableNames();
    sampling_variable_names_ = requested_sampling_variables.empty() ? group_variable_names_ : requested_sampling_variables;
    sampling_steps_.clear();
    sampling_ranges_.clear();
    sampling_steps_.reserve(sampling_variable_names_.size());
    for (const auto& variable_name : sampling_variable_names_)
    {
      if (std::find(group_variable_names_.begin(), group_variable_names_.end(), variable_name) == group_variable_names_.end())
      {
        RCLCPP_ERROR(get_logger(), "Sampling variable '%s' is not part of group '%s'", variable_name.c_str(),
                     group_name_.c_str());
        return 2;
      }
      const auto& bounds = model->getVariableBounds(variable_name);
      double lower = 0.0;
      double upper = 0.0;
      const auto range_override = range_overrides.find(variable_name);
      if (bounds.position_bounded_ && std::isfinite(bounds.min_position_) && std::isfinite(bounds.max_position_))
      {
        const double range = bounds.max_position_ - bounds.min_position_;
        lower = bounds.min_position_ + limit_margin * range;
        upper = bounds.max_position_ - limit_margin * range;
      }
      else if (range_override != range_overrides.end())
      {
        lower = range_override->second.first;
        upper = range_override->second.second;
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Sampling variable '%s' needs finite bounds or an explicit range", variable_name.c_str());
        return 2;
      }
      const auto override = step_overrides.find(variable_name);
      sampling_steps_.push_back(override == step_overrides.end() ? joint_step : override->second);
      sampling_ranges_.emplace_back(lower, upper);
    }
    for (const auto& [variable_name, step] : step_overrides)
    {
      (void)step;
      if (std::find(sampling_variable_names_.begin(), sampling_variable_names_.end(), variable_name) ==
          sampling_variable_names_.end())
      {
        RCLCPP_ERROR(get_logger(), "Step override variable '%s' is not sampled", variable_name.c_str());
        return 2;
      }
    }
    for (const auto& [variable_name, position] : fixed_joint_positions_)
    {
      const auto* joint = model->getJointOfVariable(variable_name);
      if (!joint)
      {
        RCLCPP_ERROR(get_logger(), "Fixed-position variable '%s' is unknown", variable_name.c_str());
        return 2;
      }
      if (std::find(sampling_variable_names_.begin(), sampling_variable_names_.end(), variable_name) !=
          sampling_variable_names_.end())
      {
        RCLCPP_ERROR(get_logger(), "Fixed-position variable '%s' is also sampled", variable_name.c_str());
        return 2;
      }
      const auto& bounds = model->getVariableBounds(variable_name);
      if (bounds.position_bounded_ && (position < bounds.min_position_ || position > bounds.max_position_))
      {
        RCLCPP_ERROR(get_logger(), "Fixed position %.6f for '%s' is outside [%.6f, %.6f]", position,
                     variable_name.c_str(), bounds.min_position_, bounds.max_position_);
        return 2;
      }
    }
    for (const auto& [variable_name, position] : extra_sampling_values)
    {
      if (std::find(sampling_variable_names_.begin(), sampling_variable_names_.end(), variable_name) ==
          sampling_variable_names_.end())
      {
        RCLCPP_ERROR(get_logger(), "Extra sampling value variable '%s' is not sampled", variable_name.c_str());
        return 2;
      }
      const auto& bounds = model->getVariableBounds(variable_name);
      if (bounds.position_bounded_ && (position < bounds.min_position_ || position > bounds.max_position_))
      {
        RCLCPP_ERROR(get_logger(), "Extra sampling value %.6f for '%s' is outside [%.6f, %.6f]", position,
                     variable_name.c_str(), bounds.min_position_, bounds.max_position_);
        return 2;
      }
    }

    for (const auto& [variable_name, range] : range_overrides)
    {
      (void)range;
      if (std::find(sampling_variable_names_.begin(), sampling_variable_names_.end(), variable_name) ==
          sampling_variable_names_.end())
      {
        RCLCPP_ERROR(get_logger(), "Range override variable '%s' is not sampled", variable_name.c_str());
        return 2;
      }
    }

    if (sampling_mode == "random")
      return generateRandom(model, group, target_count, random_seed, max_random_attempts);

    std::vector<std::vector<double>> samples_per_variable;
    samples_per_variable.reserve(sampling_variable_names_.size());
    std::size_t combinations = 1;
    for (std::size_t variable_index = 0; variable_index < sampling_variable_names_.size(); ++variable_index)
    {
      const auto& variable_name = sampling_variable_names_[variable_index];
      const double step = sampling_steps_[variable_index];
      const double lower = sampling_ranges_[variable_index].first;
      const double upper = sampling_ranges_[variable_index].second;
      std::vector<double> values;
      const auto canonicalize = [](const double value) { return std::abs(value) < 1e-12 ? 0.0 : value; };
      for (double value = lower; value < upper - 0.5 * step; value += step)
        values.push_back(canonicalize(value));
      values.push_back(canonicalize(upper));
      const auto extra_value = extra_sampling_values.find(variable_name);
      if (extra_value != extra_sampling_values.end())
        values.push_back(canonicalize(extra_value->second));
      std::sort(values.begin(), values.end());
      values.erase(std::unique(values.begin(), values.end()), values.end());
      if (combinations > static_cast<std::size_t>(max_combinations) / values.size())
      {
        RCLCPP_ERROR(get_logger(), "Grid would exceed max_combinations=%d while adding '%s'", max_combinations,
                     variable_name.c_str());
        return 2;
      }
      combinations *= values.size();
      samples_per_variable.push_back(std::move(values));
    }

    std::ofstream output(output_path, std::ios::out | std::ios::trunc);
    if (!output)
    {
      RCLCPP_ERROR(get_logger(), "Cannot open '%s' for writing", output_path.c_str());
      return 2;
    }
    output << std::setprecision(17);
    output << "{\"type\":\"metadata\",\"schema_version\":2,\"group\":\"" << group_name_
           << "\",\"tip_link\":\"" << tip_link_ << "\",\"frame_id\":\"" << frame_id_ << "\",\"group_joint_names\":";
    writeStringArray(output, group_variable_names_);
    output << ",\"sampled_joint_names\":";
    writeStringArray(output, sampling_variable_names_);
    output << ",\"joint_step_rad\":" << joint_step << ",\"sampling_joint_steps\":{";
    for (std::size_t index = 0; index < sampling_variable_names_.size(); ++index)
      output << (index == 0 ? "" : ",") << "\"" << sampling_variable_names_[index] << "\":" << sampling_steps_[index];
    output << "},\"joint_limit_margin\":" << limit_margin
           << ",\"sampling_joint_extra_values\":{";
    std::vector<std::string> extra_names;
    extra_names.reserve(extra_sampling_values.size());
    for (const auto& [name, position] : extra_sampling_values)
    {
      (void)position;
      extra_names.push_back(name);
    }
    std::sort(extra_names.begin(), extra_names.end());
    for (std::size_t index = 0; index < extra_names.size(); ++index)
    {
      const auto& name = extra_names[index];
      output << (index == 0 ? "" : ",") << "\"" << name << "\":" << extra_sampling_values.at(name);
    }
    output << "}"
           << ",\"fixed_joint_positions\":{";
    std::vector<std::string> fixed_names;
    fixed_names.reserve(fixed_joint_positions_.size());
    for (const auto& [name, position] : fixed_joint_positions_)
    {
      (void)position;
      fixed_names.push_back(name);
    }
    std::sort(fixed_names.begin(), fixed_names.end());
    for (std::size_t index = 0; index < fixed_names.size(); ++index)
    {
      const auto& name = fixed_names[index];
      output << (index == 0 ? "" : ",") << "\"" << name << "\":" << fixed_joint_positions_.at(name);
    }
    output << "}"
           << ",\"total_combinations\":" << combinations << "}\n";

    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    std::vector<double> sampled_positions(sampling_variable_names_.size(), 0.0);
    collision_detection::CollisionRequest collision_request;
    // Match MoveIt's per-planning-group validity check: include this group's
    // interaction with the robot, but ignore unrelated static link pairs.
    collision_request.group_name = group_name_;
    collision_request.contacts = report_first_collision_;
    collision_request.max_contacts = report_first_collision_ ? 32 : 0;
    collision_request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult collision_result;
    generate(0, samples_per_variable, group, state, scene, collision_request, collision_result, sampled_positions, output);
    RCLCPP_INFO(get_logger(), "Generated %zu collision-free targets for %s from %zu combinations; rejected %zu self-collisions: %s",
                valid_count_, group_name_.c_str(), combinations, collision_count_, output_path.c_str());
    return 0;
  }

private:
  int generateRandom(const moveit::core::RobotModelConstPtr& model, const moveit::core::JointModelGroup* group,
                     int target_count, int random_seed, int max_random_attempts)
  {
    std::ofstream output(get_parameter("output_path").as_string(), std::ios::out | std::ios::trunc);
    if (!output)
    {
      RCLCPP_ERROR(get_logger(), "Cannot open random dataset output");
      return 2;
    }
    output << std::setprecision(17);
    output << "{\"type\":\"metadata\",\"schema_version\":2,\"group\":\"" << group_name_
           << "\",\"tip_link\":\"" << tip_link_ << "\",\"frame_id\":\"" << frame_id_
           << "\",\"group_joint_names\":";
    writeStringArray(output, group_variable_names_);
    output << ",\"sampled_joint_names\":";
    writeStringArray(output, sampling_variable_names_);
    output << ",\"sampling_mode\":\"random\",\"random_seed\":" << random_seed
           << ",\"target_count_requested\":" << target_count << ",\"fixed_joint_positions\":{}"
           << "}\n";

    planning_scene::PlanningScene scene(model);
    moveit::core::RobotState state(model);
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group_name_;
    collision_request.contacts = report_first_collision_;
    collision_request.max_contacts = report_first_collision_ ? 32 : 0;
    collision_request.max_contacts_per_pair = 1;
    collision_detection::CollisionResult collision_result;
    std::mt19937_64 random_generator(static_cast<std::uint64_t>(random_seed));
    std::vector<std::uniform_real_distribution<double>> distributions;
    distributions.reserve(sampling_ranges_.size());
    for (const auto& range : sampling_ranges_)
      distributions.emplace_back(range.first, range.second);

    for (int attempt = 0; attempt < max_random_attempts && valid_count_ < static_cast<std::size_t>(target_count);
         ++attempt)
    {
      state.setToDefaultValues();
      for (const auto& [variable_name, position] : fixed_joint_positions_)
        state.setVariablePosition(variable_name, position);
      std::vector<double> sampled_positions;
      sampled_positions.reserve(sampling_variable_names_.size());
      for (std::size_t index = 0; index < sampling_variable_names_.size(); ++index)
      {
        const double value = distributions[index](random_generator);
        sampled_positions.push_back(value);
        state.setVariablePosition(sampling_variable_names_[index], value);
      }
      state.update();
      collision_result.clear();
      scene.checkSelfCollision(collision_request, collision_result, state);
      if (collision_result.collision)
      {
        ++collision_count_;
        continue;
      }

      std::vector<double> group_positions;
      state.copyJointGroupPositions(group, group_positions);
      const Eigen::Isometry3d& transform = state.getGlobalLinkTransform(tip_link_);
      Eigen::Quaterniond orientation(transform.rotation());
      orientation.normalize();
      output << "{\"type\":\"target\",\"index\":" << valid_count_ << ",\"joint_positions\":[";
      for (std::size_t index = 0; index < group_positions.size(); ++index)
        output << (index == 0 ? "" : ",") << group_positions[index];
      output << "],\"sampled_joint_positions\":[";
      for (std::size_t index = 0; index < sampled_positions.size(); ++index)
        output << (index == 0 ? "" : ",") << sampled_positions[index];
      output << "],\"position\":[" << transform.translation().x() << "," << transform.translation().y() << ","
             << transform.translation().z() << "],\"orientation_xyzw\":[" << orientation.x() << ","
             << orientation.y() << "," << orientation.z() << "," << orientation.w() << "]}\n";
      ++valid_count_;
    }
    if (valid_count_ != static_cast<std::size_t>(target_count))
    {
      RCLCPP_ERROR(get_logger(), "Generated only %zu/%d collision-free random targets after %d attempts", valid_count_,
                   target_count, max_random_attempts);
      return 2;
    }
    RCLCPP_INFO(get_logger(), "Generated %zu collision-free random targets for %s; rejected %zu self-collisions",
                valid_count_, group_name_.c_str(), collision_count_);
    return 0;
  }

  void generate(std::size_t variable_index, const std::vector<std::vector<double>>& samples_per_variable,
                const moveit::core::JointModelGroup* group, moveit::core::RobotState& state,
                const planning_scene::PlanningScene& scene, const collision_detection::CollisionRequest& collision_request,
                collision_detection::CollisionResult& collision_result, std::vector<double>& sampled_positions,
                std::ofstream& output)
  {
    if (variable_index < samples_per_variable.size())
    {
      for (const double value : samples_per_variable[variable_index])
      {
        sampled_positions[variable_index] = value;
        generate(variable_index + 1, samples_per_variable, group, state, scene, collision_request, collision_result,
                 sampled_positions, output);
      }
      return;
    }

    state.setToDefaultValues();
    for (const auto& [variable_name, position] : fixed_joint_positions_)
      state.setVariablePosition(variable_name, position);
    for (std::size_t index = 0; index < sampling_variable_names_.size(); ++index)
      state.setVariablePosition(sampling_variable_names_[index], sampled_positions[index]);
    state.update();
    collision_result.clear();
    scene.checkSelfCollision(collision_request, collision_result, state);
    if (collision_result.collision)
    {
      ++collision_count_;
      if (report_first_collision_ && !collision_reported_)
      {
        std::ostringstream pairs;
        for (const auto& [pair, contacts] : collision_result.contacts)
        {
          (void)contacts;
          if (pairs.tellp() > 0)
            pairs << ", ";
          pairs << pair.first << " <-> " << pair.second;
        }
        RCLCPP_WARN(get_logger(), "First self-collision for sampled state: %s", pairs.str().c_str());
        collision_reported_ = true;
      }
      return;
    }

    std::vector<double> group_positions;
    state.copyJointGroupPositions(group, group_positions);
    const Eigen::Isometry3d& transform = state.getGlobalLinkTransform(tip_link_);
    Eigen::Quaterniond orientation(transform.rotation());
    orientation.normalize();
    output << "{\"type\":\"target\",\"index\":" << valid_count_ << ",\"joint_positions\":[";
    for (std::size_t index = 0; index < group_positions.size(); ++index)
      output << (index == 0 ? "" : ",") << group_positions[index];
    output << "],\"sampled_joint_positions\":[";
    for (std::size_t index = 0; index < sampled_positions.size(); ++index)
      output << (index == 0 ? "" : ",") << sampled_positions[index];
    output << "],\"position\":[" << transform.translation().x() << "," << transform.translation().y() << ","
           << transform.translation().z() << "],\"orientation_xyzw\":[" << orientation.x() << "," << orientation.y()
           << "," << orientation.z() << "," << orientation.w() << "]}\n";
    ++valid_count_;
  }

  std::string group_name_;
  std::string tip_link_;
  std::string frame_id_;
  std::vector<std::string> group_variable_names_;
  std::vector<std::string> sampling_variable_names_;
  std::vector<double> sampling_steps_;
  std::vector<std::pair<double, double>> sampling_ranges_;
  std::unordered_map<std::string, double> fixed_joint_positions_;
  std::size_t valid_count_ = 0;
  std::size_t collision_count_ = 0;
  bool report_first_collision_ = false;
  bool collision_reported_ = false;
};
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<PoseDatasetGenerator>();
  const int result = node->run();
  rclcpp::shutdown();
  return result;
}
