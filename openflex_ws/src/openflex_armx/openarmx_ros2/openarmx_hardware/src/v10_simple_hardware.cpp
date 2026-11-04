// Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
//
// Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
// https://www.openarmx.com
//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
// 4.0 International License (CC BY-NC-SA 4.0).
//
// To view a copy of this license, visit:
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

#include "openarmx_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <openarmx/robstride_motor/rs_motor_control.hpp>

#include "openarmx_hardware/gripper_feedback.hpp"

namespace openarmx_hardware {

// Helper function to create MIT mode packet (equivalent to csp_set_mode_packet)
// MIT mode is MOTION_CONTROL mode (value 0) in RUN_MODE parameter
static inline openarmx::robstride_motor::CANPacket mit_set_mode_packet(
    const openarmx::robstride_motor::Motor& motor) {
  // RUN_MODE = 0x7005, 0 = MOTION_CONTROL (MIT mode)
  return openarmx::robstride_motor::CanPacketEncoder::create_write_param_command(
      motor, 0x7005, 0.0f, openarmx::robstride_motor::ParamValueType::UINT8);
}

OpenArmX_v10HW::OpenArmX_v10HW() = default;

OpenArmX_v10HW::~OpenArmX_v10HW() {
  // Stop parameter node executor thread
  if (param_spin_thread_active_) {
    param_spin_thread_active_ = false;
    if (param_executor_) {
      param_executor_->cancel();
    }
    if (param_spin_thread_.joinable()) {
      param_spin_thread_.join();
    }
  }
}

bool OpenArmX_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: false per user request)
  it = info.hardware_parameters.find("can_fd");
  std::string raw_can_fd = (it != info.hardware_parameters.end()) ? it->second : std::string("<unset>");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = false;  // Default to false now
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  const auto parse_control_mode = [](std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    return value == "csp" ? ControlMode::CSP : ControlMode::MIT;
  };

  // Keep control_mode as the arm mode. An omitted gripper mode inherits it so
  // existing robot descriptions retain their original all-MIT/all-CSP behavior.
  it = info.hardware_parameters.find("control_mode");
  control_mode_ = parse_control_mode(
      it != info.hardware_parameters.end() ? it->second : "mit");
  it = info.hardware_parameters.find("gripper_control_mode");
  if (it == info.hardware_parameters.end()) {
    gripper_control_mode_ = control_mode_;
  } else {
    gripper_control_mode_ = parse_control_mode(it->second);
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Raw can_fd param: %s", raw_can_fd.c_str());

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s, "
              "arm_control_mode=%s, gripper_control_mode=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled",
              (control_mode_ == ControlMode::MIT ? "mit" : "csp"),
              (gripper_control_mode_ == ControlMode::MIT ? "mit" : "csp"));
  return true;
}

void OpenArmX_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: openarmx_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "openarmx_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "openarmx_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize ROS2 node for dynamic parameters
  std::string node_name = "openarmx_" + arm_prefix_ + "hardware_params";
  param_node_ = std::make_shared<rclcpp::Node>(node_name);

  rcl_interfaces::msg::ParameterDescriptor arm_mode_descriptor;
  arm_mode_descriptor.description =
      "Runtime arm mode. Switching between csp and mit briefly disables arm motors.";
  param_node_->declare_parameter(
      "arm_control_mode",
      control_mode_ == ControlMode::MIT ? "mit" : "csp",
      arm_mode_descriptor);
  rcl_interfaces::msg::ParameterDescriptor gripper_mode_descriptor;
  gripper_mode_descriptor.read_only = true;
  param_node_->declare_parameter(
      "gripper_control_mode",
      gripper_control_mode_ == ControlMode::MIT ? "mit" : "csp",
      gripper_mode_descriptor);

  // Initialize KP and KD values with defaults
  // 注意：第8个值是夹爪，增大KP/KD可提高响应速度和阻尼
  kp_values_ = {50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 10.0, 50.0};  // 夹爪KP从10.0改为50.0
  kd_values_ = {2.5, 2.5, 2.5, 2.5, 0.5, 0.5, 0.5, 2.5};          // 夹爪KD从0.5改为2.5（与大关节相同）

  // Declare ROS2 parameters for KP and KD
  for (size_t i = 0; i < kp_values_.size(); ++i) {
    std::string kp_param_name = "kp_joint" + std::to_string(i + 1);
    std::string kd_param_name = "kd_joint" + std::to_string(i + 1);

    param_node_->declare_parameter(kp_param_name, kp_values_[i]);
    param_node_->declare_parameter(kd_param_name, kd_values_[i]);

    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Declared parameter %s with default value %.2f",
                kp_param_name.c_str(), kp_values_[i]);
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Declared parameter %s with default value %.2f",
                kd_param_name.c_str(), kd_values_[i]);
  }

  // Register parameter callback for dynamic reconfiguration
  param_callback_handle_ = param_node_->add_on_set_parameters_callback(
      std::bind(&OpenArmX_v10HW::parameters_callback, this, std::placeholders::_1));

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Dynamic parameter reconfiguration enabled for KP and KD values");

  // Create executor and start spinning param_node_ in a separate thread
  param_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  param_executor_->add_node(param_node_);
  param_spin_thread_active_ = true;
  param_spin_thread_ = std::thread([this]() {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Parameter node executor thread started for node: %s",
                param_node_->get_name());
    while (param_spin_thread_active_ && rclcpp::ok()) {
      param_executor_->spin_some(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Parameter node executor thread stopped");
  });

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Parameter node %s is now spinning in background thread",
              param_node_->get_name());

  // Initialize OpenArmX with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Initializing OpenArmX on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  openarmx =
      std::make_unique<openarmx::can::socket::OpenArmX>(can_interface_, can_fd_);

  // Initialize arm motors with V10 defaults
  openarmx->init_arm_motors(DEFAULT_MOTOR_TYPES, DEFAULT_SEND_CAN_IDS,
                            DEFAULT_RECV_CAN_IDS);

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "Initializing gripper...");
    openarmx->init_gripper_motor(DEFAULT_GRIPPER_MOTOR_TYPE,
                                 DEFAULT_GRIPPER_SEND_CAN_ID,
                                 DEFAULT_GRIPPER_RECV_CAN_ID);
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "OpenArmX V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  openarmx->refresh_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarmx->recv_all();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
OpenArmX_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
OpenArmX_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "Activating OpenArmX V10...");
  openarmx->set_callback_mode_all(openarmx::robstride_motor::CallbackMode::STATE);

  // Enable a short debug window to verify raw motor readings vs. published joint states
  debug_cycles_remaining_ = 50;  // ~first 50 read() calls

  using namespace openarmx::robstride_motor;
  auto& sock = openarmx->get_master_can_device_collection().get_can_socket();
  auto arm_motors = openarmx->get_arm().get_all_motors();
  auto arm_devices = openarmx->get_arm().get_all_devices();

  openarmx->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Configuring %zu arm motors for %s mode", arm_motors.size(),
              control_mode_ == ControlMode::MIT ? "MIT" : "CSP");
  for (size_t i = 0; i < arm_motors.size() && i < arm_devices.size(); ++i) {
    if (control_mode_ == ControlMode::MIT) {
      configure_motor_mit_mode(arm_motors[i], arm_devices[i], sock);
    } else {
      configure_motor_csp_mode(arm_motors[i], arm_devices[i], sock);
    }
  }

  if (hand_) {
    auto gripper_motors = openarmx->get_gripper().get_all_motors();
    auto gripper_devices = openarmx->get_gripper().get_all_devices();
    for (size_t i = 0;
         i < gripper_motors.size() && i < gripper_devices.size(); ++i) {
      if (gripper_control_mode_ == ControlMode::MIT) {
        configure_motor_mit_mode(gripper_motors[i], gripper_devices[i], sock);
      } else {
        configure_motor_csp_mode(gripper_motors[i], gripper_devices[i], sock);
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                "Configured %zu gripper motors for %s mode",
                gripper_motors.size(),
                gripper_control_mode_ == ControlMode::MIT ? "MIT" : "CSP");
  }

  openarmx->enable_all();
  const bool any_mit = control_mode_ == ControlMode::MIT ||
                       (hand_ && gripper_control_mode_ == ControlMode::MIT);
  std::this_thread::sleep_for(
      std::chrono::milliseconds(any_mit ? 1000 : 50));
  openarmx->refresh_all();
  openarmx->recv_all();

  // Initialize commands from measured positions so activation never commands zero.
  const auto direction_multipliers = get_motor_direction_multipliers();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    pos_commands_[i] = arm_motors[i]->get_position() * direction_multipliers[i];
  }
  if (hand_) {
    auto gripper_motors = openarmx->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      const double raw_motor_pos = gripper_motors[0]->get_position();
      const auto feedback = validate_gripper_motor_feedback(raw_motor_pos);
      if (!feedback.valid || gripper_motors[0]->has_error()) {
        RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                     "[%s] Refusing activation with unsafe gripper state: "
                     "position=%.6f rad enabled=%s error=%s error_code=%u",
                     arm_prefix_.c_str(), raw_motor_pos,
                     gripper_motors[0]->is_enabled() ? "true" : "false",
                     gripper_motors[0]->has_error() ? "true" : "false",
                     static_cast<unsigned>(gripper_motors[0]->get_error_code()));
        openarmx->disable_all();
        return CallbackReturn::ERROR;
      }
      if (feedback.clamped) {
        RCLCPP_WARN(rclcpp::get_logger("OpenArmX_v10HW"),
                    "[%s] Clamping gripper activation feedback from %.6f "
                    "to %.6f rad",
                    arm_prefix_.c_str(), raw_motor_pos, feedback.radians);
      }
      pos_commands_[ARM_DOF] = motor_radians_to_joint(feedback.radians);
      pos_states_[ARM_DOF] = pos_commands_[ARM_DOF];
      gripper_last_pos_state_ = pos_commands_[ARM_DOF];
      gripper_stall_counter_ = 0;
      gripper_stall_active_ = false;
      gripper_feedback_fault_latched_ = false;
      gripper_stall_locked_pos_ = pos_commands_[ARM_DOF];
      gripper_stall_hold_command_ = pos_commands_[ARM_DOF];
      gripper_last_valid_motor_torque_ = gripper_motors[0]->get_torque();
      gripper_last_valid_motor_temperature_ =
          gripper_motors[0]->get_temperature();
    }
  }

  hardware_active_ = true;
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "OpenArmX V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn OpenArmX_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  hardware_active_ = false;
  std::lock_guard<std::mutex> io_lock(hardware_io_mutex_);
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Deactivating OpenArmX V10...");

  // Disable all motors (like full_arm.cpp exit)
  openarmx->disable_all();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  openarmx->recv_all();

  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"), "OpenArmX V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type OpenArmX_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> io_lock(hardware_io_mutex_);
  // Receive all motor states
  openarmx->refresh_all();
  openarmx->recv_all();

  // Read arm joint states - FIXED: Now using vector order with direction correction
  // joint1 -> motor[0] (CAN ID 1), joint7 -> motor[6] (CAN ID 7)
  auto arm_motors = openarmx->get_arm().get_motors();
  const auto direction_multipliers = get_motor_direction_multipliers();
  for (size_t i = 0; i < ARM_DOF && i < arm_motors.size(); ++i) {
    const double raw_pos = arm_motors[i]->get_position();
    const double raw_vel = arm_motors[i]->get_velocity();
    const double raw_tau = arm_motors[i]->get_torque();
    const double sign = direction_multipliers[i];
    pos_states_[i] = raw_pos * sign;
    vel_states_[i] = raw_vel * sign;
    tau_states_[i] = raw_tau * sign;

    // Transient debug logging to diagnose unexpected offsets/signs
    if (debug_cycles_remaining_ > 0 && i == 0) {
      RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                  "[%s] joint%zu raw=%.6f rad, sign=%.1f, published=%.6f (can_id=%u)",
                  arm_prefix_.c_str(), i + 1, raw_pos, sign, pos_states_[i],
                  static_cast<unsigned>(arm_motors[i]->get_send_can_id()));
    }
  }

  if (debug_cycles_remaining_ > 0) {
    --debug_cycles_remaining_;
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    auto gripper_motors = openarmx->get_gripper().get_motors();
    if (!gripper_motors.empty()) {
      // TODO the mappings are approximates
      // Convert motor position (radians) to joint value (0-0.044m)
      auto* const gripper_motor = gripper_motors[0];
      const double raw_motor_pos = gripper_motor->get_position();
      const auto feedback = validate_gripper_motor_feedback(raw_motor_pos);
      if (!feedback.valid || gripper_motor->has_error()) {
        gripper_feedback_fault_latched_ = true;
        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("OpenArmX_v10HW"), *param_node_->get_clock(),
            2000,
            "[%s] Gripper motor fault; rejecting feedback: position=%.6f rad "
            "last_valid=%.4f m last_torque=%.3f Nm last_temp=%.1f C "
            "driver_enabled=%s driver_error=%s driver_error_code=%u",
            arm_prefix_.c_str(), raw_motor_pos, gripper_last_pos_state_,
            gripper_last_valid_motor_torque_,
            gripper_last_valid_motor_temperature_,
            gripper_motor->is_enabled() ? "true" : "false",
            gripper_motor->has_error() ? "true" : "false",
            static_cast<unsigned>(gripper_motor->get_error_code()));
      } else if (feedback.clamped) {
        RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("OpenArmX_v10HW"), *param_node_->get_clock(),
            2000,
            "[%s] Clamping plausible gripper overtravel feedback from %.6f "
            "to %.6f rad",
            arm_prefix_.c_str(), raw_motor_pos, feedback.radians);
      }
      if (gripper_feedback_fault_latched_) {
        // Keep the command-side last valid position, but publish an invalid
        // state so upper layers cannot mistake a faulted gripper for a hold.
        pos_states_[ARM_DOF] = std::numeric_limits<double>::quiet_NaN();
      } else {
        pos_states_[ARM_DOF] = motor_radians_to_joint(feedback.radians);
        gripper_last_valid_motor_torque_ = gripper_motor->get_torque();
        gripper_last_valid_motor_temperature_ = gripper_motor->get_temperature();
      }

      // Unimplemented: Velocity and torque mapping
      vel_states_[ARM_DOF] = 0;  // gripper_motors[0]->get_velocity();
      tau_states_[ARM_DOF] = 0;  // gripper_motors[0]->get_torque();
    }
  }

  return hardware_interface::return_type::OK;
}

void OpenArmX_v10HW::configure_motor_csp_mode(
    openarmx::robstride_motor::Motor* motor,
    openarmx::robstride_motor::RSCANDevice* device,
    openarmx::canbus::CANSocket& socket) {
  using namespace openarmx::robstride_motor;

  // 1) Switch to CSP mode
  auto mode_pkt = csp_set_mode_packet(*motor);
  auto mode_frame = device->create_can_frame(mode_pkt.send_can_id, mode_pkt.data);
  socket.write_can_frame(mode_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // 2) Get motor limits from motor type
  auto motor_type = motor->get_motor_type();
  size_t idx = static_cast<size_t>(motor_type);
  const auto& limits = MOTOR_LIMIT_PARAMS[idx];

  // 3) Set speed limit
  auto spd_pkt = csp_set_speed_limit_packet(*motor, static_cast<float>(0.5));
  auto spd_frame = device->create_can_frame(spd_pkt.send_can_id, spd_pkt.data);
  socket.write_can_frame(spd_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));

  // 4) Set current limit
  auto cur_pkt = csp_set_current_limit_packet(*motor, static_cast<float>(limits.tMax));
  auto cur_frame = device->create_can_frame(cur_pkt.send_can_id, cur_pkt.data);
  socket.write_can_frame(cur_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

void OpenArmX_v10HW::configure_motor_mit_mode(
    openarmx::robstride_motor::Motor* motor,
    openarmx::robstride_motor::RSCANDevice* device,
    openarmx::canbus::CANSocket& socket) {
  using namespace openarmx::robstride_motor;

  // Switch to MIT mode (motion control mode)
  auto mode_pkt = mit_set_mode_packet(*motor);
  auto mode_frame = device->create_can_frame(mode_pkt.send_can_id, mode_pkt.data);
  socket.write_can_frame(mode_frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

bool OpenArmX_v10HW::switch_arm_control_mode(
    ControlMode target_mode, std::string& reason) {
  if (!hardware_active_) {
    reason = "arm hardware is not active";
    return false;
  }
  std::lock_guard<std::mutex> io_lock(hardware_io_mutex_);
  if (!hardware_active_) {
    reason = "arm hardware became inactive during mode switch";
    return false;
  }
  if (target_mode == control_mode_) {
    reason = "arm control mode is already active";
    return true;
  }

  using namespace openarmx::robstride_motor;
  auto& arm = openarmx->get_arm();
  auto motors = arm.get_all_motors();
  auto devices = arm.get_all_devices();
  if (motors.size() < ARM_DOF || devices.size() < ARM_DOF) {
    reason = "arm motor/device list is incomplete";
    return false;
  }

  const auto direction_multipliers = get_motor_direction_multipliers();
  const ControlMode original_mode = control_mode_;
  std::vector<double> hold_positions(ARM_DOF, 0.0);
  for (size_t i = 0; i < ARM_DOF; ++i) {
    hold_positions[i] = pos_states_[i];
    pos_commands_[i] = hold_positions[i];
    vel_commands_[i] = 0.0;
  }

  auto& socket =
      openarmx->get_master_can_device_collection().get_can_socket();
  const auto configure_mode = [&](ControlMode mode) {
    for (size_t i = 0; i < ARM_DOF; ++i) {
      if (mode == ControlMode::MIT) {
        configure_motor_mit_mode(motors[i], devices[i], socket);
      } else {
        configure_motor_csp_mode(motors[i], devices[i], socket);
      }
    }
  };

  try {
    if (!arm.disable_all()) {
      arm.enable_all();
      reason = "failed to disable all arm motors before mode switch";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    configure_mode(target_mode);
    if (!arm.enable_all()) {
      configure_mode(original_mode);
      arm.enable_all();
      reason = "failed to re-enable all arm motors after mode switch";
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    control_mode_ = target_mode;
    if (target_mode == ControlMode::MIT) {
      std::vector<MotionControlParam> hold_commands(ARM_DOF);
      std::lock_guard<std::mutex> gain_lock(kp_kd_mutex_);
      for (size_t i = 0; i < ARM_DOF; ++i) {
        hold_commands[i].kp = kp_values_[i];
        hold_commands[i].kd = kd_values_[i];
        hold_commands[i].position =
            hold_positions[i] * direction_multipliers[i];
        hold_commands[i].velocity = 0.0;
        hold_commands[i].torque =
            tau_commands_[i] * direction_multipliers[i];
      }
      arm.send_motion_control_commands(hold_commands);
    } else {
      for (size_t i = 0; i < ARM_DOF; ++i) {
        const float target = static_cast<float>(
            hold_positions[i] * direction_multipliers[i]);
        auto packet = csp_set_target_position_packet(*motors[i], target);
        auto frame =
            devices[i]->create_can_frame(packet.send_can_id, packet.data);
        socket.write_can_frame(frame);
      }
    }
    openarmx->recv_all(1000);
  } catch (const std::exception& error) {
    try {
      configure_mode(original_mode);
      arm.enable_all();
      control_mode_ = original_mode;
    } catch (const std::exception& rollback_error) {
      reason = std::string("arm mode switch failed: ") + error.what() +
               "; rollback also failed: " + rollback_error.what();
      return false;
    }
    reason = std::string("arm mode switch failed and was rolled back: ") +
             error.what();
    return false;
  }

  reason = target_mode == ControlMode::MIT
               ? "arm switched to mit"
               : "arm switched to csp";
  RCLCPP_WARN(rclcpp::get_logger("OpenArmX_v10HW"),
              "[%s] %s", arm_prefix_.c_str(), reason.c_str());
  return true;
}

hardware_interface::return_type OpenArmX_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  std::lock_guard<std::mutex> io_lock(hardware_io_mutex_);
  const auto direction_multipliers = get_motor_direction_multipliers();
  auto& socket =
      openarmx->get_master_can_device_collection().get_can_socket();
  if (control_mode_ == ControlMode::MIT) {
    std::vector<openarmx::robstride_motor::MotionControlParam> arm_params(ARM_DOF);
    std::lock_guard<std::mutex> lock(kp_kd_mutex_);
    for (size_t i = 0; i < ARM_DOF; ++i) {
      openarmx::robstride_motor::MotionControlParam param;
      param.kp = kp_values_[i];
      param.kd = kd_values_[i];
      param.position = pos_commands_[i] * direction_multipliers[i];
      param.velocity = vel_commands_[i] * direction_multipliers[i];
      param.torque = tau_commands_[i] * direction_multipliers[i];
      arm_params[i] = param;
    }
    openarmx->get_arm().send_motion_control_commands(arm_params);
  } else {
    using namespace openarmx::robstride_motor;
    auto& arm = openarmx->get_arm();
    auto motors = arm.get_all_motors();
    auto devices = arm.get_all_devices();
    for (size_t i = 0;
         i < ARM_DOF && i < motors.size() && i < devices.size(); ++i) {
      float target = static_cast<float>(pos_commands_[i] * direction_multipliers[i]);
      auto pkt = csp_set_target_position_packet(*motors[i], target);
      auto frame = devices[i]->create_can_frame(pkt.send_can_id, pkt.data);
      socket.write_can_frame(frame);
    }
  }

  if (hand_ && joint_names_.size() > ARM_DOF) {
    if (gripper_control_mode_ == ControlMode::MIT) {
      std::lock_guard<std::mutex> lock(kp_kd_mutex_);
      const bool gripper_state_valid =
          std::isfinite(pos_states_[ARM_DOF]) &&
          pos_states_[ARM_DOF] >=
              GRIPPER_JOINT_1_POSITION - GRIPPER_STATE_VALID_TOLERANCE &&
          pos_states_[ARM_DOF] <=
              GRIPPER_JOINT_0_POSITION + GRIPPER_STATE_VALID_TOLERANCE;
      // Keep a small bounded preload after contact. The lower hold gain avoids
      // sustaining the full close gain against an immovable object.
      if (!gripper_state_valid) {
        gripper_stall_counter_ = 0;
        const double safe_hold_position = gripper_stall_active_
                                              ? gripper_stall_hold_command_
                                              : gripper_last_pos_state_;
        pos_commands_[ARM_DOF] =
            std::clamp(safe_hold_position, GRIPPER_JOINT_1_POSITION,
                       GRIPPER_JOINT_0_POSITION);
        RCLCPP_ERROR_THROTTLE(
            rclcpp::get_logger("OpenArmX_v10HW"), *param_node_->get_clock(), 2000,
            "[%s] Invalid gripper feedback %.4f m; holding safe command %.4f m",
            arm_prefix_.c_str(), pos_states_[ARM_DOF],
            pos_commands_[ARM_DOF]);
      } else if (gripper_stall_active_) {
        const double cmd_delta =
            pos_commands_[ARM_DOF] - gripper_stall_locked_pos_;
        if (cmd_delta > GRIPPER_STALL_RELEASE_THRESHOLD) {
          gripper_stall_active_ = false;
          gripper_stall_counter_ = 0;
          RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                      "[%s] Gripper stall released, new cmd=%.4f",
                      arm_prefix_.c_str(), pos_commands_[ARM_DOF]);
        } else {
          pos_commands_[ARM_DOF] = gripper_stall_hold_command_;
          RCLCPP_INFO_THROTTLE(rclcpp::get_logger("OpenArmX_v10HW"),
                               *param_node_->get_clock(), 2000,
                               "[%s] Gripper stall holding: contact=%.4f cmd=%.4f",
                               arm_prefix_.c_str(), gripper_stall_locked_pos_,
                               gripper_stall_hold_command_);
        }
      } else {
        const double pos_error =
            std::abs(pos_commands_[ARM_DOF] - pos_states_[ARM_DOF]);
        const double pos_change =
            std::abs(pos_states_[ARM_DOF] - gripper_last_pos_state_);
        const bool closing_against_contact =
            pos_commands_[ARM_DOF] <
            pos_states_[ARM_DOF] - GRIPPER_STALL_POS_ERROR_THRESHOLD;
        if (closing_against_contact &&
            pos_error > GRIPPER_STALL_POS_ERROR_THRESHOLD &&
            pos_change < GRIPPER_STALL_POS_CHANGE_THRESHOLD) {
          ++gripper_stall_counter_;
          if (gripper_stall_counter_ >= GRIPPER_STALL_CYCLES) {
            gripper_stall_active_ = true;
            gripper_stall_locked_pos_ = pos_states_[ARM_DOF];
            gripper_stall_hold_command_ = std::max(
                GRIPPER_JOINT_1_POSITION,
                gripper_stall_locked_pos_ - GRIPPER_STALL_PRELOAD);
            pos_commands_[ARM_DOF] = gripper_stall_hold_command_;
            RCLCPP_INFO(
                rclcpp::get_logger("OpenArmX_v10HW"),
                "[%s] Gripper contact detected: contact=%.4f hold_cmd=%.4f "
                "hold_kp=%.1f",
                arm_prefix_.c_str(), gripper_stall_locked_pos_,
                gripper_stall_hold_command_,
                std::min(kp_values_[ARM_DOF], GRIPPER_STALL_HOLD_KP));
          }
        } else {
          gripper_stall_counter_ = 0;
        }
      }
      if (gripper_state_valid) {
        gripper_last_pos_state_ = pos_states_[ARM_DOF];
      }

      openarmx::robstride_motor::MotionControlParam gripper_param;
      const double configured_kp = std::isfinite(kp_values_[ARM_DOF])
                                       ? std::clamp(kp_values_[ARM_DOF], 0.0, 500.0)
                                       : 0.0;
      const double configured_kd = std::isfinite(kd_values_[ARM_DOF])
                                       ? std::clamp(kd_values_[ARM_DOF], 0.0, 5.0)
                                       : 0.0;
      const bool use_low_hold_gain =
          gripper_stall_active_ || gripper_feedback_fault_latched_;
      gripper_param.kp = use_low_hold_gain
                             ? std::min(configured_kp, GRIPPER_STALL_HOLD_KP)
                             : configured_kp;
      gripper_param.kd = configured_kd;
      gripper_param.position =
          joint_to_motor_radians(pos_commands_[ARM_DOF]);
      gripper_param.velocity = 0.0;
      gripper_param.torque = 0.0;
      openarmx->get_gripper().send_motion_control_commands({gripper_param});
    } else {
      using namespace openarmx::robstride_motor;
      auto g_motors = openarmx->get_gripper().get_all_motors();
      auto g_devices = openarmx->get_gripper().get_all_devices();
      if (!g_motors.empty() && !g_devices.empty()) {
        const float motor_command = static_cast<float>(
            joint_to_motor_radians(pos_commands_[ARM_DOF]));
        auto pkt = csp_set_target_position_packet(*g_motors[0], motor_command);
        auto frame = g_devices[0]->create_can_frame(pkt.send_can_id, pkt.data);
        socket.write_can_frame(frame);
      }
    }
  }

  openarmx->recv_all(1000);
  return hardware_interface::return_type::OK;
}

void OpenArmX_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
              "Returning to zero position...");

  // Lock to safely read KP/KD values
  std::lock_guard<std::mutex> lock(kp_kd_mutex_);

  // Return arm to zero with motion control - use hardware order for zero position
  std::vector<openarmx::robstride_motor::MotionControlParam> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    openarmx::robstride_motor::MotionControlParam param;
    param.kp = kp_values_[i];
    param.kd = kd_values_[i];
    param.position = 0.0;
    param.velocity = 0.0;
    param.torque = 0.0;
    arm_params.push_back(param);
  }
  openarmx->get_arm().send_motion_control_commands(arm_params);

  // Return gripper to zero if enabled
  if (hand_) {
    openarmx::robstride_motor::MotionControlParam gripper_param;
    gripper_param.kp = kp_values_[ARM_DOF];
    gripper_param.kd = kd_values_[ARM_DOF];
    gripper_param.position = GRIPPER_JOINT_0_POSITION;
    gripper_param.velocity = 0.0;
    gripper_param.torque = 0.0;
    openarmx->get_gripper().send_motion_control_commands({gripper_param});
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  openarmx->recv_all();
}

// Gripper mapping helper functions
double OpenArmX_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double OpenArmX_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

// Motor direction correction based on URDF axis definitions and Python reference
std::vector<double> OpenArmX_v10HW::get_motor_direction_multipliers() const {
  return std::vector<double>{-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
}

// Dynamic parameter callback implementation
rcl_interfaces::msg::SetParametersResult OpenArmX_v10HW::parameters_callback(
    const std::vector<rclcpp::Parameter>& parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto& param : parameters) {
    std::string param_name = param.get_name();

    if (param_name == "arm_control_mode") {
      std::string value = param.as_string();
      std::transform(value.begin(), value.end(), value.begin(), ::tolower);
      if (value != "mit" && value != "csp") {
        result.successful = false;
        result.reason = "arm_control_mode must be 'mit' or 'csp'";
        return result;
      }
      std::string switch_reason;
      const ControlMode target_mode =
          value == "mit" ? ControlMode::MIT : ControlMode::CSP;
      if (!switch_arm_control_mode(target_mode, switch_reason)) {
        result.successful = false;
        result.reason = switch_reason;
        return result;
      }
      result.reason = switch_reason;
    }
    // Check if it's a KP parameter
    else if (param_name.find("kp_joint") == 0) {
      // Extract joint index from parameter name (kp_joint1 -> index 0)
      size_t joint_idx = std::stoi(param_name.substr(8)) - 1;

      if (joint_idx < kp_values_.size()) {
        std::lock_guard<std::mutex> lock(kp_kd_mutex_);
        double new_value = param.as_double();
        double old_value = kp_values_[joint_idx];
        kp_values_[joint_idx] = new_value;

        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                    "Updated %s: %.2f -> %.2f",
                    param_name.c_str(), old_value, new_value);
      } else {
        result.successful = false;
        result.reason = "Joint index out of range for " + param_name;
        RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                     "Failed to update %s: joint index %zu out of range",
                     param_name.c_str(), joint_idx);
      }
    }
    // Check if it's a KD parameter
    else if (param_name.find("kd_joint") == 0) {
      // Extract joint index from parameter name (kd_joint1 -> index 0)
      size_t joint_idx = std::stoi(param_name.substr(8)) - 1;

      if (joint_idx < kd_values_.size()) {
        std::lock_guard<std::mutex> lock(kp_kd_mutex_);
        double new_value = param.as_double();
        double old_value = kd_values_[joint_idx];
        kd_values_[joint_idx] = new_value;

        RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
                    "Updated %s: %.2f -> %.2f",
                    param_name.c_str(), old_value, new_value);
      } else {
        result.successful = false;
        result.reason = "Joint index out of range for " + param_name;
        RCLCPP_ERROR(rclcpp::get_logger("OpenArmX_v10HW"),
                     "Failed to update %s: joint index %zu out of range",
                     param_name.c_str(), joint_idx);
      }
    }
  }

  // Print current KP/KD values for all joints after any parameter change
  // if (result.successful && !parameters.empty()) {
  //   RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //               "========== Current KP/KD Values ==========");

  //   // Print arm joints (1-7)
  //   RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //               "Arm Joints:");
  //   for (size_t i = 0; i < ARM_DOF && i < kp_values_.size(); ++i) {
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //                 "  Joint %zu: KP=%.2f, KD=%.2f",
  //                 i + 1, kp_values_[i], kd_values_[i]);
  //   }

  //   // Print gripper (joint 8) if available
  //   if (kp_values_.size() > ARM_DOF) {
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //                 "Gripper:");
  //     RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //                 "  Joint 8 (Gripper): KP=%.2f, KD=%.2f",
  //                 kp_values_[ARM_DOF], kd_values_[ARM_DOF]);
  //   }

  //   RCLCPP_INFO(rclcpp::get_logger("OpenArmX_v10HW"),
  //               "==========================================");
  // }

  return result;
}

}  // namespace openarmx_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(openarmx_hardware::OpenArmX_v10HW,
                       hardware_interface::SystemInterface)
