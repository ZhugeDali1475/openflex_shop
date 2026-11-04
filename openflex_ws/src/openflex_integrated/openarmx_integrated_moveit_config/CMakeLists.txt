# Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International
#
# Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.
# https://www.openarmx.com
#
# This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike
# 4.0 International License (CC BY-NC-SA 4.0).
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.

cmake_minimum_required(VERSION 3.22)
project(openarmx_integrated_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(class_loader REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_simple_controller_manager REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tl_expected REQUIRED)
find_package(openarmx_integrated_moveit_msgs REQUIRED)

# Some MoveIt 2.5 packages export `tl::expected`, while the Humble
# `tl_expected` package provides `tl_expected::tl_expected`.
if(NOT TARGET tl::expected)
  add_library(tl::expected ALIAS tl_expected::tl_expected)
endif()

add_library(openflex_ik_pipeline SHARED
  src/openflex_ik_pipeline.cpp)
target_compile_features(openflex_ik_pipeline PUBLIC cxx_std_17)
target_include_directories(openflex_ik_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(openflex_ik_pipeline Eigen3 moveit_core)

add_library(openflex_kinematics SHARED
  src/openflex_kinematics_plugin.cpp)
target_compile_features(openflex_kinematics PUBLIC cxx_std_17)
target_include_directories(openflex_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(openflex_kinematics
  Eigen3
  class_loader
  moveit_core
  moveit_msgs
  pluginlib
  rclcpp)
target_link_libraries(openflex_kinematics openflex_ik_pipeline)

add_library(openarmx_moveit_controller_manager SHARED
  src/openarmx_moveit_controller_manager.cpp)
target_compile_features(openarmx_moveit_controller_manager PUBLIC cxx_std_17)
ament_target_dependencies(openarmx_moveit_controller_manager
  moveit_core
  moveit_simple_controller_manager
  nav2_msgs
  pluginlib
  rclcpp
  tf2_eigen)

add_executable(generate_pose_dataset
  src/generate_pose_dataset.cpp)
target_compile_features(generate_pose_dataset PUBLIC cxx_std_17)
ament_target_dependencies(generate_pose_dataset
  moveit_core
  moveit_ros_planning
  rclcpp)

add_executable(openflex_pick_task_executor
  src/openflex_pick_task_executor.cpp)
target_compile_features(openflex_pick_task_executor PUBLIC cxx_std_17)
target_include_directories(openflex_pick_task_executor PRIVATE include)
ament_target_dependencies(openflex_pick_task_executor
  Eigen3
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  nav2_msgs
  openarmx_integrated_moveit_msgs
  pluginlib
  rclcpp
  rclcpp_action
  sensor_msgs)
target_link_libraries(openflex_pick_task_executor openflex_ik_pipeline)

add_executable(openflex_joint_posture_executor
  src/openflex_joint_posture_executor.cpp)
target_compile_features(openflex_joint_posture_executor PUBLIC cxx_std_17)
ament_target_dependencies(openflex_joint_posture_executor
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  openarmx_integrated_moveit_msgs
  rclcpp
  rclcpp_action
  sensor_msgs)

pluginlib_export_plugin_description_file(
  moveit_core openflex_kinematics_plugin_description.xml)
install(TARGETS openflex_ik_pipeline openflex_kinematics
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS openarmx_moveit_controller_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS generate_pose_dataset
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS openflex_pick_task_executor openflex_joint_posture_executor
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(FILES
  openflex_kinematics_plugin_description.xml
  README.md
  THIRD_PARTY_NOTICES.md
  WHOLE_BODY_PLANNING_CN.md
  DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/moveit_command_bridge.py
    scripts/base_navigation_bridge.py
    scripts/ensure_controllers_ready.py
    scripts/switch_lift_controller.py
    scripts/wait_for_robot_state.py
    scripts/wait_for_moveit.py
  DESTINATION lib/${PROJECT_NAME})
