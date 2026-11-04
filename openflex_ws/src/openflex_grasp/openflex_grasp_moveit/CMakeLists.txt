cmake_minimum_required(VERSION 3.22)
project(openflex_grasp_moveit)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(openarmx_integrated_moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(cartesian_approach_executor src/cartesian_approach_executor.cpp)
target_compile_features(cartesian_approach_executor PUBLIC cxx_std_17)
ament_target_dependencies(cartesian_approach_executor
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  openarmx_integrated_moveit_msgs
  rclcpp
  rclcpp_action
  shape_msgs)
target_link_libraries(cartesian_approach_executor Eigen3::Eigen)

install(TARGETS cartesian_approach_executor DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} PATTERN "__pycache__" EXCLUDE PATTERN "*.pyc" EXCLUDE)

ament_package()
