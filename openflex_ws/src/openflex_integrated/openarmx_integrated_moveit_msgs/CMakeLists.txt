cmake_minimum_required(VERSION 3.22)
project(openarmx_integrated_moveit_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CartesianApproach.action"
  "action/ExecuteJointPosture.action"
  "action/PlanPregrasp.action"
  DEPENDENCIES geometry_msgs moveit_msgs sensor_msgs)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
