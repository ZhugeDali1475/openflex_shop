cmake_minimum_required(VERSION 3.8)
project(openflex_thesis_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectObservation.msg"
  "msg/ObjectMemory.msg"
  "msg/AffordanceCandidate.msg"
  "msg/AffordanceCandidateArray.msg"
  "msg/AffordanceState.msg"
  "msg/GraspCandidate.msg"
  "msg/GraspCandidateArray.msg"
  "msg/AnyGraspRequest.msg"
  "msg/ReadyRegion.msg"
  "msg/ManipulationPlan.msg"
  "msg/ControlMode.msg"
  "msg/MissionState.msg"
  "msg/PolicyControl.msg"
  "msg/GraspStatus.msg"
  "msg/FixedPlaceControl.msg"
  "msg/PlaceStatus.msg"
  "action/EvaluateCandidates.action"
  "action/GenerateGrasps.action"
  "action/ComputeTcWam.action"
  "action/ExecuteLocalAlignment.action"
  "action/ExecuteMoveItGrasp.action"
  "action/ExecuteLocalPick.action"
  "action/ExecuteFixedPlace.action"
  "action/RunPickDeliver.action"
  "srv/SetControlMode.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
