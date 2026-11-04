cmake_minimum_required(VERSION 3.8)
project(openarmx_task_msgs)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteTransport.action"
  "action/ExecuteCompetitionPhase.action"
  "action/DockToStation.action"
  "action/ShelfFineAlign.action"
  "action/ShelfTraverse.action"
  "action/RetreatFromStation.action"
  "action/DetectObject.action"
  "action/InspectRetailScene.action"
  "action/MoveLiftToLevel.action"
  "action/MoveToShelfView.action"
  "action/PickObject.action"
  "action/PlanPick.action"
  "action/ExecutePick.action"
  "action/PlaceObject.action"
  "action/ReadOrder.action"
  "action/SetPerceptionMode.action"
  "action/VerifyPlacement.action"
  "msg/CompetitionWorkItem.msg"
  "msg/OrderLine.msg"
  "msg/TaskError.msg"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
