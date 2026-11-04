cmake_minimum_required(VERSION 3.8)
project(openarmx_battery_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
  include
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

set(PLUGIN_HEADERS
  include/openarmx_battery_monitor/battery_overlay_display.hpp
)

add_library(${PROJECT_NAME} SHARED
  src/battery_overlay_display.cpp
  ${PLUGIN_HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Widgets
)

ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rviz_common
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  scripts
  systemd
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  openarmx_battery_monitor/jd_battery_bus_diag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jd_battery_bus_diag
)

install(PROGRAMS
  openarmx_battery_monitor/jd_battery_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jd_battery_node
)

install(PROGRAMS
  openarmx_battery_monitor/jd_battery_multi_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jd_battery_multi_node
)

install(PROGRAMS
  openarmx_battery_monitor/jd_battery_master_dual_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME jd_battery_master_dual_node
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  pluginlib
  rviz_common
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
