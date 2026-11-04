cmake_minimum_required(VERSION 3.8)
project(swerve_navigation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # User-site pytest plugins can be newer than Ubuntu/ROS Humble's pytest and
  # make even `pytest --version` fail during CMake configure. This unit test has
  # no external plugin requirements, so keep discovery deterministic.
  set(ENV{PYTEST_DISABLE_PLUGIN_AUTOLOAD} "1")
  ament_add_pytest_test(
    pointcloud_qos_relay_test
    test/test_pointcloud_qos_relay.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    cmd_vel_safety_gate_core_test
    test/test_cmd_vel_safety_gate_core.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    navigation_launch_wiring_test
    test/test_navigation_launch_wiring.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
  ament_add_pytest_test(
    safe_odom_mux_freshness_test
    test/test_safe_odom_mux_freshness.py
    ENV "PYTEST_DISABLE_PLUGIN_AUTOLOAD=1"
  )
endif()

install(
  DIRECTORY launch config rviz scripts behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES
    scripts/cmd_vel_safety_gate_core.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/pointcloud_to_livox.py
    scripts/livox_to_pointcloud2.py
    scripts/record_gps_datum.py
    scripts/gps_initial_pose.py
    scripts/plan_only_goal.py
    scripts/pointcloud_qos_relay.py
    scripts/laser_scan_qos_relay.py
    scripts/local_window_marker.py
    scripts/safe_odom_mux.py
    scripts/cmd_vel_safety_gate.py
    scripts/localization_costmap_guard.py
    scripts/wait_for_localization_ready.py
    scripts/cmd_vel_replay_conditioner.py
    scripts/future_odom_pose_tracker.py
    scripts/vla_path_to_nav2_follow_path.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
