import math
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import openflex_navigation_gui.competition_window as competition_window_module

from PyQt5.QtCore import QSettings, Qt
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QDialog, QWidget

from openflex_navigation_gui.benchmark import (
    BenchmarkAcceptance,
    BenchmarkProfile,
    BenchmarkWaypoint,
)
from openflex_navigation_gui.competition_mission import (
    load_mission,
    mission_path,
    save_mission,
)
from openflex_navigation_gui.competition_window import (
    CompetitionMainWindow,
    nav2_stack_readiness_problems,
)
from openflex_navigation_gui.main_window import NavigationRosNode, PathPlot, Telemetry
from openflex_navigation_gui.model import resolve_map_paths
from openflex_navigation_gui.shelf_anchor_calibration import ShelfAnchorCalibration


def _profile(map_root: Path) -> BenchmarkProfile:
    start = BenchmarkWaypoint(
        "START", "起点", 0.5, 0.5, 0.0, 0.2, "start_position"
    )
    acceptance = BenchmarkAcceptance(
        goal_position_m=0.05,
        goal_yaw_rad=math.radians(3.0),
        max_goal_position_m=0.1,
        max_goal_yaw_rad=math.radians(6.0),
        path_cross_track_m=0.1,
        max_icp_step_m=0.1,
        max_icp_step_yaw_rad=math.radians(5.0),
        stopped_linear_mps=0.02,
        stopped_angular_radps=0.02,
        stopped_hold_sec=0.5,
        minimum_success_rate=0.9,
        required_zero_collision_tours=1,
    )
    return BenchmarkProfile(
        profile_id="test",
        display_name="test",
        purpose="test",
        map_root=map_root,
        map_name="arena_a",
        launch_file="test.launch.py",
        initial_x=0.0,
        initial_y=0.0,
        initial_yaw=0.0,
        acceptance=acceptance,
        waypoints={"START": start},
        tours={},
    )


def _make_map(root: Path, name: str) -> None:
    paths = resolve_map_paths(root, name)
    paths.map_2d_dir.mkdir(parents=True)
    paths.map_3d_dir.mkdir(parents=True)
    paths.pgm.write_text("P2\n2 2\n255\n254 254\n254 254\n", encoding="ascii")
    paths.yaml.write_text(
        "image: map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n",
        encoding="ascii",
    )
    paths.pcd.write_text("# test\n", encoding="ascii")


def test_nav2_preflight_requires_internal_bt_action_servers():
    nodes = {
        "/behavior_server",
        "/bt_navigator",
        "/controller_server",
        "/planner_server",
        "/smoother_server",
        "/velocity_smoother",
    }
    actions = {
        "/compute_path_to_pose",
        "/follow_path",
        "/navigate_to_pose",
        "/smooth_path",
    }

    problems = nav2_stack_readiness_problems(nodes, actions)

    assert problems == ["安全等待恢复 Action 未就绪：/wait"]
    actions.add("/wait")
    assert nav2_stack_readiness_problems(nodes, actions) == []


def test_stale_runtime_process_check_detects_orphaned_grasp_stack(monkeypatch):
    monkeypatch.setattr(
        competition_window_module.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            stdout=(
                "101 /usr/bin/python3 unrelated_worker.py\n"
                "202 python3 -m openflex_grasp.detect_object_server --ros-args\n"
            )
        ),
    )

    stale = CompetitionMainWindow._stale_runtime_processes()

    assert stale == [
        "202 python3 -m openflex_grasp.detect_object_server --ros-args"
    ]


def test_stale_navigation_process_check_detects_lio_and_ignores_unrelated(
    monkeypatch,
):
    monkeypatch.setattr(
        competition_window_module.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(
            stdout=(
                "101 /usr/bin/python3 unrelated_worker.py\n"
                "202 /ws/install/fast_livo/lib/fast_livo/fastlivo_mapping "
                "--ros-args\n"
            )
        ),
    )

    stale = CompetitionMainWindow._stale_navigation_processes()

    assert stale == [
        "202 /ws/install/fast_livo/lib/fast_livo/fastlivo_mapping --ros-args"
    ]


def test_competition_flow_uses_test9_a_anchor_camera_geometry(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(_profile(map_root), map_name="supermarket_test_09")
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    flow = window.competition_flow_tab
    monkeypatch.setattr(
        window,
        "_map_paths",
        lambda: resolve_map_paths(map_root, "supermarket_test_09"),
    )
    monkeypatch.setattr(window, "_goal_rejection_reason", lambda *_args: "")
    window._mission_waypoints = {
        "SHELF-A": BenchmarkWaypoint(
            "SHELF-A", "A", 1.945, 1.323, math.pi, 0.05, "shelf_approach"
        ),
    }
    snapshot = {
        "scene_revision": 12,
        "requested_map_matches": True,
        "map_binding": {"map_name": "supermarket_test_09", "map_fingerprint": "test"},
        "shelf_types": [],
        "shelves": [
            {
                "id": 1, "name": "1", "world_x": -1.5, "world_y": 0.5,
                "yaw": 0.06, "shelf_width": 0.77, "shelf_length": 1.83,
            },
            {
                "id": 2, "name": "2", "world_x": 0.5706867418984594,
                "world_y": 0.8047947063020753, "yaw": 0.06356114806348771,
                "shelf_width": 0.77, "shelf_length": 1.83,
            },
        ],
        "delivery_tables": [],
        "skus": [{
            "sku": "cola", "qwen_grounding_prompt": "red cola bottle",
            "grasp_method": "夹爪",
        }],
        "slots": [
            {
                "slot_id": "2-1-2-70",
                "shelf_id": 2,
                "face": 1,
                "level": 2,
                "y_cm": 70.0,
                "expected_sku": "tea",
                "actual_sku": "cola",
                "local_x": 0.6275,
                "local_y": 0.70,
                "local_z": 0.84,
                "world_x": 1.1524,
                "world_y": 1.5432,
                "world_z": 0.84,
            }
        ],
    }
    flow._scene_snapshot = snapshot
    flow._scene_identity = flow._current_map_identity()
    flow._scene_reload_needed = False
    flow._configure_anchor(snapshot)
    calibration = ShelfAnchorCalibration(
        "supermarket_test_09", flow._scene_identity[1], "SHELF-A", 2, 1,
        1.945, 1.323, math.pi, 1.845, 1.323, math.pi,
        0.5706867418984594, 0.8047947063020753, 0.06356114806348771,
        "2026-08-16T12:00:00+08:00",
    )
    flow._anchors[(2, 1)]["calibration"] = calibration
    flow._anchors[(2, 1)]["problem"] = ""
    flow._anchor_problem = ""
    assert (2, 1) in flow._anchors, flow.anchor_status_label.text()
    available = flow._available_skus(snapshot)
    assert available == ["cola"]
    flow._populate_sku_combo(available)
    flow.sku_combo.setCurrentIndex(flow.sku_combo.findData("cola"))

    selected = flow._selected_slot_info
    assert flow.sku_combo.findData("tea") == -1
    assert selected is not None, flow.slot_info_label.text()
    assert selected["slot_id"] == "2-1-2-70"
    assert selected["shelf_id"] == 2
    assert selected["face"] == 1
    assert selected["arm_level"] == 3
    assert selected["station"] == "SHELF-A"
    assert selected["waypoint"].y == pytest.approx(1.464, abs=0.01)
    assert selected["qwen_grounding_prompt"] == "red cola bottle"
    assert "物理第 3 层" in flow.slot_info_label.text()
    assert flow._camera_calibrations[2].translation_xyz_m == pytest.approx(
        (0.302, -0.206, 1.011)
    )
    assert "目标安全检查通过" in flow.slot_info_label.text()
    assert flow._camera_calibrations[3].arm_level == 4
    assert flow._camera_calibrations[3].safe_roi_fraction == pytest.approx(0.10)
    window.close()
    app.processEvents()


def test_test9_flow_waits_for_scene_after_mission_reload(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-mission-reload",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    flow = window.competition_flow_tab
    window._mission_waypoints["SHELF-A"] = BenchmarkWaypoint(
        "SHELF-A", "货架 A", 0.5, 0.5, 0.0, 0.2, "shelf_approach"
    )
    monkeypatch.setattr(window, "_goal_rejection_reason", lambda *_args: "")

    flow.invalidate_scene()

    assert flow._selected_sku is None
    assert flow._selected_slot_info is None
    assert flow.sku_combo.currentData() is None
    assert flow._scene_reload_needed
    assert flow._delivery_station == "jiaofu2"
    assert "等待当前地图的场景数据" in flow.slot_info_label.text()
    window.close()
    app.processEvents()


def test_physical_start_is_used_for_localization_reference(tmp_path):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    paths = resolve_map_paths(map_root, "supermarket_test_09")
    start = BenchmarkWaypoint(
        "START", "物理起点", 0.0, -0.3, math.pi / 2, 0.0, "start_position"
    )
    pre_start = BenchmarkWaypoint(
        "PRE_START", "定位等待区", 0.4, 0.7, math.pi / 2, 0.0, "waypoint"
    )
    save_mission(
        mission_path(paths), "supermarket_test_09", [start, pre_start]
    )
    profile = replace(
        _profile(map_root),
        profile_id="pre-start-initialization",
        map_name="supermarket_test_09",
    )

    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")

    assert window._start_waypoint.waypoint_id == "START"
    assert window.initial_x.value() == pytest.approx(start.x)
    assert window.initial_y.value() == pytest.approx(start.y)
    assert window.initial_yaw.value() == pytest.approx(start.yaw, abs=0.001)
    window.close()
    app.processEvents()


def test_phase1_clears_fence_relative_to_actual_pose_before_delivery(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(_profile(map_root), map_name="supermarket_test_09")
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    flow = window.competition_flow_tab
    delivery = BenchmarkWaypoint(
        "jiaofu2", "交付台", -2.256, 0.194, -1.604, 0.05, "delivery_table"
    )
    pre_start = BenchmarkWaypoint(
        "PRE_START", "回程等待点", -0.118, 0.051, 1.56, 0.05, "waypoint"
    )
    physical_start = BenchmarkWaypoint(
        "START", "物理起点", -0.121, -0.249, 1.56, 0.05, "start_position"
    )
    window._mission_waypoints = {
        "START": physical_start,
        "jiaofu2": delivery,
        "PRE_START": pre_start,
    }
    flow.sku_combo.addItem("cola", "cola")
    flow.sku_combo.setCurrentIndex(flow.sku_combo.findData("cola"))
    monkeypatch.setattr(flow, "_build_order_queue", lambda: True)
    monkeypatch.setattr(window, "_process_running", lambda key: key == "navigation")
    monkeypatch.setattr(window, "_benchmark_preflight_problems", lambda: [])

    published = []
    navigated = []

    class FakeRos:
        def publish_start_clearance(self, speed):
            published.append(float(speed))

    window.ros_node = FakeRos()
    window._dispatch_waypoint = lambda target: navigated.append(target) or True
    state = window.telemetry
    now = time.monotonic()
    state.localization_state_seen = True
    state.localization_ok = True
    state.localization_true_since = now - 2.0
    state.last_seen["/odom"] = now
    state.base_linear_speed = 0.0
    state.base_angular_speed = 0.0
    state.wheel_x = 4.0
    state.wheel_y = -2.0
    state.wheel_yaw = math.pi / 2.0
    state.has_map_pose = True
    state.map_x = -0.42
    state.map_y = -0.61
    state.map_yaw = math.pi / 2.0

    flow._start_phase1()
    assert flow._phase1_stage == "waiting_start_buffer_ready"
    assert navigated == []

    flow._update_start_buffer_move()
    assert flow._phase1_stage == "start_buffer"
    flow._update_start_buffer_move()
    assert published[-1] > 0.0

    state.last_seen["/odom"] = time.monotonic()
    state.wheel_y = -1.69
    state.map_y = -0.30
    flow._update_start_buffer_move()
    assert flow._phase1_stage == "start_buffer_settling"
    assert published[-1] == 0.0

    flow._update_start_buffer_move()
    flow._start_buffer_settle_since = time.monotonic() - 0.4
    state.last_seen["/odom"] = time.monotonic()
    flow._update_start_buffer_move()
    assert flow._phase1_stage == "waiting_start_buffer_navigation_ready"

    flow._update_start_buffer_move()
    assert flow._phase1_stage == "navigating_ticket"
    assert [target.waypoint_id for target in navigated] == ["jiaofu2"]
    assert all(target.waypoint_id != "PRE_START" for target in navigated)
    assert navigated[0].target_clearance_m == pytest.approx(0.20)
    assert delivery.target_clearance_m == pytest.approx(0.05)
    assert flow._start_buffer_completed_map_pose == pytest.approx((-0.42, -0.30))

    window.ros_node = None
    window.close()
    app.processEvents()


def test_path_plot_loads_ros_map_with_world_coordinate_bounds(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    paths = resolve_map_paths(map_root, "arena_a")
    plot = PathPlot(Telemetry())
    plot.resize(700, 500)
    plot.set_map(paths.pgm, paths.yaml)
    plot.show()
    app.processEvents()

    assert plot._map_image is not None
    assert plot._map_world_bounds == pytest.approx((0.0, 2.0, 0.0, 2.0))
    pixel = plot._to_pixel(1.25, 0.75)
    assert plot._from_pixel(*pixel) == pytest.approx((1.25, 0.75))
    plot.close()


def test_path_plot_loads_goal_safety_overlay_for_any_map(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    paths = resolve_map_paths(map_root, "arena_a")
    plot = PathPlot(Telemetry())
    plot.resize(700, 500)
    plot.set_map(paths.pgm, paths.yaml)
    assert plot._goal_safety is None

    from openflex_navigation_gui.goal_safety import GoalSafetyGeometry

    plot.set_map(paths.pgm, paths.yaml, GoalSafetyGeometry(0.46, 0.45, 0.05))
    assert plot._goal_safety is not None
    plot.close()


def test_path_plot_drag_selects_heading(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    paths = resolve_map_paths(map_root, "arena_a")
    plot = PathPlot(Telemetry())
    plot.resize(700, 500)
    plot.set_map(paths.pgm, paths.yaml)
    plot.show()
    app.processEvents()
    selected = []
    plot.pose_selected.connect(lambda x, y, yaw: selected.append((x, y, yaw)))
    start = plot._to_pixel(0.5, 0.5)
    end = plot._to_pixel(1.0, 0.5)
    from PyQt5.QtCore import QPoint
    QTest.mousePress(plot, Qt.LeftButton, pos=QPoint(round(start[0]), round(start[1])))
    QTest.mouseMove(plot, QPoint(round(end[0]), round(end[1])), 20)
    QTest.mouseRelease(plot, Qt.LeftButton, pos=QPoint(round(end[0]), round(end[1])))
    assert selected
    assert selected[-1][0] == pytest.approx(0.5, abs=0.03)
    assert selected[-1][1] == pytest.approx(0.5, abs=0.03)
    assert selected[-1][2] == pytest.approx(0.0, abs=0.08)
    plot.close()


def test_graph_query_failure_does_not_block_tf_sampling():
    sampled = []

    class Logger:
        def debug(self, _message):
            pass

    class FakeNode:
        telemetry = Telemetry()

        @staticmethod
        def get_node_names_and_namespaces():
            raise RuntimeError("graph discovery unavailable")

        @staticmethod
        def get_service_names_and_types():
            raise RuntimeError("action discovery unavailable")

        @staticmethod
        def get_logger():
            return Logger()

        @staticmethod
        def _sample_transform(parent, child, base_pose):
            sampled.append((parent, child, base_pose))

        def sample_tf(self):
            NavigationRosNode.sample_tf(self)

    NavigationRosNode.sample_graph_and_tf(FakeNode())

    assert sampled == [
        ("map", "base_link", True),
        ("map", "odom", False),
    ]


def test_shelf_view_result_keeps_child_failure_instead_of_logger_error():
    from rclpy.logging import get_logger

    node = object.__new__(NavigationRosNode)
    node.telemetry = Telemetry()
    node._active_shelf_view_sequence = 1
    node.shelf_view_goal_handle = object()
    node.get_logger = lambda: get_logger("test_shelf_view_mixed_result_severity")
    node._clear_shelf_view_state = lambda sequence: setattr(
        node, "_active_shelf_view_sequence", 0
    )

    success = SimpleNamespace(
        status=4,
        result=SimpleNamespace(
            success=True,
            lift_position=0.0,
            message="move posture reached",
        ),
    )
    NavigationRosNode._shelf_view_result(
        node, SimpleNamespace(result=lambda: success), 1
    )
    assert node.telemetry.shelf_view_status == "succeeded"

    node._active_shelf_view_sequence = 2
    failure_message = (
        "direct taught posture failed (lift: final target did not settle)"
    )
    failure = SimpleNamespace(
        status=6,
        result=SimpleNamespace(
            success=False,
            lift_position=0.28437,
            message=failure_message,
        ),
    )
    NavigationRosNode._shelf_view_result(
        node, SimpleNamespace(result=lambda: failure), 2
    )

    assert node.telemetry.shelf_view_status == "failed"
    assert node.telemetry.shelf_view_message == failure_message


def test_telemetry_refresh_samples_tf_at_display_rate(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")

    class FakeRosNode:
        def __init__(self):
            self.sample_count = 0

        def sample_tf(self):
            self.sample_count += 1

    fake = FakeRosNode()
    window.ros_node = fake
    window._update_telemetry()

    assert window.telemetry_timer.interval() == 200
    assert fake.sample_count == 1
    window.ros_node = None
    window.close()
    app.processEvents()


def test_goal_error_uses_sent_goal_instead_of_stale_plan(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    state = window.telemetry
    state.has_map_pose = True
    state.map_x = 1.0
    state.map_y = 0.0
    state.map_yaw = 0.0
    state.goal_x = 1.05
    state.goal_y = 0.0
    state.goal_yaw = 0.0
    state.goal_valid = True
    state.path = [(9.0, 9.0, math.pi)]

    window._update_telemetry()

    assert window.metric_goal.value.text().startswith("5.0 cm")
    window.close()
    app.processEvents()


def test_station_editor_saves_order_and_reloads_per_map(tmp_path: Path, monkeypatch):
    settings_dir = tmp_path / "settings"
    QSettings.setDefaultFormat(QSettings.IniFormat)
    QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, str(settings_dir))
    QSettings("OpenArmX", "OpenFlexCompetitionNavigationGUI").clear()
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    _make_map(map_root, "arena_b")
    profile = _profile(map_root)

    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    assert window.map_name_combo.currentText() == "arena_a"
    assert window.path_plot._map_world_bounds == pytest.approx((0.0, 2.0, 0.0, 2.0))
    window._new_station()
    window.station_id.setText("SHELF_A")
    window.station_label.setText("A 货架")
    window.station_role.setCurrentIndex(
        window.station_role.findData("shelf_approach")
    )
    window.station_x.setValue(1.25)
    window.station_y.setValue(0.75)
    window.station_yaw_deg.setValue(90.0)
    window._upsert_station()
    assert list(window._mission_waypoints) == ["START", "SHELF_A"]

    window.waypoint_table.selectRow(1)
    window._move_selected_station(-1)
    assert list(window._mission_waypoints) == ["SHELF_A", "START"]

    assert window.competition_map_combo.currentText() == "arena_a"
    assert window.competition_map_combo.findText("arena_b") >= 0
    window.competition_map_combo.setCurrentText("arena_b")
    assert window.competition_map_combo.currentText() == "arena_b"
    assert window._mission_waypoints == {}
    window.competition_map_combo.setCurrentText("arena_a")
    assert list(window._mission_waypoints) == ["SHELF_A", "START"]
    assert window._start_waypoint is not None
    assert window._start_waypoint.waypoint_id == "START"

    saved = load_mission(mission_path(resolve_map_paths(map_root, "arena_a")), "arena_a")
    assert [station.waypoint_id for station in saved] == ["SHELF_A", "START"]
    assert saved[0].x == pytest.approx(1.25)
    assert saved[0].yaw == pytest.approx(math.pi / 2)

    window.waypoint_table.selectRow(0)
    window.station_x.setValue(1.30)
    monkeypatch.setattr(window, "_persist_mission", lambda: False)
    window._upsert_station()
    assert window._mission_waypoints["SHELF_A"].x == pytest.approx(1.25)
    window.close()
    app.processEvents()


@pytest.mark.parametrize(
    ("passed", "detail", "expected_stage"),
    ((True, "通过", "complete"), (False, "货架精停失败", "failed")),
)
def test_mock_pick_chain_uses_final_waypoint_outcome(
    tmp_path: Path,
    monkeypatch,
    passed: bool,
    detail: str,
    expected_stage: str,
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    destination = BenchmarkWaypoint(
        "NEXT", "下一站", 1.0, 1.0, 0.0, 0.2, "waypoint"
    )
    window.ros_node = SimpleNamespace()
    window._pick_navigation_test_stage = "navigating"
    window._pick_navigation_test_destination = destination
    window._active_target = None
    window._goal_terminal_handled = True
    window._last_waypoint_outcome = (destination.waypoint_id, passed, detail)

    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == expected_stage
    assert detail in window.pick_chain_status.text() or passed
    window.ros_node = None
    window.close()
    app.processEvents()


def test_phase1_failure_releases_joint_control_without_stopping_fullbody(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    canceled = []

    class FakeRos:
        def fullbody_position_controllers_ready(self):
            return True

        def competition_actions_active(self):
            return False

        def publish_teleop(self, *_command):
            pass

        def __getattr__(self, name):
            if name.startswith("cancel_") and name.endswith("_goal"):
                return lambda: canceled.append(name)
            raise AttributeError(name)

    window.ros_node = FakeRos()
    window._pick_navigation_test_stage = "ready"
    window._active_motion_phase = "shelf_alignment"
    window._active_target = BenchmarkWaypoint(
        "SHELF", "货架", 0.5, 0.5, 0.0, 0.2, "shelf_approach"
    )
    window.telemetry.node_names = {"/controller_manager"}
    window.telemetry.last_seen["/joint_states"] = time.monotonic()
    window.telemetry.base_linear_speed = 0.0
    window.telemetry.base_angular_speed = 0.0
    monkeypatch.setattr(window, "_process_running", lambda _key: False)

    window.competition_flow_tab._fail_phase1("抓取确认失败")

    assert window._pick_navigation_test_stage == "failed"
    assert window._active_motion_phase == "idle"
    assert window._active_target is None
    assert set(canceled) == {
        "cancel_navigation_goal",
        "cancel_shelf_alignment_goal",
        "cancel_shelf_traverse_goal",
        "cancel_shelf_view_goal",
        "cancel_grasp_debug_goal",
        "cancel_mock_pick_goal",
        "cancel_right_gripper_goal",
        "cancel_station_retreat_goal",
    }
    allowed, reason = window._joint_motion_allowed()
    assert allowed
    assert "可独立" in reason
    assert "保持整机控制" in window.competition_flow_tab.phase1_status_label.text()

    window.ros_node = None
    window.close()
    app.processEvents()


def test_shelf_alignment_action_success_is_authoritative(tmp_path: Path):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    target = BenchmarkWaypoint(
        "SHELF", "货架", 0.5, 0.5, 0.0, 0.2, "shelf_approach"
    )
    now = time.monotonic()
    window._active_target = target
    window._active_target_row = None
    window._active_motion_phase = "shelf_alignment"
    window._goal_terminal_handled = False
    window._active_started_at = now - 2.0
    window._stop_stable_since = now - 1.0
    window.telemetry.shelf_align_status = "succeeded"
    # The action server accepts this close-range value through its 3 mm
    # hysteresis (5 cm target, 1 cm nominal tolerance). The GUI must not apply
    # a second, stricter threshold and turn that success into "out of spec".
    window.telemetry.shelf_clearance = 0.0375
    window.telemetry.shelf_yaw_error = 0.0
    window.telemetry.has_map_pose = True
    # Fine alignment intentionally moved far enough to fail the coarse pose
    # gate; the successful shelf-relative Action result is authoritative.
    window.telemetry.map_x = 1.0
    window.telemetry.map_y = 1.0
    window.telemetry.map_yaw = 0.0
    window.telemetry.base_linear_speed = 0.0
    window.telemetry.base_angular_speed = 0.0
    window.telemetry.last_seen["/odom_safe"] = now

    window._handle_benchmark_result()

    assert window._last_waypoint_outcome == ("SHELF", True, "通过")
    window.close()
    app.processEvents()


def test_shelf_alignment_failure_exposes_the_action_message(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    target = BenchmarkWaypoint(
        "SHELF", "货架", 0.5, 0.5, 0.0, 0.2, "shelf_approach"
    )
    window._active_target = target
    window._active_target_row = None
    window._active_motion_phase = "shelf_alignment"
    window._goal_terminal_handled = False
    window._active_started_at = time.monotonic() - 20.0
    window._pick_navigation_test_stage = "navigating_to_shelf"
    window._pick_navigation_test_source_id = "SHELF"
    window.telemetry.shelf_align_status = "failed"
    window.telemetry.shelf_align_message = "shelf fine alignment timed out"
    window.telemetry.has_map_pose = True

    window._handle_benchmark_result()

    assert "shelf fine alignment timed out" in window.pick_chain_status.text()
    assert window._last_waypoint_outcome is not None
    assert "shelf fine alignment timed out" in window._last_waypoint_outcome[2]
    window.close()
    app.processEvents()


def test_test9_navigation_aligns_only_yaw_before_item_traverse(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(_profile(map_root), map_name="supermarket_test_09")
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    target = BenchmarkWaypoint(
        "SHELF-A",
        "A",
        1.945,
        1.323,
        math.pi,
        0.05,
        "shelf_approach",
        fine_align_enabled=True,
        target_clearance_m=0.10,
    )
    sent = []

    class FakeRos:
        def send_shelf_alignment_goal(self, station_id, yaw, **kwargs):
            sent.append((station_id, yaw, kwargs))
            return True

    now = time.monotonic()
    window.ros_node = FakeRos()
    window._pick_navigation_test_stage = "navigating_to_shelf"
    window._pick_navigation_test_source_id = "SHELF-A"
    window._active_target = target
    window._active_target_row = None
    window._active_motion_phase = "navigation"
    window._goal_terminal_handled = False
    window._active_started_at = now - 2.0
    window._stop_stable_since = now - 1.0
    window.telemetry.action_status = "succeeded"
    window.telemetry.localization_ok = True
    window.telemetry.base_linear_speed = 0.0
    window.telemetry.base_angular_speed = 0.0
    window.telemetry.last_seen["/odom_safe"] = now
    window.telemetry.last_seen["/scan"] = now
    window.telemetry.last_seen["/livox/body_cloud_safety"] = now

    window._handle_benchmark_result()

    assert window._active_motion_phase == "shelf_alignment"
    assert sent[0][0:2] == ("SHELF-A", math.pi)
    assert sent[0][2]["yaw_only"] is True
    assert sent[0][2]["target_clearance"] == pytest.approx(0.10)
    assert "仅校正角度" in window.tour_status.text()
    window.ros_node = None
    window.close()
    app.processEvents()


def test_map_picker_dispatches_transient_goal(tmp_path: Path, monkeypatch):
    settings_dir = tmp_path / "settings"
    QSettings.setDefaultFormat(QSettings.IniFormat)
    QSettings.setPath(QSettings.IniFormat, QSettings.UserScope, str(settings_dir))
    QSettings("OpenArmX", "OpenFlexCompetitionNavigationGUI").clear()
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")

    class FakePicker:
        selected_pose = (1.25, -0.75, math.pi / 3.0)

        def __init__(self, *_args):
            pass

        def setWindowTitle(self, _title):
            pass

        def exec_(self):
            return QDialog.Accepted

    dispatched = []
    monkeypatch.setattr(
        "openflex_navigation_gui.competition_window.WaypointPickerDialog",
        FakePicker,
    )
    monkeypatch.setattr(window, "_benchmark_preflight_problems", lambda: [])
    monkeypatch.setattr(
        window,
        "_dispatch_waypoint",
        lambda waypoint, *, track_table=True: dispatched.append(
            (waypoint, track_table)
        )
        or True,
    )

    window._pick_and_send_map_goal()

    assert len(dispatched) == 1
    waypoint, track_table = dispatched[0]
    assert waypoint.waypoint_id == "MAP_GOAL"
    assert waypoint.x == pytest.approx(1.25)
    assert waypoint.yaw == pytest.approx(math.pi / 3.0)
    assert not track_table
    assert "X 1.250" in window.map_goal_status.text()
    window.close()
    app.processEvents()


def test_fullbody_mode_keeps_mapping_and_reports_layered_interfaces(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    monkeypatch.setenv("OPENFLEX_COMPETITION_LAUNCH_PACKAGE", "openarmx_task_executor")
    monkeypatch.setenv(
        "OPENFLEX_COMPETITION_LAUNCH_FILE", "supermarket_full_system.launch.py"
    )
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")

    assert window.control_tabs.tabText(0) == "建图"
    assert window.control_tabs.isTabEnabled(0)
    assert window.control_tabs.tabText(1) == "整机控制"
    assert window.control_tabs.tabText(2) == "比赛流程"
    assert window.control_tabs.tabText(3) == "比赛导航"
    assert window.control_tabs.currentIndex() == 1
    assert window.workspace_tabs.tabText(0) == "机器人控制"
    assert window.workspace_tabs.tabText(1) == "关节控制"
    assert window.workspace_tabs.tabText(2) == "2D 场景配准"
    assert window.workspace_tabs.tabText(3) == "场景管理"
    assert window.nav_rviz.isChecked()
    assert not window.nav_rviz.isEnabled()
    assert window.relocalization_group.title() == "定位恢复"
    assert window.fullbody_start_button.text() == "启动全身控制"
    assert window.navigation_start_button.text() == "启动定位导航"

    window.telemetry.node_names = {"/controller_manager", "/move_group"}
    window.telemetry.action_servers = {
        "/navigate_to_pose",
        "/detect_object",
        "/execute_transport",
    }
    window._update_fullbody_controls()
    assert window.layer_status_labels["hardware"].text() == "在线"
    assert window.layer_status_labels["moveit"].text() == "在线"
    assert window.layer_status_labels["navigation"].text() == "在线"
    assert window.layer_status_labels["detection"].text() == "在线"
    assert window.layer_status_labels["pick"].text() == "等待 A/B 接入"
    assert window.layer_status_labels["shelf_view"].text() == "等待货架姿态服务"
    assert window.layer_status_labels["mock_pick"].text() == "等待模拟服务器"
    assert window.layer_status_labels["retreat"].text() == "等待退避服务器"
    assert window.layer_status_labels["place"].text() == "等待 B 接入"
    assert window.layer_status_labels["task"].text() == "在线"
    assert window.pick_source_combo.count() == 1
    assert window.pick_destination_combo.count() == 1
    assert not window.pick_chain_button.isEnabled()

    window.telemetry.action_servers.update(
        {
            "/competition/mock_pick_object",
            "/competition/move_to_shelf_view",
            "/retreat_from_station",
        }
    )
    window._update_fullbody_controls()
    assert window.layer_status_labels["shelf_view"].text() == "货架观察可用"
    assert window.layer_status_labels["mock_pick"].text() == "模拟成功可用"
    assert window.layer_status_labels["retreat"].text() == "在线"

    window.close()
    app.processEvents()


def test_fullbody_flow_tab_does_not_overflow_behind_map(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    window.resize(1280, 800)
    window.show()
    flow_index = next(
        index
        for index in range(window.control_tabs.count())
        if window.control_tabs.tabText(index) == "比赛流程"
    )
    window.control_tabs.setCurrentIndex(flow_index)
    app.processEvents()

    flow_scroll = window.control_tabs.widget(flow_index)
    assert flow_scroll.horizontalScrollBar().maximum() == 0
    assert flow_scroll.widget().width() <= flow_scroll.viewport().width()

    flow = window.competition_flow_tab
    flow._populate_sku_combo(
        ["very_long_product_sku_name_that_must_not_resize_the_control_panel"]
    )
    for combo in (window.pick_source_combo, window.pick_destination_combo):
        combo.clear()
        combo.addItem(
            "SHELF_SUPERMARKET_STATION_WITH_A_VERY_LONG_OPERATOR_LABEL",
            "LONG_STATION",
        )
    window.pick_chain_status.setText(
        "联调状态：正在导航到货架，并等待精停完成后进入第三层观察姿态"
    )
    flow.phase1_status_label.setText(
        "状态：正在导航到 SHELF_SUPERMARKET_LONG_NAME 并准备抓取 long_product_sku"
    )
    app.processEvents()

    assert flow_scroll.horizontalScrollBar().maximum() == 0
    assert flow_scroll.widget().width() <= flow_scroll.viewport().width()
    viewport_rect = flow_scroll.viewport().rect()
    for child in flow_scroll.widget().findChildren(QWidget):
        if not child.isVisible():
            continue
        child_rect = child.rect()
        top_left = child.mapTo(flow_scroll.viewport(), child_rect.topLeft())
        bottom_right = child.mapTo(flow_scroll.viewport(), child_rect.bottomRight())
        assert top_left.x() >= viewport_rect.left()
        assert bottom_right.x() <= viewport_rect.right()
    window.close()
    app.processEvents()


def test_competition_navigation_can_select_and_remember_new_map(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    settings = QSettings("OpenArmX", "OpenFlexCompetitionNavigationGUI")
    settings.clear()
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    profile = _profile(map_root)
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")

    _make_map(map_root, "arena_b")
    window._refresh_maps()
    assert window.competition_map_combo.findText("arena_b") >= 0

    window.competition_map_combo.setCurrentText("arena_b")
    assert window.map_name_combo.currentText() == "arena_b"
    assert "arena_b" in window.mission_path_label.text()
    window.close()
    app.processEvents()

    restored = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    assert restored.competition_map_combo.currentText() == "arena_b"
    assert restored.map_name_combo.currentText() == "arena_b"
    restored.close()
    app.processEvents()
    settings.clear()


def test_fullbody_navigation_always_starts_navigation_rviz(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    settings = QSettings("OpenArmX", "OpenFlexCompetitionNavigationGUI")
    settings.clear()
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    window.nav_rviz.setChecked(False)
    window.auto_record.setChecked(False)
    window.telemetry.node_names = {"/controller_manager", "/move_group"}
    monkeypatch.setattr(window, "_process_running", lambda key: key == "fullbody")
    monkeypatch.setattr(window, "_stale_navigation_processes", lambda: [])
    warnings = []
    monkeypatch.setattr(
        "openflex_navigation_gui.competition_window.QMessageBox.warning",
        lambda _parent, title, message: warnings.append((title, message)),
    )
    launches = []
    monkeypatch.setattr(
        window,
        "_start_process",
        lambda key, label, program, args: launches.append((key, label, program, args))
        or True,
    )

    window._start_navigation()

    assert launches, warnings
    assert warnings == []
    assert launches[-1][0] == "navigation"
    assert launches[-1][3][1:3] == [
        "swerve_navigation",
        "test.launch.py",
    ]
    assert "use_rviz:=true" in launches[-1][3]
    assert "start_robot_hardware:=false" in launches[-1][3]
    assert "start_safe_odom_mux:=false" in launches[-1][3]
    window.close()
    app.processEvents()
    settings.clear()


def test_fullbody_start_does_not_start_mapping_or_navigation(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    monkeypatch.setattr(window, "_all_can_interfaces_up", lambda: True)
    monkeypatch.setattr(window, "_has_external_chassis_stack", lambda: False)
    launches = []
    monkeypatch.setattr(
        window,
        "_start_process",
        lambda key, label, program, args: launches.append((key, label, program, args))
        or True,
    )

    window._start_fullbody()

    assert len(launches) == 1
    key, _label, program, args = launches[0]
    assert key == "fullbody"
    assert program == "ros2"
    assert args[1:3] == [
        "openarmx_task_executor",
        "supermarket_fullbody_runtime.launch.py",
    ]
    assert "start_shelf_view_server:=true" in args
    assert not any("map_yaml" in arg or "pcd_path" in arg for arg in args)
    window.close()
    app.processEvents()


def test_fullbody_start_rejects_external_grasp_execution_stack(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    window.telemetry.node_names = {
        # This perception owner used to be omitted from the duplicate-stack
        # guard even though a second instance creates the same /detect_object
        # Action server.
        "/openflex_detect_object",
    }
    monkeypatch.setattr(window, "_all_can_interfaces_up", lambda: True)
    monkeypatch.setattr(window, "_has_external_chassis_stack", lambda: False)
    warnings = []
    monkeypatch.setattr(
        "openflex_navigation_gui.competition_window.QMessageBox.warning",
        lambda _parent, title, message: warnings.append((title, message)),
    )
    launches = []
    monkeypatch.setattr(
        window,
        "_start_process",
        lambda *args: launches.append(args) or True,
    )

    window._start_fullbody()

    assert not launches
    assert warnings
    assert "旧抓取运行栈" in warnings[0][0]
    assert "/openflex_detect_object" in warnings[0][1]
    assert "禁止两套 Action 服务端" in warnings[0][1]
    window.close()
    app.processEvents()


def test_pick_preparation_allows_flow_after_shelf_align_warning(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    shelf = BenchmarkWaypoint(
        "SHELF", "货架", 0.5, 0.5, 0.0, 0.2, "shelf_approach"
    )
    next_station = BenchmarkWaypoint(
        "NEXT", "下一站", 1.0, 1.0, 0.0, 0.2, "delivery_table"
    )
    window._mission_waypoints = {"SHELF": shelf, "NEXT": next_station}
    window._refresh_waypoint_table()
    window.pick_shelf_level.setCurrentIndex(window.pick_shelf_level.findData(3))

    class FakeActionClient:
        @staticmethod
        def server_is_ready():
            return True

    class FakeRos:
        shelf_view_action_client = FakeActionClient()

        def __init__(self):
            self.sent = []

        def send_shelf_view_goal(self, *args):
            self.sent.append(args)
            return True

    fake_ros = FakeRos()
    window.ros_node = fake_ros
    window.telemetry.localization_ok = True
    monkeypatch.setattr(window, "_process_running", lambda key: key == "navigation")
    monkeypatch.setattr(window, "_benchmark_preflight_problems", lambda: [])
    dispatched = []
    monkeypatch.setattr(
        window,
        "_dispatch_waypoint",
        lambda waypoint, *, track_table=True: dispatched.append(waypoint) or True,
    )

    window._start_pick_preparation()
    assert window._pick_navigation_test_stage == "navigating_to_shelf"
    assert dispatched == [shelf]

    window._active_target = shelf
    window._active_target_row = None
    window._active_motion_phase = "shelf_alignment"
    window._goal_terminal_handled = False
    window._active_started_at = time.monotonic() - 2.0
    window._stop_stable_since = time.monotonic() - 1.0
    window.telemetry.shelf_align_status = "succeeded"
    window.telemetry.shelf_clearance = 0.05
    # Flow-only integration should continue after the shelf-align action
    # succeeds even when route-quality statistics still show a warning.
    window.telemetry.shelf_yaw_error = math.radians(0.8)
    window._active_max_cte = 0.20
    window.telemetry.has_map_pose = True
    window.telemetry.map_x = shelf.x
    window.telemetry.map_y = shelf.y
    window.telemetry.map_yaw = shelf.yaw
    window.telemetry.base_linear_speed = 0.0
    window.telemetry.base_angular_speed = 0.0
    window.telemetry.last_seen["/odom_safe"] = time.monotonic()
    window._pick_navigation_test_anchor_return_pose = (0.45, 0.50, 0.0)
    window._pick_navigation_test_item_target_pose = (0.45, 0.80, 0.0)

    window._handle_benchmark_result()

    assert window._pick_navigation_test_stage == "moving_to_pregrasp"
    assert fake_ros.sent == [("SHELF", 3, "right")]
    assert window._pick_navigation_test_anchor_return_pose == pytest.approx(
        (0.50, 0.50, 0.0)
    )
    assert window._pick_navigation_test_item_target_pose == pytest.approx(
        (0.50, 0.80, 0.0)
    )
    assert "精停超差但继续" in window.pregrasp_result_status.text()
    window.ros_node = None
    window.close()
    app.processEvents()


def test_test9_moves_from_observation_posture_to_item_camera_point(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-item-traverse",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    anchor = BenchmarkWaypoint(
        "SHELF-A",
        "货架 A",
        1.945,
        1.323,
        math.pi,
        0.05,
        "shelf_approach",
        fine_align_enabled=True,
        target_clearance_m=0.10,
        retreat_after_pick=True,
        retreat_distance_m=0.15,
        retreat_speed_mps=0.04,
    )
    events = []

    class FakeRos:
        def send_shelf_traverse_goal(self, station_id, *pose, **kwargs):
            events.append((station_id, pose, kwargs))
            window.telemetry.shelf_traverse_status = "sending"
            return True

        def send_shelf_alignment_goal(self, station_id, yaw, **kwargs):
            events.append(("align", station_id, yaw, kwargs))
            window.telemetry.shelf_align_status = "sending"
            return True

    window.ros_node = FakeRos()
    window._mission_waypoints = {"SHELF-A": anchor}
    window._pick_navigation_test_source_id = "SHELF-A"
    window._pick_navigation_test_item_target_pose = (
        1.798,
        1.388,
        math.pi,
    )
    window._pick_navigation_test_anchor_return_pose = (
        1.798,
        1.236,
        math.pi,
    )
    window._pick_navigation_test_level = 2
    window._pick_navigation_test_return_destination = BenchmarkWaypoint(
        "JIAOFU", "交付台", 2.0, 2.0, 0.0, 0.2, "delivery_table"
    )
    window._pick_navigation_test_stage = "moving_to_pregrasp"
    window.telemetry.shelf_view_status = "succeeded"
    window.telemetry.has_map_pose = True
    window.telemetry.map_x = 1.798
    window.telemetry.map_y = 1.236

    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "traversing_to_item"
    assert events[0][0] == "SHELF-A"
    assert events[0][1] == (1.798, 1.388, math.pi)
    assert events[0][2]["target_clearance"] == pytest.approx(0.10)
    assert events[0][2]["max_lateral_speed"] == 0.04
    assert events[0][2]["lateral_only"] is True

    window.telemetry.shelf_traverse_status = "succeeded"
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "aligning_to_item_grasp_clearance"
    assert events[1][0:3] == ("align", "SHELF-A", math.pi)
    assert events[1][3]["target_clearance"] == pytest.approx(0.05)
    assert events[1][3]["clearance_tolerance"] == pytest.approx(0.010)

    window.telemetry.shelf_align_status = "succeeded"
    window.telemetry.shelf_clearance = 0.05
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "ready"
    assert window._pick_navigation_test_at_grasp_clearance
    assert "商品相机点到位" in window.pick_chain_status.text()
    window.ros_node = None
    window.close()
    app.processEvents()


def test_mock_success_returns_to_level2_before_retreat(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    destination = BenchmarkWaypoint(
        "NEXT", "下一站", 1.0, 1.0, 0.0, 0.2, "delivery_table"
    )
    window._pick_navigation_test_stage = "picking"
    window._pick_navigation_test_level = 3
    window._pick_navigation_test_source_id = "SHELF"
    window._pick_navigation_test_destination = destination
    window.telemetry.pick_test_status = "succeeded"
    window.telemetry.pick_test_attached = True

    class FakeRos:
        def __init__(self):
            self.shelf_view = []
            self.retreat = []

        def send_shelf_view_goal(self, *args):
            self.shelf_view.append(args)
            return True

        def send_station_retreat_goal(self, *args, **kwargs):
            self.retreat.append((args, kwargs))
            return True

    fake_ros = FakeRos()
    window.ros_node = fake_ros
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "returning_transport"
    assert fake_ros.shelf_view == [("SHELF", 2, "right")]
    assert fake_ros.retreat == []

    window.telemetry.shelf_view_status = "succeeded"
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "retreating"
    assert fake_ros.retreat
    window.ros_node = None
    window.close()
    app.processEvents()


def test_pick_preparation_reaches_move_posture_before_test9_navigation(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-pick-preparation",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    shelf = BenchmarkWaypoint(
        "SHELF-A", "货架 A", 0.5, 0.5, 0.0, 0.2, "shelf_approach"
    )
    delivery = BenchmarkWaypoint(
        "JIAOFU", "交付台", 1.0, 1.0, 0.0, 0.2, "delivery_table"
    )
    window._mission_waypoints = {"SHELF-A": shelf, "JIAOFU": delivery}
    window._refresh_waypoint_table()
    window.pick_source_combo.setCurrentIndex(
        window.pick_source_combo.findData("SHELF-A")
    )
    window.pick_destination_combo.setCurrentIndex(
        window.pick_destination_combo.findData("JIAOFU")
    )
    window.pick_shelf_level.setCurrentIndex(window.pick_shelf_level.findData(4))

    class FakeActionClient:
        @staticmethod
        def server_is_ready():
            return True

    class FakeRos:
        shelf_view_action_client = FakeActionClient()

        def __init__(self):
            self.shelf_view = []

        def send_shelf_view_goal(self, *args):
            self.shelf_view.append(args)
            return True

    fake_ros = FakeRos()
    window.ros_node = fake_ros
    preflight_problems = ["定位未稳定"]
    monkeypatch.setattr(
        window, "_benchmark_preflight_problems", lambda: list(preflight_problems)
    )
    dispatched = []
    monkeypatch.setattr(
        window,
        "_dispatch_waypoint",
        lambda waypoint, **_kwargs: dispatched.append(waypoint) or True,
    )

    window._start_pick_preparation(pre_navigation_level=0, transport_level=0)

    assert window._pick_navigation_test_stage == "moving_to_transport_before_shelf"
    assert fake_ros.shelf_view == [("SHELF-A", 0, "both")]
    assert dispatched == []

    window.telemetry.shelf_view_status = "succeeded"
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "waiting_navigation_ready_before_shelf"
    assert dispatched == []

    preflight_problems.clear()
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "navigating_to_shelf"
    assert dispatched == [shelf]
    window.ros_node = None
    window.close()
    app.processEvents()


def test_test9_pre_navigation_posture_releases_manual_joint_control(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-control-handoff",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    shelf = BenchmarkWaypoint(
        "SHELF-A", "货架 A", 0.5, 0.5, 0.0, 0.2, "shelf_approach"
    )
    delivery = BenchmarkWaypoint(
        "JIAOFU", "交付台", 1.0, 1.0, 0.0, 0.2, "delivery_table"
    )
    window._mission_waypoints = {"SHELF-A": shelf, "JIAOFU": delivery}
    window._refresh_waypoint_table()
    window.pick_source_combo.setCurrentIndex(
        window.pick_source_combo.findData("SHELF-A")
    )
    window.pick_destination_combo.setCurrentIndex(
        window.pick_destination_combo.findData("JIAOFU")
    )
    window.pick_shelf_level.setCurrentIndex(window.pick_shelf_level.findData(4))

    class FakeActionClient:
        @staticmethod
        def server_is_ready():
            return True

    class FakeRos:
        shelf_view_action_client = FakeActionClient()

        def __init__(self):
            self.manual_state = "acquired"
            self.release_calls = 0
            self.shelf_view = []

        def manual_joint_control_status(self):
            return self.manual_state, self.manual_state

        def release_manual_joint_control(self):
            self.release_calls += 1
            self.manual_state = "releasing"

        def poll_manual_joint_control(self):
            if self.manual_state == "releasing":
                self.manual_state = "released"

        def send_shelf_view_goal(self, *args):
            self.shelf_view.append(args)
            return True

    fake_ros = FakeRos()
    window.ros_node = fake_ros
    monkeypatch.setattr(
        window, "_benchmark_preflight_problems", lambda: ["定位未稳定"]
    )

    window._start_pick_preparation(pre_navigation_level=0, transport_level=0)

    assert window._pick_navigation_test_stage == (
        "releasing_manual_control_before_transport"
    )
    assert fake_ros.release_calls == 1
    assert fake_ros.shelf_view == []

    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "moving_to_transport_before_shelf"
    assert fake_ros.shelf_view == [("SHELF-A", 0, "both")]
    window.ros_node = None
    window.close()
    app.processEvents()


def test_place_confirmation_retreats_then_returns_to_start(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    delivery = BenchmarkWaypoint(
        "JIAOFU",
        "交付台",
        1.0,
        1.0,
        0.0,
        0.2,
        "delivery_table",
        retreat_after_place=True,
        retreat_distance_m=0.18,
        retreat_speed_mps=0.03,
    )
    start = BenchmarkWaypoint(
        "START", "起点", 0.0, 0.0, 0.0, 0.2, "start_position"
    )

    class FakeRos:
        def __init__(self):
            self.retreat = []

        def send_station_retreat_goal(self, *args, **kwargs):
            self.retreat.append((args, kwargs))
            return True

    fake_ros = FakeRos()
    window.ros_node = fake_ros
    window._pick_navigation_test_stage = "ready_to_place"
    window._pick_navigation_test_destination = delivery
    window._pick_navigation_test_return_destination = start

    assert window._continue_after_place_confirmation()
    assert window._pick_navigation_test_stage == "retreating_after_place"
    assert fake_ros.retreat == [
        (("JIAOFU",), {"distance": 0.18, "max_speed": 0.03, "timeout_sec": 8.0})
    ]

    dispatched = []
    monkeypatch.setattr(
        window,
        "_dispatch_waypoint",
        lambda waypoint, **_kwargs: dispatched.append(waypoint) or True,
    )
    window.telemetry.retreat_test_status = "succeeded"
    window.telemetry.retreat_test_distance = 0.18
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "returning_to_start"
    assert dispatched == [start]
    window.ros_node = None
    window.close()
    app.processEvents()


def test_test9_c_point_places_releases_returns_move_then_retreats(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-c-place-sequence",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    delivery = BenchmarkWaypoint(
        "jiaofu2",
        "C",
        -2.256,
        0.194,
        -1.603957582582789,
        0.05,
        "delivery_table",
        fine_align_enabled=True,
        retreat_after_place=True,
        retreat_distance_m=0.15,
        retreat_speed_mps=0.04,
    )
    start = BenchmarkWaypoint(
        "START", "起点", 0.0, 0.0, 0.0, 0.2, "start_position"
    )
    events = []

    class FakeRos:
        def send_shelf_view_goal(self, station_id, level, arm_name):
            events.append(("posture", station_id, level, arm_name))
            window.telemetry.shelf_view_status = "sending"
            return True

        def send_right_gripper_goal(self, position):
            events.append(("gripper", position))
            window.telemetry.right_gripper_status = "sending"
            return True

        def send_station_retreat_goal(self, station_id, **kwargs):
            events.append(("retreat", station_id, kwargs))
            window.telemetry.retreat_test_status = "sending"
            return True

    window.ros_node = FakeRos()
    window._pick_navigation_test_stage = "navigating"
    window._pick_navigation_test_destination = delivery
    window._pick_navigation_test_return_destination = start
    window._active_target = None
    window._goal_terminal_handled = True
    # Route-quality statistics may be out of tolerance even though the local
    # C-point alignment succeeded. That benchmark warning must not block the
    # physical placement sequence.
    window._last_waypoint_outcome = ("jiaofu2", False, "超差")
    window.telemetry.shelf_align_status = "succeeded"

    window._update_mock_pick_navigation_test()
    assert window._pick_navigation_test_stage == "moving_to_place_transition"
    assert events == [("posture", "jiaofu2", 10, "both")]

    window.telemetry.shelf_view_status = "succeeded"
    window._update_mock_pick_navigation_test()
    assert window._pick_navigation_test_stage == "moving_to_place"
    assert events[-1] == ("posture", "jiaofu2", 11, "both")
    assert not any(event[0] == "gripper" for event in events)

    window.telemetry.shelf_view_status = "succeeded"
    window._update_mock_pick_navigation_test()
    assert window._pick_navigation_test_stage == "releasing_after_place"
    assert events[-1] == ("gripper", pytest.approx(0.044))

    window.telemetry.right_gripper_status = "succeeded"
    window.telemetry.right_gripper_position = 0.044
    window._update_mock_pick_navigation_test()
    assert window._pick_navigation_test_stage == "returning_move_after_place"
    assert events[-1] == ("posture", "jiaofu2", 0, "both")

    window.telemetry.shelf_view_status = "succeeded"
    window._update_mock_pick_navigation_test()
    assert window._pick_navigation_test_stage == "retreating_after_place"
    assert events[-1] == (
        "retreat",
        "jiaofu2",
        {"distance": 0.15, "max_speed": 0.04, "timeout_sec": 8.0},
    )

    monkeypatch.setattr(
        window,
        "_dispatch_waypoint",
        lambda waypoint, **_kwargs: events.append(("navigate", waypoint.waypoint_id))
        or True,
    )
    window.telemetry.retreat_test_status = "succeeded"
    window.telemetry.retreat_test_distance = 0.15
    window._update_mock_pick_navigation_test()
    assert window._pick_navigation_test_stage == "returning_to_start"
    assert events[-1] == ("navigate", "START")
    assert [event[0] for event in events] == [
        "posture",
        "posture",
        "gripper",
        "posture",
        "retreat",
        "navigate",
    ]

    window.ros_node = None
    window.close()
    app.processEvents()


def test_test9_gripper_hold_monitor_rejects_drift_and_invalid_feedback(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-gripper-hold-monitor",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    window._pick_navigation_test_grasp_position = 0.017
    window.telemetry.last_seen["/joint_states"] = time.monotonic()
    window.telemetry.joint_positions["openarmx_right_finger_joint1"] = 0.016

    assert window._right_gripper_hold_problem() == ""

    window.telemetry.joint_positions["openarmx_right_finger_joint1"] = 0.030
    assert "未保持抓取" in window._right_gripper_hold_problem()

    window.telemetry.joint_positions["openarmx_right_finger_joint1"] = -0.527
    assert "位置越界" in window._right_gripper_hold_problem()

    window.close()
    app.processEvents()


def test_test9_manual_pick_confirmation_rejects_nearly_open_gripper(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-open-gripper-confirmation",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    window.ros_node = object()
    window._pick_navigation_test_stage = "ready"
    window.telemetry.last_seen["/joint_states"] = time.monotonic()
    window.telemetry.joint_positions["openarmx_right_finger_joint1"] = 0.0437

    assert not window._confirm_manual_pick_success()
    assert window._pick_navigation_test_stage == "ready"
    assert window._pick_navigation_test_grasp_position is None
    assert "仍接近全开" in window.pick_result_status.text()

    window.ros_node = None
    window.close()
    app.processEvents()


def test_pick_retreats_directly_from_item_before_navigation(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(_profile(map_root), map_name="supermarket_test_09")
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    anchor = BenchmarkWaypoint(
        "SHELF-A",
        "A",
        1.945,
        1.323,
        math.pi,
        0.05,
        "shelf_approach",
        fine_align_enabled=True,
        target_clearance_m=0.10,
        retreat_after_pick=True,
        retreat_distance_m=0.15,
        retreat_speed_mps=0.04,
    )
    events = []

    class FakeRos:
        def send_station_retreat_goal(self, station_id, **kwargs):
            events.append(("retreat", station_id, kwargs))
            window.telemetry.retreat_test_status = "sending"
            return True

    window.ros_node = FakeRos()
    window._mission_waypoints = {"SHELF-A": anchor}
    window._pick_navigation_test_source_id = "SHELF-A"
    window._pick_navigation_test_anchor_return_pose = (1.845, 1.323, math.pi)
    window._pick_navigation_test_at_grasp_clearance = True
    window._pick_navigation_test_retreat_distance = 0.15
    window._pick_navigation_test_retreat_speed = 0.04
    destination = BenchmarkWaypoint(
        "JIAOFU", "交付台", 2.0, 2.0, 0.0, 0.2, "delivery_table"
    )
    window._pick_navigation_test_destination = destination
    window._dispatch_waypoint = lambda target: events.append(("navigate", target)) or True
    window._pick_navigation_test_stage = "returning_transport"
    window.telemetry.map_x = 1.845
    window.telemetry.map_y = 1.80
    window.telemetry.shelf_view_status = "succeeded"

    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "retreating"
    assert events[0] == (
        "retreat",
        "SHELF-A",
        {
            "distance": pytest.approx(0.30),
            "max_speed": 0.03,
            "timeout_sec": pytest.approx(13.0),
        },
    )

    window.telemetry.retreat_test_status = "succeeded"
    window.telemetry.retreat_test_distance = 0.30
    window._update_mock_pick_navigation_test()

    assert window._pick_navigation_test_stage == "navigating"
    assert events[1] == ("navigate", destination)
    assert not window._pick_navigation_test_at_grasp_clearance
    window.ros_node = None
    window.close()
    app.processEvents()


def test_competition_flow_waits_for_operator_to_run_grasp_debug(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(_profile(map_root), map_name="supermarket_test_09")
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    flow = window.competition_flow_tab
    events = []

    class FakeRos:
        grasp_debug_status = "idle"
        grasp_debug_message = ""
        grasp_debug_plan_owner = ""

        def send_grasp_detect_goal(self, *_args, **_kwargs):
            events.append("unexpected detect")

        def send_grasp_plan_goal(self, *_args, **_kwargs):
            events.append("unexpected plan")

        def send_grasp_execute_goal(self, *_args, **_kwargs):
            events.append("unexpected execute")

    fake_ros = FakeRos()
    window.ros_node = fake_ros
    flow._phase1_stage = "executing"
    flow._real_grasp_stage = "waiting_observation"
    window._pick_navigation_test_stage = "ready"
    flow._selected_slot_info = {
        "sku": "东方树叶茉莉花茶",
        "qwen_grounding_prompt": "白底绿标；东方树叶；茉莉花图案",
    }
    window.grasp_debug_page._apply_sku_catalog(
        [
            {
                "sku": "东方树叶茉莉花茶",
                "qwen_grounding_prompt": "白底绿标；东方树叶；茉莉花图案",
                "grasp_method": "夹爪",
            }
        ],
        "",
    )

    flow._update_real_grasp()
    assert events == []
    assert flow._real_grasp_stage == "waiting_manual_grasp"
    assert flow.grasp_success_checkbox.isEnabled()
    assert window.grasp_debug_page.sku_combo.currentText() == "东方树叶茉莉花茶"
    assert window.grasp_debug_page.arm_combo.currentText() == "right"

    flow._update_real_grasp()
    assert events == []

    rejected = []
    monkeypatch.setattr(
        window,
        "_confirm_manual_pick_success",
        lambda: rejected.append(window._pick_navigation_test_stage) or False,
    )
    flow.grasp_success_checkbox.setChecked(True)
    flow._update_real_grasp()
    assert rejected == ["ready"]
    assert flow._real_grasp_stage == "waiting_manual_grasp"
    assert flow._phase1_stage == "executing"
    assert flow.grasp_success_checkbox.isEnabled()
    assert not flow.grasp_success_checkbox.isChecked()

    confirmed = []
    monkeypatch.setattr(
        window,
        "_confirm_manual_pick_success",
        lambda: confirmed.append(window._pick_navigation_test_stage) or True,
    )
    flow.grasp_success_checkbox.setChecked(True)
    flow._update_real_grasp()
    assert confirmed == ["ready"]
    assert flow._real_grasp_stage == "complete"
    assert events == []

    window.ros_node = None
    window.close()
    app.processEvents()


def test_test9_manual_pick_confirmation_rejects_nearly_closed_empty_gripper(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-empty-gripper-confirmation",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")
    window.ros_node = object()
    window._pick_navigation_test_stage = "ready"
    window.telemetry.last_seen["/joint_states"] = time.monotonic()
    window.telemetry.joint_positions["openarmx_right_finger_joint1"] = 0.001

    assert not window._confirm_manual_pick_success()
    assert window._pick_navigation_test_stage == "ready"
    assert window._pick_navigation_test_grasp_position is None
    assert "接近全闭" in window.pick_result_status.text()

    window.ros_node = None
    window.close()
    app.processEvents()


@pytest.mark.parametrize("grasp_position", [0.002, 0.041])
def test_test9_manual_pick_confirmation_accepts_grasp_range_boundaries(
    tmp_path: Path, monkeypatch, grasp_position: float
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "supermarket_test_09")
    profile = replace(
        _profile(map_root),
        profile_id="test9-closed-gripper-confirmation",
        map_name="supermarket_test_09",
    )
    window = CompetitionMainWindow(None, profile, tmp_path / "profile.yaml")

    class FakeRos:
        def __init__(self):
            self.shelf_view = []

        def send_shelf_view_goal(self, *args):
            self.shelf_view.append(args)
            return True

    fake_ros = FakeRos()
    window.ros_node = fake_ros
    window._pick_navigation_test_stage = "ready"
    window._pick_navigation_test_source_id = "SHELF-A"
    window._pick_navigation_test_level = 4
    window._pick_navigation_test_transport_level = 0
    window.telemetry.last_seen["/joint_states"] = time.monotonic()
    window.telemetry.joint_positions[
        "openarmx_right_finger_joint1"
    ] = grasp_position

    assert window._confirm_manual_pick_success()
    assert window._pick_navigation_test_stage == "returning_transport"
    assert window._pick_navigation_test_grasp_position == pytest.approx(
        grasp_position
    )
    assert fake_ros.shelf_view == [("SHELF-A", 0, "both")]

    window.ros_node = None
    window.close()
    app.processEvents()


def test_mapping_reuses_running_fullbody_hardware(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    monkeypatch.setattr(window, "_process_running", lambda key: key == "fullbody")
    launches = []
    monkeypatch.setattr(
        window,
        "_start_process",
        lambda key, label, program, args: launches.append((key, label, program, args))
        or True,
    )

    window._start_mapping()

    assert launches
    assert launches[-1][0] == "mapping"
    assert "start_robot_hardware:=false" in launches[-1][3]
    assert not window.teleop_enabled.isChecked()
    window.close()
    app.processEvents()


def test_map_relocalization_publishes_selected_initial_pose(tmp_path: Path, monkeypatch):
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")

    class FakeRosNode:
        def __init__(self):
            self.initial_poses = []

        def publish_initial_pose(self, x, y, yaw):
            self.initial_poses.append((x, y, yaw))

        def publish_teleop(self, *_args):
            pass

    class FakePicker:
        selected_pose = (1.25, -0.75, math.pi / 3.0)

        def __init__(self, *_args):
            pass

        def setWindowTitle(self, _title):
            pass

        def exec_(self):
            return QDialog.Accepted

    fake_ros = FakeRosNode()
    window.ros_node = fake_ros
    monkeypatch.setattr(window, "_process_running", lambda key: key == "navigation")
    monkeypatch.setattr(
        "openflex_navigation_gui.competition_window.WaypointPickerDialog",
        FakePicker,
    )

    window._pick_and_publish_initial_pose()

    assert fake_ros.initial_poses[0] == pytest.approx(
        (1.25, -0.75, math.pi / 3.0), abs=0.001
    )
    window.ros_node = None
    window.close()
    app.processEvents()


def test_fullbody_vr_control_uses_explicit_base_permission(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    window.telemetry.node_names = {"/controller_manager", "/move_group"}
    window.control_mode.setCurrentIndex(1)
    window.vr_base_control.setChecked(False)
    launches = []
    monkeypatch.setattr(
        window,
        "_start_process",
        lambda key, label, program, args: launches.append((key, label, program, args))
        or True,
    )

    window._start_vr()

    assert launches
    key, _label, program, args = launches[-1]
    assert key == "vr"
    assert program == "ros2"
    assert "enable_joystick_control:=false" in args
    assert "vr_chassis:=false" in args
    assert "enable_waist_control:=false" in args
    assert "enable_arm_teleop:=true" in args

    window.close()
    app.processEvents()


def test_fullbody_vr_during_freedrive_keeps_auxiliary_controls(
    tmp_path: Path, monkeypatch
):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    window.telemetry.node_names = {"/controller_manager", "/move_group"}
    window.control_mode.setCurrentIndex(1)
    window.vr_base_control.setChecked(True)
    freedrive = {"running": True}
    window.joint_control_page.freedrive_running = lambda: freedrive["running"]
    monkeypatch.setattr(window, "_process_running", lambda key: key == "navigation")
    launches = []
    monkeypatch.setattr(
        window,
        "_start_process",
        lambda key, label, program, args: launches.append((key, label, program, args))
        or True,
    )

    window._start_vr()

    assert launches
    _key, _label, _program, args = launches[-1]
    assert "enable_joystick_control:=true" in args
    assert "vr_chassis:=true" in args
    assert "enable_arm_teleop:=false" in args
    assert "enable_button_lift:=false" not in args
    assert window._vr_arm_teleop_enabled is False

    freedrive["running"] = False
    window.close()
    app.processEvents()


def test_fullbody_manual_drive_does_not_require_mapping(tmp_path: Path, monkeypatch):
    monkeypatch.setenv("OPENFLEX_FULLBODY_COMPETITION", "1")
    app = QApplication.instance() or QApplication([])
    map_root = tmp_path / "maps"
    _make_map(map_root, "arena_a")
    window = CompetitionMainWindow(None, _profile(map_root), tmp_path / "profile.yaml")
    window.telemetry.node_names = {"/controller_manager"}
    monkeypatch.setattr(window, "_process_running", lambda key: key == "fullbody")

    assert window._mapping_available()
    assert not window._process_running("mapping")
    assert not window._process_running("navigation")

    monkeypatch.setattr(
        window,
        "_process_running",
        lambda key: key in ("fullbody", "mapping"),
    )
    assert window._mapping_available()
    window._update_fullbody_controls()
    assert window.teleop_enabled.isEnabled()
    assert "独立于建图" in window.teleop_enabled.toolTip()

    monkeypatch.setattr(window, "_process_running", lambda key: key in ("fullbody", "navigation"))
    assert not window._mapping_available()
    window.close()
    app.processEvents()
