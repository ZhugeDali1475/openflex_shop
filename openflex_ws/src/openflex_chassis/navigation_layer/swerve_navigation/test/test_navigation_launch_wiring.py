import ast
from pathlib import Path

import yaml


LAUNCH_FILE = Path(__file__).parents[1] / "launch" / "full_system.launch.py"
COMPETITION_NAV_CONFIG = (
    Path(__file__).parents[1] / "config" / "nav2_params_supermarket_test_03.yaml"
)
BRIDGE_CONFIG = (
    Path(__file__).parents[3]
    / "mapping_localization_layer"
    / "fast_livo"
    / "config"
    / "bridge_navigation.yaml"
)


def test_competition_bt_allows_loaded_child_servers_to_acknowledge():
    params = yaml.safe_load(
        COMPETITION_NAV_CONFIG.read_text(encoding="utf-8")
    )["bt_navigator"]["ros__parameters"]

    assert params["default_server_timeout"] >= 500


def test_full_system_starts_localization_costmap_guard():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    guard_nodes = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        executable = keywords.get("executable")
        name = keywords.get("name")
        if (
            isinstance(executable, ast.Constant)
            and executable.value == "localization_costmap_guard.py"
        ):
            guard_nodes.append(name)

    assert len(guard_nodes) == 1
    assert isinstance(guard_nodes[0], ast.Constant)
    assert guard_nodes[0].value == "localization_costmap_guard"


def test_nav2_start_is_gated_by_stable_localization_inputs():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    tree = ast.parse(source)

    gate_nodes = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        executable = keywords.get("executable")
        if (
            isinstance(executable, ast.Constant)
            and executable.value == "wait_for_localization_ready.py"
        ):
            gate_nodes.append(node)

    assert len(gate_nodes) == 1
    assert "target_action=localization_ready_gate" in source
    assert "on_exit=start_nav2_after_localization" in source
    assert "if event.returncode != 0" in source
    assert "TimerAction(period=3.0, actions=[nav2_navigation_launch])" not in source


def _literal_node_parameters(tree, executable):
    matches = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        node_executable = keywords.get("executable")
        if not (
            isinstance(node_executable, ast.Constant)
            and node_executable.value == executable
        ):
            continue
        parameters = keywords.get("parameters")
        if parameters is not None:
            matches.append(ast.literal_eval(parameters)[0])
    return matches


def test_navigation_and_safety_relays_filter_full_transport_pose_height():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    relays = {
        params["output_topic"]: params
        for params in _literal_node_parameters(tree, "pointcloud_qos_relay.py")
    }

    assert set(relays) >= {
        "/livox/body_cloud_nav2",
        "/livox/body_cloud_safety",
    }
    for params in relays.values():
        assert params["self_filter_enabled"] is True
        assert params["self_filter_min_z"] <= -0.30
        assert params["self_filter_max_z"] >= 1.80


def test_scan_projection_keeps_full_height_shelf_returns():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    parameters = _literal_node_parameters(tree, "pointcloud_to_laserscan_node")

    assert len(parameters) == 1
    assert parameters[0]["min_height"] <= -0.15
    assert parameters[0]["max_height"] >= 1.80


def test_lio_bridge_filter_covers_measured_arm_and_lift_returns():
    params = yaml.safe_load(BRIDGE_CONFIG.read_text(encoding="utf-8"))["/**"][
        "ros__parameters"
    ]
    values = params["body_cloud_self_filter_boxes"]
    boxes = [values[index:index + 6] for index in range(0, len(values), 6)]
    measured_returns = (
        (-0.055, 0.224, 0.992),
        (0.170, 0.225, 0.752),
        (-0.231, -0.041, 0.868),
    )

    for x, y, z in measured_returns:
        assert any(
            min_x <= x <= max_x
            and min_y <= y <= max_y
            and min_z <= z <= max_z
            for min_x, max_x, min_y, max_y, min_z, max_z in boxes
        )
