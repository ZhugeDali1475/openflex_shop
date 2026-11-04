import ast
from pathlib import Path


LAUNCH_FILE = (
    Path(__file__).parents[1]
    / "launch"
    / "integrated_robot_bringup_moveit.launch.py"
)


def _controller_recovery_arguments():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "Node":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        executable = keywords.get("executable")
        if (
            isinstance(executable, ast.Constant)
            and executable.value == "ensure_controllers_ready.py"
        ):
            return ast.literal_eval(keywords["arguments"])
    raise AssertionError("controller recovery node is missing")


def _robot_launch_arguments():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "IncludeLaunchDescription":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        launch_arguments = keywords.get("launch_arguments")
        if not (
            isinstance(launch_arguments, ast.Call)
            and isinstance(launch_arguments.func, ast.Attribute)
            and launch_arguments.func.attr == "items"
            and isinstance(launch_arguments.func.value, ast.Dict)
        ):
            continue
        mapping = launch_arguments.func.value
        return {
            ast.literal_eval(key): ast.literal_eval(value)
            for key, value in zip(mapping.keys, mapping.values)
            if isinstance(key, ast.Constant) and isinstance(value, ast.Constant)
        }
    raise AssertionError("robot bringup include is missing")


def test_moveit_startup_recovers_active_swerve_controller():
    arguments = _controller_recovery_arguments()

    assert any(
        arguments[index:index + 2] == ["--active", "swerve_drive_controller"]
        for index in range(len(arguments) - 1)
    )


def test_moveit_startup_recovers_active_lift_state_controller():
    arguments = _controller_recovery_arguments()

    assert any(
        arguments[index:index + 2] == ["--active", "lift_state_controller"]
        for index in range(len(arguments) - 1)
    )


def test_moveit_startup_is_the_only_controller_startup_owner():
    launch_arguments = _robot_launch_arguments()
    arguments = _controller_recovery_arguments()

    assert launch_arguments["start_controller_spawners"] == "false"
    assert any(
        arguments[index:index + 2] == ["--configured", "velocity_controller"]
        for index in range(len(arguments) - 1)
    )
    assert any(
        arguments[index:index + 2] == ["--repair-delay", "0.0"]
        for index in range(len(arguments) - 1)
    )


def test_moveit_clients_wait_for_planning_scene_readiness():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))

    handlers = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "OnProcessExit":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        target = keywords.get("target_action")
        callback = keywords.get("on_exit")
        if (
            isinstance(target, ast.Name)
            and target.id == "move_group_ready"
            and isinstance(callback, ast.Name)
            and callback.id == "start_clients_after_move_group"
        ):
            handlers.append(node)

    assert len(handlers) == 1


def test_moveit_readiness_is_started_only_after_controller_recovery():
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))

    gated_handlers = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "OnProcessExit":
            continue
        keywords = {keyword.arg: keyword.value for keyword in node.keywords}
        target = keywords.get("target_action")
        callback = keywords.get("on_exit")
        if (
            isinstance(target, ast.Name)
            and target.id == "controller_recovery"
            and isinstance(callback, ast.Name)
            and callback.id == "start_readiness_after_controller_recovery"
        ):
            gated_handlers.append(node)

    assert len(gated_handlers) == 1
