from types import SimpleNamespace

from PyQt5.QtWidgets import QApplication

from openflex_navigation_gui.grasp_debug_tab import GraspDebugTab
from openflex_navigation_gui.main_window import NavigationRosNode


class _ReadyClient:
    def __init__(self):
        self.goal = None

    @staticmethod
    def server_is_ready():
        return True

    @staticmethod
    def wait_for_server(timeout_sec=0.0):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        self.goal = goal
        return SimpleNamespace(add_done_callback=lambda _callback: None)


def test_grasp_debug_defaults_add_upward_shelf_retreat():
    app = QApplication.instance() or QApplication([])
    tab = GraspDebugTab(SimpleNamespace(grasp_debug_plan_result=None))
    tab._timer.stop()
    tab._visualizer_retry_timer.stop()

    assert tab.initial_raise.value() == 0.0
    assert tab.pregrasp_height.value() == 30.0
    assert tab.grasp_height.value() == 30.0
    assert tab.retreat_height.value() == 40.0
    assert tab.approach_distance.value() == 0.0

    tab.close()
    app.processEvents()


def test_grasp_debug_height_control_sends_meters_and_invalidates_plan():
    app = QApplication.instance() or QApplication([])
    calls = []

    def record_plan(*args, **kwargs):
        calls.append((args, kwargs))

    ros_node = SimpleNamespace(
        grasp_debug_plan_result=object(),
        send_grasp_plan_goal=record_plan,
    )
    tab = GraspDebugTab(ros_node)
    tab._timer.stop()
    tab._visualizer_retry_timer.stop()
    tab.sku_edit.setText("可乐")
    tab.initial_raise.setValue(10.0)
    tab.pregrasp_height.setValue(20.0)
    tab.grasp_height.setValue(30.0)
    tab.retreat_height.setValue(40.0)
    tab.approach_distance.setValue(125.0)

    assert ros_node.grasp_debug_plan_result is None
    tab._send_plan()
    assert calls == [
        (
            ("可乐", "right"),
            {
                "initial_raise_m": 0.01,
                "pregrasp_height_offset_m": 0.02,
                "grasp_height_offset_m": 0.03,
                "retreat_height_offset_m": 0.04,
                "approach_distance_m": 0.125,
            },
        )
    ]

    tab.close()
    app.processEvents()


def test_debug_page_cannot_invalidate_competition_plan():
    node = object.__new__(NavigationRosNode)
    node.grasp_debug_plan_result = object()
    node.grasp_debug_plan_owner = "competition"
    node.grasp_debug_status = "plan_ready"
    node.grasp_debug_message = "ready"
    node._grasp_plan_sequence = 3
    node._active_grasp_plan_sequence = 3

    assert not NavigationRosNode.invalidate_grasp_plan(node, "debug")
    assert node.grasp_debug_plan_result is not None
    assert node.grasp_debug_plan_owner == "competition"

    assert NavigationRosNode.invalidate_grasp_plan(node, "competition")
    assert node.grasp_debug_plan_result is None
    assert node.grasp_debug_plan_owner == ""


def test_grasp_debug_sku_selection_sends_database_qwen_prompt():
    app = QApplication.instance() or QApplication([])
    calls = []
    ros_node = SimpleNamespace(
        grasp_debug_plan_result=None,
        send_grasp_detect_goal=lambda *args: calls.append(args),
    )
    tab = GraspDebugTab(ros_node)
    tab._timer.stop()
    tab._visualizer_retry_timer.stop()
    tab._apply_sku_catalog(
        [
            {
                "sku": "绿豆冰沙",
                "qwen_grounding_prompt": "绿色；绿豆；杯装",
                "grasp_method": "夹爪",
            }
        ],
        "",
    )

    assert tab.sku_combo.currentText() == "绿豆冰沙"
    assert tab.qwen_prompt_edit.text() == "绿色；绿豆；杯装"
    assert tab.grasp_method_edit.text() == "夹爪"
    tab._send_detect()
    assert calls == [("绿豆冰沙", "right", "绿色；绿豆；杯装")]

    tab.close()
    app.processEvents()


def test_grasp_debug_rejects_unsupported_suction_sku():
    app = QApplication.instance() or QApplication([])
    calls = []
    ros_node = SimpleNamespace(
        grasp_debug_plan_result=None,
        send_grasp_detect_goal=lambda *args: calls.append(args),
    )
    tab = GraspDebugTab(ros_node)
    tab._timer.stop()
    tab._visualizer_retry_timer.stop()
    tab._apply_sku_catalog(
        [
            {
                "sku": "测试吸盘商品",
                "qwen_grounding_prompt": "白色盒装",
                "grasp_method": "吸盘",
            }
        ],
        "",
    )

    tab._send_detect()
    assert calls == []
    assert "只支持夹爪" in tab.status_label.text()

    tab.close()
    app.processEvents()


def test_navigation_goal_carries_grasp_height_offset():
    node = object.__new__(NavigationRosNode)
    plan_client = _ReadyClient()
    node.grasp_debug_result = SimpleNamespace(detection_id="detection-1")
    node.plan_pick_action_client = plan_client
    node.plan_pregrasp_action_client = _ReadyClient()
    node.cartesian_approach_action_client = _ReadyClient()
    node.grasp_debug_plan_result = None
    node.grasp_debug_status = "idle"
    node.grasp_debug_message = ""

    assert NavigationRosNode.send_grasp_plan_goal(
        node,
        "可乐",
        "right",
        initial_raise_m=0.01,
        pregrasp_height_offset_m=0.02,
        grasp_height_offset_m=0.03,
        retreat_height_offset_m=0.04,
        approach_distance_m=0.125,
    )
    assert plan_client.goal.height_offset_m == 0.0
    assert plan_client.goal.initial_raise_m == 0.01
    assert plan_client.goal.pregrasp_height_offset_m == 0.02
    assert plan_client.goal.grasp_height_offset_m == 0.03
    assert plan_client.goal.retreat_height_offset_m == 0.04
    assert plan_client.goal.approach_distance_m == 0.125
    assert plan_client.goal.max_dof == plan_client.goal.ARM_WITH_LIFT
    assert "起始 10.0" in node.grasp_debug_message
    assert "回撤 40.0 mm" in node.grasp_debug_message
    assert "进给距离 125.0 mm" in node.grasp_debug_message


def test_navigation_detect_goal_carries_sku_and_qwen_prompt():
    node = object.__new__(NavigationRosNode)
    detect_client = _ReadyClient()
    node.detect_object_action_client = detect_client
    node.grasp_debug_result = object()
    node.grasp_debug_plan_result = object()
    node.grasp_debug_status = "idle"
    node.grasp_debug_message = ""

    assert NavigationRosNode.send_grasp_detect_goal(
        node,
        "绿豆冰沙",
        "right",
        "绿色；绿豆；杯装",
    )
    assert detect_client.goal.object_id == "绿豆冰沙"
    assert detect_client.goal.sku_name == "绿豆冰沙"
    assert detect_client.goal.qwen_grounding_prompt == "绿色；绿豆；杯装"
