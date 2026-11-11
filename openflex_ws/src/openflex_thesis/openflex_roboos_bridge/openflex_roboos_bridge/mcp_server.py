import argparse
import base64
import json
from typing import Literal
from urllib.error import URLError
from urllib.request import ProxyHandler, Request, build_opener

from mcp.server.fastmcp import FastMCP, Image
from mcp.types import ToolAnnotations


class SnapshotClient:
    def __init__(self, port=18552):
        self.base_url = f"http://127.0.0.1:{port}"
        self.opener = build_opener(ProxyHandler({}))

    def read(self, path):
        try:
            with self.opener.open(Request(self.base_url + path), timeout=3) as response:
                payload = response.read(8 * 1024 * 1024 + 1)
            if len(payload) > 8 * 1024 * 1024:
                raise ValueError("Snapshot exceeds size limit")
            result = json.loads(payload)
            if result.get("schema_version") != 1 or result.get("read_only") is not True:
                raise ValueError("Unexpected snapshot contract")
            return result
        except (URLError, OSError, ValueError) as error:
            return {"schema_version": 1, "read_only": True, "status": "unavailable",
                    "holding": None, "sources": {}, "physical_verification": False,
                    "error": f"ROS snapshot unavailable: {error}"}

    def navigation(self, route, arguments):
        try:
            request = Request("http://127.0.0.1:18553" + route,
                              data=json.dumps(arguments, allow_nan=False).encode(),
                              headers={"Content-Type": "application/json"}, method="POST")
            with self.opener.open(request, timeout=5) as response:
                payload = response.read(1024 * 1024 + 1)
            if len(payload) > 1024 * 1024:
                raise ValueError("Navigation response too large")
            return json.loads(payload)
        except (URLError, OSError, ValueError) as error:
            return {"status": "unknown", "success": False,
                    "error": str(error), "warning": "Do not retry with a new request_id; query original operation"}


def create_server(port=18551, snapshot_port=18552, enable_navigation=False, enable_manipulation=False, enable_placement=False):
    if enable_manipulation and not enable_navigation:
        raise ValueError('Manipulation requires the shared navigation/operation gateway')
    if enable_placement and not enable_manipulation:
        raise ValueError('Placement requires explicitly enabled navigation and manipulation')
    server = FastMCP("openflex_readonly", host="127.0.0.1", port=port,
                     stateless_http=True, json_response=True)
    client = SnapshotClient(snapshot_port)
    annotations = ToolAnnotations(readOnlyHint=True, destructiveHint=False,
                                  idempotentHint=True, openWorldHint=False)

    @server.tool(annotations=annotations)
    def get_robot_state() -> dict:
        """读取 OpenFlex 真实 ROS 状态。缺失或过期保持未知；MuJoCo 状态不是物理证据，不表示任务成功。"""
        return client.read("/state")

    @server.tool(annotations=annotations, structured_output=False)
    def observe_scene() -> list:
        """读取实时 MuJoCo 头部 RGB 图像及独立标记的 Oracle 目标数据。无新鲜图像时不返回旧图，不操作机器人。"""
        observation = client.read("/observation")
        camera = observation.get("sources", {}).get("camera", {})
        image = camera.get("data") if camera.get("status") == "fresh" else None
        image_data = image.pop("data", None) if image else None
        content = [json.dumps(observation, ensure_ascii=False, allow_nan=False)]
        if image_data:
            content.append(Image(data=base64.b64decode(image_data, validate=True), format="jpeg"))
        return content

    if enable_navigation:
        @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=True,
                                                 idempotentHint=True, openWorldHint=False))
        def navigate_to_target(target_id: Literal["pickup_zone", "delivery_zone"], request_id: str,
                               timeout_sec: float = 60.0) -> dict:
            """只在隔离 MuJoCo 中导航到登记站点；持物只允许交付位，需含货物轨迹检查和持续夹持验证。running 不等于成功；必须查询同一 request_id。禁止自由坐标。"""
            return client.navigation("/navigate", {"target_id": target_id, "request_id": request_id,
                                                    "timeout_sec": timeout_sec})

        @server.tool(annotations=annotations)
        def get_navigation_result(request_id: str) -> dict:
            """查询真实导航结果；只有 succeeded 且 success=true、stopped_confirmed=true 才算仿真导航完成，unknown 不得重试动作。"""
            return client.navigation("/status", {"request_id": request_id})

        @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=False,
                                                 idempotentHint=True, openWorldHint=False))
        def cancel_current_task(request_id: str) -> dict:
            """请求取消指定导航，不表示已停稳。随后查询原 request_id，核实终态与停稳证据。"""
            return client.navigation("/cancel", {"request_id": request_id})

    if enable_manipulation:
        @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=True,
                                                 idempotentHint=True, openWorldHint=False))
        def grasp_object(object_id: Literal['thesis_item_01'], request_id: str, timeout_sec: float = 600.0) -> dict:
            """隔离仿真单商品抓取：先全机器人和持物轨迹检查，再串行导航对位、精抓、独立夹持验证与撤回。返回 running 必须查询，失败不能当作持物成功。"""
            return client.navigation('/grasp', {'object_id': object_id, 'request_id': request_id, 'timeout_sec': timeout_sec})

        @server.tool(annotations=annotations)
        def get_skill_result(request_id: str) -> dict:
            """查询原请求的持久化真实技能结果。只有 succeeded、success=true 且 stopped_confirmed=true 才算技能成功；不是整项取送任务成功。"""
            return client.navigation('/status', {'request_id': request_id})

    if enable_placement:
        @server.tool(annotations=ToolAnnotations(readOnlyHint=False, destructiveHint=True,
                                                 idempotentHint=True, openWorldHint=False))
        def place_to_affordance(object_id: Literal['thesis_item_01'], affordance_id: Literal['delivery_zone'],
                                request_id: str, timeout_sec: float = 600.0) -> dict:
            """隔离仿真实际放置：持物碰撞检查、机械臂移动、原位开爪、真实接触落稳和撤臂后复核。不改写物体位姿；running 必须查询真实终态。"""
            return client.navigation('/place', {'object_id': object_id, 'affordance_id': affordance_id,
                                               'request_id': request_id, 'timeout_sec': timeout_sec})

    return server


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=18551)
    parser.add_argument("--snapshot-port", type=int, default=18552)
    parser.add_argument("--enable-navigation", action="store_true")
    parser.add_argument('--enable-manipulation', action='store_true')
    parser.add_argument('--enable-placement', action='store_true')
    args = parser.parse_args()
    create_server(args.port, args.snapshot_port, args.enable_navigation, args.enable_manipulation, args.enable_placement).run(transport="streamable-http")


if __name__ == "__main__":
    main()
