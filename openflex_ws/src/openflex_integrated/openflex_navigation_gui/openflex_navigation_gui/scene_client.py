"""Asynchronous ROS service transport for the competition GUI scene store."""


class SceneRosClient:
    def __init__(self, node) -> None:
        from supermarket_scene_interfaces.msg import (
            DeliveryTablePoseUpdate,
            ShelfPoseUpdate,
        )
        from supermarket_scene_interfaces.srv import ApplySceneLayout, GetSceneSnapshot

        self._snapshot_type = GetSceneSnapshot
        self._layout_type = ApplySceneLayout
        self._shelf_update_type = ShelfPoseUpdate
        self._table_update_type = DeliveryTablePoseUpdate
        self._snapshot_client = node.create_client(GetSceneSnapshot, "get_scene_snapshot")
        self._layout_client = node.create_client(ApplySceneLayout, "apply_scene_layout")

    @property
    def snapshot_service_ready(self) -> bool:
        return self._snapshot_client.service_is_ready()

    @property
    def layout_service_ready(self) -> bool:
        return self._layout_client.service_is_ready()

    def request_snapshot(self, map_name: str = "", map_fingerprint: str = ""):
        request = self._snapshot_type.Request()
        request.map_name = map_name
        request.map_fingerprint = map_fingerprint
        return self._snapshot_client.call_async(request)

    def apply_layout(self, layout: dict):
        request = self._layout_type.Request()
        request.map_name = layout["map_name"]
        request.map_fingerprint = layout["map_fingerprint"]
        request.expected_scene_revision = layout["expected_scene_revision"]
        for value in layout["shelf_updates"]:
            item = self._shelf_update_type()
            item.shelf_id = value["shelf_id"]
            item.world_x = value["world_x"]
            item.world_y = value["world_y"]
            item.yaw = value["yaw"]
            request.shelf_updates.append(item)
        for value in layout["delivery_table_updates"]:
            item = self._table_update_type()
            item.table_id = value["table_id"]
            item.world_x = value["world_x"]
            item.world_y = value["world_y"]
            item.yaw = value["yaw"]
            request.delivery_table_updates.append(item)
        return self._layout_client.call_async(request)

    @staticmethod
    def snapshot_from_response(response) -> dict:
        if not response.success:
            raise RuntimeError(response.error)
        binding = None
        if response.has_map_binding:
            binding = {
                "map_name": response.map_binding.map_name,
                "map_fingerprint": response.map_binding.map_fingerprint,
                "updated_at": response.map_binding.updated_at,
            }
        return {
            "scene_revision": response.scene_revision,
            "map_binding": binding,
            "requested_map_matches": response.requested_map_matches,
            "shelf_types": [
                {
                    field: getattr(value, field)
                    for field in (
                        "id", "name", "shelf_length", "shelf_width", "shelf_height",
                        "num_levels", "bottom_clearance", "level_spacing", "panel_thick",
                        "back_thick", "shelf_depth_normal", "shelf_depth_bottom",
                    )
                }
                for value in response.shelf_types
            ],
            "shelves": [
                {
                    field: getattr(value, field)
                    for field in (
                        "id", "name", "world_x", "world_y", "yaw", "shelf_type_id",
                        "shelf_length", "shelf_width", "shelf_height", "num_levels",
                        "bottom_clearance", "level_spacing", "panel_thick", "back_thick",
                        "shelf_depth_normal", "shelf_depth_bottom",
                        "face_0_representatives", "face_1_representatives",
                    )
                }
                for value in response.shelves
            ],
            "delivery_tables": [
                {
                    field: getattr(value, field)
                    for field in (
                        "id", "name", "world_x", "world_y", "yaw",
                        "length", "width", "height", "top_thickness",
                    )
                }
                for value in response.delivery_tables
            ],
            "skus": [
                {
                    field: getattr(value, field)
                    for field in (
                        "sku", "category", "mesh_file", "tex_file", "owlv2_prompt",
                        "qwen_grounding_prompt", "reference_image_path", "grasp_method",
                    )
                }
                for value in response.skus
            ],
            "slots": [SceneRosClient._slot_from_message(value) for value in response.slots],
        }

    @staticmethod
    def _slot_from_message(value) -> dict:
        return {
            "slot_id": value.slot_id,
            "shelf_id": value.shelf_id,
            "face": value.face,
            "level": value.level,
            "y_cm": value.y_cm,
            "expected_sku": value.expected_sku,
            "actual_sku": value.actual_sku if value.has_actual_sku else None,
            "status": value.status,
            "width_cm": value.width_cm if value.has_width_cm else None,
            "height_cm": value.height_cm if value.has_height_cm else None,
            "image_dir": value.image_dir,
            "image_revision": value.image_revision,
            "local_x": value.local_x,
            "local_y": value.local_y,
            "local_z": value.local_z,
            "world_x": value.world_x,
            "world_y": value.world_y,
            "world_z": value.world_z,
            "yaw": value.shelf_yaw,
        }
