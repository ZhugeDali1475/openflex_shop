from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "src" / "lio_bridge_node.cpp"
MAPPER_SOURCE = Path(__file__).resolve().parents[1] / "src" / "LIVMapper.cpp"


def test_main_keeps_bridge_node_alive_while_executor_spins():
    source = SOURCE.read_text(encoding="utf-8")

    assert "const auto node = std::make_shared<LioBridgeNode>();" in source
    assert "executor.add_node(node);" in source
    assert "executor.add_node(std::make_shared<LioBridgeNode>())" not in source


def test_world_cloud_publishers_use_lidar_frame_time():
    source = MAPPER_SOURCE.read_text(encoding="utf-8")
    start = source.index("void LIVMapper::publish_frame_world")
    end = source.index("void LIVMapper::publish_visual_sub_map", start)
    publish_frame_world = source[start:end]

    assert "sec2Stamp(LidarMeasures.last_lio_update_time)" in publish_frame_world
    assert "laserCloudmsg.header.stamp = frame_stamp;" in publish_frame_world
    assert "laserCloudmsg.header.stamp = this->node->get_clock()->now()" not in publish_frame_world
