"""Generate and publish the registered bottom-half supermarket Nav2 map."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from openarmx_supermarket_sim.map_builder import build_nav2_map


def _prepare_map(context):
    scene = LaunchConfiguration("scene").perform(context)
    layout = LaunchConfiguration("layout").perform(context)
    stations = LaunchConfiguration("stations").perform(context)
    output_directory = LaunchConfiguration("output_directory").perform(context)
    yaml_path, _pgm_path, grid = build_nav2_map(scene, layout, stations, output_directory)
    return [
        SetLaunchConfiguration("generated_map_yaml", str(yaml_path)),
        LogInfo(
            msg=(
                f"Generated registered supermarket map {grid.spec.width}x{grid.spec.height} "
                f"at {grid.spec.resolution:.3f} m/cell: {yaml_path}"
            )
        ),
    ]


def generate_launch_description():
    sim_share = Path(get_package_share_directory("openarmx_supermarket_sim"))
    executor_share = Path(get_package_share_directory("openarmx_task_executor"))
    default_scene = "/home/openflex/openflex_shop/supermarket_scene/supermarket_scene_from_db.xml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("scene", default_value=default_scene),
            DeclareLaunchArgument(
                "layout", default_value=str(sim_share / "config" / "scene_layout_bottom.yaml")
            ),
            DeclareLaunchArgument(
                "stations",
                default_value=str(
                    executor_share / "config" / "stations_supermarket_mujoco.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "output_directory", default_value="/tmp/openflex_supermarket_map"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_prepare_map),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "yaml_filename": LaunchConfiguration("generated_map_yaml"),
                    }
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_map",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "autostart": True,
                        "node_names": ["map_server"],
                    }
                ],
            ),
        ]
    )
