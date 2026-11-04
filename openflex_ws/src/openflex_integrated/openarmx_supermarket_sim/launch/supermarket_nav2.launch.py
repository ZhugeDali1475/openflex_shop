"""Run Nav2 on the occupancy map projected from the MuJoCo scene."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    sim_share = Path(get_package_share_directory("openarmx_supermarket_sim"))
    nav2_share = Path(get_package_share_directory("nav2_bringup"))
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(sim_share / "launch" / "supermarket_map.launch.py")
                ),
                launch_arguments={"use_sim_time": "true"}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(nav2_share / "launch" / "navigation_launch.py")
                ),
                launch_arguments={
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_composition": "false",
                    "params_file": str(sim_share / "config" / "nav2_sim.yaml"),
                }.items(),
            ),
        ]
    )
