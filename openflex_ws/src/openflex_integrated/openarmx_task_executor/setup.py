from setuptools import find_packages, setup

package_name = "openarmx_task_executor"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name,
            [
                "package.xml",
                "README.md",
                "HARDWARE_INTEGRATION_CHECKLIST.md",
                "SUPERMARKET_COMPETITION_FRAMEWORK_CN.md",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/stations.yaml",
                "config/stations_supermarket_mujoco.yaml",
                "config/stations_supermarket_test_07.yaml",
                "config/stations_supermarket_test_08.yaml",
                "config/stations_supermarket_test_09.yaml",
                "config/competition.yaml",
                "config/inventory_supermarket_scene.example.yaml",
            ],
        ),
        (
            "share/" + package_name + "/launch",
            [
                "launch/task_executor.launch.py",
                "launch/station_docking.launch.py",
                "launch/shelf_fine_align.launch.py",
                "launch/station_retreat.launch.py",
                "launch/competition_task_api.launch.py",
                "launch/competition_coordinator.launch.py",
                "launch/supermarket_full_system.launch.py",
                "launch/supermarket_fullbody_runtime.launch.py",
            ],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "task_executor = openarmx_task_executor.task_executor_node:main",
            "station_docking = openarmx_task_executor.docking_node:main",
            "shelf_fine_align = openarmx_task_executor.shelf_fine_align_node:main",
            "station_retreat = openarmx_task_executor.station_retreat_node:main",
            "lift_level_test = openarmx_task_executor.lift_level_test_node:main",
            "shelf_view = openarmx_task_executor.shelf_view_node:main",
            "competition_mock_pick = openarmx_task_executor.mock_pick_node:main",
            "competition_coordinator = openarmx_task_executor.competition_coordinator_node:main",
            "shopping_ticket_reader = openarmx_task_executor.shopping_ticket_reader_node:main",
        ],
    },
)
