from glob import glob
from setuptools import find_packages, setup


package_name = "openflex_thesis_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="OpenArmX",
    maintainer_email="openarmrobot@gmail.com",
    description="Safe command ownership and pick-deliver mission control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_arbiter = openflex_thesis_control.velocity_arbiter_node:main",
            "local_alignment_supervisor = openflex_thesis_control.local_pick_supervisor_node:main",
            "moveit_grasp_executor = openflex_thesis_control.moveit_grasp_executor_node:main",
            "fixed_place_supervisor = openflex_thesis_control.fixed_place_supervisor_node:main",
            "mission_orchestrator = openflex_thesis_control.mission_orchestrator_node:main",
            "mock_dependencies = openflex_thesis_control.mock_dependencies_node:main",
            "demo_mission_client = openflex_thesis_control.demo_mission_client:main",
        ],
    },
)
