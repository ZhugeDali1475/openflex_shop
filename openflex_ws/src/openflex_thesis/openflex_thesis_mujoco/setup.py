from glob import glob
from setuptools import find_packages, setup


package_name = "openflex_thesis_mujoco"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "requirements.txt"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/models", glob("models/*")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="OpenArmX",
    maintainer_email="openarmrobot@gmail.com",
    description="MuJoCo digital experiment environment for the OpenFleX TC-WAM thesis.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "build_thesis_scene = openflex_thesis_mujoco.model_builder:main",
            "mujoco_bridge = openflex_thesis_mujoco.bridge_node:main",
            "lio_odom_tf_bridge = openflex_thesis_mujoco.lio_odom_tf_bridge:main",
            "oracle_anygrasp = openflex_thesis_mujoco.oracle_anygrasp_node:main",
            "scripted_local_policy = openflex_thesis_mujoco.scripted_local_policy_node:main",
            "sim_navigation = openflex_thesis_mujoco.sim_navigation_node:main",
            "sim_mission_client = openflex_thesis_mujoco.sim_mission_client:main",
        ],
    },
)
