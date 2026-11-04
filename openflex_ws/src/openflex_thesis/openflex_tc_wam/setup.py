from glob import glob
from setuptools import find_packages, setup


package_name = "openflex_tc_wam"

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
    description="TC-WAM generation, physical evaluation, scoring, and object memory.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "manual_object_source = openflex_tc_wam.manual_object_source_node:main",
            "object_observation_adapter = openflex_tc_wam.object_observation_adapter_node:main",
            "object_memory = openflex_tc_wam.object_memory_node:main",
            "anygrasp_gateway = openflex_tc_wam.anygrasp_gateway_node:main",
            "mock_anygrasp = openflex_tc_wam.mock_anygrasp_node:main",
            "mock_evaluator = openflex_tc_wam.mock_evaluator_node:main",
            "moveit_evaluator = openflex_tc_wam.moveit_evaluator_node:main",
            "tc_wam_server = openflex_tc_wam.tc_wam_server_node:main",
            "auto_compute = openflex_tc_wam.auto_compute_node:main",
        ],
    },
)
