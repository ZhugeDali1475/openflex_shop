from glob import glob
from setuptools import find_packages, setup


package_name = "openarmx_supermarket_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/maps", glob("maps/*")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="OpenArmX",
    maintainer_email="openarmrobot@gmail.com",
    description="Truth-mode simulator for the OpenArmX supermarket competition.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "competition_sim = openarmx_supermarket_sim.sim_node:main",
            "competition_physics = openarmx_supermarket_sim.physics_bridge_node:main",
            "build_competition_model = openarmx_supermarket_sim.competition_model_builder:main",
            "build_supermarket_map = openarmx_supermarket_sim.map_builder:main",
            "competition_preflight = openarmx_supermarket_sim.preflight:main",
        ],
    },
)
