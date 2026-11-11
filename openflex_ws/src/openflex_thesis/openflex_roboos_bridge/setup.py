from setuptools import find_packages, setup


package_name = "openflex_roboos_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenArmX",
    maintainer_email="openarmrobot@gmail.com",
    description="Read-only ROS snapshots for OpenFlex MCP.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "readonly_snapshot = openflex_roboos_bridge.ros_snapshot:main",
    ]},
)
