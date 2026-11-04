from setuptools import setup

package_name = "openarmx_battery_monitor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/single_battery_overlay.launch.py",
                "launch/dual_battery_overlay.launch.py",
                "launch/auto_pack_overlay.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/single_battery_overlay.rviz",
                "config/dual_battery_overlay.rviz",
                "config/auto_pack_overlay.rviz",
            ],
        ),
        (
            "share/" + package_name + "/scripts",
            [
                "scripts/run_battery_monitor_systemd.sh",
                "scripts/install_openarmx_battery_monitor_service.sh",
            ],
        ),
        (
            "share/" + package_name + "/systemd",
            [
                "systemd/openarmx-battery-monitor.service",
                "systemd/openarmx-battery-monitor.env",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenArm Team",
    maintainer_email="openarm_dev@enactic.ai",
    description="JD battery RS485 monitor for single or multi-battery RViz overlay display.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "jd_battery_node = openarmx_battery_monitor.jd_battery_node:main",
            "jd_battery_multi_node = openarmx_battery_monitor.jd_battery_multi_node:main",
            "jd_battery_master_dual_node = openarmx_battery_monitor.jd_battery_master_dual_node:main",
            "jd_battery_bus_diag = openarmx_battery_monitor.jd_battery_bus_diag:main",
        ],
    },
)
