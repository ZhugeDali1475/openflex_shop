from setuptools import find_packages, setup


package_name = "openflex_navigation_gui"


setup(
    name=package_name,
    version="0.1.0",
    maintainer="OpenArmX",
    maintainer_email="openarmrobot@gmail.com",
    description="OpenFlex mapping, localization, navigation, and recording desktop console.",
    license="CC-BY-NC-SA-4.0",
    url="https://openarmx.com",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README-CN.md"]),
        (
            "share/" + package_name + "/config",
            ["config/right_wrist_shelf_camera.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "openflex_navigation_gui = openflex_navigation_gui.main_window:main",
            "openflex_competition_navigation_gui = openflex_navigation_gui.competition_window:main",
            "openflex_freedrive = openflex_navigation_gui.freedrive_runner:main",
        ],
    },
)
