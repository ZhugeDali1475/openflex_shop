from glob import glob

from setuptools import find_packages, setup


package_name = "openflex_grasp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README_CN.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/web", glob("web/*")),
        ("lib/" + package_name, glob("scripts/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="openflex",
    maintainer_email="openarmrobot@gmail.com",
    description="Portable product perception and GraspNetBaseline grasp execution",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={},
)
