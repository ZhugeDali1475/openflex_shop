from setuptools import setup

setup(
    name="openflex_grasp_supermarket_scene_adapter",
    version="0.1.0",
    packages=["openflex_grasp_supermarket_scene_adapter"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/openflex_grasp_supermarket_scene_adapter"]),
        ("share/openflex_grasp_supermarket_scene_adapter", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
)
