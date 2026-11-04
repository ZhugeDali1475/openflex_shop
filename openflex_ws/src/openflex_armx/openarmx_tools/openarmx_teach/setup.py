import sys

from setuptools import setup
from setuptools.command.develop import develop as _develop


def _drop_arg(flag, takes_value=False):
    while flag in sys.argv:
        idx = sys.argv.index(flag)
        if takes_value and idx + 1 < len(sys.argv):
            del sys.argv[idx:idx + 2]
        else:
            del sys.argv[idx]


class _ColconDevelopCompat(_develop):
    """Accept legacy colcon develop flags removed by newer setuptools."""

    user_options = _develop.user_options + [
        ("uninstall", None, "compatibility no-op for colcon"),
        ("editable", None, "compatibility no-op for colcon"),
        ("build-directory=", None, "compatibility no-op for colcon"),
        ("script-dir=", None, "compatibility no-op for colcon"),
    ]
    boolean_options = list(getattr(_develop, "boolean_options", [])) + [
        "uninstall",
        "editable",
    ]

    def initialize_options(self):
        super().initialize_options()
        self.uninstall = False
        self.editable = False
        self.build_directory = None
        self.script_dir = None


while "--editable" in sys.argv:
    sys.argv.remove("--editable")
while "--uninstall" in sys.argv:
    sys.argv.remove("--uninstall")
_drop_arg("--build-directory", takes_value=True)

package_name = "openarmx_teach"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OpenArm User",
    maintainer_email="user@example.com",
    description="Utility scripts for recording and playing OpenArm joint states and trajectories.",
    license="CC-BY-NC-SA-4.0",
    extras_require={"test": ["pytest"]},
    cmdclass={"develop": _ColconDevelopCompat},
    entry_points={
        "console_scripts": [
            "record_joint_states_always = openarmx_teach.record_joint_states_always:main",
            "play_joint_trajectory = openarmx_teach.play_joint_trajectory:main",
        ],
    },
)
