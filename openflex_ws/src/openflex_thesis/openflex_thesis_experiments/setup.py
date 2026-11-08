from glob import glob
from setuptools import find_packages, setup


package_name = "openflex_thesis_experiments"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name, ["requirements-learning.txt"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="OpenArmX",
    maintainer_email="openarmrobot@gmail.com",
    description="Reproducible thesis experiment protocols and logging.",
    license="Apache-2.0",
    tests_require=["pytest"],
    extras_require={"learning": ["scikit-learn>=1.1", "joblib>=1.1"]},
    entry_points={
        "console_scripts": [
            "trial_logger = openflex_thesis_experiments.trial_logger_node:main",
            "generate_trial_matrix = openflex_thesis_experiments.trial_matrix:main",
            "summarize_trials = openflex_thesis_experiments.summarize_trials:main",
            "thesis_preflight = openflex_thesis_experiments.preflight:main",
            "train_recovery = openflex_thesis_experiments.recovery_training:main",
            "run_recovery_benchmark = openflex_thesis_experiments.recovery_benchmark:main",
            "generate_recovery_matrix = openflex_thesis_experiments.recovery_protocol:main",
            "report_recovery = openflex_thesis_experiments.recovery_report:main",
        ],
    },
)
