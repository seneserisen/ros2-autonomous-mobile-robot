from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = "faultnav_robot"

setup(
    name=PACKAGE_NAME,
    version="0.2.0",
    packages=find_packages(where="src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sadik Enes Erisen",
    maintainer_email="s.eneserisen@gmail.com",
    description="Python-first ROS 2 mobile-robot odometry and navigation experiments.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "command_odometry = faultnav_robot.odometry_node:main",
            "faultnav-experiment = faultnav_robot.experiment_cli:main",
        ],
    },
)
