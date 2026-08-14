from glob import glob

from setuptools import setup

package_name = "imperative_navigation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("share/" + package_name + "/algorithm", glob("algorithm/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rosmaster user",
    maintainer_email="user@example.com",
    description="Gazebo Sim test harness for the imperative navigation demo.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "imperative_controller = imperative_navigation.controller_node:main",
            "imperative_m1_controller = imperative_navigation.m1_controller_node:main",
            "imperative_cmd_watchdog = imperative_navigation.cmd_vel_watchdog_node:main",
            "experiment_logger = imperative_navigation.experiment_logger:main",
            "dynamic_obstacle_mover = imperative_navigation.dynamic_obstacle_mover:main",
            "software_lidar = imperative_navigation.software_lidar:main",
        ],
    },
)
