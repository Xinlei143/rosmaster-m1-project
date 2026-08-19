from glob import glob

from setuptools import setup

package_name = "m1_nav2_support"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rosmaster user",
    maintainer_email="user@example.com",
    description="Gazebo Sim adapters and safety watchdog for Nav2 MPPI on Rosmaster M1.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "m1_cmd_watchdog = m1_nav2_support.cmd_vel_watchdog_node:main",
            "dynamic_obstacle_mover = m1_nav2_support.dynamic_obstacle_mover:main",
            "software_lidar = m1_nav2_support.software_lidar:main",
        ],
    },
)
