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
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
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
            "odom_slip_simulator = m1_nav2_support.odom_slip_simulator:main",
            "m1_motion_diagnostic = m1_nav2_support.motion_diagnostic:main",
            "m1_costmap_freshness_diagnostic = m1_nav2_support.costmap_freshness_diagnostic:main",
            "gpu_lidar_probe = m1_nav2_support.gpu_lidar_probe:main",
            "gpu_lidar_readback_ab = m1_nav2_support.gpu_lidar_readback_ab:main",
            "gpu_lidar_scene_matrix = m1_nav2_support.gpu_lidar_scene_matrix:main",
            "gpu_lidar_ros_capture = m1_nav2_support.gpu_lidar_ros_capture:main",
            "gpu_lidar_dual_ab = m1_nav2_support.gpu_lidar_dual_ab:main",
        ],
    },
)
