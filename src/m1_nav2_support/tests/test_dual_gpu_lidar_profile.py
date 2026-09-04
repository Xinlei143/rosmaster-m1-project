"""Static contracts for the Ogre1 dual-180 GPU LiDAR workaround."""

from pathlib import Path


ROOT = Path(__file__).parents[3]
SUPPORT = ROOT / "src" / "m1_nav2_support"
DESCRIPTION = ROOT / "src" / "yahboomcar_description"
URDF = DESCRIPTION / "urdf" / "yahboomcar_M1_gazebo.urdf.xacro"
LAUNCH = SUPPORT / "launch" / "m1_gazebo.launch.py"
PACKAGE = SUPPORT / "package.xml"
SETUP = SUPPORT / "setup.py"
OGRE2_WORLD = SUPPORT / "worlds" / "m1_ogre2_regression.sdf"
OGRE1_WORLD = SUPPORT / "worlds" / "m1_ogre.sdf"


def _normalized_world(path):
    return path.read_text().replace(
        "<render_engine>ogre2</render_engine>",
        "<render_engine>RENDER_ENGINE</render_engine>",
    ).replace(
        "<render_engine>ogre</render_engine>",
        "<render_engine>RENDER_ENGINE</render_engine>",
    )


def test_regression_and_treatment_worlds_only_differ_by_sensor_engine():
    assert "<render_engine>ogre2</render_engine>" in OGRE2_WORLD.read_text()
    assert "<render_engine>ogre</render_engine>" in OGRE1_WORLD.read_text()
    assert _normalized_world(OGRE2_WORLD) == _normalized_world(OGRE1_WORLD)


def test_xacro_keeps_single_360_and_adds_two_180_degree_sensors():
    text = URDF.read_text()

    assert '<xacro:arg name="dual_gpu_lidar" default="true"/>' in text
    assert '<xacro:unless value="$(arg dual_gpu_lidar)">' in text
    assert '<sensor name="tmini_plus" type="gpu_lidar">' in text
    assert "<samples>667</samples>" in text
    assert '<xacro:if value="$(arg dual_gpu_lidar)">' in text
    assert '<sensor name="tmini_plus_front" type="gpu_lidar">' in text
    assert '<sensor name="tmini_plus_rear" type="gpu_lidar">' in text
    assert text.count("<samples>334</samples>") == 2
    assert text.count("<min_angle>-1.57079632679</min_angle>") == 2
    assert text.count("<max_angle>1.57079632679</max_angle>") == 2
    assert "<topic>/scan_front</topic>" in text
    assert "<topic>/scan_rear</topic>" in text


def test_rear_scan_frame_is_coincident_and_rotated_by_pi():
    text = URDF.read_text()

    assert '<link name="laser_scan_rear_link"/>' in text
    assert '<joint name="laser_scan_rear_joint" type="fixed">' in text
    assert '<parent link="laser_scan_link"/>' in text
    assert '<child link="laser_scan_rear_link"/>' in text
    assert '<origin xyz="0 0 0" rpy="0 0 3.14159265359"/>' in text
    assert '<gz_frame_id>laser_scan_rear_link</gz_frame_id>' in text


def test_launch_selects_matching_world_and_mutually_exclusive_scan_publishers():
    text = LAUNCH.read_text()

    assert '"render_engine", default_value="ogre"' in text
    assert '"dual_gpu_lidar", default_value="true"' in text
    assert (
        'world_name = "m1_ogre.sdf" if render_engine == "ogre" '
        'else "m1_ogre2_regression.sdf"') in text
    assert '" dual_gpu_lidar:=", LaunchConfiguration("dual_gpu_lidar")' in text
    assert '"/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"' in text
    assert '"/scan_front@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"' in text
    assert '"/scan_rear@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"' in text
    assert 'executable="dual_laser_merger_node"' in text
    assert '"merged_scan_topic": "/scan"' in text
    assert '"target_frame": "laser_scan_link"' in text
    assert '"enable_calibration": False' in text
    assert '"enable_shadow_filter": False' in text
    assert '"enable_average_filter": False' in text


def test_support_package_installs_worlds_and_declares_merger_dependency():
    assert "<exec_depend>dual_laser_merger</exec_depend>" in PACKAGE.read_text()
    assert 'glob("worlds/*.sdf")' in SETUP.read_text()
