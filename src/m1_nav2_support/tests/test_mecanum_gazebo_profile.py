"""Static checks for the simulation-only Mecanum contact profile."""

from pathlib import Path


ROOT = Path(__file__).parents[3]
URDF = ROOT / "src" / "yahboomcar_description" / "urdf" / "yahboomcar_M1_gazebo.urdf.xacro"
WORLD = ROOT / "src" / "m1_nav2_support" / "worlds" / "m1.sdf"


def test_all_mecanum_wheels_release_the_roller_direction():
    text = URDF.read_text()
    assert text.count("<mu2>0.0</mu2>") == 4
    assert "<mu2>0.1</mu2>" not in text


def test_x_pattern_friction_directions_are_preserved():
    text = URDF.read_text()
    assert text.count("<fdir1 ignition:expressed_in=\"base_footprint\">1 -1 0</fdir1>") == 2
    assert text.count("<fdir1 ignition:expressed_in=\"base_footprint\">1 1 0</fdir1>") == 2


def test_ground_has_explicit_high_friction():
    text = WORLD.read_text()
    assert "<mu>50.0</mu>" in text
