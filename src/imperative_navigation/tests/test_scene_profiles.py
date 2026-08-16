from pathlib import Path

from imperative_navigation.scene_profiles import get_scene_profile


def test_house_and_cafe_have_distinct_dynamic_shapes_and_routes():
    for scene in ("house", "cafe"):
        profile = get_scene_profile(scene)
        assert profile["world_name"] == scene
        assert len(profile["obstacles"]) == 3
        assert [obstacle["shape"] for obstacle in profile["obstacles"]] == [
            "cylinder", "box", "long_box"
        ]
        assert all(len(obstacle["route"]) >= 4 for obstacle in profile["obstacles"])
        assert len(profile["dynamic_radii"]) == 3


def test_scene_world_and_dynamic_models_are_packaged_in_source_tree():
    package_root = Path(__file__).parents[1]
    assert (package_root / "worlds" / "house.world").is_file()
    assert (package_root / "worlds" / "cafe.world").is_file()
    for model in (
        "imperative_dynamic_cylinder",
        "imperative_dynamic_box",
        "imperative_dynamic_long_box",
    ):
        assert (package_root / "models" / model / "model.sdf").is_file()
        assert (package_root / "models" / model / "model.config").is_file()
