"""Load the original planner source packaged alongside this ROS node."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def load_algorithm():
    algorithm_path = Path(get_package_share_directory("imperative_navigation")) / "algorithm" / "Imperative_learning_2D_moving.py"
    spec = spec_from_file_location("imperative_planner", algorithm_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load planner source: {algorithm_path}")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
