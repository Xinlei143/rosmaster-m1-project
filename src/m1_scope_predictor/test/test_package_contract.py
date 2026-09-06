from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src" / "m1_scope_predictor"


def test_predictor_entry_point_and_launch_assets_are_installed():
    setup = (PACKAGE / "setup.py").read_text()
    assert "scope_predictor = m1_scope_predictor.predictor_node:main" in setup
    assert "scope_runtime_parity = m1_scope_predictor.parity:main" in setup
    assert "launch/scope_online.launch.py" in setup
    assert "config/scope_online.yaml" in setup
    assert "rviz/scope_online.rviz" in setup


def test_predictor_is_observer_only():
    source = (PACKAGE / "m1_scope_predictor" / "predictor_node.py").read_text()
    assert "Twist" not in source
    assert '"/scope/current_ogm"' in source
    assert '"/scope/prediction"' in source
    assert '"/scope/uncertainty"' in source
    assert '"/scope/diagnostics"' in source


def test_parity_module_can_run_as_a_script():
    source = (PACKAGE / "m1_scope_predictor" / "parity.py").read_text()
    assert 'if __name__ == "__main__":' in source
