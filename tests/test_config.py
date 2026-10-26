from pathlib import Path

from starfishbot.config import load_config

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def test_reference_config_loads_with_relative_paths() -> None:
    config = load_config(REPOSITORY_ROOT / "configs" / "random_leg_angles.toml")
    assert config.repository_root == REPOSITORY_ROOT
    assert config.scene_path == REPOSITORY_ROOT / "scenes" / "Asymmetry_Leg_gait.ttt"
    assert config.robot.num_legs == 6
    assert config.experiment.seed == 20260115
