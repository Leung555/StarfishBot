import json
from pathlib import Path

import pytest

from starfishbot.config import load_config

ROOT = Path(__file__).resolve().parents[1]


def test_small_config_uses_robot_defaults(tmp_path):
    settings = tmp_path / "config.json"
    settings.write_text("{}")
    config = load_config(settings)
    assert config.repository_root == ROOT
    assert config.scene_path == ROOT / "scenes/Asymmetry_Leg_gait.ttt"
    assert config.robot.num_legs == 6
    assert config.experiment.seed == 42
    assert config.experiment.samples * config.experiment.steps_per_sample == 10


def test_config_outside_git_checkout_uses_project_paths(tmp_path):
    path = tmp_path / "settings.json"
    path.write_text('{"robot": {"joint_bias_rad": [0, -0.5, 2]}}')
    config = load_config(path)
    assert config.robot.joint_bias_rad == (0, -0.5, 2)
    assert config.scene_path.is_file()
    assert config.repository_root == ROOT


@pytest.mark.parametrize(
    ("section", "field", "value", "message"),
    [
        ("experiment", "samples", 0, "must be positive"),
        ("experiment", "samples", 1.5, "must be an integer"),
        ("experiment", "samples", True, "must be an integer"),
        ("robot", "leg_radius", "large", "must contain only numbers"),
        ("robot", "leg_radius", -1, "leg_radius must be positive"),
        ("robot", "leg_radius", float("nan"), "must be finite"),
        ("gait", "max_legs_lifted", 6, "max_legs_lifted"),
        ("gait", "lifting_period_steps", 0, "periods must be positive"),
        ("gait", "stance_transition_steps", -1, "cannot be negative"),
        ("experiment", "minimum_angle_separation_deg", 61, "impossible"),
        ("experiment", "name", "../escape", "directory name"),
        ("robot", "default_angles_deg", 1, "array"),
    ],
)
def test_invalid_settings_rejected(tmp_path, section, field, value, message):
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps({section: {field: value}}))
    with pytest.raises(ValueError, match=message):
        load_config(path)


@pytest.mark.parametrize("data", [[], {"experimant": {}}, {"robot": 1}])
def test_invalid_sections_are_rejected(tmp_path, data):
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_config(path)
