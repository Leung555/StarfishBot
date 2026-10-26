"""JSON settings and defaults for the bundled six-legged robot."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import get_type_hints


@dataclass(frozen=True)
class ExperimentSettings:
    name: str = "smoke"
    mode: str = "random_configurations"
    seed: int = 42
    samples: int = 1
    steps_per_sample: int = 10
    angle_min_deg: float = 0.0
    angle_max_deg: float = 360.0
    minimum_angle_separation_deg: float = 25.0


@dataclass(frozen=True)
class RobotSettings:
    num_legs: int = 6
    joints_per_leg: int = 3
    joint_bias_rad: tuple[float, ...] = (0.0, -0.52, 2.09)
    lift_offsets_rad: tuple[float, ...] = (0.0, -0.8, 0.0)
    default_angles_deg: tuple[float, ...] = (0, 60, 120, 180, 240, 300)
    leg_radius: float = 1.0
    robot_path: str = "/hexapod"
    leg_mount_joint_pattern: str = ":/Revolute_joint[{leg}]"
    actuator_joint_pattern: str = ":/joint{joint}_{leg}"


@dataclass(frozen=True)
class GaitSettings:
    max_legs_lifted: int = 1
    lifting_period_steps: int = 40
    fast_lifting_period_steps: int = 4
    stance_transition_steps: int = 20
    stability_threshold: float = 0.2


@dataclass(frozen=True)
class MotorSettings:
    mode: str = "binary"
    cpg_mi: float = 0.0


@dataclass(frozen=True)
class SimulatorSettings:
    scene: str = "scenes/Asymmetry_Leg_gait.ttt"


@dataclass(frozen=True)
class OutputSettings:
    directory: str = "results"


@dataclass(frozen=True)
class RunConfig:
    path: Path
    repository_root: Path
    experiment: ExperimentSettings
    robot: RobotSettings
    gait: GaitSettings
    motor: MotorSettings
    simulator: SimulatorSettings
    output: OutputSettings

    @property
    def scene_path(self) -> Path:
        path = Path(self.simulator.scene)
        return path if path.is_absolute() else self.repository_root / path

    @property
    def output_directory(self) -> Path:
        path = Path(self.output.directory)
        return path if path.is_absolute() else self.repository_root / path


def load_config(path: str | Path) -> RunConfig:
    """Load and validate an experiment configuration file."""
    config_path = Path(path).expanduser().resolve()
    data = json.loads(config_path.read_text(encoding="utf-8"))
    sections = {
        "experiment": ExperimentSettings,
        "robot": RobotSettings,
        "gait": GaitSettings,
        "motor": MotorSettings,
        "simulator": SimulatorSettings,
        "output": OutputSettings,
    }
    if not isinstance(data, dict) or set(data) - sections.keys():
        raise ValueError(
            "settings must be a JSON object with known sections: " + ", ".join(sections)
        )
    settings = {}
    for name, cls in sections.items():
        values = data.get(name, {})
        if not isinstance(values, dict):
            raise ValueError(f"{name} must be a JSON object")
        values = {
            key: tuple(value) if isinstance(value, list) else value for key, value in values.items()
        }
        settings[name] = cls(**values)

    experiment, robot, gait, motor = (
        settings[key] for key in ("experiment", "robot", "gait", "motor")
    )
    simulator, output = settings["simulator"], settings["output"]

    # Dataclasses do not enforce JSON types; reject bad values before connecting.
    for section_name, section in (
        ("experiment", experiment),
        ("robot", robot),
        ("gait", gait),
        ("motor", motor),
        ("simulator", simulator),
        ("output", output),
    ):
        for name, expected_type in get_type_hints(type(section)).items():
            value = getattr(section, name)
            label = f"{section_name}.{name}"
            if expected_type is str and not isinstance(value, str):
                raise ValueError(f"{label} must be a string")
            if expected_type is int and type(value) is not int:
                raise ValueError(f"{label} must be an integer")
            if expected_type == tuple[float, ...] and not isinstance(value, tuple):
                raise ValueError(f"{label} must be an array of numbers")
            if expected_type in (float, tuple[float, ...]):
                values = value if isinstance(value, tuple) else (value,)
                if any(type(item) not in (int, float) for item in values):
                    raise ValueError(f"{label} must contain only numbers")
                if any(not math.isfinite(item) for item in values):
                    raise ValueError(f"{label} must be finite")
    if (
        not experiment.name
        or Path(experiment.name).name != experiment.name
        or experiment.name in {".", ".."}
    ):
        raise ValueError(
            "experiment.name must be a nonempty directory name without path separators"
        )
    if robot.num_legs < 2 or robot.joints_per_leg <= 0:
        raise ValueError("num_legs must be at least 2 and joints_per_leg must be positive")
    if robot.leg_radius <= 0:
        raise ValueError("leg_radius must be positive")
    if not 1 <= gait.max_legs_lifted < robot.num_legs:
        raise ValueError("max_legs_lifted must be between 1 and num_legs - 1")
    if gait.lifting_period_steps <= 0 or gait.fast_lifting_period_steps <= 0:
        raise ValueError("lifting periods must be positive")
    if gait.stance_transition_steps < 0:
        raise ValueError("stance_transition_steps cannot be negative")
    separation = experiment.minimum_angle_separation_deg
    if separation < 0 or separation * robot.num_legs > 360:
        raise ValueError("minimum_angle_separation_deg is impossible for the requested leg count")
    if experiment.mode != "random_configurations":
        raise ValueError("experiment.mode must be random_configurations")
    if experiment.samples <= 0 or experiment.steps_per_sample <= 0:
        raise ValueError("samples and steps_per_sample must be positive")
    if experiment.angle_min_deg >= experiment.angle_max_deg:
        raise ValueError("angle_min_deg must be smaller than angle_max_deg")
    if robot.num_legs != len(robot.default_angles_deg):
        raise ValueError("default_angles_deg length must equal num_legs")
    if robot.joints_per_leg != len(robot.joint_bias_rad):
        raise ValueError("joint_bias_rad length must equal joints_per_leg")
    if robot.joints_per_leg != len(robot.lift_offsets_rad):
        raise ValueError("lift_offsets_rad length must equal joints_per_leg")
    if motor.mode not in {"binary", "cpg"}:
        raise ValueError("motor.mode must be binary or cpg")

    return RunConfig(
        path=config_path,
        repository_root=Path(__file__).resolve().parents[1],
        experiment=experiment,
        robot=robot,
        gait=gait,
        motor=motor,
        simulator=simulator,
        output=output,
    )
