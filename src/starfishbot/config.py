"""Typed TOML configuration for simulator experiments."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class ExperimentSettings:
    name: str
    mode: str
    seed: int
    samples: int
    steps_per_sample: int
    angle_min_deg: float
    angle_max_deg: float
    minimum_angle_separation_deg: float


@dataclass(frozen=True)
class RobotSettings:
    num_legs: int
    joints_per_leg: int
    joint_bias_rad: tuple[float, ...]
    lift_offsets_rad: tuple[float, ...]
    default_angles_deg: tuple[float, ...]
    leg_radius: float
    robot_path: str
    leg_mount_joint_pattern: str
    actuator_joint_pattern: str


@dataclass(frozen=True)
class GaitSettings:
    max_legs_lifted: int
    lifting_period_steps: int
    fast_lifting_period_steps: int
    stance_transition_steps: int
    stability_threshold: float


@dataclass(frozen=True)
class MotorSettings:
    mode: str
    cpg_mi: float


@dataclass(frozen=True)
class SimulatorSettings:
    scene: str


@dataclass(frozen=True)
class OutputSettings:
    directory: str


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


def _repository_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / ".git").exists():
            return candidate
    raise ValueError(f"could not find repository root above {start}")


def _section(data: dict[str, Any], name: str) -> dict[str, Any]:
    value = data.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"missing [{name}] section")
    return value


def load_config(path: str | Path) -> RunConfig:
    """Load and validate an experiment configuration file."""
    config_path = Path(path).expanduser().resolve()
    with config_path.open("rb") as stream:
        data = tomllib.load(stream)

    experiment = ExperimentSettings(**_section(data, "experiment"))
    robot_raw = _section(data, "robot")
    robot = RobotSettings(
        **{
            **robot_raw,
            "joint_bias_rad": tuple(robot_raw["joint_bias_rad"]),
            "lift_offsets_rad": tuple(robot_raw["lift_offsets_rad"]),
            "default_angles_deg": tuple(robot_raw["default_angles_deg"]),
        }
    )
    gait = GaitSettings(**_section(data, "gait"))
    motor = MotorSettings(**_section(data, "motor"))
    simulator = SimulatorSettings(**_section(data, "simulator"))
    output = OutputSettings(**_section(data, "output"))

    if experiment.mode not in {"random_configurations", "angle_schedule"}:
        raise ValueError("experiment.mode must be random_configurations or angle_schedule")
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
        repository_root=_repository_root(config_path.parent),
        experiment=experiment,
        robot=robot,
        gait=gait,
        motor=motor,
        simulator=simulator,
        output=output,
    )
