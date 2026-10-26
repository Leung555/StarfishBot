"""Configuration-driven CoppeliaSim experiment runner."""

from __future__ import annotations

import csv
import logging
import math
import random
import shutil
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .config import RunConfig
from .control import CPG, MotorMapper
from .gait_planner import GaitPlanner
from .metadata import base_metadata, write_metadata
from .sampling import generate_random_angles
from .stability import per_leg_test_margins, signed_stability_margin

LOGGER = logging.getLogger("starfishbot.experiment")


def _create_run_directory(config: RunConfig) -> Path:
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    run_directory = config.output_directory / f"{config.experiment.name}-{timestamp}"
    run_directory.mkdir(parents=True, exist_ok=False)
    return run_directory


def _configure_logging(log_path: Path, verbose: bool) -> None:
    LOGGER.setLevel(logging.DEBUG)
    for handler in LOGGER.handlers[:]:
        handler.close()
        LOGGER.removeHandler(handler)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    LOGGER.addHandler(file_handler)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.DEBUG if verbose else logging.INFO)
    console_handler.setFormatter(formatter)
    LOGGER.addHandler(console_handler)


def _coppeliasim_version(sim: Any) -> int | str | None:
    try:
        return sim.getIntProperty(sim.handle_app, "productVersionNb")
    except Exception:
        try:
            return sim.getInt32Param(sim.intparam_program_version)
        except Exception:
            return None


def _orientation_degrees(sim: Any, robot_handle: int) -> dict[str, float]:
    world = getattr(sim, "handle_world", -1)
    alpha, beta, gamma = sim.getObjectOrientation(robot_handle, world)
    yaw, pitch, roll = sim.alphaBetaGammaToYawPitchRoll(alpha, beta, gamma)
    return {
        "alpha_deg": math.degrees(alpha),
        "beta_deg": math.degrees(beta),
        "gamma_deg": math.degrees(gamma),
        "yaw_deg": math.degrees(yaw),
        "pitch_deg": math.degrees(pitch),
        "roll_deg": math.degrees(roll),
    }


def _set_leg_angles(sim: Any, handles: list[int], angles_deg: list[float]) -> None:
    for handle, angle in zip(handles, angles_deg, strict=True):
        wrapped_angle = angle - 360.0 if angle > 180.0 else angle
        sim.setJointTargetPosition(handle, math.radians(wrapped_angle))


def _object_handles(sim: Any, config: RunConfig) -> tuple[int, list[int], list[int]]:
    robot_handle = sim.getObject(config.robot.robot_path)
    mount_handles = [
        sim.getObject(config.robot.leg_mount_joint_pattern.format(leg=leg))
        for leg in range(config.robot.num_legs)
    ]
    actuator_handles = [
        sim.getObject(config.robot.actuator_joint_pattern.format(joint=joint + 1, leg=leg))
        for leg in range(config.robot.num_legs)
        for joint in range(config.robot.joints_per_leg)
    ]
    return robot_handle, mount_handles, actuator_handles


def _csv_header(config: RunConfig) -> list[str]:
    header = ["sample_index", "step_index", "global_step", "sim_time_s"]
    header.extend(f"angle_{index}_deg" for index in range(config.robot.num_legs))
    header.extend(
        [
            "proxy_support_margin_radius",
            "commanded_lifted_legs",
            "scheduled_lifted_legs",
            "min_sample_proxy_margin_radius",
            "mean_sample_proxy_margin_radius",
        ]
    )
    header.extend(f"leg_{index}_test_margin_radius" for index in range(config.robot.num_legs))
    header.extend(["alpha_deg", "beta_deg", "gamma_deg", "yaw_deg", "pitch_deg", "roll_deg"])
    return header


def _run_samples(
    sim: Any,
    writer: csv.DictWriter[str],
    config: RunConfig,
    robot_handle: int,
    mount_handles: list[int],
    actuator_handles: list[int],
) -> int:
    rng = random.Random(config.experiment.seed)
    global_step = 0
    total_steps = config.experiment.samples * config.experiment.steps_per_sample

    motor_mapper = MotorMapper(
        config.robot.num_legs,
        config.robot.joints_per_leg,
        mode=config.motor.mode,
    )
    motor_mapper.set_lift_offsets(config.robot.lift_offsets_rad)

    for sample_index in range(config.experiment.samples):
        angles = generate_random_angles(
            rng,
            config.robot.num_legs,
            angle_min_deg=config.experiment.angle_min_deg,
            angle_max_deg=config.experiment.angle_max_deg,
            minimum_separation_deg=config.experiment.minimum_angle_separation_deg,
        )
        _set_leg_angles(sim, mount_handles, angles)
        planner = GaitPlanner(
            angles,
            leg_radius=config.robot.leg_radius,
            max_legs_lifted=config.gait.max_legs_lifted,
            lifting_period=config.gait.lifting_period_steps,
            fast_lifting_period=config.gait.fast_lifting_period_steps,
            stance_transition_time=config.gait.stance_transition_steps,
        )
        cpg = CPG(config.motor.cpg_mi) if config.motor.mode == "cpg" else None
        leg_test_margins = per_leg_test_margins(angles, leg_radius=config.robot.leg_radius)
        sample_rows: list[dict[str, Any]] = []

        for step_index in range(config.experiment.steps_per_sample):
            gait_commands, lifted_legs = planner.step(angles, config.gait.stability_threshold)
            cpg_outputs = None
            if cpg is not None:
                cpg.step()
                cpg_outputs = cpg.output()
            motor_commands = motor_mapper.map_to_motors(
                gait_commands,
                config.robot.joint_bias_rad,
                cpg_outputs=cpg_outputs,
            )
            for handle, command in zip(actuator_handles, motor_commands, strict=True):
                sim.setJointTargetPosition(handle, command)

            sim.step()
            orientation = _orientation_degrees(sim, robot_handle)
            margin = signed_stability_margin(
                angles,
                lifted_legs,
                leg_radius=config.robot.leg_radius,
            )
            row: dict[str, Any] = {
                "sample_index": sample_index,
                "step_index": step_index,
                "global_step": global_step,
                "sim_time_s": sim.getSimulationTime(),
                "proxy_support_margin_radius": margin,
                "commanded_lifted_legs": ";".join(
                    str(leg) for leg, in_stance in enumerate(gait_commands) if not in_stance
                ),
                "scheduled_lifted_legs": ";".join(map(str, lifted_legs)),
                **orientation,
            }
            row.update({f"angle_{index}_deg": angle for index, angle in enumerate(angles)})
            row.update(
                {
                    f"leg_{index}_test_margin_radius": value
                    for index, value in enumerate(leg_test_margins)
                }
            )
            sample_rows.append(row)
            global_step += 1
            if global_step % 200 == 0:
                LOGGER.info(
                    "step=%d/%d sample=%d/%d sim_time=%.2fs",
                    global_step,
                    total_steps,
                    sample_index + 1,
                    config.experiment.samples,
                    row["sim_time_s"],
                )

        margins = [float(row["proxy_support_margin_radius"]) for row in sample_rows]
        minimum = min(margins)
        mean = sum(margins) / len(margins)
        for row in sample_rows:
            row["min_sample_proxy_margin_radius"] = minimum
            row["mean_sample_proxy_margin_radius"] = mean
            writer.writerow(row)
        LOGGER.info(
            "sample=%d/%d angles=%s min_margin=%.5f mean_margin=%.5f",
            sample_index + 1,
            config.experiment.samples,
            angles,
            minimum,
            mean,
        )
    return global_step


def run_experiment(config: RunConfig, *, verbose: bool = False) -> Path:
    """Run one experiment and return its timestamped result directory."""
    if not config.scene_path.is_file():
        raise FileNotFoundError(f"scene not found: {config.scene_path}")

    run_directory = _create_run_directory(config)
    _configure_logging(run_directory / "simulation.log", verbose)
    shutil.copy2(config.path, run_directory / "config.json")
    metadata = base_metadata(config)
    metadata_path = run_directory / "metadata.json"
    write_metadata(metadata_path, metadata)

    simulation_started = False
    sim = None
    mount_handles: list[int] = []
    try:
        from coppeliasim_zmqremoteapi_client import RemoteAPIClient

        client = RemoteAPIClient()
        sim = client.require("sim")
        LOGGER.info("loading scene %s", config.scene_path)
        sim.loadScene(str(config.scene_path))
        robot_handle, mount_handles, actuator_handles = _object_handles(sim, config)
        metadata["coppeliasim_version"] = _coppeliasim_version(sim)
        metadata["orientation_convention"] = {
            "source": "sim.getObjectOrientation relative to world",
            "native": "CoppeliaSim alpha-beta-gamma Tait-Bryan angles",
            "converted": "sim.alphaBetaGammaToYawPitchRoll returns yaw, pitch, roll",
        }
        sim.setStepping(True)
        sim.startSimulation()
        simulation_started = True

        measurements_path = run_directory / "measurements.csv"
        header = _csv_header(config)
        with measurements_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=header)
            writer.writeheader()
            total_steps = _run_samples(
                sim,
                writer,
                config,
                robot_handle,
                mount_handles,
                actuator_handles,
            )

        metadata["status"] = "completed"
        metadata["completed_at_utc"] = datetime.now(timezone.utc).isoformat()
        metadata["total_steps"] = total_steps
        LOGGER.info("completed %d steps; results=%s", total_steps, run_directory)
    except Exception as error:
        metadata["status"] = "failed"
        metadata["failed_at_utc"] = datetime.now(timezone.utc).isoformat()
        metadata["error"] = f"{type(error).__name__}: {error}"
        LOGGER.exception("experiment failed")
        raise
    finally:
        if sim is not None and mount_handles:
            try:
                _set_leg_angles(sim, mount_handles, list(config.robot.default_angles_deg))
            except Exception:
                LOGGER.exception("could not restore default leg angles")
        if sim is not None and simulation_started:
            try:
                sim.stopSimulation()
            except Exception:
                LOGGER.exception("could not stop simulation cleanly")
        write_metadata(metadata_path, metadata)
    return run_directory
