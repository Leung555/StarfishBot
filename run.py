"""Command-line interface for StarfishBot experiments."""

from __future__ import annotations

import argparse
import sys
from collections.abc import Sequence
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("config.json")),
        help="JSON settings file (default: config.json beside run.py)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="validate configuration, scene file, and sampling without connecting to CoppeliaSim",
    )
    parser.add_argument("--verbose", action="store_true", help="show detailed progress")
    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(arguments)
    try:
        from starfishbot.config import load_config
        from starfishbot.experiment import run_experiment
    except ModuleNotFoundError as error:
        requirements = Path(__file__).with_name("requirements.txt")
        parser.error(
            f"Missing dependency: {error.name}. Install with: "
            f'"{sys.executable}" -m pip install -r "{requirements}"'
        )
    try:
        config = load_config(args.config)
        if not config.scene_path.is_file():
            raise ValueError(f"scene not found: {config.scene_path}")
        if args.check:
            import random

            from starfishbot.sampling import generate_random_angles

            rng = random.Random(config.experiment.seed)
            for _ in range(config.experiment.samples):
                generate_random_angles(
                    rng,
                    config.robot.num_legs,
                    angle_min_deg=config.experiment.angle_min_deg,
                    angle_max_deg=config.experiment.angle_max_deg,
                    minimum_separation_deg=config.experiment.minimum_angle_separation_deg,
                )
            print(f"Configuration OK: {config.experiment.name}")
            print(f"Scene: {config.scene_path}")
            rows = config.experiment.samples * config.experiment.steps_per_sample
            print(f"Expected measurement rows: {rows}")
            return 0
    except (OSError, ValueError, TypeError, KeyError, RuntimeError) as error:
        parser.error(str(error))
    output = run_experiment(config, verbose=args.verbose)
    print(output)
    return 0


if __name__ == "__main__":
    if sys.version_info < (3, 10):  # noqa: UP036 -- source checkout can use older Python
        raise SystemExit("StarfishBot needs Python 3.10 or newer. Check your Python interpreter.")
    raise SystemExit(main())
