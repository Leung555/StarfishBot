"""Command-line interface for StarfishBot experiments."""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from .config import load_config
from .experiment import run_experiment


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="path to an experiment TOML file")
    parser.add_argument("--verbose", action="store_true", help="show detailed progress")
    return parser


def main(arguments: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(arguments)
    output = run_experiment(load_config(args.config), verbose=args.verbose)
    print(output)
    return 0
