"""Legacy entry point for the angle-change experiment.

Prefer ``python scripts/run_experiment.py --config configs/angle_change.toml``.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from starfishbot.cli import main  # noqa: E402


if __name__ == "__main__":
    arguments = ["--config", str(REPOSITORY_ROOT / "configs" / "angle_change.toml")]
    arguments.extend(sys.argv[1:])
    raise SystemExit(main(arguments))
