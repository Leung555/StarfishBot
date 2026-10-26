"""Compatibility import for historical notebooks and scripts."""

from __future__ import annotations

import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "src"))

from starfishbot.gait_planner import GaitPlanner  # noqa: E402,F401

__all__ = ["GaitPlanner"]
