#!/usr/bin/env python3
"""Run StarfishBot directly from a source checkout without installation."""

from __future__ import annotations

import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    sys.path.insert(0, str(REPOSITORY_ROOT / "src"))
    from starfishbot.cli import main as cli_main

    return cli_main()


if __name__ == "__main__":
    raise SystemExit(main())
