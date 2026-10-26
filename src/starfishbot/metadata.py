"""Reproducibility metadata helpers."""

from __future__ import annotations

import hashlib
import importlib.metadata
import json
import platform
import subprocess
import sys
from datetime import UTC, datetime
from pathlib import Path
from typing import Any


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _git(repository_root: Path, *arguments: str) -> str | None:
    try:
        result = subprocess.run(
            ["git", *arguments],
            cwd=repository_root,
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip()


def dependency_versions(names: tuple[str, ...]) -> dict[str, str | None]:
    versions: dict[str, str | None] = {}
    for name in names:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    return versions


def _display_path(path: Path, repository_root: Path) -> str:
    try:
        return str(path.relative_to(repository_root))
    except ValueError:
        return str(path)


def base_metadata(config: Any) -> dict[str, Any]:
    status = _git(config.repository_root, "status", "--porcelain")
    return {
        "schema_version": 1,
        "created_at_utc": datetime.now(UTC).isoformat(),
        "status": "initializing",
        "experiment_name": config.experiment.name,
        "experiment_mode": config.experiment.mode,
        "random_seed": config.experiment.seed,
        "git": {
            "commit": _git(config.repository_root, "rev-parse", "HEAD"),
            "branch": _git(config.repository_root, "branch", "--show-current"),
            "remote": _git(config.repository_root, "remote", "get-url", "origin"),
            "dirty": bool(status),
        },
        "environment": {
            "python": sys.version,
            "platform": platform.platform(),
            "dependencies": dependency_versions(
                (
                    "coppeliasim-zmqremoteapi-client",
                    "shapely",
                    "numpy",
                    "matplotlib",
                    "pandas",
                )
            ),
        },
        "files": {
            "config": _display_path(config.path, config.repository_root),
            "config_sha256": file_sha256(config.path),
            "scene": _display_path(config.scene_path, config.repository_root),
            "scene_sha256": file_sha256(config.scene_path),
        },
        "method_assumptions": [
            "Foot contacts are idealised on a circle at the configured leg radius.",
            "The centre-of-mass projection is fixed at the robot-frame origin.",
            "The reported stability margin is a geometric proxy, not a dynamic tipping margin.",
            "Lifted legs are commanded states and are not confirmed by contact sensors.",
        ],
    }


def write_metadata(path: Path, metadata: dict[str, Any]) -> None:
    path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
