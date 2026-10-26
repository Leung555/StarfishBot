"""Deterministic sampling helpers for leg configurations."""

from __future__ import annotations

import random


def circular_angle_separations(angles_deg: list[float]) -> list[float]:
    """Return the smaller circular separation between adjacent sorted angles."""
    ordered = sorted(angle % 360.0 for angle in angles_deg)
    if len(ordered) < 2:
        return []
    return [
        (ordered[(index + 1) % len(ordered)] - angle) % 360.0 for index, angle in enumerate(ordered)
    ]


def generate_random_angles(
    rng: random.Random,
    num_legs: int,
    *,
    angle_min_deg: float = 0.0,
    angle_max_deg: float = 360.0,
    minimum_separation_deg: float = 0.0,
    max_attempts: int = 10_000,
) -> list[float]:
    """Sample sorted angles with a reproducible random generator."""
    if num_legs <= 0:
        raise ValueError("num_legs must be positive")
    if minimum_separation_deg < 0:
        raise ValueError("minimum_separation_deg cannot be negative")
    if minimum_separation_deg * num_legs > 360.0:
        raise ValueError("minimum separation is impossible for the requested leg count")

    for _ in range(max_attempts):
        angles = sorted(round(rng.uniform(angle_min_deg, angle_max_deg)) for _ in range(num_legs))
        if all(
            separation >= minimum_separation_deg
            for separation in circular_angle_separations(angles)
        ):
            return angles
    raise RuntimeError(f"could not sample valid angles after {max_attempts} attempts")
