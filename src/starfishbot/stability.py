"""Geometric stability proxy used by the StarfishBot gait planner.

This module deliberately calls the result a *proxy*. It assumes that foot
contacts lie on a circle around a fixed centre-of-mass projection. It does not
query physical foot contacts or the dynamic centre of mass from CoppeliaSim.
"""

from __future__ import annotations

import math
from collections.abc import Iterable, Sequence

from shapely.geometry import MultiPoint, Point
from shapely.geometry.base import BaseGeometry


def contact_points(
    angles_deg: Sequence[float], leg_radius: float = 1.0
) -> list[tuple[float, float]]:
    """Convert leg mounting angles to idealised planar contact points."""
    if leg_radius <= 0:
        raise ValueError("leg_radius must be positive")
    if not angles_deg:
        raise ValueError("at least one leg angle is required")
    return [
        (
            leg_radius * math.cos(math.radians(angle)),
            leg_radius * math.sin(math.radians(angle)),
        )
        for angle in angles_deg
    ]


def support_geometry(
    angles_deg: Sequence[float],
    excluded_legs: Iterable[int] = (),
    leg_radius: float = 1.0,
) -> BaseGeometry:
    """Return the convex hull of all idealised contacts still in stance."""
    excluded = set(excluded_legs)
    invalid = [index for index in excluded if index < 0 or index >= len(angles_deg)]
    if invalid:
        raise IndexError(f"leg indices out of range: {sorted(invalid)}")
    points = [
        point
        for index, point in enumerate(contact_points(angles_deg, leg_radius))
        if index not in excluded
    ]
    if not points:
        return MultiPoint([])
    return MultiPoint(points).convex_hull


def signed_margin_from_geometry(
    geometry: BaseGeometry, centre: tuple[float, float] = (0.0, 0.0)
) -> float:
    """Return distance to the hull boundary, negative when outside the hull.

    A line, point, or empty hull has no support area and is therefore treated
    as unstable. A centre exactly on a degenerate hull has a margin of zero.
    """
    centre_point = Point(centre)
    if geometry.is_empty:
        return float("-inf")
    if geometry.geom_type != "Polygon":
        return -float(geometry.distance(centre_point))
    distance = float(geometry.boundary.distance(centre_point))
    return distance if geometry.covers(centre_point) else -distance


def signed_stability_margin(
    angles_deg: Sequence[float],
    excluded_legs: Iterable[int] = (),
    *,
    leg_radius: float = 1.0,
    centre: tuple[float, float] = (0.0, 0.0),
) -> float:
    """Calculate the signed geometric stability proxy for a leg arrangement."""
    geometry = support_geometry(angles_deg, excluded_legs, leg_radius)
    return signed_margin_from_geometry(geometry, centre)


def per_leg_test_margins(
    angles_deg: Sequence[float],
    *,
    already_lifted: Iterable[int] = (),
    leg_radius: float = 1.0,
    centre: tuple[float, float] = (0.0, 0.0),
) -> list[float]:
    """Calculate the proxy margin produced by additionally lifting each leg."""
    lifted = set(already_lifted)
    return [
        signed_stability_margin(
            angles_deg,
            lifted | {index},
            leg_radius=leg_radius,
            centre=centre,
        )
        for index in range(len(angles_deg))
    ]
