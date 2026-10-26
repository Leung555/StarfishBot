import math

import pytest

from starfishbot.stability import signed_stability_margin

SYMMETRIC_HEXAPOD = [0, 60, 120, 180, 240, 300]


def test_symmetric_hexapod_margin_is_inradius() -> None:
    margin = signed_stability_margin(SYMMETRIC_HEXAPOD)
    assert margin == pytest.approx(math.sqrt(3) / 2)


def test_lifting_one_symmetric_leg_reduces_margin() -> None:
    margin = signed_stability_margin(SYMMETRIC_HEXAPOD, {0})
    assert margin == pytest.approx(0.5)


def test_centre_outside_support_is_negative() -> None:
    margin = signed_stability_margin([0, 30, 60])
    assert margin < 0


def test_invalid_leg_index_is_rejected() -> None:
    with pytest.raises(IndexError):
        signed_stability_margin(SYMMETRIC_HEXAPOD, {6})


def test_degenerate_hull_margin_is_negative_distance():
    from shapely.geometry import LineString, MultiPoint, Point

    from starfishbot.stability import signed_margin_from_geometry

    assert signed_margin_from_geometry(LineString([(-1, 0), (1, 0)])) == 0
    assert signed_margin_from_geometry(LineString([(-1, 1), (1, 1)])) == -1
    assert signed_margin_from_geometry(Point(2, 0)) == -2
    assert signed_margin_from_geometry(MultiPoint([])) == -math.inf
