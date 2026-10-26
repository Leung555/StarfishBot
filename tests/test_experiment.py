import math

import pytest

from starfishbot.experiment import _orientation_degrees


class FakeSim:
    handle_world = -1

    def getObjectOrientation(self, robot_handle: int, relative_to: int) -> list[float]:
        assert robot_handle == 10
        assert relative_to == self.handle_world
        return [0.1, 0.2, 0.3]

    def alphaBetaGammaToYawPitchRoll(self, alpha: float, beta: float, gamma: float) -> list[float]:
        assert (alpha, beta, gamma) == (0.1, 0.2, 0.3)
        return [0.4, 0.5, 0.6]


def test_orientation_uses_official_conversion_and_return_order() -> None:
    orientation = _orientation_degrees(FakeSim(), 10)
    assert orientation == pytest.approx(
        {
            "alpha_deg": math.degrees(0.1),
            "beta_deg": math.degrees(0.2),
            "gamma_deg": math.degrees(0.3),
            "yaw_deg": math.degrees(0.4),
            "pitch_deg": math.degrees(0.5),
            "roll_deg": math.degrees(0.6),
        }
    )
