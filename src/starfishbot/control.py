"""Neural oscillator and motor mapping for StarfishBot."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import ClassVar


class CPG:
    """Two-neuron SO(2) central pattern generator with MI modulation."""

    def __init__(self, mi: float = 0.0) -> None:
        self.mi = mi
        self.w11 = 1.4
        self.w22 = 1.4
        self.w12 = 0.18 + mi
        self.w21 = -0.18 - mi
        self.o1 = 0.01
        self.o2 = 0.01

    def step(self) -> None:
        self.o1 = math.tanh(self.w11 * self.o1 + self.w12 * self.o2)
        self.o2 = math.tanh(self.w22 * self.o2 + self.w21 * self.o1)

    def output(self) -> tuple[float, float]:
        return self.o1, self.o2

    def get_CPG_output(self) -> list[float]:
        """Compatibility alias for the original research scripts."""
        return list(self.output())


class MotorMapper:
    """Map stance states to joint targets using an explicit control mode.

    ``binary`` preserves the original experiment's fixed lift offset. ``cpg``
    scales that offset with the first oscillator output mapped from [-1, 1]
    to [0, 1]. The mode is explicit so oscillator data is never silently
    accepted and ignored.
    """

    VALID_MODES: ClassVar[frozenset[str]] = frozenset({"binary", "cpg"})

    def __init__(
        self,
        num_legs: int,
        num_motors_per_leg: int,
        *,
        mode: str = "binary",
    ) -> None:
        if num_legs <= 0 or num_motors_per_leg <= 0:
            raise ValueError("motor and leg counts must be positive")
        if mode not in self.VALID_MODES:
            raise ValueError(f"mode must be one of {sorted(self.VALID_MODES)}")
        self.num_legs = num_legs
        self.num_motors_per_leg = num_motors_per_leg
        self.mode = mode
        self.lift_offsets = [0.0] * num_motors_per_leg

    def set_lift_offsets(self, offsets: Sequence[float]) -> None:
        if len(offsets) != self.num_motors_per_leg:
            raise ValueError("one lift offset is required for each motor")
        self.lift_offsets = [float(value) for value in offsets]

    def set_weight_leg(self, weights: Sequence[float] = (0.0, 0.0, 0.0)) -> None:
        """Compatibility alias for the original motor mapper."""
        self.set_lift_offsets(weights)

    def map_to_motors(
        self,
        gait_pattern: Sequence[bool],
        joints_bias: Sequence[float],
        *,
        cpg_outputs: Sequence[float] | None = None,
    ) -> list[float]:
        """Return flattened joint targets for every leg.

        ``True`` in ``gait_pattern`` means stance and ``False`` means lifted.
        """
        if len(gait_pattern) != self.num_legs:
            raise ValueError("gait_pattern length must equal num_legs")
        if len(joints_bias) != self.num_motors_per_leg:
            raise ValueError("joints_bias length must equal num_motors_per_leg")

        activation = 1.0
        if self.mode == "cpg":
            if cpg_outputs is None or not cpg_outputs:
                raise ValueError("cpg mode requires at least one oscillator output")
            activation = max(0.0, min(1.0, (float(cpg_outputs[0]) + 1.0) / 2.0))

        commands: list[float] = []
        for in_stance in gait_pattern:
            if in_stance:
                commands.extend(float(value) for value in joints_bias)
            else:
                commands.extend(
                    float(bias) + activation * offset
                    for bias, offset in zip(joints_bias, self.lift_offsets, strict=True)
                )
        return commands
