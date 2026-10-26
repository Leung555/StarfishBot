"""Stability-aware leg scheduling."""

from __future__ import annotations

from collections.abc import Sequence

from .stability import signed_stability_margin


class GaitPlanner:
    """Schedule lifted legs using an idealised geometric stability proxy."""

    def __init__(
        self,
        angles: Sequence[float],
        leg_radius: float = 1.0,
        max_legs_lifted: int = 1,
        lifting_period: int = 20,
        *,
        fast_lifting_period: int = 4,
        stance_transition_time: int = 20,
    ) -> None:
        if not angles:
            raise ValueError("at least one leg is required")
        if not 1 <= max_legs_lifted < len(angles):
            raise ValueError("max_legs_lifted must be between 1 and num_legs - 1")
        if lifting_period <= 0 or fast_lifting_period <= 0:
            raise ValueError("lifting periods must be positive")
        if stance_transition_time < 0:
            raise ValueError("stance_transition_time cannot be negative")

        self.num_legs = len(angles)
        self.leg_radius = float(leg_radius)
        self.max_legs_lifted = max_legs_lifted
        self.lifting_period = lifting_period
        self.fast_lifting_period = fast_lifting_period
        self.stance_transition_time = stance_transition_time
        self.lifted_legs_timer = {index: 0 for index in range(self.num_legs)}
        self.leg_stance_state = {index: True for index in range(self.num_legs)}
        self.leg_stance_state_commands = {index: True for index in range(self.num_legs)}
        self.this_cycle_lifted: list[int] = []

    def _lift_leg(self, leg_index: int, lifting_period: int) -> None:
        self.lifted_legs_timer[leg_index] = lifting_period
        self.leg_stance_state[leg_index] = False
        self.leg_stance_state_commands[leg_index] = False
        self.this_cycle_lifted.append(leg_index)
        if len(self.this_cycle_lifted) == self.num_legs:
            self.this_cycle_lifted.clear()

    def _update_lift_timers(self) -> None:
        for leg, remaining in self.lifted_legs_timer.items():
            if remaining <= 0:
                continue
            remaining -= 1
            self.lifted_legs_timer[leg] = remaining
            if remaining == 0:
                self.leg_stance_state[leg] = True
                self.leg_stance_state_commands[leg] = True
            elif remaining < self.stance_transition_time:
                self.leg_stance_state_commands[leg] = True

    def step(
        self, angles: Sequence[float], stability_threshold: float = 0.2
    ) -> tuple[list[bool], list[int]]:
        """Advance the schedule and return motor commands plus scheduled exclusions.

        During landing, a leg receives a stance command but remains excluded
        from the support proxy until its lift timer expires.
        """
        if len(angles) != self.num_legs:
            raise ValueError("the number of angles cannot change after initialization")

        self._update_lift_timers()
        lifted = [leg for leg, in_stance in self.leg_stance_state.items() if not in_stance]

        if len(lifted) < self.max_legs_lifted:
            candidates: list[tuple[int, float]] = []
            for leg in range(self.num_legs):
                if leg in lifted or leg in self.this_cycle_lifted:
                    continue
                margin = signed_stability_margin(
                    angles,
                    set(lifted) | {leg},
                    leg_radius=self.leg_radius,
                )
                candidates.append((leg, margin))

            if candidates:
                leg, margin = max(candidates, key=lambda item: item[1])
                period = (
                    self.lifting_period
                    if margin > stability_threshold
                    else self.fast_lifting_period
                )
                self._lift_leg(leg, period)

        # Recompute after state transitions and the possible new lift. This fixes
        # the original one-step mismatch between commands, logs, and margins.
        lifted = [leg for leg, in_stance in self.leg_stance_state.items() if not in_stance]
        commands = [self.leg_stance_state_commands[index] for index in range(self.num_legs)]
        return commands, lifted
