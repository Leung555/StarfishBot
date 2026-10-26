# Methodology

## Research question

StarfishBot investigates how the angular arrangement of six reconfigurable legs affects a geometric support-margin proxy and the gait schedule selected by a stability-aware controller.

## Idealised contact model

For leg `i`, mounting angle `theta_i`, and configured leg radius `r`, the idealised planar contact is:

```text
p_i = (r cos(theta_i), r sin(theta_i))
```

For a commanded lifted-leg set `L`, the support geometry is the convex hull of all `p_i` where `i` is not in `L`. The assumed centre-of-mass projection is fixed at `(0, 0)` in this idealised robot frame.

The signed proxy margin is the shortest distance from the assumed centre to the hull boundary. It is positive inside the support polygon, negative outside, and zero on the boundary or on a degenerate support geometry.

## Gait selection

At each controller step, the planner:

1. Updates active lift timers.
2. Determines the currently scheduled lifted legs.
3. Tests the proxy margin produced by additionally lifting each eligible leg.
4. Selects the candidate with the largest margin.
5. Uses the normal lift duration when the margin exceeds the configured threshold and the fast duration otherwise.
6. Returns the post-update command and lifted-leg states used for both motor commands and logging.

Only the configured maximum number of legs may be scheduled as lifted at once. A cycle rule prevents a leg from being selected again until all legs have been selected once.

## Motor control

Two explicit modes are supported:

- `binary`: reproduces the original experiment by adding a fixed joint offset to a commanded lifted leg.
- `cpg`: transforms the first SO(2) oscillator output from `[-1, 1]` to `[0, 1]` and uses it to scale the configured lift offset.

The `cpg` mapping is an experimental control rule, not a validated biological model. Results from different motor modes must not be combined without recording and analysing the mode as an experimental factor.

## Simulation timing

For every measurement row, the runner selects a gait state, sends joint targets, advances CoppeliaSim by one stepping-mode step, and then records simulation time and orientation. The support proxy describes the commanded lifted-leg schedule; it does not confirm physical contact.

## Orientation

`sim.getObjectOrientation` returns CoppeliaSim Tait-Bryan angles `[alpha, beta, gamma]`. These values are not directly relabelled as roll, pitch, and yaw. The runner records the native angles and calls `sim.alphaBetaGammaToYawPitchRoll`, whose output order is yaw, pitch, roll.

References:

- [CoppeliaSim positions and orientations](https://manual.coppeliarobotics.com/en/positionOrientationTransformation.htm)
- [`sim.getObjectOrientation`](https://manual.coppeliarobotics.com/en/sim/simGetObjectOrientation.htm)
- [`sim.alphaBetaGammaToYawPitchRoll`](https://manual.coppeliarobotics.com/en/sim/simAlphaBetaGammaToYawPitchRoll.htm)

## Limitations

The current margin is a geometric proxy rather than a physical static or dynamic stability margin because:

- Foot contacts are inferred from mounting angles and a constant radius.
- The centre-of-mass projection is fixed at the origin.
- Contact state, compliance, slip, inertial effects, and external disturbances are not measured.
- Commanded lift state may differ from physical contact state during transitions.

Conclusions must use language such as “idealised geometric support margin” until the proxy is validated against simulated contact points, centre-of-mass projection, and tipping outcomes.
