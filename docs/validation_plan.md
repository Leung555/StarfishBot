# Validation plan

The automated tests verify deterministic software behavior, but they do not validate the physical interpretation of the stability proxy. Complete the following before treating it as a physical stability measure.

## Geometry validation

- Compare idealised contact points against CoppeliaSim foot positions for symmetric and asymmetric configurations.
- Calculate the projected simulated centre of mass and compare it with the assumed origin.
- Compare the proxy margin with a margin calculated from actual contacting feet.
- Test boundary, collinear, and near-degenerate contact arrangements.

## State validation

- Add or identify contact sensors for every foot.
- Compare commanded lifted legs with detected contact loss and restoration.
- Measure transition delays and revise the stance-transition model accordingly.

## Outcome validation

- Apply controlled disturbances at multiple proxy margins.
- Record tipping, recovery, slip, and body orientation.
- Estimate how well the proxy predicts each outcome with confidence intervals.
- Repeat trials under multiple seeds and initial conditions.

## Control validation

- Evaluate `binary` and `cpg` motor modes as separate treatments.
- Verify joint target limits and sign conventions for every leg.
- Add trajectory plots demonstrating the expected CPG phase and lift profile.

Until these checks are complete, use “geometric support-margin proxy” in figures, tables, and conclusions.
