# Notebooks

These notebooks preserve exploratory analysis from the prototype repository. They contain saved outputs and may refer to historical CSV column names and local datasets under `asymmetry_leg_config/data/`.

Before using a notebook for a publication result:

1. Update it to read a selected timestamped run from `results/`.
2. Map columns using `docs/data_dictionary.md`.
3. Move reusable transformations into `src/starfishbot/` and add tests.
4. Run the notebook from a clean kernel from top to bottom.
5. Retain output only for a reviewed, publication-ready result.

The repository’s automated lint and unit-test jobs intentionally exclude exploratory notebook cells. Analysis correctness must be reviewed separately.
