# Experiment protocol

## Before a run

1. Check out or record the intended Git commit.
2. Ensure the working tree is clean, or document why a dirty state is required.
3. Create a new TOML configuration instead of editing a published run’s copied configuration.
4. Record a deliberate random seed.
5. Start CoppeliaSim and confirm the ZeroMQ Remote API service is available.
6. Verify the scene loads and all configured object paths resolve.

## Execute

```bash
python scripts/run_experiment.py --config configs/random_leg_angles.toml
```

Do not reuse or overwrite an existing run directory. The runner creates a timestamped directory and always writes the final success or failure status to its metadata.

## Quality checks after a run

1. Confirm `metadata.json` reports `completed`.
2. Confirm the expected row count: `samples × steps_per_sample`.
3. Confirm `global_step` is continuous and `sim_time_s` is monotonic.
4. Inspect missing, infinite, and out-of-range values.
5. Plot the native and converted orientation columns as a convention check.
6. Preserve the config, metadata, measurements, and log together.

## Publication

Archive the exact dataset with a checksum and DOI. Report the code commit, configuration filename, seed set, CoppeliaSim version, scene checksum, number of independent runs, exclusion criteria, and statistical method. Add the DOI to `README.md` and `CITATION.cff`.
