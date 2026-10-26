# Contributing to StarfishBot

## Development setup

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e ".[analysis,dev]"
```

Before submitting a change, run:

```bash
ruff check .
ruff format --check .
pytest
```

## Research changes

Changes that affect an experiment must include:

- A configuration file or an update to an existing configuration
- A description of any changed assumptions, variables, or units
- Tests for deterministic algorithm changes
- The Git commit, configuration, seed, and simulator version in reported results

Do not overwrite raw results. Create a new run, keep the generated metadata beside the measurements, and archive publication datasets in a DOI-backed repository.

## Notebooks

Keep reusable calculations in `src/starfishbot/`. Notebooks should call those functions and focus on interpretation and figures. Clear accidental debugging output before committing; retain output only when it is part of a reviewed, publication-ready analysis.

## Commit scope

Keep code, scene, configuration, and documentation changes logically separated when practical. Do not commit virtual environments, bytecode, editor settings, generated logs, or complete local result collections.
