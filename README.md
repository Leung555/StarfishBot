# StarfishBot

StarfishBot is a reproducible CoppeliaSim research project for studying how asymmetric leg arrangements affect gait scheduling and an idealised geometric support-margin proxy in a reconfigurable six-legged robot.

The repository provides stability-aware leg scheduling, binary and experimental CPG motor mappings, configuration-driven simulation experiments, structured run metadata, analysis notebooks, and CoppeliaSim scenes.

> The current stability value is an **idealised geometric proxy**, not a validated physical tipping margin. Foot positions are inferred from mounting angles on a circle, the centre-of-mass projection is fixed at the origin, and commanded lift states are not confirmed by contact sensors. See [Methodology](docs/methodology.md) and the [Validation plan](docs/validation_plan.md).

## Repository layout

```text
StarfishBot/
├── src/starfishbot/             # Reusable algorithms and experiment runner
├── configs/                     # Versioned TOML experiment definitions
├── notebooks/                   # Exploratory and result-oriented analyses
├── scripts/                     # Source-checkout command-line entry points
├── tests/                       # Deterministic unit tests
├── docs/                        # Method, protocol, schema, and validation plan
├── data/                        # Data policy and small reviewed examples
├── results/                     # Local generated runs; ignored by Git
├── scenes/                      # CoppeliaSim scene files
├── asymmetry_leg_config/        # Backward-compatible historical entry points
├── pyproject.toml               # Environment, package, test, and lint settings
└── CITATION.cff                 # Software citation metadata
```

## Requirements

- Python 3.11 or newer
- [CoppeliaSim](https://www.coppeliarobotics.com/)
- CoppeliaSim ZeroMQ Remote API service

## Installation

```bash
git clone https://github.com/Leung555/StarfishBot.git
cd StarfishBot
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install -e ".[analysis,dev]"
```

For simulation without notebooks or development tools, install only the runtime package:

```bash
python3 -m pip install -e .
```

## Run an experiment

1. Start CoppeliaSim and make sure its ZeroMQ Remote API service is available.
2. Review or copy one of the versioned configurations in `configs/`.
3. Run from the repository root:

```bash
python scripts/run_experiment.py --config configs/random_leg_angles.toml
```

The angle-change protocol is available as:

```bash
python scripts/run_experiment.py --config configs/angle_change.toml
```

The package also installs a command:

```bash
starfishbot-run --config configs/random_leg_angles.toml
```

Scene and result paths are relative to the repository by default; no machine-specific absolute path is required.

## Reproducible outputs

Every run creates a new timestamped folder under `results/` containing:

```text
results/random-leg-angles-<UTC timestamp>/
├── config.toml
├── measurements.csv
├── metadata.json
└── simulation.log
```

Metadata records the random seed, Git commit and dirty state, Python and dependency versions, CoppeliaSim version when available, scene/configuration SHA-256 checksums, method assumptions, timestamps, and completion status.

Measurement headers contain units and record both:

- Native CoppeliaSim `alpha`, `beta`, and `gamma` angles
- Converted `yaw`, `pitch`, and `roll` values

CoppeliaSim documents that `sim.getObjectOrientation` returns alpha-beta-gamma Tait-Bryan angles. The runner uses the official `sim.alphaBetaGammaToYawPitchRoll` conversion instead of relabelling array elements. See the [CoppeliaSim orientation manual](https://manual.coppeliarobotics.com/en/positionOrientationTransformation.htm).

The complete schema is in the [data dictionary](docs/data_dictionary.md), and the run procedure is in the [experiment protocol](docs/experiment_protocol.md).

## Control modes

The default `binary` mode preserves the original fixed joint offset for a lifted leg. The optional `cpg` mode scales the lift offset with the first SO(2) oscillator output. The mode is explicit in each configuration and stored with every run.

Treat the two modes as different experimental conditions. The CPG mapping remains experimental and requires physical validation.

## Analysis

Install the `analysis` dependencies and start Jupyter:

```bash
jupyter notebook
```

The existing exploratory notebooks are under `notebooks/`. Reusable analysis should be moved into importable modules under `src/starfishbot/`; notebooks should focus on figures and interpretation. The notebooks currently target historical CSV schemas, so update their column mappings before using them with the new `measurements.csv` schema.

Full generated datasets are intentionally excluded from Git. Keep only small reviewed examples under `data/examples/`, and archive publication datasets in a DOI-backed repository such as Zenodo or OSF.

## Development and testing

```bash
ruff check .
ruff format --check .
pytest --cov=starfishbot
```

GitHub Actions runs linting and unit tests on Python 3.11 and 3.12. Simulator integration is intentionally separate because it requires a running CoppeliaSim instance and the project scene.

See [Contributing](CONTRIBUTING.md) before changing an experiment or dataset.

## Research documentation

- [Methodology and assumptions](docs/methodology.md)
- [Literature review and candidate contributions](docs/literature_review.md)
- [Experiment protocol](docs/experiment_protocol.md)
- [Measurement data dictionary](docs/data_dictionary.md)
- [Physical validation plan](docs/validation_plan.md)
- [Data policy](data/README.md)

## Citation

Citation metadata is provided in [CITATION.cff](CITATION.cff). Add the associated paper and archived dataset DOI when they are available.

## License

StarfishBot is licensed under the [GNU General Public License v3.0](LICENSE).
