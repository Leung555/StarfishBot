# StarfishBot

A small Python controller for the bundled six-legged robot in CoppeliaSim.
It samples leg mounting angles, selects which leg to lift using a geometric
support-margin estimate, and saves measurements to CSV.

## 1. Install Python dependencies

Use **Python 3.10 or newer**. Check with `python3 --version`.
Open a terminal in the `StarfishBot` folder, then run these commands one at a time
(Linux / macOS):

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

That is the complete Python installation. The two direct dependencies are
`coppeliasim-zmqremoteapi-client` (connects to the simulator) and `shapely`
(calculates support geometry). Pip installs their dependencies automatically.

The commands below use `.venv/bin/python` explicitly so they always use the
correct environment. You do not need to activate it or install the project as a
package. If using an editor, select `.venv/bin/python` as its Python interpreter.

On **Windows**, create the environment with `py -3 -m venv .venv`, then replace
`.venv/bin/python` in every command with `.venv\Scripts\python.exe`.

## 2. Check the setup

```bash
.venv/bin/python run.py --check
```

Expected output includes:

```text
Configuration OK: smoke
Expected measurement rows: 600
```

This reads the settings, checks that the scene file exists, and checks angle
sampling. It does not connect to CoppeliaSim or create results.

## 3. Run the robot

Install the separate [CoppeliaSim desktop application](https://www.coppeliarobotics.com/)
if you do not already have it. Pip installs only the Python connection library.

1. Open CoppeliaSim, save any open scene, and leave the simulation stopped.
2. Ensure its ZeroMQ Remote API service is running on the default local port (23000).
3. Run:

   ```bash
   .venv/bin/python run.py
   ```

The program loads `scenes/Asymmetry_Leg_gait.ttt`, runs the configured experiment
(currently 2 samples of 300 steps), requests the
default mounting angles, and stops the simulation. It prints the result folder,
for example `results/smoke-.../`. Open `measurements.csv` there to see the results.
`metadata.json` should report `"status": "completed"` and a `total_steps` value
equal to `samples × steps_per_sample` (currently 600).
The folder also contains `config.json` and `simulation.log`.
Progress is printed every 200 completed simulation steps across the whole run
and saved to `simulation.log`, alongside sample summaries and the final total.

## Change the experiment

Edit **`config.json`**. For a longer run, change `samples` to `100` and
`steps_per_sample` to `300` (30,000 measurement rows). Keep `seed` fixed to repeat
the same sampled angles. Run `--check` after editing.

The file contains only common settings; robot, gait, motor, scene, and output
settings have defaults in [starfishbot/config.py](starfishbot/config.py).
Advanced overrides use the corresponding section names, such as
`"motor": {"mode": "cpg"}`. The default motor mode is `binary`.
To use a separate JSON file:

```bash
.venv/bin/python run.py --config my_experiment.json
```

Relative scene and output paths start at the project folder. Samples in one run
share physical simulator state; each sample resets the gait planner and CPG.
Mounting angles in the CSV are targets. The support margin uses idealised contacts
and a fixed centre of mass; physical stability still needs validation.

## Tests (optional)

```bash
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pytest
```

Tests exercise geometry, gait, settings, and successful/failed runs with a fake
simulator. They do not require CoppeliaSim. CI also runs lint and formatting checks.

## Common problems

| Problem | What to do |
| --- | --- |
| `No module named ...` | Use `.venv/bin/python`, then repeat the requirements installation. |
| Old instructions mention `tomllib` or TOML | Use this version's `run.py` and `config.json`; TOML is no longer used. |
| `venv` cannot be created | Check Python is 3.10+ and has its venv component installed. On Ubuntu/Debian this may require `sudo apt install python3-venv`. |
| Run waits while connecting | Confirm CoppeliaSim and its ZeroMQ service are running. Interrupt with Ctrl+C if needed; the client currently has no project-level connection timeout. |
| Object lookup fails | Use the bundled scene; custom scenes must match the paths in `starfishbot/config.py`. |
| JSON settings error | Use double quotes, no comments, and no trailing commas. |

## How the controller works

1. Sample sorted mounting angles using the configured random seed.
2. Treat each leg as a contact on a circle of radius `leg_radius` around `(0, 0)`.
3. Test the support margin after excluding each eligible leg and choose the
   candidate with the largest margin. Each leg is selected once per gait cycle.
4. Send joint targets, advance the simulator one step, then record orientation
   and the support estimate.

The support margin is the signed distance from `(0, 0)` to the support hull:
positive inside a polygon, negative outside, and zero on its boundary. A line or
point has no support area and gives the negative distance to that geometry;
an empty hull gives negative infinity. Values scale with `leg_radius` and are
not normalised by it. They represent metres only if the configured radius does.

A landing leg can receive a stance target while its timer still excludes it
from support. The CSV therefore records motor commands and scheduled exclusions
separately. These are not measured foot contacts. The default `binary` motor mode
uses fixed lift offsets; the optional `cpg` mode scales them using an oscillator.

The estimate assumes idealised contacts and a fixed centre of mass. It does not
measure actual contact, slip, compliance, or dynamic tipping. Validate those
against simulator sensors and physical outcomes before making stability claims.
There is no settling period between samples; samples share physical simulator
state. A fixed seed repeats sampled angles, not necessarily physical outcomes.

## Read the results

Each run creates a new timestamped folder under `results/`. Preserve its four
files together. `metadata.json` records status, complete settings (including
defaults), seed, Git state, dependency/simulator versions, and file checksums.
A run is usable only after checking its status and measurements; a folder alone
does not establish success.

`measurements.csv` uses schema 2. Leg indices start at zero; `i` below ranges
from `0` to `num_legs - 1`.

| Column(s) | Meaning |
| --- | --- |
| `sample_index`, `step_index`, `global_step` | Zero-based sample, step within sample, and step across the run |
| `sim_time_s` | Simulator time in seconds, after the measured step |
| `angle_i_deg` | Mounting target for leg `i`, in degrees |
| `proxy_support_margin_radius` | Signed support estimate for the scheduled excluded-leg set |
| `commanded_lifted_legs` | Legs receiving a lift target; semicolon-separated indices, empty for none |
| `scheduled_lifted_legs` | Legs excluded from support, including landing transitions |
| `min_sample_proxy_margin_radius`, `mean_sample_proxy_margin_radius` | Minimum and mean over the whole sample, repeated on each row |
| `leg_i_test_margin_radius` | Estimate with only leg `i` excluded, independent of the current gait |
| `alpha_deg`, `beta_deg`, `gamma_deg` | Native CoppeliaSim orientation relative to the world, in degrees |
| `yaw_deg`, `pitch_deg`, `roll_deg` | Orientation converted by CoppeliaSim's `alphaBetaGammaToYawPitchRoll`, in degrees |

Check for exactly `samples × steps_per_sample` rows, continuous `global_step`,
increasing simulation time, and missing or infinite values. Configuration joint
biases and lift offsets use radians; logged orientation and mounting angles use
degrees. In older schema 1 output, `commanded_lifted_legs` contained scheduled
exclusions; do not mix schemas without accounting for that difference.

## Minimal project layout

```text
StarfishBot/
├── README.md              # This is the only project manual
├── LICENSE                # GPL v3 license
├── run.py                 # Entry point
├── config.json            # Experiment settings
├── requirements.txt       # Run dependencies
├── requirements-dev.txt   # Optional test/lint dependencies
├── starfishbot/            # Controller and experiment code
├── scenes/
│   └── Asymmetry_Leg_gait.ttt
├── tests/                 # Automated regression tests
├── pyproject.toml         # Test/lint settings
└── .github/workflows/ci.yml # Automated checks on GitHub
```

`results/` is created when you run an experiment and is ignored by Git. `.venv/`
holds your installed dependencies. `.gitignore` excludes generated files;
`.gitattributes` tells Git to treat the scene as binary. These are supporting
files, not additional manuals. Old wrappers, notebooks, archives, unused scenes,
and separate documentation have been removed from the active project.

Licensed under [GPL v3](LICENSE). For research use, cite StarfishBot contributors,
[StarfishBot](https://github.com/Leung555/StarfishBot), the code commit used, and
the archived dataset or associated paper where available.
