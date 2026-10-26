# Measurement data dictionary

Each run writes `measurements.csv`. The schema version is stored in `metadata.json`.

| Column | Type | Unit | Meaning |
| --- | --- | --- | --- |
| `sample_index` | integer | — | Zero-based configuration or phase index |
| `step_index` | integer | step | Zero-based step within the sample |
| `global_step` | integer | step | Zero-based step across the complete run |
| `sim_time_s` | float | second | CoppeliaSim time after advancing the measured step |
| `angle_i_deg` | float | degree | Configured mounting angle for leg `i` |
| `proxy_support_margin_radius` | float | configured radius unit | Signed idealised support margin for the commanded lifted set |
| `commanded_lifted_legs` | string | — | Semicolon-separated zero-based leg indices; empty means none |
| `min_sample_proxy_margin_radius` | float | configured radius unit | Minimum proxy margin in the sample |
| `mean_sample_proxy_margin_radius` | float | configured radius unit | Arithmetic mean proxy margin in the sample |
| `leg_i_test_margin_radius` | float | configured radius unit | Proxy margin if leg `i` alone were excluded |
| `alpha_deg` | float | degree | Native CoppeliaSim alpha angle relative to the world frame |
| `beta_deg` | float | degree | Native CoppeliaSim beta angle relative to the world frame |
| `gamma_deg` | float | degree | Native CoppeliaSim gamma angle relative to the world frame |
| `yaw_deg` | float | degree | Yaw returned by CoppeliaSim’s ABG-to-YPR conversion |
| `pitch_deg` | float | degree | Pitch returned by CoppeliaSim’s ABG-to-YPR conversion |
| `roll_deg` | float | degree | Roll returned by CoppeliaSim’s ABG-to-YPR conversion |

`i` ranges from `0` to `num_legs - 1`. Angles are recorded in degrees; joint targets in configuration files are radians.

## Run metadata

`metadata.json` records:

- Run status and UTC timestamps
- Experiment name, mode, and random seed
- Git commit, branch, remote, and dirty state
- Python, operating-system, and dependency versions
- Configuration and scene paths plus SHA-256 checksums
- CoppeliaSim version when available
- Orientation convention
- Method assumptions and any failure message
