# Results

The experiment runner creates one immutable, timestamped directory per run here. Each directory contains:

- `config.toml`: exact configuration used
- `metadata.json`: seed, Git state, dependency versions, checksums, assumptions, and run status
- `measurements.csv`: step-level measurements with units in the headers
- `simulation.log`: progress and error log

Generated run directories are ignored by Git. Archive datasets supporting a publication in a DOI-backed service such as Zenodo or OSF, and include the archive DOI in the README and citation metadata.
