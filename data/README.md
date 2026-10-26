# Data policy

This directory is reserved for documented research data shared with the repository.

- `examples/` may contain a small, versioned dataset sufficient to exercise analysis code.
- `raw/` is for immutable local input data and is ignored by Git.
- `processed/` is for reproducible derived data and is ignored by Git.
- Full publication datasets should be archived outside Git with a DOI and checksum.

Do not edit raw data manually. Record the script, configuration, source commit, and checksum used to produce every processed dataset.
