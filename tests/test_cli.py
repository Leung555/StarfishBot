from pathlib import Path

import pytest

from run import main

ROOT = Path(__file__).resolve().parents[1]


def test_check_does_not_run_simulator(monkeypatch, capsys, tmp_path):
    def unexpected_run(*args, **kwargs):
        pytest.fail("--check must not run a simulation")

    monkeypatch.setattr("starfishbot.experiment.run_experiment", unexpected_run)
    settings = tmp_path / "config.json"
    settings.write_text("{}")
    assert main(["--config", str(settings), "--check"]) == 0
    assert "Expected measurement rows: 10" in capsys.readouterr().out


def test_missing_config_has_readable_error(capsys):
    with pytest.raises(SystemExit) as error:
        main(["--config", "missing-config.json", "--check"])
    assert error.value.code == 2
    assert "missing-config.json" in capsys.readouterr().err


def test_help_and_missing_dependency_message_without_site_packages():
    import os
    import subprocess
    import sys

    environment = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    help_result = subprocess.run(
        [sys.executable, "-S", str(ROOT / "run.py"), "--help"],
        capture_output=True,
        text=True,
        env=environment,
    )
    assert help_result.returncode == 0
    check_result = subprocess.run(
        [sys.executable, "-S", str(ROOT / "run.py"), "--check"],
        capture_output=True,
        text=True,
        env=environment,
    )
    assert check_result.returncode == 2
    assert "Missing dependency: shapely" in check_result.stderr
    assert "pip install -r" in check_result.stderr
    assert "Traceback" not in check_result.stderr


def test_run_from_downloaded_folder_without_git_or_package_install(tmp_path):
    import os
    import shutil
    import subprocess
    import sys

    checkout = tmp_path / "download"
    checkout.mkdir()
    shutil.copy2(ROOT / "run.py", checkout)
    (checkout / "config.json").write_text("{}")
    shutil.copytree(
        ROOT / "starfishbot", checkout / "starfishbot", ignore=shutil.ignore_patterns("__pycache__")
    )
    shutil.copytree(ROOT / "scenes", checkout / "scenes")
    result = subprocess.run(
        [sys.executable, str(checkout / "run.py"), "--check"],
        cwd=tmp_path,
        env={key: value for key, value in os.environ.items() if key != "PYTHONPATH"},
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
    assert "Configuration OK: smoke" in result.stdout
    assert str(checkout / "scenes") in result.stdout
    assert not (checkout / "results").exists()
