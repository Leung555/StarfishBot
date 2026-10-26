import math

import pytest

from starfishbot.experiment import _orientation_degrees


class FakeSim:
    handle_world = -1

    def getObjectOrientation(self, robot_handle: int, relative_to: int) -> list[float]:
        assert robot_handle == 10
        assert relative_to == self.handle_world
        return [0.1, 0.2, 0.3]

    def alphaBetaGammaToYawPitchRoll(self, alpha: float, beta: float, gamma: float) -> list[float]:
        assert (alpha, beta, gamma) == (0.1, 0.2, 0.3)
        return [0.4, 0.5, 0.6]


def test_orientation_uses_official_conversion_and_return_order() -> None:
    orientation = _orientation_degrees(FakeSim(), 10)
    assert orientation == pytest.approx(
        {
            "alpha_deg": math.degrees(0.1),
            "beta_deg": math.degrees(0.2),
            "gamma_deg": math.degrees(0.3),
            "yaw_deg": math.degrees(0.4),
            "pitch_deg": math.degrees(0.5),
            "roll_deg": math.degrees(0.6),
        }
    )


class RunSim(FakeSim):
    def __init__(self, *, fail=False):
        self.steps = 0
        self.stopped = False
        self.targets = []
        self.fail = fail

    def loadScene(self, path):
        self.scene = path

    def getObject(self, path):
        return 10

    def getInt32Param(self, parameter):
        return 0

    def setStepping(self, enabled):
        assert enabled

    def startSimulation(self):
        self.started = True

    def stopSimulation(self):
        self.stopped = True

    def setJointTargetPosition(self, handle, value):
        self.targets.append(value)

    def step(self):
        if self.fail:
            raise RuntimeError("simulated step failure")
        self.steps += 1

    def getSimulationTime(self):
        return self.steps * 0.05


@pytest.mark.parametrize("fail", [False, True])
def test_run_artifacts_and_cleanup(tmp_path, monkeypatch, fail):
    import csv
    import json
    import sys
    from dataclasses import replace
    from pathlib import Path
    from types import SimpleNamespace

    from starfishbot.config import load_config
    from starfishbot.experiment import run_experiment

    root = Path(__file__).resolve().parents[1]
    config = load_config(root / "config.json")
    config = replace(
        config,
        experiment=replace(config.experiment, samples=2, steps_per_sample=6),
        gait=replace(
            config.gait,
            lifting_period_steps=4,
            fast_lifting_period_steps=4,
            stance_transition_steps=3,
        ),
        output=replace(config.output, directory=str(tmp_path)),
    )
    sim = RunSim(fail=fail)
    monkeypatch.setitem(
        sys.modules,
        "coppeliasim_zmqremoteapi_client",
        SimpleNamespace(RemoteAPIClient=lambda: SimpleNamespace(require=lambda name: sim)),
    )
    if fail:
        with pytest.raises(RuntimeError, match="simulated step failure"):
            run_experiment(config)
    else:
        run_experiment(config)
    run = next(tmp_path.iterdir())
    metadata = json.loads((run / "metadata.json").read_text())
    assert metadata["status"] == ("failed" if fail else "completed")
    assert metadata["schema_version"] == 2
    assert sim.stopped
    assert sim.targets[-6:] == pytest.approx(
        [0, math.pi / 3, 2 * math.pi / 3, math.pi, -2 * math.pi / 3, -math.pi / 3]
    )
    assert (run / "config.json").read_bytes() == config.path.read_bytes()
    assert (run / "simulation.log").is_file()
    if fail:
        assert "simulated step failure" in metadata["error"]
    else:
        with (run / "measurements.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        assert metadata["total_steps"] == len(rows) == 12
        assert [int(row["global_step"]) for row in rows] == list(range(12))
        assert [float(row["sim_time_s"]) for row in rows] == pytest.approx(
            [0.05 * step for step in range(1, 13)]
        )
        assert any(row["commanded_lifted_legs"] != row["scheduled_lifted_legs"] for row in rows)
        for sample in range(2):
            group = [row for row in rows if int(row["sample_index"]) == sample]
            margins = [float(row["proxy_support_margin_radius"]) for row in group]
            assert all(
                float(row["min_sample_proxy_margin_radius"]) == min(margins) for row in group
            )
            assert all(
                float(row["mean_sample_proxy_margin_radius"])
                == pytest.approx(sum(margins) / len(margins))
                for row in group
            )
