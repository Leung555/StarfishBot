import pytest

from starfishbot.control import CPG, MotorMapper


def test_cpg_changes_state() -> None:
    cpg = CPG()
    initial = cpg.output()
    cpg.step()
    assert cpg.output() != initial


def test_binary_mapper_preserves_fixed_lift_behavior() -> None:
    mapper = MotorMapper(2, 3, mode="binary")
    mapper.set_lift_offsets([0.0, -0.8, 0.0])
    commands = mapper.map_to_motors([True, False], [0.0, -0.52, 2.09])
    assert commands == pytest.approx([0.0, -0.52, 2.09, 0.0, -1.32, 2.09])


def test_cpg_mapper_uses_oscillator_output() -> None:
    mapper = MotorMapper(1, 1, mode="cpg")
    mapper.set_lift_offsets([2.0])
    low = mapper.map_to_motors([False], [1.0], cpg_outputs=[-1.0, 0.0])
    high = mapper.map_to_motors([False], [1.0], cpg_outputs=[1.0, 0.0])
    assert low == [1.0]
    assert high == [3.0]


def test_cpg_mapper_rejects_missing_output() -> None:
    mapper = MotorMapper(1, 1, mode="cpg")
    with pytest.raises(ValueError):
        mapper.map_to_motors([False], [0.0])
