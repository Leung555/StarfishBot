from starfishbot.gait_planner import GaitPlanner

ANGLES = [0, 60, 120, 180, 240, 300]


def test_returned_lifted_state_includes_new_command_immediately() -> None:
    planner = GaitPlanner(
        ANGLES,
        lifting_period=2,
        fast_lifting_period=2,
        stance_transition_time=0,
    )
    commands, lifted = planner.step(ANGLES)
    assert lifted == [0]
    assert commands[0] is False


def test_returned_lifted_state_is_recomputed_after_timer_transition() -> None:
    planner = GaitPlanner(
        ANGLES,
        lifting_period=2,
        fast_lifting_period=2,
        stance_transition_time=0,
    )
    planner.step(ANGLES)
    planner.step(ANGLES)
    _, lifted = planner.step(ANGLES)
    assert 0 not in lifted
    assert len(lifted) == 1


def test_lift_limit_is_respected() -> None:
    planner = GaitPlanner(ANGLES, max_legs_lifted=1, lifting_period=10)
    for _ in range(5):
        _, lifted = planner.step(ANGLES)
        assert len(lifted) <= 1


def test_landing_command_precedes_scheduled_support_restoration():
    planner = GaitPlanner(ANGLES, lifting_period=4, fast_lifting_period=4, stance_transition_time=3)
    planner.step(ANGLES)
    planner.step(ANGLES)
    commands, scheduled = planner.step(ANGLES)
    assert commands[0] is True
    assert 0 in scheduled
