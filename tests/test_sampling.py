import random

from starfishbot.sampling import circular_angle_separations, generate_random_angles


def test_random_angles_are_reproducible() -> None:
    first = generate_random_angles(random.Random(42), 6, minimum_separation_deg=25)
    second = generate_random_angles(random.Random(42), 6, minimum_separation_deg=25)
    assert first == second


def test_random_angles_respect_circular_separation() -> None:
    angles = generate_random_angles(random.Random(7), 6, minimum_separation_deg=25)
    assert min(circular_angle_separations(angles)) >= 25
