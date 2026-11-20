"""Robot-independent nominal dynamical system."""

from collections.abc import Sequence
from math import hypot


def compute_velocity(
    position: Sequence[float],
    goal: Sequence[float],
    gain: float,
    speed_limit: float,
) -> tuple[float, ...]:
    """Compute a goal-directed velocity, capped at ``speed_limit``."""
    if len(position) == 0 or len(position) != len(goal):
        raise ValueError("Position and goal must have the same nonzero dimension.")

    error = tuple(target - current for current, target in zip(position, goal))
    velocity = tuple(gain * component for component in error)
    speed = hypot(*velocity)
    scale = min(1.0, speed_limit / speed) if speed > 0.0 else 0.0
    return tuple(component * scale for component in velocity)


def nominal_path(
    start: Sequence[float],
    goal: Sequence[float],
    gain: float,
    speed_limit: float,
    dt: float,
    tolerance: float,
    maximum_steps: int,
) -> list[tuple[float, ...]]:
    """Integrate the nominal DS from ``start`` until it reaches ``goal``."""
    position = tuple(float(value) for value in start)
    goal = tuple(float(value) for value in goal)
    if len(position) == 0 or len(position) != len(goal):
        raise ValueError("Start and goal must have the same nonzero dimension.")
    path = []

    for _ in range(maximum_steps):
        if hypot(*(target - current for current, target in zip(position, goal))) <= tolerance:
            path.append(goal)
            return path
        velocity = compute_velocity(position, goal, gain, speed_limit)
        position = tuple(
            current + dt * speed
            for current, speed in zip(position, velocity)
        )
        path.append(position)

    raise RuntimeError("Nominal system failed to reach B within maximum_steps.")
