"""Robot-independent nominal dynamical system."""

from collections.abc import Sequence
from math import hypot, isfinite


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
    return cap_velocity(velocity, speed_limit)


def cap_velocity(
    velocity: Sequence[float],
    speed_limit: float,
) -> tuple[float, ...]:
    """Limit a velocity vector's magnitude to ``speed_limit``."""
    if (
        len(velocity) == 0
        or not all(isfinite(value) for value in velocity)
        or not isfinite(speed_limit)
        or speed_limit <= 0.0
    ):
        raise ValueError("Velocity and speed limit must be finite and valid.")
    speed = hypot(*velocity)
    scale = min(1.0, speed_limit / speed) if speed > 0.0 else 0.0
    return tuple(component * scale for component in velocity)


def modulate_velocity_ellipsoid(
    position: Sequence[float],
    velocity: Sequence[float],
    center: Sequence[float],
    semi_axes: Sequence[float],
    reactivity: float = 1.0,
    tail_effect: bool = False,
) -> tuple[float, ...]:
    """Modulate a nominal velocity to avoid one axis-aligned ellipsoid.

    The modulation follows the normal/tangent eigenvalue construction in
    ``obs_modulation_ellipsoid.m`` for a single ellipsoid with unit power terms.
    """
    dimensions = len(position)
    if (
        dimensions == 0
        or len(velocity) != dimensions
        or len(center) != dimensions
        or len(semi_axes) != dimensions
    ):
        raise ValueError(
            "Position, velocity, center, and ellipsoid axes must have "
            "equal nonzero dimensions."
        )
    if (
        not all(isfinite(axis) and axis > 0.0 for axis in semi_axes)
        or not isfinite(reactivity)
        or reactivity <= 0.0
        or not all(isfinite(value) for value in (*position, *velocity, *center))
    ):
        raise ValueError("Sphere parameters and vectors must be finite and valid.")

    relative_position = tuple(
        coordinate - obstacle_coordinate
        for coordinate, obstacle_coordinate in zip(position, center)
    )
    gamma = sum(
        (component / axis) ** 2
        for component, axis in zip(relative_position, semi_axes)
    )
    normal_gradient = tuple(
        component / (axis * axis)
        for component, axis in zip(relative_position, semi_axes)
    )
    gradient_norm = hypot(*normal_gradient)
    if gradient_norm <= 1e-12:
        raise ValueError("The obstacle normal is undefined at its center.")
    normal = tuple(component / gradient_norm for component in normal_gradient)
    normal_velocity = sum(
        component * normal_component
        for component, normal_component in zip(velocity, normal)
    )
    attenuation = min(1.0, gamma ** (-1.0 / reactivity))
    normal_scale = 1.0 - attenuation
    tangent_scale = 1.0 + attenuation

    if not tail_effect and normal_velocity >= 0.0:
        normal_scale = 1.0
    elif gamma < 1.0:
        tangent_scale = 2.0
        if normal_velocity < 0.0:
            normal_scale = 0.0

    return tuple(
        tangent_scale * component
        + (normal_scale - tangent_scale) * normal_velocity * normal_component
        for component, normal_component in zip(velocity, normal)
    )


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
