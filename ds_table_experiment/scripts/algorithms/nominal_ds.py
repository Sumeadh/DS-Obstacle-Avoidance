import numpy as np
from math import hypot, isfinite
from collections.abc import Sequence


def compute_velocity(
    position: Sequence[float],
    goal: Sequence[float],
    gain: float,
    speed_limit: float,
) -> tuple[float, ...]:
    """Compute a goal-directed nominal velocity with a magnitude limit."""
    if len(position) == 0 or len(position) != len(goal):
        raise ValueError("Position and goal must have the same nonzero dimension.")
    if (
        not isfinite(gain)
        or gain <= 0.0
        or not isfinite(speed_limit)
        or speed_limit <= 0.0
        or not all(isfinite(value) for value in (*position, *goal))
    ):
        raise ValueError("Position, goal, gain, and speed limit must be finite and valid.")

    velocity = tuple(
        gain * (target - current)
        for current, target in zip(position, goal)
    )
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


def nominal_path(
    start: Sequence[float],
    goal: Sequence[float],
    gain: float,
    speed_limit: float,
    dt: float,
    tolerance: float,
    maximum_steps: int,
) -> list[tuple[float, ...]]:
    """Integrate the nominal DS until it reaches ``goal``."""
    position = tuple(float(value) for value in start)
    goal = tuple(float(value) for value in goal)
    if len(position) == 0 or len(position) != len(goal):
        raise ValueError("Start and goal must have the same nonzero dimension.")
    if dt <= 0.0 or tolerance <= 0.0 or maximum_steps <= 0:
        raise ValueError("Integration parameters must be positive.")

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


def modulate_velocity(
    position,
    nominal_velocity,
    center,
    semi_axes,
    orientation=None,
    safety_factor=None,
    reactivity=1.0,
    tail_effect=False,
):
    """
    Khansari-Zadeh & Billard (2012) single-ellipsoid modulation.

    Returns the modulated velocity and the intermediate matrices.
    """
    x = np.asarray(position, dtype=float)
    v = np.asarray(nominal_velocity, dtype=float)
    c = np.asarray(center, dtype=float)
    a = np.asarray(semi_axes, dtype=float)

    d = len(x)
    if d == 0 or any(z.shape != (d,) for z in (v, c, a)):
        raise ValueError("All vectors must have equal nonzero dimensions.")
    if not all(np.all(np.isfinite(z)) for z in (x, v, c, a)):
        raise ValueError("Inputs must be finite.")
    if np.any(a <= 0) or reactivity <= 0:
        raise ValueError("Invalid obstacle axes or reactivity.")

    R = np.eye(d) if orientation is None else np.asarray(orientation, dtype=float)
    eta = np.ones(d) if safety_factor is None else np.asarray(safety_factor, dtype=float)

    if R.shape != (d, d) or eta.shape != (d,):
        raise ValueError("Invalid orientation or safety-factor dimensions.")
    if not np.all(np.isfinite(R)) or not np.all(np.isfinite(eta)):
        raise ValueError("Orientation and safety factor must be finite.")
    if (
        not np.allclose(R.T @ R, np.eye(d), atol=1e-7)
        or np.linalg.det(R) < 0.0
    ):
        raise ValueError("Orientation must be a proper rotation matrix.")
    if np.any(eta < 1.0):
        raise ValueError("Safety factors must be >= 1.")

    relative_position = R.T @ (x - c)
    effective_axes = a * eta
    gamma = np.sum((relative_position / effective_axes) ** 2)

    if gamma <= 1e-12:
        raise ValueError("Normal is undefined at obstacle center.")

    gradient_local = 2.0 * relative_position / effective_axes**2
    gradient_global = R @ gradient_local
    normal = gradient_global / np.linalg.norm(gradient_global)

    _, _, Vh = np.linalg.svd(normal.reshape(1, -1))
    tangents = Vh[1:].T
    E = np.column_stack((normal, tangents))

    attenuation = 1.0 / abs(gamma) ** (1.0 / reactivity)
    lambda_normal = 1.0 - attenuation
    lambda_tangent = 1.0 + attenuation

    normal_velocity = normal @ v
    if not tail_effect and normal_velocity >= 0.0:
        lambda_normal = 1.0

    eigenvalues = np.full(d, lambda_tangent)
    eigenvalues[0] = lambda_normal
    D = np.diag(eigenvalues)
    M = E @ D @ E.T
    modulated_velocity = M @ v

    matrices = {
        "relative_position": relative_position,
        "Gamma": float(gamma),
        "gradient": gradient_global,
        "normal": normal,
        "E": E,
        "D": D,
        "M": M,
        "eigenvalues": eigenvalues,
        "normal_velocity": float(normal_velocity),
    }
    return modulated_velocity, matrices


def modulate_velocity_with_equilibrium(
    position,
    goal,
    gain,
    speed_limit,
    center,
    semi_axes,
    dt,
    orientation=None,
    safety_factor=None,
    reactivity=1.0,
    tail_effect=False,
    alpha=0.01,
    boundary_tolerance=1e-3,
    velocity_tolerance=1e-6,
    maximum_recovery_steps=100,
):
    """
    Apply Eq. (19), with Algorithm 1 recovery for boundary equilibria.

    The tangential perturbations are virtual numerical probes. They do not
    alter the measured robot position. The returned velocity is evaluated at
    the first probe satisfying either escape condition.
    """
    x = np.asarray(position, dtype=float)
    target = np.asarray(goal, dtype=float)

    if x.ndim != 1 or target.shape != x.shape or x.size == 0:
        raise ValueError("Position and goal must have equal nonzero dimensions.")
    if (
        not isfinite(dt) or dt <= 0.0
        or not isfinite(alpha) or alpha <= 0.0
        or not isfinite(boundary_tolerance) or boundary_tolerance <= 0.0
        or not isfinite(velocity_tolerance) or velocity_tolerance <= 0.0
        or maximum_recovery_steps <= 0
    ):
        raise ValueError("Invalid equilibrium-recovery parameters.")

    nominal = compute_velocity(x, target, gain, speed_limit)
    modulated, matrices = modulate_velocity(
        x, nominal, center, semi_axes,
        orientation=orientation,
        safety_factor=safety_factor,
        reactivity=reactivity,
        tail_effect=tail_effect,
    )

    gamma = matrices["Gamma"]
    speed = float(np.linalg.norm(modulated))

    # Algorithm 1 is only triggered at a boundary equilibrium.
    if abs(gamma - 1.0) > boundary_tolerance or speed > velocity_tolerance:
        return modulated, matrices, False

    normal = matrices["normal"]
    basis = matrices["E"]
    dimensions = x.size

    # Try the available tangent basis directions. Both signs are tested so
    # that the arbitrary sign convention of an SVD basis cannot block escape.
    for tangent_index in range(1, dimensions):
        tangent = basis[:, tangent_index]
        for direction_sign in (1.0, -1.0):
            direction = direction_sign * tangent
            probe_position = x.copy()

            for iteration in range(maximum_recovery_steps):
                probe_position = probe_position + alpha * direction * dt
                probe_nominal = compute_velocity(
                    probe_position, target, gain, speed_limit
                )
                probe_velocity, probe_matrices = modulate_velocity(
                    probe_position,
                    probe_nominal,
                    center,
                    semi_axes,
                    orientation=orientation,
                    safety_factor=safety_factor,
                    reactivity=reactivity,
                    tail_effect=tail_effect,
                )

                tangent_escape = float(direction @ probe_velocity) > 0.0
                normal_escape = float(normal @ probe_velocity) > 0.0

                if tangent_escape or normal_escape:
                    probe_matrices["equilibrium_recovery"] = {
                        "recovered": True,
                        "tangent_direction": direction.copy(),
                        "probe_steps": iteration + 1,
                        "probe_position": probe_position.copy(),
                        "tangent_escape": tangent_escape,
                        "normal_escape": normal_escape,
                    }
                    return probe_velocity, probe_matrices, True

    matrices["equilibrium_recovery"] = {
        "recovered": False,
        "probe_steps_per_direction": maximum_recovery_steps,
    }
    return modulated, matrices, False


def obstacle_ellipsoid(size, gazebo_center, safety_margin):
    """
    Single source of truth for the modulation ellipsoid.

    Returns ``(math_center, semi_axes)``. The ellipsoid is anchored at z=0
    (its mathematical centre), not at the Gazebo box centre; its z semi-axis
    reaches ``top_of_box + safety_margin``. Controller and visualizer must
    both call this so the plotted boundary is the boundary actually used.
    """
    size = np.asarray(size, dtype=float)
    center = np.asarray(gazebo_center, dtype=float)
    top_of_box = center[2] + size[2] / 2.0
    semi_axes = np.array((
        np.sqrt(3.0) * size[0] / 2.0 + safety_margin,
        np.sqrt(3.0) * size[1] / 2.0 + safety_margin,
        top_of_box + safety_margin,
    ))
    math_center = np.array((center[0], center[1], 0.0))
    return math_center, semi_axes
