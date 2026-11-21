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
    Khansari-Zadeh & Billard (2012)
    Single static ellipsoid obstacle modulation.

    Returns:
        modulated_velocity
        matrices: Gamma, normal, E, D, M and eigenvalues
    """

    # --------------------------------------------------
    # 1. Input preparation
    # --------------------------------------------------

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

    if (not np.allclose(R.T @ R, np.eye(d), atol=1e-7)
            or np.linalg.det(R) < 0.0):
        raise ValueError("Orientation must be a proper rotation matrix.")

    if np.any(eta < 1.0):
        raise ValueError("Safety factors must be >= 1.")

    # --------------------------------------------------
    # 2. Relative position in obstacle frame
    # --------------------------------------------------

    relative_position = R.T @ (x - c)

    # --------------------------------------------------
    # 3. Obstacle function Gamma
    #    Safety margin is included here.
    # --------------------------------------------------

    effective_axes = a * eta

    gamma = np.sum((relative_position / effective_axes) ** 2)

    if gamma <= 1e-12:
        raise ValueError("Normal is undefined at obstacle center.")

    # --------------------------------------------------
    # 4. Surface normal
    #    Gradient in obstacle frame
    # --------------------------------------------------

    gradient_local = (
        2.0 * relative_position / effective_axes**2
    )

    # Transform gradient into global frame
    gradient_global = R @ gradient_local

    normal = gradient_global / np.linalg.norm(gradient_global)

    # --------------------------------------------------
    # 5. Construct orthonormal basis E
    #
    #    E = [normal, tangent_1, ..., tangent_(d-1)]
    # --------------------------------------------------

    _, _, Vh = np.linalg.svd(normal.reshape(1, -1))

    tangents = Vh[1:].T

    E = np.column_stack((normal, tangents))

    # --------------------------------------------------
    # 6. Eigenvalue calculation
    # --------------------------------------------------

    attenuation = 1.0 / abs(gamma) ** (1.0 / reactivity)

    lambda_normal = 1.0 - attenuation
    lambda_tangent = 1.0 + attenuation

    # --------------------------------------------------
    # 7. Tail-effect handling
    # --------------------------------------------------

    normal_velocity = normal @ v

    if not tail_effect and normal_velocity >= 0.0:
        lambda_normal = 1.0

    # --------------------------------------------------
    # 8. Construct eigenvalue matrix D
    # --------------------------------------------------

    eigenvalues = np.full(d, lambda_tangent)
    eigenvalues[0] = lambda_normal

    D = np.diag(eigenvalues)

    # --------------------------------------------------
    # 9. Modulation matrix M = E D E^-1
    # --------------------------------------------------

    M = E @ D @ E.T

    # --------------------------------------------------
    # 10. Modulate nominal velocity
    # --------------------------------------------------

    modulated_velocity = M @ v

    matrices = {
        "relative_position": relative_position,
        "Gamma": gamma,
        "gradient": gradient_global,
        "normal": normal,
        "E": E,
        "D": D,
        "M": M,
        "eigenvalues": eigenvalues,
        "normal_velocity": normal_velocity,
    }

    return modulated_velocity, matrices
