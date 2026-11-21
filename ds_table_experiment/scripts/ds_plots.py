"""Pure-matplotlib plotting helpers for the DS obstacle-avoidance experiment.

No ROS imports here, so the same code can be used live (visualize_results.py)
or offline on a recorded trajectory.

Conventions
-----------
Gamma(x) = sum(((x - c) / (a * eta))**2)   (Khansari-Zadeh & Billard)
    Gamma < 1  : inside the modulation ellipsoid
    Gamma = 1  : on its boundary
    Gamma > 1  : outside
All inside/outside decisions are made with the full 3D Gamma, never from a
2D projection.
"""

import numpy as np
from algorithms.nominal_ds import (
    cap_velocity,
    compute_velocity,
    modulate_velocity_with_equilibrium,
    obstacle_ellipsoid,
)

BOUNDARY_TOL = 0.02  # |Gamma-1| <= tol counts as "on the boundary"


# ----------------------------------------------------------------- geometry
def gamma_of(points, center, effective_axes):
    points = np.atleast_2d(points)
    return np.sum(((points - center) / effective_axes) ** 2, axis=1)


def classify(gamma, tol=BOUNDARY_TOL):
    """0 = outside, 1 = on boundary (sliding), 2 = inside."""
    out = np.zeros(len(gamma), dtype=int)
    out[np.abs(gamma - 1.0) <= tol] = 1
    out[gamma < 1.0 - tol] = 2
    return out


def box_clearance(points, box_center, box_size):
    """Signed distance from points to the *physical* box (negative = inside)."""
    half = np.asarray(box_size) / 2.0
    off = np.abs(np.atleast_2d(points) - box_center) - half
    outside = np.linalg.norm(np.maximum(off, 0.0), axis=1)
    return outside + np.minimum(off.max(axis=1), 0.0)


class ControllerModel:
    """Everything needed to evaluate exactly what Controller.servo_step does."""

    def __init__(self, box_size, box_center, margin, safety_factor,
                 reactivity, tail_effect, gain, speed_limit, dt):
        self.box_size = np.asarray(box_size, float)
        self.box_center = np.asarray(box_center, float)
        self.center, self.semi_axes = obstacle_ellipsoid(
            self.box_size, self.box_center, margin)
        self.eta = np.asarray(safety_factor, float)
        self.axes = self.semi_axes * self.eta          # effective axes
        self.reactivity = float(reactivity)
        self.tail_effect = bool(tail_effect)
        self.gain, self.speed_limit, self.dt = gain, speed_limit, dt

    def gamma(self, points):
        return gamma_of(points, self.center, self.axes)

    def velocities(self, point, goal):
        """(nominal, modulated) using the controller's own function."""
        nominal = np.asarray(
            compute_velocity(point, goal, self.gain, self.speed_limit))
        mod, _, _ = modulate_velocity_with_equilibrium(
            point, goal, self.gain, self.speed_limit, self.center,
            self.semi_axes, self.dt, safety_factor=self.eta,
            reactivity=self.reactivity, tail_effect=self.tail_effect)
        return nominal, np.asarray(cap_velocity(mod, self.speed_limit))


# ------------------------------------------------------------------ 3D plot
def draw_3d(ax, model, positions, goal, gamma, z_floor=0.25):
    ax.clear()
    cls = classify(gamma)
    ax.plot(*positions.T, color="tab:blue", lw=1.8, label="Robot trajectory")
    for k, col, lab in ((1, "orange", "On boundary (Γ≈1)"),
                        (2, "red", "INSIDE ellipsoid (Γ<1)")):
        m = cls == k
        if m.any():
            ax.scatter(*positions[m].T, color=col, s=8, label=lab)
    ax.scatter(*positions[0], marker="o", s=35, color="tab:blue", label="Start")
    ax.scatter(*positions[-1], marker="^", s=40, color="tab:green", label="Current")
    ax.scatter(*goal, marker="*", s=100, color="gold", label="Goal")

    # physical box (what Gazebo shows)
    lo = model.box_center - model.box_size / 2
    hi = model.box_center + model.box_size / 2
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                        for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    for i in range(8):
        for j in range(i + 1, 8):
            if np.sum(corners[i] != corners[j]) == 1:
                ax.plot(*corners[[i, j]].T, color="k", lw=1.2)
    ax.plot([], [], color="k", lw=1.2, label="Physical obstacle (box)")

    # modulation ellipsoid wireframe (only the part above z_floor)
    c, a = model.center, model.axes
    t = np.linspace(0, 2 * np.pi, 120)
    lines = []
    for v in np.linspace(0.15, np.pi - 0.15, 14):          # parallels
        lines.append(np.c_[c[0] + a[0] * np.sin(v) * np.cos(t),
                           c[1] + a[1] * np.sin(v) * np.sin(t),
                           c[2] + a[2] * np.cos(v) * np.ones_like(t)])
    for u in np.linspace(0, np.pi, 8, endpoint=False):     # meridians
        s = np.linspace(0, 2 * np.pi, 120)
        lines.append(np.c_[c[0] + a[0] * np.sin(s) * np.cos(u),
                           c[1] + a[1] * np.sin(s) * np.sin(u),
                           c[2] + a[2] * np.cos(s)])
    for ln in lines:
        ln = ln.copy()
        ln[ln[:, 2] < z_floor] = np.nan
        ax.plot(*ln.T, color="tab:purple", lw=0.5, alpha=0.5)
    ax.plot([], [], color="tab:purple", lw=1, label="Modulation ellipsoid")

    pts = np.vstack([positions, goal[None], lo[None], hi[None]])
    xl = (min(pts[:, 0].min(), c[0] - a[0]) - 0.05, c[0] + a[0] + 0.05)
    yl = (min(pts[:, 1].min(), c[1] - a[1]) - 0.1, max(pts[:, 1].max(), c[1] + a[1]) + 0.1)
    zl = (z_floor, c[2] + a[2] + 0.05)
    ax.set(xlim=xl, ylim=yl, zlim=zl, xlabel="X (m)", ylabel="Y (m)",
           zlabel="Z (m)", title="3D End-Effector Trajectory")
    ax.set_box_aspect((xl[1] - xl[0], yl[1] - yl[0], zl[1] - zl[0]))
    ax.legend(fontsize=6, loc="upper left")


# ----------------------------------------------------------- distance plots
def draw_distance(ax_dist, ax_gamma, model, times, positions):
    gamma = model.gamma(positions)
    cls = classify(gamma)
    rho = np.linalg.norm(positions - model.center, axis=1)
    rho_b = rho / np.sqrt(gamma)          # centre->boundary distance along the ray

    ax_dist.clear()
    ax_dist.plot(times, rho, "b-", lw=1.6, label=r"$\|x-\epsilon^o\|$")
    ax_dist.plot(times, rho_b, "r--", lw=1.2,
                 label="boundary distance along current ray")
    ax_dist.set(ylabel="Distance to ellipsoid centre (m)",
                title="Distance from Obstacle vs Time")
    ax_dist.grid(True, ls=":", alpha=.7)
    ax_dist.legend(fontsize=7, loc="best")

    ax_gamma.clear()
    ax_gamma.plot(times, np.sqrt(gamma), "b-", lw=1.6)
    ax_gamma.axhline(1.0, color="r", ls="--", lw=1.2, label="modulation boundary (Γ=1)")
    ax_gamma.axhspan(0, 1.0, color="red", alpha=0.08)
    ax_gamma.axhspan(1.0, 1.0 + BOUNDARY_TOL, color="orange", alpha=0.25)
    for k, col in ((1, "orange"), (2, "red")):
        m = cls == k
        if m.any():
            ax_gamma.scatter(times[m], np.sqrt(gamma[m]), s=6, color=col, zorder=4)
    top = max(1.3, min(np.sqrt(gamma.max()) * 1.05, 4.0))
    ax_gamma.set(xlabel="Time (s)", ylabel=r"$\sqrt{\Gamma(x)}$", ylim=(0, top),
                 title="Inside / outside modulation region")
    ax_gamma.text(0.02, 0.06, "inside (Γ<1)", transform=ax_gamma.transAxes,
                  fontsize=7, color="red")
    ax_gamma.grid(True, ls=":", alpha=.7)
    ax_gamma.legend(fontsize=7, loc="upper right")
    clr = box_clearance(positions, model.box_center, model.box_size)
    ax_gamma.set_title(
        f"min Γ = {gamma.min():.4f},  samples inside = {(cls == 2).sum()},  "
        f"min box clearance = {clr.min()*100:.1f} cm", fontsize=8)


# ------------------------------------------------------------ vector fields
_PLANES = {  # name: (horizontal idx, vertical idx, fixed idx)
    "xy": (0, 1, 2), "xz": (0, 2, 1), "yz": (1, 2, 0)}
_LABELS = "XYZ"


def draw_field(ax, plane, model, goal, positions, gamma, slice_point, n=17):
    ax.clear()
    h, v, f = _PLANES[plane]
    fixed = float(slice_point[f])
    c, a = model.center, model.axes

    # window: obstacle + data
    allp = np.vstack([positions, goal[None]])
    lims = []
    for d in (h, v):
        lo = min(c[d] - a[d], allp[:, d].min()) - 0.1
        hi = max(c[d] + a[d], allp[:, d].max()) + 0.1
        if d == 2:                       # ellipsoid is z=0-anchored: skip floor
            lo = max(lo, min(allp[:, 2].min(), model.box_center[2] - model.box_size[2] / 2) - 0.3)
        lims.append((lo, hi))
    U, V = np.meshgrid(np.linspace(*lims[0], n), np.linspace(*lims[1], n))
    nu = np.full_like(U, np.nan); nv = nu.copy(); mu = nu.copy(); mv = nu.copy()
    for r in range(n):
        for k in range(n):
            p = np.zeros(3); p[h], p[v], p[f] = U[r, k], V[r, k], fixed
            if model.gamma(p)[0] < 1.0:          # controller undefined inside
                continue
            nom, mod = model.velocities(p, goal)
            nu[r, k], nv[r, k], mu[r, k], mv[r, k] = nom[h], nom[v], mod[h], mod[v]
    ax.quiver(U, V, nu, nv, color="0.6", alpha=.7, label="Nominal DS")
    ax.quiver(U, V, mu, mv, color="red", alpha=.85, label="Modulated DS (controller)")

    th = np.linspace(0, 2 * np.pi, 200)
    # silhouette = largest cross-section = what a 2D projection "shadow" looks like
    ax.plot(c[h] + a[h] * np.cos(th), c[v] + a[v] * np.sin(th), color="b",
            ls="--", lw=1, alpha=.6, label="Ellipsoid silhouette (max extent)")
    rem = 1.0 - ((fixed - c[f]) / a[f]) ** 2
    if rem > 0:
        s = np.sqrt(rem)
        ax.plot(c[h] + a[h] * s * np.cos(th), c[v] + a[v] * s * np.sin(th),
                color="b", lw=1.8, label=f"Boundary at {_LABELS[f]}={fixed:.2f}")
    # physical box: silhouette + cross-section
    lo = model.box_center - model.box_size / 2; hi = model.box_center + model.box_size / 2
    ax.add_patch(plt_rect(lo[h], lo[v], hi[h] - lo[h], hi[v] - lo[v],
                          ec="k", fc="none", ls=":", lw=1))
    if lo[f] <= fixed <= hi[f]:
        ax.add_patch(plt_rect(lo[h], lo[v], hi[h] - lo[h], hi[v] - lo[v],
                              ec="k", fc="0.3", alpha=.5, lw=1.2))

    cls = classify(gamma)
    ax.plot(positions[:, h], positions[:, v], color="k", lw=1.0, alpha=.7,
            label="Robot trajectory (projected)")
    for k, col, lab in ((1, "orange", "on boundary"), (2, "red", "INSIDE")):
        m = cls == k
        if m.any():
            ax.scatter(positions[m, h], positions[m, v], s=7, color=col, zorder=5, label=lab)
    ax.scatter(positions[0, h], positions[0, v], marker="o", color="tab:blue", s=30, zorder=6, label="Start")
    ax.scatter(positions[-1, h], positions[-1, v], marker="^", color="tab:green", s=30, zorder=6, label="Current")
    ax.scatter(slice_point[h], slice_point[v], marker="D", facecolor="none",
               edgecolor="m", s=45, zorder=6, label="Closest approach")
    ax.scatter(goal[h], goal[v], marker="*", color="gold", edgecolor="k", s=70, zorder=6, label="Goal")
    ax.set_title(f"{_LABELS[h]}{_LABELS[v]} plane ({_LABELS[f]} = {fixed:.2f} m)", fontsize=9)
    ax.set_xlabel(_LABELS[h] + " (m)", fontsize=8); ax.set_ylabel(_LABELS[v] + " (m)", fontsize=8)
    ax.set_xlim(*lims[0]); ax.set_ylim(*lims[1])
    ax.set_aspect("equal", adjustable="box"); ax.tick_params(labelsize=7)
    ax.grid(True, ls=":", alpha=.5)


def plt_rect(x, y, w, hgt, **kw):
    from matplotlib.patches import Rectangle
    return Rectangle((x, y), w, hgt, **kw)


def draw_all_fields(axes, model, goal, positions):
    gamma = model.gamma(positions)
    slice_point = positions[int(np.argmin(gamma))]   # same point in all 3 panels
    for plane, ax in zip(("xy", "xz", "yz"), axes):
        draw_field(ax, plane, model, goal, positions, gamma, slice_point)
    axes[0].legend(fontsize=5, loc="upper right", ncol=2)
