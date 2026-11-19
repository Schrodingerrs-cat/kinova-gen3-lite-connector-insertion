"""Smooth joint trajectories (pure NumPy, no ROS): quintic time scaling, so position, velocity and acceleration are continuous and zero at both ends."""
import numpy as np


def quintic(tau):
    """Time scaling tau in [0, 1] -> (s, ds/dtau, d2s/dtau2): s = 10t^3 - 15t^4 + 6t^5."""
    return 10 * tau**3 - 15 * tau**4 + 6 * tau**5, 30 * tau**2 - 60 * tau**3 + 30 * tau**4, 60 * tau - 180 * tau**2 + 120 * tau**3


def duration_for(dq_max_rad, speed_rad_s, v_peak_rad_s, t_min=1.0):
    """Move time: average speed `speed`, but the quintic peak (1.875 x average) never above v_peak; never below t_min."""
    return float(max(t_min, dq_max_rad / speed_rad_s, 1.875 * dq_max_rad / v_peak_rad_s))


def joint_move(q0, q1, T, rate_hz=10):
    """Straight joint-space move in T s. Returns times, positions, velocities, accelerations (rows = trajectory points)."""
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    tau = np.linspace(0.0, 1.0, max(8, int(T * rate_hz)) + 1)
    s, ds, dds = quintic(tau)
    d = q1 - q0
    return tau * T, q0 + np.outer(s, d), np.outer(ds, d) / T, np.outer(dds, d) / T**2


def path_move(pts, T):
    """Waypoints of a Cartesian line (joint space) run with the quintic profile over T s. Waypoints keep their geometry; their times follow the profile
    (slow at the ends, fast in the middle). Returns times, positions, velocities, accelerations."""
    P = np.asarray(pts, float)
    seg = np.abs(np.diff(P, axis=0)).max(axis=1)
    s = np.r_[0.0, np.cumsum(seg)] / max(seg.sum(), 1e-12)
    grid = np.linspace(0.0, 1.0, 2001)
    tau = np.interp(s, quintic(grid)[0], grid)                  # invert s(tau)
    _, ds, dds = quintic(tau)
    dq_ds = np.gradient(P, s, axis=0, edge_order=2)              # joint change per unit path
    d2q_ds2 = np.gradient(dq_ds, s, axis=0, edge_order=2)
    v = dq_ds * (ds / T)[:, None]
    a = d2q_ds2 * ((ds / T) ** 2)[:, None] + dq_ds * (dds / T**2)[:, None]
    return tau * T, P, v, a
