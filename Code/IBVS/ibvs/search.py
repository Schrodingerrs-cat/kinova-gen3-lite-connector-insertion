import numpy as np


def candidates(c):
    """(dx, dy, dyaw) in the socket frame, nominal first, all inside the measured bounds, at most max_attempts.
    Grid steps ordered by ring (Chebyshev index) then by distance under the prior sigma."""
    nx, ny, nw = (int(c["max_abs_x_m"] // c["step_x_m"]), int(c["max_abs_y_m"] // c["step_y_m"]),
                  int(c["max_abs_yaw_rad"] // c["step_yaw_rad"]))
    grid = [(i, j, k) for i in range(-nx, nx + 1) for j in range(-ny, ny + 1) for k in range(-nw, nw + 1)]

    sx, sw = c["sigma_xy_m"], c["sigma_yaw_rad"]

    def key(g):
        d = (g[0] * c["step_x_m"] / sx) ** 2 + (g[1] * c["step_y_m"] / sx) ** 2 + (g[2] * c["step_yaw_rad"] / sw) ** 2
        return (max(map(abs, g)), d, g)
    grid.sort(key=key)
    out = [(i * c["step_x_m"], j * c["step_y_m"], k * c["step_yaw_rad"]) for i, j, k in grid]
    return out[: c["max_attempts"]]
