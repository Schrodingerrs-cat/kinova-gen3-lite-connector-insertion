"""MuJoCo scene: wall with open socket cavities, plug, cable chain, optional Gen3 Lite.
World z-up, metres. Wall front face y=WALL_Y, insertion direction +y.
All clearances/cable params are UNCALIBRATED (see plan fidelity ledger)."""
import os, numpy as np, mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
WALL_Y, WALL_T, SOCKET_Z = 0.550, 0.003, 0.127
BOX, CYL = mujoco.mjtGeom.mjGEOM_BOX, mujoco.mjtGeom.mjGEOM_CYLINDER
CAP = mujoco.mjtGeom.mjGEOM_CAPSULE

# tip (dx, dz) across axis, tip length, seated depth, body (dx, dz), body length; metres, from CAD sections
CONN = {
    "usb":  dict(x=0.075,  tip=(0.0045, 0.012),  tip_len=0.0153, depth=0.0153, body=(0.0075, 0.015), body_len=0.020),
    "jack": dict(x=0.0,    tip=(0.0035, 0.0035), tip_len=0.0191, depth=0.0191, body=(0.0065, 0.0065), body_len=0.020, round=True),
    "ac":   dict(x=-0.075, tip=(0.014, 0.0175),  tip_len=0.024,  depth=0.024,  body=(0.020, 0.0175), body_len=0.030),
}
CABLE = dict(n=24, length=0.30, radius=0.003, mass=0.03, stiff=0.02, damp=0.002)  # UNCALIBRATED


def _box(body, pos, half, rgba=(0.6, 0.6, 0.65, 1), name=None):
    return body.add_geom(type=BOX, pos=pos, size=half, rgba=rgba, name=name, friction=[0.4, 0.005, 0.0001])


BOARD_P0 = np.array([0.0, WALL_Y, SOCKET_Z])  # board frame origin; board is a mocap body so its pose can be randomised per episode


def add_wall(spec, clearance):
    """wall + socket cavities as children of mocap body 'board'; positions relative to BOARD_P0"""
    board = spec.worldbody.add_body(name="board", mocap=True, pos=BOARD_P0)
    t, band = 0.006, 0.02
    gaps = []
    for c in CONN.values():
        hx, hz = c["tip"][0] / 2 + clearance, c["tip"][1] / 2 + clearance
        d, cx = c["depth"], c["x"]
        _box(board, (cx - hx - t / 2, d / 2 + 0.001, 0), (t / 2, d / 2 + 0.001, hz + t))
        _box(board, (cx + hx + t / 2, d / 2 + 0.001, 0), (t / 2, d / 2 + 0.001, hz + t))
        _box(board, (cx, d / 2 + 0.001, hz + t / 2), (hx, d / 2 + 0.001, t / 2))
        _box(board, (cx, d / 2 + 0.001, -hz - t / 2), (hx, d / 2 + 0.001, t / 2))
        _box(board, (cx, d + 0.003, 0), (hx + t, 0.002, hz + t))
        gaps.append((cx - hx - t, cx + hx + t))
    gaps.sort()
    edges = [-0.125] + [e for g in gaps for e in g] + [0.125]
    wall = (0.9, 0.6, 0.1, 1)
    for lo, hi in zip(edges[::2], edges[1::2]):
        if hi - lo > 1e-6:
            _box(board, ((lo + hi) / 2, WALL_T / 2, 0), ((hi - lo) / 2, WALL_T / 2, band), wall)
    _box(board, (0, WALL_T / 2, band + 0.04), (0.125, WALL_T / 2, 0.04), wall)
    _box(board, (0, WALL_T / 2, -band - 0.09), (0.125, WALL_T / 2, 0.09), wall)


HAND = dict(kp=2000.0, kv=20.0, fmax=10.0)  # gripper stand-in for contact tests: force-limited


def add_plug(spec, conn, pos, quat, hand=False):
    c = CONN[conn]
    p = spec.worldbody.add_body(name="plug", pos=pos, quat=quat)
    if hand:
        for ax, nm in zip(np.eye(3), "xyz"):
            p.add_joint(type=mujoco.mjtJoint.mjJNT_SLIDE, name="h" + nm, axis=ax, damping=HAND["kv"])
            spec.add_actuator(name="a" + nm, target="h" + nm, trntype=mujoco.mjtTrn.mjTRN_JOINT, gainprm=[HAND["kp"]] + [0] * 9,
                              biastype=mujoco.mjtBias.mjBIAS_AFFINE, biasprm=[0, -HAND["kp"]] + [0] * 8,
                              forcelimited=1, forcerange=[-HAND["fmax"], HAND["fmax"]])
    else:
        p.add_freejoint()
    tl, bl = c["tip_len"], c["body_len"]
    fr = [0.4, 0.005, 0.0001]
    if c.get("round"):
        p.add_geom(type=CYL, pos=[0, tl / 2, 0], euler=[90, 0, 0], size=[c["tip"][0] / 2, tl / 2, 0], rgba=[0.1, 0.7, 0.2, 1], friction=fr, mass=0.002)
        p.add_geom(type=CYL, pos=[0, -bl / 2, 0], euler=[90, 0, 0], size=[c["body"][0] / 2, bl / 2, 0], rgba=[0.1, 0.7, 0.2, 1], friction=fr, mass=0.004)
    else:
        p.add_geom(type=BOX, pos=[0, tl / 2, 0], size=[c["tip"][0] / 2, tl / 2, c["tip"][1] / 2], rgba=[0.1, 0.4, 0.9, 1], friction=fr, mass=0.002)
        p.add_geom(type=BOX, pos=[0, -bl / 2, 0], size=[c["body"][0] / 2, bl / 2, c["body"][1] / 2], rgba=[0.1, 0.4, 0.9, 1], friction=fr, mass=0.006)
    return p, -bl


def add_cable(spec, plug, y0, length, curl):
    """capsule chain behind the plug; `curl` = total turning (rad) so the cable can lie as a loop.
    far end is free"""
    n, r = CABLE["n"], CABLE["radius"]
    seg = length / n
    dth = np.degrees(curl / n)
    parent = plug
    for i in range(n):
        kw = dict(pos=[0, y0, 0]) if i == 0 else dict(pos=[0, -seg, 0], euler=[0, 0, dth])
        b = parent.add_body(name=f"cable{i}", **kw)
        b.add_joint(type=mujoco.mjtJoint.mjJNT_BALL, stiffness=CABLE["stiff"], damping=CABLE["damp"], armature=1e-5)
        b.add_geom(type=CAP, fromto=[0, 0, 0, 0, -seg, 0], size=[r, 0, 0], rgba=[0.15, 0.15, 0.15, 1], mass=CABLE["mass"] / n, friction=[CABLE.get('friction', 0.5), 0.005, 0.0001])
        parent = b
    return parent


# domain randomisation ranges (user: only cable drag + contact physics, from 0 up to a max). UNCALIBRATED upper bounds.
DR = dict(cable_stiff=(0.0, 0.05), cable_damp=(0.0, 0.01), cable_mass=(0.0005, 0.06), cable_friction=(0.0, 1.0), contact_friction=(0.1, 0.8))


def sample_dr(rng):
    return {k: float(rng.uniform(*v)) for k, v in DR.items()}


def build(conn="usb", clearance=0.0005, robot=False, plug_pos=None, plug_quat=(1, 0, 0, 0), cable=True, dt=0.0005, cable_len=None, curl=0.0, hand=False, dr=None):
    dr = dr or {}
    CABLE.update(stiff=dr.get('cable_stiff', CABLE['stiff']), damp=dr.get('cable_damp', CABLE['damp']), mass=dr.get('cable_mass', CABLE['mass']), friction=dr.get('cable_friction', 0.5))
    import robot as _robot
    spec = _robot.load() if robot else mujoco.MjSpec()
    spec.option.timestep = dt
    spec.option.noslip_iterations = 3
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.gravity = [0, 0, -9.81]
    spec.default.geom.margin = 0.0001
    spec.default.geom.solref = [0.002, 1]
    add_wall(spec, clearance)
    floor = spec.worldbody.add_geom(type=BOX, pos=[0, 0.25, -0.03], size=[0.3, 0.45, 0.005], rgba=[0.5, 0.5, 0.5, 1])
    plug_pos = plug_pos if plug_pos is not None else (CONN[conn]["x"], WALL_Y - 0.10, SOCKET_Z)
    plug, rear = add_plug(spec, conn, plug_pos, plug_quat, hand)
    if cable:
        L = cable_len or CABLE["length"]
        add_cable(spec, plug, rear, L, curl)
        # cable end is free (user: cable is just placed in the cradle, not attached)
    return spec
