"""Plan Sec.3 adversarial scenes: aligned seats, misaligned stops, blocked cavity does not tunnel."""
import sys, os, numpy as np, mujoco
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import scene

def run(conn, dx=0.0, dz=0.0, block=False, clearance=0.0005, speed=0.01, cable=True):
    c = scene.CONN[conn]
    y0 = scene.WALL_Y - 0.005 - c["tip_len"]  # tip starts 5 mm before the wall
    spec = scene.build(conn, clearance, plug_pos=(c["x"] + dx, y0, scene.SOCKET_Z + dz), cable=cable, cable_len=0.40, hand=True)
    if block:  # plate closing the cavity mouth
        spec.worldbody.add_geom(type=scene.BOX, pos=[c["x"], scene.WALL_Y + 0.001, scene.SOCKET_Z], size=[0.03, 0.001, 0.03])
    m = spec.compile(); d = mujoco.MjData(m)
    target = scene.WALL_Y + c["depth"] - c["tip_len"]  # plug origin y when seated
    pid = m.body("plug").id
    maxpen = 0
    for k in range(int(4.0 / m.opt.timestep)):
        t = k * m.opt.timestep
        d.ctrl[1] = min(speed * t, target + 0.004 - y0)  # command 4 mm past seat
        mujoco.mj_step(m, d)
        maxpen = max(maxpen, -min([cn.dist for cn in d.contact] + [0]))
    tip_y = d.xpos[pid][1] + c["tip_len"]
    return dict(tip_depth_mm=(tip_y - scene.WALL_Y) * 1e3, need_mm=c["depth"] * 1e3, lateral_mm=np.hypot(d.xpos[pid][0] - c["x"], d.xpos[pid][2] - scene.SOCKET_Z) * 1e3, max_pen_mm=maxpen * 1e3)

if __name__ == "__main__":
    for conn in ["usb", "jack", "ac"]:
        a = run(conn); mis = run(conn, dx=0.004); blk = run(conn, block=True)
        print(conn, "aligned", {k: round(v, 2) for k, v in a.items()})
        print(conn, "mis 4mm", {k: round(v, 2) for k, v in mis.items()})
        print(conn, "blocked", {k: round(v, 2) for k, v in blk.items()})
