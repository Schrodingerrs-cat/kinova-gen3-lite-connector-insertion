# G2: joint angles (within limits) so URDF link meshes match CAD link meshes; prints residuals
import os, numpy as np, mujoco, trimesh
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
m = mujoco.MjSpec.from_file(f"{HERE}/assets/robot/gen3_lite.urdf").compile(); d = mujoco.MjData(m)
cad = {"shoulder_link": "shoulder", "arm_link": "arm", "forearm_link": "forearm", "lower_wrist_link": "lower_wrist", "upper_wrist_link": "upper_wrist", "end_effector_link": "gripper_model"}
rng = np.random.default_rng(0)
tgt, loc = {}, {}
for link, part in cad.items():
    p = trimesh.load(f"{HERE}/assets/cad_parts/{part}.stl").sample(40000)
    tgt[link] = cKDTree(np.c_[p[:, 0], -p[:, 2], p[:, 1]])  # CAD (x,y,z) -> world (x,-z,y)
    gm = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body(link).id and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    md = m.geom_dataid[gm[0]]
    loc[link] = (m.mesh_vert[m.mesh_vertadr[md]:m.mesh_vertadr[md] + m.mesh_vertnum[md]][::2], gm[0])
def link_pts(q):
    d.qpos[:6] = q; mujoco.mj_kinematics(m, d)
    return {l: v @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g] for l, (v, g) in loc.items()}
def resid(q):
    P = link_pts(q); return np.concatenate([tgt[l].query(P[l])[0] for l in cad])
lo, hi = m.jnt_range[:6].T
q123 = np.array([-1.5935, 1.1695, -1.9289])
best = None
for s in range(40):
    q0 = np.r_[q123, rng.uniform(lo[3:], hi[3:])]
    sol = least_squares(lambda w: resid(np.r_[q123, w]), q0[3:], bounds=(lo[3:], hi[3:]), loss="soft_l1", f_scale=0.01)
    sol.x = np.r_[q123, sol.x]
    if best is None or sol.cost < best.cost: best = sol
q = best.x; P = link_pts(q)
print("q (rad):", np.round(q, 4).tolist(), " limits ok:", bool(np.all(q >= lo) and np.all(q <= hi)))
for l in cad: print(f"{l:18s} mean nearest-point error {tgt[l].query(P[l])[0].mean()*1000:.2f} mm")
np.save(f"{HERE}/assets/q_cad_pose.npy", q)
