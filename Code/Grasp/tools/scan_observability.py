#!/usr/bin/env python3
"""How well does a set of camera poses pin down a connector's pose? Offline, no robot: python3 tools/scan_observability.py [connector]
For every model surface sample the views can see (normal towards the camera, 70-500 mm, above the panel) the point-to-plane information J J^T / sigma^2 is summed;
J = [n, p x n] (6 pose parameters), sigma grows with range squared (stereo depth), samples per mm^2 fall with range squared and with the viewing angle.
The inverse gives the expected 1-sigma pose error for noise sigma0 at 200 mm. Not a replacement for the leave-one-view-out gate: it says which views are worth taking."""
import pathlib
import sys

import numpy as np
import trimesh
import yaml
from scipy.spatial.transform import Rotation as Rot

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from housing_grasp import cad_fit  # noqa: E402

F_PX = 434.7


def evaluate(M, N, area_per_sample_mm2, cams_m, T_model, sigma0_mm=0.8, seated=False, fov_deg=40):
    """M, N model samples (mm, model frame). cams_m = list of 4x4 T_BC (m). T_model: model -> base (mm). Returns dict of expected 1-sigma errors and coverage."""
    P, Nb = M @ T_model[:3, :3].T + T_model[:3, 3], N @ T_model[:3, :3].T
    ref = P.mean(0)
    I, seen = np.zeros((6, 6)), np.zeros(len(P), bool)
    for T in cams_m:
        c, ez = T[:3, 3] * 1000, T[:3, 2]
        v = c - P
        z = np.linalg.norm(v, axis=1)
        cos = (Nb * v).sum(1) / z
        ang = np.degrees(np.arccos(np.clip(((P - c) / z[:, None]) @ ez, -1, 1)))
        vis = (cos > 0.2) & (z > 70) & (z < 500) & (P[:, 2] > T_model[2, 3] - 1) & (ang < fov_deg)
        if not vis.any():
            continue
        seen |= vis
        sig = sigma0_mm * (z[vis] / 200.0) ** 2
        w = area_per_sample_mm2 * cos[vis] * (F_PX / z[vis]) ** 2 / sig ** 2
        J = np.c_[Nb[vis], np.cross(P[vis] - ref, Nb[vis])]
        I += (J * w[:, None]).T @ J
    cols = [0, 1, 2, 5] if seated else list(range(6))      # seated on the panel: tilt is known (columns 3, 4 removed)
    Isub = I[np.ix_(cols, cols)]
    ev = np.linalg.eigvalsh(Isub)
    out = dict(coverage_pct=float(100 * seen.mean()), views_seeing=0)
    if ev.min() < 1e-9 * max(ev.max(), 1e-12):
        C = np.linalg.pinv(Isub, rcond=1e-9)
        out["note"] = "some direction unobservable (e.g. spin of a round connector)"
    else:
        C = np.linalg.inv(Isub)
    sd = np.sqrt(np.clip(np.diag(C), 0, None))
    names = ["x", "y", "z", "rx", "ry", "rz"]
    for k, i in enumerate(cols):
        out["sigma_" + names[i]] = float(sd[k] * (180 / np.pi if i >= 3 else 1.0))     # mm or deg
    return out


def fk_cameras(joints_deg, cfg):
    """Camera poses (m) of recorded joint poses: URDF forward kinematics + the hand-eye of config/grasp.yaml."""
    import xml.etree.ElementTree as ET
    root = ET.parse("/opt/ros/humble/share/kortex_description/robots/gen3_lite.urdf").getroot()
    J = {j.get("name"): j for j in root.iter("joint") if j.find("origin") is not None}

    def O(j):
        o = j.find("origin")
        M = np.eye(4)
        M[:3, :3] = Rot.from_euler("xyz", [float(x) for x in o.get("rpy").split()]).as_matrix()
        M[:3, 3] = [float(x) for x in o.get("xyz").split()]
        return M
    arm = [O(J["joint_%d" % i]) for i in range(1, 7)]
    tool = O(J["end_effector"]) @ O(J["tool_frame_joint"])
    X = np.eye(4)
    X[:3, :3], X[:3, 3] = Rot.from_quat(cfg["camera"]["hand_eye_quat_xyzw"]).as_matrix(), cfg["camera"]["hand_eye_xyz_m"]
    out = []
    for q in joints_deg:
        M = np.eye(4)
        for o, a in zip(arm, np.radians(q)):
            c, s = np.cos(a), np.sin(a)
            M = M @ o @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        out.append(M @ tool @ X)
    return out


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "micro_jack"
    ccfg, spec = cad_fit.load_config(name)
    cfg = yaml.safe_load(open(HERE / "config" / "grasp.yaml"))
    M, N = cad_fit.model_cloud(spec)
    sc = trimesh.load(cad_fit.ROOT / spec["mesh"])
    mesh = sc.to_geometry() if hasattr(sc, "to_geometry") else sc
    area = mesh.area * 1e6 / 40000
    T = np.eye(4)
    T[:3, 3] = [260.0, -20.0, -spec["seat_u_mm"]]                    # standing in the panel at z = 0, near where it sat in the runs
    tgt = np.array([0.26, -0.02, 0.01])
    Rh = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])             # home camera looking straight down
    sets = {}
    sets["home view only (0.35 m above)"] = [np.block([[Rh, np.array([[0.26], [-0.02], [0.35]])], [np.zeros((1, 3)), np.ones((1, 1))]])]
    j7 = np.array(cfg["scan"]["views_joints_deg"])
    sets["7 recorded scan poses (the current scan)"] = fk_cameras(j7, cfg)
    for tilt, d in ((35, 0.20), (45, 0.20), (35, 0.25)):
        sets["3 close views %d deg, %.2f m, az 0/120/240" % (tilt, d)] = [cad_fit.close_view_pose(tgt, az, d, tilt, Rh) for az in (0, 120, 240)]
    sets["2 close views 35 deg, 0.20 m, az 0/180"] = [cad_fit.close_view_pose(tgt, az, 0.20, 35, Rh) for az in (0, 180)]
    sets["4 close views 35 deg, 0.20 m, az 0/90/180/270"] = [cad_fit.close_view_pose(tgt, az, 0.20, 35, Rh) for az in (0, 90, 180, 270)]
    sets["3 near-vertical at 0.31 m (Jay's branch)"] = [cad_fit.close_view_pose(tgt, az, 0.31, 8, Rh) for az in (0, 120, 240)]
    sets["7 recorded + 3 close (35 deg, 0.20 m)"] = sets["7 recorded scan poses (the current scan)"] + sets["3 close views 35 deg, 0.20 m, az 0/120/240"]
    print("connector %s, expected 1-sigma pose error (depth noise 0.8 mm at 200 mm, grows with range^2)" % name)
    print("%-48s | cover | free pose: tilt deg  pos mm (x y z)      | seated on panel: pos mm (x y z)   yaw deg" % "view set")
    for k, cams in sets.items():
        a, b = evaluate(M, N, area, cams, T), evaluate(M, N, area, cams, T, seated=True)
        tilt = max(a.get("sigma_rx", np.inf), a.get("sigma_ry", np.inf))
        yaw = b.get("sigma_rz", np.inf)
        print("%-48s | %4.0f%% |  %6.2f      %5.2f %5.2f %5.2f     |  %5.2f %5.2f %5.2f      %s" % (k, a["coverage_pct"], tilt, a.get("sigma_x", np.inf), a.get("sigma_y", np.inf), a.get("sigma_z", np.inf),
              b.get("sigma_x", np.inf), b.get("sigma_y", np.inf), b.get("sigma_z", np.inf), "unobservable (round)" if spec["symmetry"] == "axial" else "%.2f" % yaw))


if __name__ == "__main__":
    main()
