"""Level the camera on the wall: wall normal parallel to the camera axis, marker row horizontal. Closed loop on the camera view."""
import sys, json, subprocess, yaml, numpy as np, cv2, pathlib
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, "Code/IBVS")
from ibvs.kinematics import UrdfChain
from ibvs.alignment import camera_joint_jacobian
from ibvs.frames import matrix
S = pathlib.Path(sys.argv[1]); J = [f"joint_{i}" for i in range(1, 7)]
fk = UrdfChain((S/"cad/gen3_lite.urdf").read_text(), "base_link", "tool_frame", J)
g = yaml.safe_load(open("Code/Grasp/config/grasp.yaml")); T_EC = matrix(g["camera"]["hand_eye_xyz_m"], g["camera"]["hand_eye_quat_xyzw"])
ACT = "/joint_trajectory_controller/follow_joint_trajectory"
det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), cv2.aruco.DetectorParameters())

def measure(tag):
    d = S/("lv_"+tag); subprocess.run(["python3", str(S/"snapshot.py"), str(d)], capture_output=True, check=True)
    info = json.load(open(d/"camera_info.json")); K = np.array(info["K"]).reshape(3, 3); D = np.array(info["D"])
    depth = np.load(d/"depth_mm.npy").astype(float)/1000; gray = cv2.imread(str(d/"color.png"), 0)
    c, ids, _ = det.detectMarkers(gray)
    mk = {int(i): x.reshape(4, 2) for i, x in zip(ids.ravel(), c)} if ids is not None else {}
    assert all(i in mk for i in (10, 11, 12)), "markers 10,11,12 not all visible: %s" % sorted(mk)
    ctr = np.array([mk[i].mean(0) for i in (10, 11, 12)])
    row = np.polyfit(ctr[:, 0], ctr[:, 1], 1); roll = np.arctan(row[0])                 # image-plane angle of the marker row
    h, w = depth.shape
    x0, x1 = max(0, int(ctr[:, 0].min())), min(w, int(ctr[:, 0].max())); y0 = int(max(mk[i][:, 1].max() for i in mk)) + 20; y1 = min(h, y0 + 260)   # wall below the markers
    assert y1 - y0 > 60 and x1 - x0 > 60, "wall region not in view (y %d..%d, x %d..%d)" % (y0, y1, x0, x1)
    v, u = np.mgrid[y0:y1, x0:x1]; z = depth[y0:y1, x0:x1]; ok = (z > .07) & (z < 1.0)
    uv = np.column_stack([u[ok], v[ok]]).astype(np.float64); xy = cv2.undistortPoints(uv.reshape(-1, 1, 2), K, D).reshape(-1, 2)
    P = np.column_stack([xy, np.ones(len(xy))])*z[ok][:, None]
    rng = np.random.default_rng(0); best = None
    for _ in range(300):
        a, b, cc = P[rng.choice(len(P), 3, replace=False)]; n = np.cross(b-a, cc-a); l = np.linalg.norm(n)
        if l < 1e-9: continue
        n /= l; inl = np.abs((P-a)@n) < .004
        if best is None or inl.sum() > best.sum(): best = inl
    Q = P[best]; n = np.linalg.svd(Q-Q.mean(0), full_matrices=False)[2][-1]; n = n if n[2] < 0 else -n
    rvec = np.array([n[1], -n[0], roll])                                                # camera rotation that squares the view
    q = np.array([json.load(open(d/"joints.json"))[j] for j in J])
    dist = float(np.median(Q[:, 2]))
    print("[%s] wall distance %.0f mm, plane inliers %d/%d, normal tilt about cam x %.2f deg, y %.2f deg, marker-row roll %.2f deg" % (tag, dist*1e3, best.sum(), len(P), np.degrees(rvec[0]), np.degrees(rvec[1]), np.degrees(roll)), flush=True)
    return q, rvec

def send(q0, q1):
    dq = np.degrees(np.abs(q1-q0)).max(); T = max(4.0, dq/5.0+1.0); pts = []
    for s in np.linspace(0, 1, 11)[1:]:
        p = 10*s**3-15*s**4+6*s**5; v = (30*s**2-60*s**3+30*s**4)/T; t = 0.1+T*s
        pts.append(dict(positions=(q0+(q1-q0)*p).tolist(), velocities=((q1-q0)*v).tolist(), time_from_start=dict(sec=int(t), nanosec=int((t % 1)*1e9))))
    goal = json.dumps(dict(trajectory=dict(joint_names=J, points=pts)))
    r = subprocess.run(["ros2", "action", "send_goal", ACT, "control_msgs/action/FollowJointTrajectory", goal], capture_output=True, text=True)
    assert "SUCCEEDED" in r.stdout, r.stdout[-300:]

hist = []
for it in range(5):
    q, r = measure("it%d" % it); hist.append(np.degrees(np.abs(r)).max())
    if hist[-1] < 0.5: print("LEVEL: all angles under 0.5 deg"); break
    if hist[-1] > 30: print("ABORT: measured tilt over 30 deg, not trusting it"); break
    if it and hist[-1] > hist[-2]*1.1: print("ABORT: error grew (%.2f -> %.2f deg), hand-eye rotation too wrong for this correction" % (hist[-2], hist[-1])); break
    Jc = camera_joint_jacobian(fk, q, T_EC)
    dq = np.linalg.solve(Jc, np.r_[0, 0, 0, 0.8*r])                                      # rotate about the camera centre, translation held
    if np.degrees(np.abs(dq)).max() > 30: print("ABORT: needed joint step %.0f deg too big" % np.degrees(np.abs(dq)).max()); break
    q1 = q+dq; Tt = fk(q1); fing = (Tt@matrix([0, 0, .035], [0, 0, 0, 1]))[:3, 3]
    assert fing[0] < 0.50 and fing[2] > 0.0, "fingertips would reach %s" % fing.round(3)
    print("   joint step deg", np.degrees(dq).round(2).tolist(), "fingertips after", fing.round(3).tolist(), flush=True)
    send(q, q1)
json.dump(dict(joints_deg=np.degrees(q).round(4).tolist(), history_max_angle_deg=[round(float(h), 3) for h in hist]), open(S/"leveled_pose.json", "w"), indent=1)
