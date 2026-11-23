# render candidate ways to hold the plug (weld, user-approved for insertion) + what the eye-in-hand camera sees
import sys, os, numpy as np, mujoco, imageio.v2 as iio
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import scene, robot
HE_P, HE_Q = [-0.0080293, -0.0495603, -0.0909769], [0.0097493, -0.0046708, -0.0127007, 0.9998609]  # Code/Calibration/calibration.yaml
T = lambda R, p: np.vstack([np.hstack([R, np.reshape(p, (3, 1))]), [0, 0, 0, 1]])
T_ee_tool = T(Rot.from_euler("z", 90, degrees=True).as_matrix(), [0, 0, 0.130])
T_tool_cam = T(Rot.from_quat(HE_Q).as_matrix() @ Rot.from_euler("x", 180, degrees=True).as_matrix(), HE_P)  # optical -> MuJoCo cam
q = np.load(os.path.join(robot.HERE, "assets/q_cad_pose.npy"))
# plug frame: +y tip, x thin side. R columns = plug x,y,z axes written in tool frame (tool z = approach, tool x = finger closing)
def R_of(x, y): x, y = np.array(x, float), np.array(y, float); return np.c_[x, y, np.cross(x, y)]
OPTIONS = {
  "A_cad_pose": dict(R=R_of([1, 0, 0], [0, 0, 1]), z=0.051, y=0.0, note="plug axis along approach, pads on thin 7.5 mm faces, tip 31 mm past TCP (from CAD)"),
  "B_rolled_90": dict(R=R_of([0, 1, 0], [0, 0, 1]), z=0.051, y=0.0, note="same but pads on the wide 15 mm faces"),
  "C_boot_grip": dict(R=R_of([1, 0, 0], [0, 0, 1]), z=0.066, y=0.0, note="gripped further back, tip 46 mm past TCP, more cable-side occlusion-free view"),
  "D_tilted_down": dict(R=R_of([1, 0, 0], [0, np.sin(np.radians(-20)), np.cos(np.radians(-20))]), z=0.051, y=0.0, note="plug pitched 20 deg down from approach axis"),
}
def fk_ee():
    s = robot.load(); m = s.compile(); d = mujoco.MjData(m); d.qpos[:6] = q; mujoco.mj_forward(m, d)
    b = m.body("end_effector_link").id; return T(d.xmat[b].reshape(3, 3), d.xpos[b])
T_w_ee = fk_ee()
os.makedirs(sys.argv[1], exist_ok=True)
for name, o in OPTIONS.items():
    T_tool_plug = T(o["R"], [0, o["y"], o["z"]]); T_ee_plug = T_ee_tool @ T_tool_plug; T_w_plug = T_w_ee @ T_ee_plug
    spec = scene.build("usb", 0.0005, robot=True, plug_pos=T_w_plug[:3, 3], plug_quat=Rot.from_matrix(T_w_plug[:3, :3]).as_quat()[[3, 0, 1, 2]],
                       cable=True, cable_len=0.30, curl=0.0)
    # cable: hang from the plug rear. weld plug to the hand, no plug-finger collisions
    qe = Rot.from_matrix(T_ee_plug[:3, :3]).as_quat()[[3, 0, 1, 2]]
    spec.add_equality(type=mujoco.mjtEq.mjEQ_WELD, name="hold", name1="end_effector_link", name2="plug", objtype=mujoco.mjtObj.mjOBJ_BODY,
                      data=[0, 0, 0, *T_ee_plug[:3, 3], *qe, 1.0], solref=[0.004, 1])
    T_ee_cam = T_ee_tool @ T_tool_cam
    spec.body("end_effector_link").add_camera(name="eih", pos=T_ee_cam[:3, 3], quat=Rot.from_matrix(T_ee_cam[:3, :3]).as_quat()[[3, 0, 1, 2]], fovy=58)
    for b in ["right_finger_prox_link", "right_finger_dist_link", "left_finger_prox_link", "left_finger_dist_link", "end_effector_link", "upper_wrist_link"]:
        spec.add_exclude(bodyname1=b, bodyname2="plug")
    m = spec.compile(); d = mujoco.MjData(m)
    d.qpos[:6] = q; d.ctrl[:6] = q; d.ctrl[6] = 0.80
    for _ in range(3000): mujoco.mj_step(m, d)
    ok = bool(np.isfinite(d.qpos).all()); print(name, "finite:", ok, "max|qvel|", float(abs(d.qvel).max()) if ok else "nan", "plug-to-hand drift mm", round(float(np.linalg.norm(d.xpos[m.body("plug").id] - (d.xpos[m.body("end_effector_link").id] + d.xmat[m.body("end_effector_link").id].reshape(3,3) @ T_ee_plug[:3, 3]))) * 1000, 2) if ok else "nan")
    r = mujoco.Renderer(m, 480, 640)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE; cam.lookat[:] = d.xpos[m.body("plug").id]; cam.distance = 0.22; cam.azimuth = 150; cam.elevation = -25
    r.update_scene(d, cam); a = r.render()
    cam.azimuth = 60; r.update_scene(d, cam); b = r.render()
    r.update_scene(d, camera="eih"); c = r.render()
    r.close(); iio.imwrite(f"{sys.argv[1]}/{name}.png", np.hstack([a, b, c]))
    print(name, o["note"])
