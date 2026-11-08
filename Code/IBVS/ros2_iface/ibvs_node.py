"""ROS 2 runner for the IBVS servo and the B0/A1/A2 cycle. Needs the arm driver + camera (Code/Grasp/run_ros.sh minus the grasp node).

  python3 ros2_iface/ibvs_node.py servo            dry run: computes and logs commands from the live camera, publishes nothing
  python3 ros2_iface/ibvs_node.py servo --live     non-contact servo: switches to the twist controller and publishes Twist
  python3 ros2_iface/ibvs_node.py cycle A2 --live  full cycle with guarded insertion. Needs CALIBRATED config + safety flags.
Ctrl-C always sends a zero twist.
"""
import argparse
import pathlib
import sys
import time

import cv2
import numpy as np
import rclpy
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import Bool

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1] / "Grasp" / "ros2_iface"))
from robot_ros import Robot  # noqa: E402
from ibvs.config import load_config  # noqa: E402
from ibvs.control import Convergence, axes_matrix, limit, servo_twist  # noqa: E402
from ibvs.depth import corner_depths, fill_depths  # noqa: E402
from ibvs.frames import adjoint, matrix  # noqa: E402
from ibvs.guards import check_axial, check_servo  # noqa: E402
from ibvs.logger import TrialLogger  # noqa: E402
from ibvs.markers import Tracker  # noqa: E402
from ibvs.search import candidates  # noqa: E402
from ibvs.state_machine import run_cycle  # noqa: E402
from ibvs.verify import Seat, seat_state  # noqa: E402


class Ibvs:
    def __init__(self, cfg, live, log):
        self.c, self.live, self.log = cfg, live, log
        self.r = Robot("ibvs", background=True)
        self.trk = Tracker(cfg["camera"], cfg["markers"])
        self.A = axes_matrix(cfg["ibvs"]["socket_axes_in_C"])
        self.T_EC = matrix(cfg["camera"]["T_EC_xyz_m"], cfg["camera"]["T_EC_quat_xyzw"])
        self.pub = self.r.create_publisher(Twist, cfg["robot"]["twist_topic"], 1)
        self.sw = self.r.create_client(SwitchController, "/controller_manager/switch_controller")
        self.samples, self.travel, self.last_stamp, self.joint_t = [], 0.0, 0.0, 0.0
        self.r.create_subscription(JointState, "/joint_states", lambda m: setattr(self, "joint_t", self.now()), 10)
        if "verification" in cfg and cfg["verification"].get("topic"):
            self.r.create_subscription(Bool, cfg["verification"]["topic"], lambda m: self.samples.append((self.now(), m.data)), 10)

    # ---- helpers ----
    def now(self):
        return self.r.get_clock().now().nanoseconds * 1e-9

    def stamp(self, msg):
        return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def zero(self):
        if self.live:
            self.pub.publish(Twist())

    def switch(self, on, off):
        if not self.live:
            return True
        req = SwitchController.Request(start_controllers=[on], stop_controllers=[off], strictness=SwitchController.Request.STRICT)
        fut = self.sw.call_async(req)
        self.r.spin_until(fut.done, 5.0)
        return fut.result() is not None and fut.result().ok

    def frame(self):
        """Newest color (gray) and depth (m) with the color stamp, or None."""
        c, d = self.r._img.get("color"), self.r._img.get("depth")
        if c is None or d is None or self.stamp(c) == self.last_stamp:
            return None
        self.last_stamp = self.stamp(c)
        img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, -1)
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if c.encoding == "rgb8" else cv2.COLOR_BGR2GRAY)
        depth = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) * 1e-3
        return gray, depth, self.stamp(c)

    def tool_xyz(self):
        return self.r.tool_matrix()[:3, 3]

    def move_tool_to(self, T, min_s=0.5):
        m = self.c["motion"]
        q = self.r.ik(T, np.degrees(self.r.q))
        if q is None or np.abs(q - np.degrees(self.r.q)).max() > m["max_joint_dev_deg"]:
            return "NO_NEAR_IK"
        box = self.c["robot"]["workspace_box_B_m"]
        x, y, z = T[:3, 3]
        if not (box[0] <= x <= box[1] and box[2] <= y <= box[3] and box[4] <= z <= box[5]):
            return "OUT_OF_BOUNDS"
        dq = np.abs(q - np.degrees(self.r.q)).max()
        self.r.move_joints(q, max(min_s, dq / m["joint_speed_deg_s"]))
        return True

    def axis_B(self):
        return self.r.tool_matrix()[:3, :3] @ self.T_EC[:3, :3] @ np.array(self.c["ibvs"]["socket_axes_in_C"])[:, 2]

    # ---- IBVS ----
    def servo_align(self):
        c, ib = self.c, self.c["ibvs"]
        if not self.switch(c["robot"]["twist_controller"], c["robot"]["traj_controller"]):
            return "CONTROLLER_SWITCH"
        conv, prev, t0, tl, missing, why = Convergence(ib["tol_px"], ib["converge_frames"]), np.zeros(3), self.now(), self.now(), 0, None
        try:
            while why is None:
                f = self.frame()
                if f is None:
                    self.r.spin_until(lambda: False, 0.005)
                    continue
                gray, depth, stamp = f
                px, valid = self.trk.detect(gray)
                missing = 0 if valid.sum() >= c["markers"]["min_valid_markers"] else missing + 1
                why = check_servo(self.now(), stamp, self.joint_t, self.tool_xyz(), c["guards"], c["robot"]["workspace_box_B_m"],
                                  t0, ib["timeout_s"], missing, c["markers"]["max_missing_frames"])
                if why or missing:
                    self.zero()
                    continue
                s, s_star, rms = self.trk.features(px, valid)
                Z = fill_depths(corner_depths(depth, px[valid].reshape(-1, 2), ib["depth_min_m"], ib["depth_max_m"]))
                if Z is None:
                    why = "DEPTH_INVALID"
                    break
                u3 = servo_twist(s, s_star, Z, self.A, ib["lambda_gain"], ib["damping"], ib["max_cond"])
                if u3 is None:
                    why = "ILL_CONDITIONED"
                    break
                dt, tl = self.now() - tl, self.now()
                u = limit(u3, prev, ib["max_lin_m_s"], ib["max_yaw_rad_s"], ib["max_acc"], dt)
                xi = self.A @ u
                if c["robot"]["command_frame"] == "tool":
                    xi = adjoint(self.T_EC) @ xi
                if self.live:
                    self.pub.publish(Twist(linear=Vector3(x=xi[0], y=xi[1], z=xi[2]), angular=Vector3(x=xi[3], y=xi[4], z=xi[5])))
                prev = u
                self.log.row(phase="SERVO_ALIGN", q=list(self.r.q), features_px=list(px.reshape(-1)),
                             desired_features_px=list(self.trk.target_px.reshape(-1)), feature_confidence=float(valid.mean()),
                             depth=list(Z), u_command=list(u3), u_executed=list(u), stop_reason=f"rms_px={rms:.2f}")
                if conv.update(rms):
                    return True
        finally:
            self.zero()
            self.switch(c["robot"]["traj_controller"], c["robot"]["twist_controller"])
        return why

    # ---- cycle primitives ----
    def check_calibration(self):
        info = self.r._img.get("info")
        if info is None or np.abs(np.array(info.k).reshape(3, 3) - self.trk.K).max() > 1.0:
            return "K_MISMATCH"
        return True

    def pick(self):
        w = self.c["waypoints"]
        self.r.gripper(0.0)
        for k in ("pre_grasp_deg", "grasp_deg"):
            self.r.move_joints(w[k], 4.0)
        self.r.gripper(w["grip_close_cmd"], 20.0)
        self.r.move_joints(w["lift_deg"], 4.0)
        return True

    def verify_grasp(self):
        w = self.c["waypoints"]
        return self.r.grip_pos_msr < w["grip_empty_frac"] * w["grip_close_cmd"] or "GRASP_EMPTY"

    def preview(self):
        self.r.move_joints(self.c["waypoints"]["inspection_deg"], 4.0)
        self.last_stamp = 0.0
        self.r.spin_until(lambda: self.frame_peek(), 2.0)
        return self.guard_check()

    def frame_peek(self):
        f = self.frame()
        if f is not None:
            self._peek = f
        return f is not None

    def guard_check(self):
        if not self.frame_peek():
            return "NO_FRAME"
        _, valid = self.trk.detect(self._peek[0])
        return bool(valid.sum() >= self.c["markers"]["min_valid_markers"]) or "MARKERS_NOT_VISIBLE"

    def attempt(self):
        ax, total = self.c["axial"], 0.0
        self.travel = 0.0
        while True:
            T = self.r.tool_matrix()
            n = self.axis_B()
            T2 = T.copy()
            T2[:3, 3] += ax["step_m"] * n
            r = self.move_tool_to(T2, ax["step_m"] / ax["speed_m_s"])
            if r is not True:
                return r
            measured = float((self.r.tool_matrix()[:3, 3] - T[:3, 3]) @ n)
            why = check_axial(ax["step_m"], measured, total, ax)
            total += max(measured, 0.0)
            self.travel = total
            if self.samples and self.samples[-1][1] == self.c["verification"]["success_value"]:
                return "OK"
            if why == "NO_PROGRESS":
                return why
            if why == "MAX_TRAVEL":
                return "OK"

    def verify_seated(self):
        v = self.c["verification"]
        self.r.spin_until(lambda: False, v["stable_dwell_s"] + v["stale_s"])
        self.samples[:] = [s for s in self.samples if self.now() - s[0] < 10.0]
        return seat_state(self.samples, self.now(), v)

    def retract(self):
        T = self.r.tool_matrix()
        T[:3, 3] -= (self.travel + self.c["motion"]["retract_m"]) * self.axis_B()
        self.travel = 0.0
        return self.move_tool_to(T, 1.0)

    def apply_search_offset(self, cand):
        dx, dy, dyaw = cand
        R_CS = np.array(self.c["ibvs"]["socket_axes_in_C"])
        d = np.eye(4)
        d[:3, :3] = Rot.from_rotvec(R_CS[:, 2] * dyaw).as_matrix()
        d[:3, 3] = R_CS @ [dx, dy, 0.0]
        T_BC = self.r.tool_matrix() @ self.T_EC @ d      # same pivot (camera origin) as the servo yaw
        return self.move_tool_to(T_BC @ np.linalg.inv(self.T_EC))

    def abort(self, reason):
        self.zero()
        self.r.stop()
        print("ABORT:", reason)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=["servo", "cycle"])
    ap.add_argument("mode", nargs="?", choices=["B0", "A1", "A2"])
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--config", default=str(HERE.parent / "config" / "ibvs.yaml"))
    ap.add_argument("--out", default=str(HERE.parent / "runs" / time.strftime("%Y%m%d_%H%M%S")))
    a = ap.parse_args()
    if a.what == "cycle" and not (a.live and a.mode):
        sys.exit("cycle needs a mode (B0|A1|A2) and --live")
    cfg = load_config(a.config, contact=a.what == "cycle")
    out = pathlib.Path(a.out)
    out.mkdir(parents=True)
    rclpy.init()
    log = TrialLogger(out / "trial.csv", run_id=out.name, config_hash=cfg["config_hash"], controller=a.mode or "servo")
    node = Ibvs(cfg, a.live, log)
    if not node.r.wait_ready(15.0):
        sys.exit("arm not ready")
    try:
        if a.what == "servo":
            print("servo:", node.servo_align())
        else:
            print("cycle:", run_cycle(node, a.mode, candidates(cfg["search"]), log, cfg["search"]["max_total_time_s"], node.now))
    finally:
        node.zero()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
