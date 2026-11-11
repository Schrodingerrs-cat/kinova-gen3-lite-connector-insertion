"""Arm + gripper commands, same way as HW7: no MoveIt execution.

MoveIt only answers questions (/compute_ik, /compute_cartesian_path). The joint waypoints go straight to
/joint_trajectory_controller/follow_joint_trajectory, gripper straight to the gripper action (robot_ros.py).
Reason: the vendor MoveIt config names the controller gen3_lite_joint_trajectory_controller, the driver
runs joint_trajectory_controller, so MoveIt execute never matched (and its timeout killed slow moves).
"""
import time

import numpy as np
from control_msgs.action import GripperCommand

MAX_STEP_RAD = 0.1      # one 5 mm line step may not jump a joint more than this (singularity / flip)
DT = 0.04               # gripper ramp period, s
STALL_GAP = 0.05        # fingers lag the command by more than this (fraction of full close) = contact


def smoothstep(x):
    return x * x * x * (10 - 15 * x + 6 * x * x)


class Arm:
    def __init__(self, robot, motion, T_tool_tcp, floor_z, grip_full):
        self.r, self.m, self.T_tool_tcp, self.floor_z, self.full = robot, motion, T_tool_tcp, floor_z, grip_full

    def ready(self, timeout=10.0):
        ok = self.r.wait_ready(timeout)
        ok = ok and self.r.ik_srv.wait_for_service(timeout) and self.r.cart_srv.wait_for_service(timeout)
        return bool(ok and self.r.spin_until(lambda: self.r.grip_pos_msr is not None, timeout))

    # ---- state ----
    def q_deg(self):
        return np.degrees(self.r.q)

    def tool_T(self):
        return self.r.tool_matrix()

    def tcp_z(self, T_tool):
        return float((T_tool @ self.T_tool_tcp)[2, 3])

    # ---- arm ----
    def ik_near(self, T_tool, seed_deg, max_dev_deg):
        """Joint solution (deg) for the tool pose, or None. seed_deg: one seed or a list; the solution closest to a seed
        wins. Any joint further than max_dev from that seed = rejected (no wrist flip / spin)."""
        if self.tcp_z(T_tool) < self.floor_z:
            return None
        best = None
        for seed in np.atleast_2d(seed_deg):
            q = self.r.ik(T_tool, seed)
            if q is None:
                continue
            dev = np.abs(q - seed).max()
            if dev <= max_dev_deg and (best is None or dev < best[0]):
                best = (dev, q)
        return None if best is None else best[1]

    def go_joints(self, q_deg, duration=None):
        """Joint move, straight interpolation from now to q_deg (what is checked is what runs)."""
        dq = np.abs(np.asarray(q_deg) - self.q_deg()).max()
        if dq < 0.3:
            return
        self.r.move_joints(q_deg, duration or max(2.0, dq / self.m["joint_speed_deg_s"]))

    def go_pose(self, T_tool, max_dev_deg):
        q = self.ik_near(T_tool, self.q_deg(), max_dev_deg)
        if q is None:
            raise RuntimeError("no safe near-current joint solution for the pose %s" % np.round(T_tool[:3, 3], 3))
        self.go_joints(q)

    def plan_chain(self, T_pre, T_grasp, T_lift, max_dev_deg):
        """Check pre-grasp -> straight descent -> straight lift (tool poses, base frame) BEFORE moving. Returns (plan, why)."""
        q_pre = self.ik_near(T_pre, self.q_deg(), max_dev_deg)
        if q_pre is None:
            return None, "pre-grasp: no near-current IK (or below the table)"
        legs = []
        q_from = np.radians(q_pre)
        for name, T0, T1 in (("descent", T_pre, T_grasp), ("lift", T_grasp, T_lift)):
            if self.tcp_z(T1) < self.floor_z:
                return None, "%s ends below the table floor" % name
            pts = self.r.line_path(T1, q_from)
            if pts is None:
                return None, "%s: straight line not feasible" % name
            if np.abs(np.diff(np.array(pts), axis=0)).max() > MAX_STEP_RAD:
                return None, "%s: joint jump along the line" % name
            legs.append((pts, max(2.0, float(np.linalg.norm(T1[:3, 3] - T0[:3, 3])) / self.m["line_speed_mps"])))
            q_from = np.array(pts[-1])
        return dict(q_pre=q_pre, down=legs[0], lift=legs[1]), "ok"

    def approach(self, plan, at_pre_grasp=None):
        """Joint move to the pre-grasp pose, then (optional callback, e.g. set the jaw opening) the straight descent."""
        self.go_joints(plan["q_pre"])
        if at_pre_grasp:
            at_pre_grasp()
        self.r.send_line(*plan["down"])

    def retreat(self, plan):
        self.r.send_line(*plan["lift"])

    def stop(self):
        self.r.stop()

    # ---- gripper (position in rad, 0 open ... `full` closed) ----
    def grip_pos(self):
        p = self.r.grip_pos_msr
        if p is None:
            raise RuntimeError("no gripper joint state")
        return float(p)

    def _cmd(self, pos, effort):
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = float(np.clip(pos, 0.0, self.full)), float(effort)
        self.r.grip.send_goal_async(goal)

    def grip_to(self, target, effort, rate=0.25):
        """Smooth ramp to target (rad). When closing, stop on contact (fingers lag the command). Returns (position, contact)."""
        a, b = self.grip_pos() / self.full, target / self.full
        n = max(2, int(abs(b - a) / (rate * DT)))
        lag = 0
        for i in range(1, n + 1):
            cmd = a + (b - a) * smoothstep(i / n)
            self._cmd(cmd * self.full, effort)
            time.sleep(DT)
            if b > a and i > 0.15 * n:
                lag = lag + 1 if cmd - self.grip_pos() / self.full > STALL_GAP else 0
                if lag >= 3:
                    self._cmd(min(1.0, self.grip_pos() / self.full + 0.04) * self.full, effort)  # small squeeze, then hold
                    time.sleep(0.5)
                    return self.grip_pos(), True
        time.sleep(0.6)
        return self.grip_pos(), False
