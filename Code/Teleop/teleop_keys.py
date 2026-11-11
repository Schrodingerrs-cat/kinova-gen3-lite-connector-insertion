#!/usr/bin/env python3
"""Jog the arm with the keyboard (terminal must have focus). python3 teleop_keys.py [--dry]   (or ./run.sh)

Self-contained: own arm.py / robot_ros.py / teleop.yaml. IK near the current joints, NO limits (no table floor, no joint-jump limit). --dry = check and print, never move.
Do not run it while another program (grasp node, calibration) is moving the arm.
"""
import argparse
import json
import os
import select
import sys
import termios
import time
import tty

import numpy as np
import rclpy
import yaml
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from arm import Arm  # noqa: E402
from robot_ros import Robot  # noqa: E402

def to_T(xyz, quat):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rot.from_quat(quat).as_matrix(), xyz
    return T


def wrap180(d):
    return (np.asarray(d, float) + 180.0) % 360.0 - 180.0


HELP = """
 Cartesian target: IK -> joint interpolation (not a straight TCP path).
 Translation keeps orientation fixed; rotations use base axes about the TCP.
 move TCP (base frame)      w/s  +x/-x      a/d  +y/-y      r/f  +z/-z
 tilt TCP in place (deg)    t/g  about y    y/h  about x    q/e  yaw about z
 one joint                  1-6 pick joint, then  [  ]  = -/+ step (deg)
 gripper                    o open a bit     c close a bit (10% of the range per press)
 save pose                  p  append current joint angles to saved_poses.json
 step size                  +  bigger   -  smaller          0 or H  go home (joint move)
 ? shows this help; Ctrl-C quits
"""
STEPS_MM = [1, 2, 5, 10, 20, 50]
GRIP_STEP = 0.1  # each o / c press moves the gripper this fraction of its full range
AXIS = {"t": (1, 1), "g": (1, -1), "y": (0, 1), "h": (0, -1), "q": (2, 1), "e": (2, -1)}
MOVE = {"w": (0, 1), "s": (0, -1), "a": (1, 1), "d": (1, -1), "r": (2, 1), "f": (2, -1)}


POSES = os.path.join(HERE, "saved_poses.json")


def save_pose(arm):
    poses = json.load(open(POSES)) if os.path.exists(POSES) else []
    poses.append(dict(joints_deg=[round(float(v), 4) for v in arm.q_deg()], gripper=round(float(arm.grip_pos()), 3),
                      time=time.strftime("%Y-%m-%d %H:%M:%S")))
    json.dump(poses, open(POSES, "w"), indent=1)
    return len(poses)


def key(timeout=0.2):
    return sys.stdin.read(1) if select.select([sys.stdin], [], [], timeout)[0] else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true")
    dry = ap.parse_args().dry
    cfg = yaml.safe_load(open(os.path.join(HERE, "teleop.yaml")))
    gp, sc = cfg["gripper"], cfg["scan"]
    T_tcp = to_T(gp["tcp_in_tool_xyz_m"], gp["tcp_in_tool_quat_xyzw"])
    rclpy.init()
    arm = Arm(Robot("teleop", background=True), cfg["motion"], T_tcp, -np.inf, gp["close_cmd"])  # no table floor
    if not arm.ready():
        sys.exit("no joint states / controller / IK: driver and move_group up? same ros_env.sh?")
    print(HELP, "DRY RUN: nothing will move" if dry else "Arm WILL move. E-stop in reach. Press ENTER to start.")
    if not dry:
        input()
    fd, old = sys.stdin.fileno(), termios.tcgetattr(sys.stdin)
    step, joint = 2, None
    print("Steps: 5 mm Cartesian translation / 5 deg rotation and joint jog", flush=True)
    try:
        tty.setcbreak(fd)
        while True:
            k = key()
            if k is None:
                continue
            msg = ""
            if k in "+=":
                step = min(step + 1, len(STEPS_MM) - 1)
            elif k in "-_":
                step = max(step - 1, 0)
            elif k in "123456":
                joint = int(k) - 1
                msg = "joint %d selected" % (joint + 1)
            elif k == "?":
                print(HELP, flush=True)
            elif k in "[]" and joint is None:
                msg = "JOINT: select 1-6 first, then [ / ]"
            elif k in "[]":
                q = arm.q_deg()
                q[joint] += (1 if k == "]" else -1) * STEPS_MM[step]  # mm steps reused as degrees
                msg = jog_to(arm, None, q, dry)
            elif k in MOVE or k in AXIS:
                T = arm.tool_T()
                Tc = T @ T_tcp  # TCP pose: moves translate it, tilts turn it about itself
                if k in MOVE:
                    Tc[MOVE[k][0], 3] += MOVE[k][1] * STEPS_MM[step] / 1e3
                else:
                    ax, sgn = AXIS[k]
                    v = np.zeros(3); v[ax] = sgn * np.radians(STEPS_MM[step])
                    Tc[:3, :3] = Rot.from_rotvec(v).as_matrix() @ Tc[:3, :3]
                msg = jog_to(arm, Tc @ np.linalg.inv(T_tcp), None, dry)
            elif k in "oc":
                full = gp["close_cmd"]
                goal = float(np.clip(arm.grip_pos() + (1 if k == "c" else -1) * GRIP_STEP * full, 0.0, full))
                if dry:
                    msg = "gripper would go to %.2f of %.2f" % (goal, full)
                else:
                    pos, contact = arm.grip_to(goal, gp["close_effort"], rate=0.25)
                    msg = "gripper %.2f of %.2f%s" % (pos, full, " (contact)" if contact else "")
            elif k == "p":
                msg = "saved pose #%d -> saved_poses.json" % save_pose(arm)
            elif k in "H0":
                home = wrap180(sc["home_joints_deg"])
                dq = float(np.abs(home - arm.q_deg()).max())
                if not dry:
                    arm.go_joints(home)
                msg = "home (%.0f deg away)%s" % (dq, " dry, not moved" if dry else "")
            p = (arm.tool_T() @ T_tcp)[:3, 3] * 1e3
            print("step %d mm / %d deg | joint %s | TCP base mm [%7.1f %7.1f %7.1f] | %s" %
                  (STEPS_MM[step], STEPS_MM[step], "none" if joint is None else joint + 1, *p, msg), flush=True)
            termios.tcflush(fd, termios.TCIFLUSH)  # drop keys queued during the move
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        arm.stop()
        os._exit(0)


def jog_to(arm, T_tool, q_goal, dry):
    """Go to a tool pose (IK-checked) or to joint angles. Returns a short status."""
    mode = "JOINT" if T_tool is None else "CARTESIAN (IK -> joints)"
    q = q_goal if T_tool is None else arm.ik_near(T_tool, arm.q_deg(), np.inf)
    if q is None:
        return mode + " REFUSED: " + arm.why
    dq = float(np.abs(q - arm.q_deg()).max())
    if dry:
        return mode + " ok (dry), max joint move %.3f deg" % dq
    try:
        moved = arm.go_joints(q, duration=max(0.5, dq / arm.m["joint_speed_deg_s"]))
    except RuntimeError as exc:
        return mode + " EXECUTION FAILED: " + str(exc)
    return mode + (" completed, max joint move %.3f deg" % dq if moved else " no change (already at target)")


if __name__ == "__main__":
    main()
