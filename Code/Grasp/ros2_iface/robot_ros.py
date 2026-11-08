"""ROS 2 hardware layer: joint states, TF, trajectories, MoveIt IK, gripper and camera topics."""
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros

BASE, TOOL, GROUP = 'base_link', 'tool_frame', 'arm'
JOINTS = [f'joint_{i}' for i in range(1, 7)]
ARM_ACTION = '/joint_trajectory_controller/follow_joint_trajectory'
GRIPPER_ACTION = '/gen3_lite_2f_gripper_controller/gripper_cmd'
COLOR, DEPTH, INFO = '/camera/camera/color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw', '/camera/camera/aligned_depth_to_color/camera_info'


def matrix(p, q):
    T = np.eye(4); T[:3, :3] = R.from_quat(q).as_matrix(); T[:3, 3] = p
    return T


class Robot(Node):
    def __init__(self, name='grasp_robot', background=False):
        """background=True: own thread keeps joints/TF/camera fresh (use inside a long-running node)."""
        super().__init__(name)
        self._ex = SingleThreadedExecutor()
        self._ex.add_node(self)
        self._bg = background
        self.q = None
        self.grip_pos_msr = None
        self.create_subscription(JointState, '/joint_states', self._js, 10)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)
        self.arm = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.grip = ActionClient(self, GripperCommand, GRIPPER_ACTION)
        self.ik_srv = self.create_client(GetPositionIK, '/compute_ik')
        self.cart_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self._img = {}
        for name, topic, typ in (('color', COLOR, Image), ('depth', DEPTH, Image), ('info', INFO, CameraInfo)):
            self.create_subscription(typ, topic, lambda m, n=name: self._img.__setitem__(n, m), qos_profile_sensor_data)
        if background:
            threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            while rclpy.ok():
                self._ex.spin_once(timeout_sec=0.05)
        except Exception:   # context shut down
            pass

    def _js(self, m):
        d = dict(zip(m.name, m.position))
        if all(j in d for j in JOINTS):
            self.q = np.array([d[j] for j in JOINTS])
            self.grip_pos_msr = d.get('right_finger_bottom_joint')

    def spin_until(self, cond, timeout):
        t0 = time.time()
        while not cond() and time.time() - t0 < timeout:
            if self._bg:
                time.sleep(0.01)
            else:
                self._ex.spin_once(timeout_sec=0.05)
        return cond()

    def wait_ready(self, timeout=15.0):
        ok = self.spin_until(lambda: self.q is not None, timeout)
        ok &= self.arm.wait_for_server(timeout_sec=timeout) and self.grip.wait_for_server(timeout_sec=timeout)
        return ok

    def joints_deg(self):
        self.spin_until(lambda: self.q is not None, 5.0)
        return np.degrees(self.q)

    def tool_matrix(self):
        """4x4 T_base_tool from TF."""
        self.spin_until(lambda: self.tfbuf.can_transform(BASE, TOOL, rclpy.time.Time()), 5.0)
        t = self.tfbuf.lookup_transform(BASE, TOOL, rclpy.time.Time()).transform
        return matrix([t.translation.x, t.translation.y, t.translation.z], [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    def send_joint_path(self, waypoints_rad, times_s, velocities=None, timeout=120.0):
        traj = JointTrajectory(); traj.joint_names = JOINTS
        for i, (q, t) in enumerate(zip(waypoints_rad, times_s)):
            p = JointTrajectoryPoint(); p.positions = [float(v) for v in q]
            if velocities is not None:
                p.velocities = [float(v) for v in velocities[i]]
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); traj.points.append(p)
        goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
        fut = self.arm.send_goal_async(goal)
        self.spin_until(fut.done, 10.0)
        gh = fut.result()
        if gh is None:
            raise RuntimeError('arm trajectory: no answer from the controller (DDS mismatch? every ROS terminal needs RMW_IMPLEMENTATION=rmw_cyclonedds_cpp)')
        if not gh.accepted:
            raise RuntimeError('arm trajectory rejected by the controller')
        self._goal = gh
        res = gh.get_result_async()
        if not self.spin_until(res.done, timeout):
            gh.cancel_goal_async()
            raise RuntimeError('arm trajectory timed out')
        if res.result().result.error_code != 0:
            raise RuntimeError(f'arm trajectory failed: {res.result().result.error_string}')

    def move_joints(self, q_deg, duration):
        """Joint-space move from the current joints to q_deg in `duration` s."""
        q0, q1 = np.array(self.q), np.radians(q_deg)
        n = max(8, int(duration * 8))
        s = np.linspace(0.0, 1.0, n + 1)
        pos = 10 * s**3 - 15 * s**4 + 6 * s**5
        vel = (30 * s**2 - 60 * s**3 + 30 * s**4) / duration
        self.send_joint_path([q0 + (q1 - q0) * p for p in pos], [0.05 + duration * x for x in s], [(q1 - q0) * v for v in vel])

    def ik(self, T, seed_deg=None, timeout=0.5, tries=3):
        """Joint angles (deg) for the tool pose T (4x4, base frame), or None. MoveIt's solver is randomized, so it is retried."""
        for _ in range(tries):
            req = GetPositionIK.Request()
            r = req.ik_request
            r.group_name, r.ik_link_name, r.avoid_collisions = GROUP, TOOL, False
            r.timeout = Duration(sec=0, nanosec=int(timeout * 1e9))
            r.pose_stamped = self._pose(T)
            st = RobotState()
            st.joint_state.name = JOINTS
            st.joint_state.position = [float(v) for v in (self.q if seed_deg is None else np.radians(seed_deg))]
            r.robot_state = st
            fut = self.ik_srv.call_async(req)
            self.spin_until(fut.done, 5.0)
            res = fut.result()
            if res is not None and res.error_code.val == MoveItErrorCodes.SUCCESS:
                d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.degrees([d[j] for j in JOINTS])
        return None

    def _pose(self, T):
        ps = PoseStamped(); ps.header.frame_id = BASE
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = [float(v) for v in T[:3, 3]]
        q = R.from_matrix(T[:3, :3]).as_quat()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = [float(v) for v in q]
        return ps

    def line_path(self, T_goal, q_start_rad=None):
        """Straight tool line from q_start (default: now) to T_goal, no motion. Returns joint waypoints (rad) or None."""
        req = GetCartesianPath.Request()
        req.group_name, req.link_name, req.max_step, req.avoid_collisions = GROUP, TOOL, 0.005, False
        req.header.frame_id = BASE
        st = RobotState(); st.joint_state.name = JOINTS
        st.joint_state.position = [float(v) for v in (self.q if q_start_rad is None else q_start_rad)]
        req.start_state = st
        req.waypoints = [self._pose(T_goal).pose]
        fut = self.cart_srv.call_async(req); self.spin_until(fut.done, 10.0)
        res = fut.result()
        if res is None or res.fraction < 0.99:
            return None
        return [list(p.positions) for p in res.solution.joint_trajectory.points]

    def send_line(self, pts, duration):
        """Run waypoints from line_path() at constant speed over `duration` s."""
        n = len(pts)
        self.send_joint_path(pts, [0.01 + duration * i / max(1, n - 1) for i in range(n)])

    def move_line(self, T_goal, duration):
        """Straight-line tool move to T_goal."""
        pts = self.line_path(T_goal)
        if pts is None:
            raise RuntimeError('straight-line path not feasible')
        self.send_line(pts, duration)

    def stop(self):
        g = getattr(self, '_goal', None)
        if g is not None:
            g.cancel_goal_async()
            self.spin_until(lambda: False, 0.3)

    def gripper(self, position, effort=20.0, timeout=10.0):
        """0.0 open ... 0.8 closed (GripperCommand)."""
        goal = GripperCommand.Goal(); goal.command.position = float(position); goal.command.max_effort = float(effort)
        fut = self.grip.send_goal_async(goal); self.spin_until(fut.done, 5.0)
        res = fut.result().get_result_async(); self.spin_until(res.done, timeout)
        return res.result().result

    def grab(self, timeout=5.0):
        """(color BGR uint8, depth metres float32, K 3x3) from the latest camera messages."""
        for k in ('color', 'depth'):
            self._img.pop(k, None)
        if not self.spin_until(lambda: len(self._img) == 3, timeout):
            raise RuntimeError('no camera frames: is realsense2_camera running with align_depth.enable:=true ?')
        c, d, i = self._img['color'], self._img['depth'], self._img['info']
        color = np.frombuffer(c.data, np.uint8).reshape(c.height, c.width, -1)
        if c.encoding == 'rgb8':
            color = color[:, :, ::-1]
        depth = np.frombuffer(d.data, np.uint16).reshape(d.height, d.width).astype(np.float32) * 1e-3
        return np.ascontiguousarray(color), depth, np.array(i.k).reshape(3, 3)
