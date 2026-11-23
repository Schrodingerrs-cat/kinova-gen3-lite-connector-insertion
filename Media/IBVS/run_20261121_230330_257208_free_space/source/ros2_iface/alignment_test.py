#!/usr/bin/env python3
"""Free-space experiment: startup -> recorded home -> post-grasp pose -> 6-DoF image alignment."""
import argparse
from array import array
import copy
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from scipy.linalg import logm

HERE = Path(__file__).resolve().parent
ROOT, PROJECT = HERE.parent, HERE.parents[2]
GRASP = HERE.parents[1] / "Grasp"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(GRASP / "ros2_iface"))
from ibvs.alignment import camera_step, fit_camera_tool_mapping, marker_plane_depths, path_times, within_limits  # noqa: E402
from ibvs.control import Convergence  # noqa: E402
from ibvs.depth import corner_depths, drop_near_field, observed_voxels, scene_near_depth  # noqa: E402
from ibvs.frames import matrix  # noqa: E402
from ibvs.markers import AutoMarkerTracker, Tracker, TrackingLost, mean_marker_depth  # noqa: E402
from ibvs.overlay import FeatureOverlay  # noqa: E402
from ibvs.recording import TimedVideo  # noqa: E402


def load_inputs(settings_path, reference_path, socket=None, marker_id=None):
    settings = yaml.safe_load(settings_path.read_text())
    for key, value in settings.items():
        if not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            raise ValueError("alignment setting %s must be finite and positive" % key)
    if settings["depth_min_m"] >= settings["depth_max_m"]:
        raise ValueError("invalid depth interval")
    for key in ("max_steps", "converge_frames", "observation_frames"):
        if int(settings[key]) != settings[key]:
            raise ValueError(key + " must be an integer")
    if reference_path.suffix in (".yaml", ".yml"):
        reference = yaml.safe_load(reference_path.read_text())
        if reference.get("feature_type") != "marker":
            raise ValueError("the YAML target must describe an ArUco marker")
        if socket is not None and socket not in reference["sockets"]:
            raise ValueError("--socket must be one of: " + ", ".join(reference["sockets"]))
        forced = marker_id if marker_id is not None else (reference["sockets"][socket] if socket is not None else None)
        w, h = reference["image_size_px"]
        # Nothing is taught: marker and goal are chosen from the live view (AutoMarkerTracker); this square is only a placeholder.
        reference["feature_tracking"] = dict(reference["feature_tracking"], ids=[forced if forced is not None else 0], marker_id=forced)
        reference["corners_image_px"] = [[w/2-30, h/2-30], [w/2+30, h/2-30], [w/2+30, h/2+30], [w/2-30, h/2+30]]
        settings["damping"] = 0.0
        settings["max_servo_joint_step_deg"] = 5.5
        FeatureOverlay(reference)
        settings["tolerance_px"] = float(reference["tolerance_px"])
        settings["converge_frames"] = int(reference["converge_frames"])
        for key in ("max_step_m", "max_step_deg", "max_steps", "timeout_s", "arrival_tolerance_deg", "arrival_timeout_s",
                    "arrival_settle_s", "error_growth_px", "error_growth_frames", "max_joint_increment_deg",
                    "tracking_loss_tolerance_s", "tracking_recovery_timeout_s", "tracking_recovery_frames",
                    "max_tracking_recoveries", "near_goal_hold_s"):
            if key in reference:
                value = reference[key]
                if not isinstance(value, (float,int)) or not np.isfinite(value) or value <= 0:
                    raise ValueError("invalid marker setting: " + key)
                if key in ("max_steps", "error_growth_frames", "tracking_recovery_frames", "max_tracking_recoveries") and int(value) != value:
                    raise ValueError(key + " must be an integer")
                settings[key] = value
        if "joint_gains" in reference:
            gains = np.asarray(reference["joint_gains"],float)
            if gains.shape != (6,) or not np.isfinite(gains).all() or np.any(gains <= 0):
                raise ValueError("six positive joint gains are required")
            settings["joint_gains"] = gains.tolist()
    else:
        if socket is not None or marker_id is not None:
            raise ValueError('--socket/--marker-id need the marker YAML reference')
        reference = json.loads(reference_path.read_text())
        image = reference_path.parent / reference["image_file"]
        if hashlib.sha256(image.read_bytes()).hexdigest() != reference["image_sha256"]:
            raise ValueError("goal image changed; extract its reference corners again")
        reference["target_pixels"] = {int(k): v for k, v in reference["target_pixels"].items()}
        if len(reference["ids"]) != 1:
            raise ValueError("this free-space test expects one four-corner marker")
    grasp = yaml.safe_load((GRASP / "config" / "grasp.yaml").read_text())
    return settings, reference, grasp


class Experiment:
    def __init__(self, args, settings, reference, grasp, output):
        import rclpy
        from control_msgs.action import FollowJointTrajectory
        from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionFK
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image
        from robot_ros import Robot, JOINTS

        class FeedbackRobot(Robot):
            def _js(self, msg):
                super()._js(msg)
                if all(j in msg.name for j in JOINTS):
                    self.latest_joints, self.joint_rx = msg, time.monotonic()

        self.args, self.c, self.reference, self.grasp, self.output = args, settings, reference, grasp, output
        self.rclpy, self.action_type = rclpy, FollowJointTrajectory
        self.children, self.log_files = [], []
        self.goal = self.pending = None
        self.video, self.frames = None, 0
        self.phase, self.last_image_stamp = "STARTUP", None
        self.after_motion_stamp = 0.0
        self.is_marker = reference.get("feature_type") == "marker"
        self.display = FeatureOverlay(reference) if self.is_marker else None
        self.detection = None
        self.last_observation = self.initial_observation = None
        self.last_marker_seen = 0.0
        self.overlay_video = None
        self.correction_video = None
        self.record_correction = False
        self.record_sequence = getattr(args,'record_sequence',False)
        self.tracking_epoch = 0
        self.servo_steps = 0
        self.record_start_stamp = None
        self.poses_completed = []
        self.window = False
        self.origin = self.tracker = None
        self.local_fk = None
        self.r = FeedbackRobot("free_space_ibvs")
        self.r.latest_joints, self.r.joint_rx = None, 0.0
        self.plan_cli = self.r.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.fk_cli = self.r.create_client(GetPositionFK, "/compute_fk")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.camera_prefix = reference.get("camera_topic_prefix", "/camera/camera").rstrip("/")
        self.image_type = Image
        self.overlay_pub = self.r.create_publisher(Image, "/ibvs/feature_image", 1) if self.is_marker else None
        self.r.create_subscription(Image, self.camera_prefix + "/color/image_raw", self.on_color, qos)
        for name, topic, typ in [("depth", self.camera_prefix + "/aligned_depth_to_color/image_raw", Image),
                                  ("info", self.camera_prefix + "/color/camera_info", CameraInfo)]:
            self.r.create_subscription(typ, topic, lambda msg, key=name: self.r._img.__setitem__(key, msg), qos)
        self.r.create_subscription(Image, self.camera_prefix + "/depth/image_rect_raw", lambda msg: None, qos)
        self.T_EC = matrix(grasp["camera"]["hand_eye_xyz_m"], grasp["camera"]["hand_eye_quat_xyzw"])
        self.T_tcp = matrix(grasp["gripper"]["tcp_in_tool_xyz_m"], grasp["gripper"]["tcp_in_tool_quat_xyzw"])
        self.floor = grasp["scan"]["table_z_m"] + settings["table_clearance_m"]
        self.csv_file = (output / "trial.csv").open("w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["timestamp_ns", "phase", "joints_deg", "corners_px", "depths_m", "rms_px", "max_corner_px",
                           "raw_corner_depths_m", "depth_measurement"])

    def pump(self, seconds=0.05):
        self.r._ex.spin_once(timeout_sec=seconds)

    def fresh(self, camera=False):
        if self.r.q is None or not np.isfinite(self.r.q).all():
            raise RuntimeError("no finite arm joint feedback")
        if time.monotonic() - self.r.joint_rx > self.c["feedback_stale_s"]:
            raise RuntimeError("JOINTS_STALE; restart the arm driver after reconnecting USB")
        now = self.r.get_clock().now().nanoseconds * 1e-9
        if not -0.1 <= now - self.stamp(self.r.latest_joints) <= self.c["feedback_stale_s"]:
            raise RuntimeError("joint-state timestamps are stale")
        if camera:
            for key in ("color", "depth"):
                msg = self.r._img.get(key)
                if msg is None or not -0.1 <= now - self.stamp(msg) <= self.c["feedback_stale_s"]:
                    raise RuntimeError(key.upper() + "_STALE")

    def refresh(self, camera=False, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            try:
                self.fresh(camera=camera)
                return
            except RuntimeError:
                pass
        self.fresh(camera=camera)

    def event(self, name, **details):
        with (self.output/'events.jsonl').open('a') as record:
            record.write(json.dumps(dict(event=name, timestamp_ns=time.time_ns(), process_id=os.getpid(), **details))+'\n')

    def hold(self, seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            self.pump()
            self.fresh(camera=True)

    def scene_depth(self, depth, color=None):
        """The gripper and whatever it holds are everything nearer than the scene range (plus a halo): not scene."""
        return drop_near_field(depth, scene_near_depth(self.c, self.T_EC, self.T_tcp))

    def positioning_scene(self):
        from observed_scene import ObservedScene
        from ibvs.depth import observed_voxels

        self.phase, self.visual_stage = 'POSITION_CHECK', 'CHECK POSITIONING CLEARANCE'
        self.refresh(camera=True)
        deadline = time.monotonic()+3
        while abs(self.stamp(self.r._img['color'])-self.stamp(self.r._img['depth'])) > self.c['camera_sync_s']:
            self.pump()
            self.fresh(camera=True)
            if time.monotonic() > deadline:
                raise RuntimeError('no synchronized depth for positioning scene')
        depth = self.r._img['depth']
        if depth.encoding != '16UC1':
            raise RuntimeError('positioning requires aligned 16UC1 depth')
        metres = np.frombuffer(depth.data,np.uint16).reshape(depth.height,depth.step//2)[:,:depth.width]*.001
        T = self.fk(self.r.q)@self.T_EC
        near = scene_near_depth(self.c, self.T_EC, self.T_tcp)       # same rule as the servo scene: gripper and held plug are not scene
        centers = observed_voxels(self.scene_depth(metres),np.asarray(self.camera['K']),np.asarray(self.camera['distortion']),
                                  T,near,self.c['depth_max_m'])
        np.savez_compressed(self.output/'positioning_scene.npz',centers_base_m=centers,camera_in_base=T,
                            depth_m=metres,K=np.asarray(self.camera['K']),distortion=np.asarray(self.camera['distortion']))
        scene = ObservedScene(self)
        scene.identifier += '_positioning'
        try:
            scene.update(centers=centers)
            scene.check(self.r.q)
        except BaseException:
            scene.close()
            raise
        self.event('POSITIONING_SCENE_READY',minimum_depth_m=near,voxels=len(centers))
        return scene

    def wait(self, future, timeout=10.0, monitor=False):
        deadline = time.monotonic() + timeout
        while not future.done():
            self.pump()
            if monitor:
                self.fresh(camera=self.phase in ("ALIGN", "CALIBRATE"))
                if (self.is_marker and self.phase == "ALIGN" and self.detection is not None and not self.detection[2][0]
                        and time.monotonic()-self.last_marker_seen > self.c["tracking_loss_tolerance_s"]):
                    raise TrackingLost("the marker was lost during the step")
            if time.monotonic() > deadline:
                if future is not self.pending:
                    future.cancel()
                raise TimeoutError("ROS request timed out in " + self.phase)
        return future.result()

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    @staticmethod
    def bgr(msg):
        if msg.encoding not in ("rgb8", "bgr8"):
            raise RuntimeError("unsupported color encoding " + msg.encoding)
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return np.ascontiguousarray(image[:, :, ::-1] if msg.encoding == "rgb8" else image)

    def record_frame(self, image, annotated, stamp):
        if not self.args.live:
            return
        if self.record_sequence:
            if self.video is None:
                self.video = TimedVideo(self.output/'camera.mp4')
            self.video.write(image,stamp)
            if annotated is not None:
                if self.overlay_video is None:
                    self.overlay_video = TimedVideo(self.output/'features.mp4',h264=True)
                self.overlay_video.write(annotated,stamp)
        if self.record_correction and annotated is not None:
            if self.correction_video is None:
                self.correction_video = TimedVideo(self.output/'ibvs_correction.mp4',h264=True)
            self.correction_video.write(annotated,stamp)
        self.frames += 1

    def on_color(self, msg):
        self.r._img["color"] = msg
        image = self.bgr(msg)
        annotated = None
        if self.is_marker and self.tracker is not None:
            pixels, valid = np.full((1,4,2), np.nan), np.array([False])
            if self.phase in ("ALIGN", "ADVANCE"):
                pixels, valid = self.tracker.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
                if self.tracker.locked is not None and self.display.marker_id != self.tracker.locked:
                    self.display.set_goal(self.tracker.locked, self.tracker.target_px[0])
                if valid[0]:
                    self.last_marker_seen = time.monotonic()
                else:
                    if self.detection is None or self.detection[2][0]:
                        self.tracking_epoch += 1
                        cv2.imwrite(str(self.output / "lost_marker.png"), image)
                    with (self.output / "tracking_loss.jsonl").open("a") as record:
                        record.write(json.dumps(dict(image_stamp=self.stamp(msg), ids_seen=self.tracker.candidates,
                                                     seconds_since_seen=time.monotonic()-self.last_marker_seen)) + "\n")
            self.detection = (self.stamp(msg), pixels, valid)
            age = max(0., self.r.get_clock().now().nanoseconds*1e-9-self.stamp(msg))
            annotated = self.display.render(image, self.camera["K"], self.camera["distortion"],
                                             self.stamp(msg), age_s=age, observation=(pixels,valid), draw_target=True,
                                             phase=getattr(self, 'visual_stage', self.phase))
            stamp = self.stamp(msg)
            if self.record_start_stamp is None:
                self.record_start_stamp = stamp
            cv2.putText(annotated, 'Time %.1f s | IBVS step %d' % (stamp-self.record_start_stamp, self.servo_steps),
                        (12, msg.height-12), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,180), 1, cv2.LINE_AA)
            out = self.image_type()
            out.header, out.width, out.height = msg.header, msg.width, msg.height
            out.encoding, out.step, out.data = "bgr8", msg.width*3, array("B", annotated.tobytes())
            self.overlay_pub.publish(out)
            self.latest_annotated = annotated
            if getattr(self.args, "show_image", False):
                if not self.window:
                    cv2.namedWindow("IBVS camera", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("IBVS camera", msg.width,msg.height)
                    self.window = True
                cv2.imshow("IBVS camera", annotated)
                if cv2.waitKey(1) & 0xff in (ord('q'), 27):
                    raise KeyboardInterrupt('camera window Q/Esc')
        self.record_frame(image,annotated,self.stamp(msg))

    def start(self, name, command):
        print("Starting " + name, flush=True)
        log = (self.output / (name + ".log")).open("w")
        self.log_files.append(log)
        self.children.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def startup(self):
        from controller_manager_msgs.srv import ListControllers

        # An existing controller with stale feedback must be restarted explicitly.
        cli = self.r.create_client(ListControllers, "/controller_manager/list_controllers")
        if not cli.wait_for_service(timeout_sec=2.0):
            if not self.args.bringup:
                raise RuntimeError("no controller manager; run with --bringup or start the arm driver")
            link = subprocess.run(["bash", str(GRASP / "arm_link.sh")], capture_output=True, text=True)
            print(link.stdout, end="", flush=True)
            if link.returncode:
                raise RuntimeError(link.stdout.strip() or link.stderr.strip())
            self.start("driver", ["ros2", "launch", "kortex_bringup", "gen3_lite.launch.py",
                                  "robot_ip:=192.168.1.10", "use_fake_hardware:=false", "launch_rviz:=false"])
            if not cli.wait_for_service(timeout_sec=35.0):
                raise RuntimeError("driver did not start; inspect driver.log")
        response = self.wait(cli.call_async(ListControllers.Request()), timeout=8)
        active = {controller.name for controller in response.controller if controller.state == "active"}
        if "joint_trajectory_controller" not in active:
            deadline = time.monotonic() + 20
            while "joint_trajectory_controller" not in active and time.monotonic() < deadline:
                self.pump(0.2)
                response = self.wait(cli.call_async(ListControllers.Request()), timeout=5)
                active = {controller.name for controller in response.controller if controller.state == "active"}
            if "joint_trajectory_controller" not in active:
                raise RuntimeError("joint trajectory controller is not active")
        deadline = time.monotonic() + 8
        while (self.r.q is None or time.monotonic() - self.r.joint_rx > self.c["feedback_stale_s"]) and time.monotonic() < deadline:
            self.pump()
        self.fresh()
        if not self.r.arm.wait_for_server(timeout_sec=5):
            raise RuntimeError("arm trajectory action is unavailable")
        if self.r.grip_pos_msr is None:
            raise RuntimeError("no gripper joint state")
        if not self.plan_cli.wait_for_service(timeout_sec=2.0):
            if not self.args.bringup:
                raise RuntimeError("MoveIt is unavailable; run with --bringup or start move_group.launch.py")
            self.start("moveit", ["ros2", "launch", "kinova_gen3_lite_moveit_config", "move_group.launch.py",
                                   "allow_trajectory_execution:=false"])
            if not self.plan_cli.wait_for_service(timeout_sec=35):
                raise RuntimeError("MoveIt did not start; inspect moveit.log")
        if not self.fk_cli.wait_for_service(timeout_sec=5) or not self.r.cart_srv.wait_for_service(timeout_sec=5):
            raise RuntimeError("MoveIt FK or Cartesian path service is unavailable")
        # Wait briefly before starting a camera to avoid claiming an already-open device.
        deadline = time.monotonic() + 2
        while "color" not in self.r._img and time.monotonic() < deadline:
            self.pump()
        if "color" not in self.r._img:
            if not self.args.bringup:
                raise RuntimeError("camera is unavailable; run with --bringup")
            command = ["ros2", "launch", "realsense2_camera", "rs_launch.py", "align_depth.enable:=true"]
            if self.is_marker:
                command += ["camera_name:=eih_cam", "camera_namespace:=/", "tf_prefix:=eih_cam",
                            "depth_module.color_profile:=848x480x30", "depth_module.depth_profile:=848x480x30"]
            self.start("camera", command)
        deadline = time.monotonic() + 20
        while not all(key in self.r._img for key in ("color", "depth", "info")) and time.monotonic() < deadline:
            self.pump()
        self.refresh(camera=True)
        info = self.r._img.get("info")
        if info is None or [info.width, info.height] != self.reference["image_size_px"]:
            raise RuntimeError("camera resolution differs from the reference")
        self.camera = dict(K=np.array(info.k).reshape(3, 3).tolist(), distortion=list(info.d))
        if self.is_marker:
            if info.header.frame_id != self.reference["camera_frame_id"]:
                raise RuntimeError("camera frame differs from the marker configuration")
            self.tracker = AutoMarkerTracker(self.camera, self.reference["feature_tracking"])
        else:
            self.tracker = Tracker(self.camera, {**self.reference, "min_side_px": self.c["min_marker_side_px"],
                                                 "max_jump_px": self.c["max_corner_jump_px"]})
        (self.output / "camera_info.json").write_text(json.dumps(self.camera, indent=2) + "\n")
        self.prepare_local_fk()
        print("Arm, MoveIt and camera are ready.", flush=True)

    def new_marker(self, marker_id, order):
        """Point the tracker at the next marker (marker_id None = choose by `order`): fresh live goal and tracking."""
        self.tracker = AutoMarkerTracker(self.camera, dict(self.reference["feature_tracking"], marker_id=marker_id, marker_order=order))
        self.display.marker_id = -1
        self.detection, self.phase = None, "ALIGN"
        self.tracking_epoch += 1
        self.after_motion_stamp = self.r.get_clock().now().nanoseconds * 1e-9

    def close_gripper(self):
        gripper = self.grasp["gripper"]
        if not self.args.live:
            print("PREVIEW: the gripper would be closed (command %.2f)." % gripper["close_cmd"], flush=True)
            return
        self.r.gripper(gripper["close_cmd"], gripper["close_effort"])
        self.r.spin_until(lambda: False, 1.0)
        position = float(self.r.grip_pos_msr)
        self.event("GRIPPER_CLOSED", position=position, empty_closes_to=gripper["close_cmd"])
        print("Gripper closed: position %.3f (an empty gripper closes to about %.2f)." % (position, gripper["close_cmd"]), flush=True)

    def state(self, q=None):
        from moveit_msgs.msg import RobotState
        from robot_ros import JOINTS

        state = RobotState()
        state.joint_state = copy.deepcopy(self.r.latest_joints)
        state.is_diff = True
        if q is not None:
            positions = list(state.joint_state.position)
            for name, angle in zip(JOINTS, q):
                positions[state.joint_state.name.index(name)] = float(angle)
            state.joint_state.position = positions
        return state

    def fk(self, q):
        if getattr(self,'local_fk',None) is not None:
            return self.local_fk(q)
        from moveit_msgs.srv import GetPositionFK
        from robot_ros import BASE, TOOL

        req = GetPositionFK.Request()
        req.header.frame_id, req.fk_link_names, req.robot_state = BASE, [TOOL], self.state(q)
        response = self.wait(self.fk_cli.call_async(req))
        if response.error_code.val != 1 or not response.pose_stamped:
            raise RuntimeError("MoveIt FK failed")
        pose = response.pose_stamped[0].pose
        return matrix([pose.position.x, pose.position.y, pose.position.z],
                      [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

    def prepare_local_fk(self):
        from rcl_interfaces.srv import GetParameters
        from robot_ros import BASE, TOOL, JOINTS
        from ibvs.kinematics import UrdfChain

        client = self.r.create_client(GetParameters,'/move_group/get_parameters')
        if not client.wait_for_service(timeout_sec=3):
            raise RuntimeError('MoveIt robot model parameter is unavailable')
        request = GetParameters.Request(names=['robot_description'])
        response = self.wait(client.call_async(request))
        description = response.values[0].string_value
        model = UrdfChain(description,BASE,TOOL,JOINTS)
        errors = []
        q = self.r.q.copy()
        for offset in (np.zeros(6),np.array([.003,-.002,.001,-.002,.003,-.001]),np.array([-.002,.001,-.003,.002,-.001,.003])):
            expected, local = self.fk(q+offset), model(q+offset)
            position = float(np.linalg.norm(expected[:3,3]-local[:3,3]))
            angle = float(Rotation.from_matrix(expected[:3,:3].T@local[:3,:3]).magnitude())
            errors.append(dict(position_m=position,rotation_rad=angle))
            if position > 1e-6 or angle > 1e-6:
                raise RuntimeError('local URDF forward kinematics does not match MoveIt')
        (self.output/'robot_description.urdf').write_text(description)
        (self.output/'fk_validation.json').write_text(json.dumps(errors,indent=2)+'\n')
        self.local_fk = model
        print('Local URDF FK verified against MoveIt; collision services remain active.',flush=True)

    def check_path(self, trajectory, local=False):
        from robot_ros import JOINTS

        if not trajectory.points or not all(name in trajectory.joint_names for name in JOINTS):
            raise RuntimeError("MoveIt returned an empty path or missing joints")
        order = [trajectory.joint_names.index(name) for name in JOINTS]
        path = np.vstack([self.r.q.copy(), [[point.positions[i] for i in order] for point in trajectory.points]])
        return self.check_joint_path(path, local)

    def direct_path(self, target_rad, local=True):
        """Straight joint-space move from the current joints; same limits as a planned path, no MoveIt planning call."""
        return self.check_joint_path(np.vstack([self.r.q.copy(), target_rad]), local)

    def check_joint_path(self, path, local=False):
        if not np.isfinite(path).all():
            raise RuntimeError("non-finite planned joints")
        changes = np.degrees(np.abs(path - path[0])).max(axis=0)
        if local and changes.max() > self.c["max_servo_joint_step_deg"]:
            rejected = dict(phase=self.phase, start_deg=np.degrees(path[0]).tolist(),
                            end_deg=np.degrees(path[-1]).tolist(), per_joint_change_deg=changes.tolist())
            with (self.output / "rejected_paths.jsonl").open("a") as record:
                record.write(json.dumps(rejected) + "\n")
            raise RuntimeError("IK requires %.2f deg on joint %d (step limit %.2f deg)" %
                               (changes.max(), int(changes.argmax())+1, self.c["max_servo_joint_step_deg"]))
        for q in path:
            within_limits(self.fk(q), self.T_EC, self.T_tcp, self.origin, self.floor,
                          self.c["max_camera_travel_m"], np.radians(self.c["max_camera_rotation_deg"]))
        return path

    def joint_plan(self, target, local=False):
        from moveit_msgs.msg import Constraints, JointConstraint
        from moveit_msgs.srv import GetMotionPlan
        from robot_ros import GROUP, JOINTS

        req = GetMotionPlan.Request()
        plan = req.motion_plan_request
        plan.group_name, plan.start_state = GROUP, self.state()
        plan.num_planning_attempts, plan.allowed_planning_time = 3, 5.0
        plan.max_velocity_scaling_factor, plan.max_acceleration_scaling_factor = 0.1, 0.1
        goal = Constraints()
        goal.joint_constraints = [JointConstraint(joint_name=name, position=float(angle), tolerance_above=0.001,
                                                  tolerance_below=0.001, weight=1.0) for name, angle in zip(JOINTS, np.radians(target))]
        plan.goal_constraints = [goal]
        result = self.wait(self.plan_cli.call_async(req), timeout=20).motion_plan_response
        if result.error_code.val != 1:
            raise RuntimeError("MoveIt joint plan failed: %d" % result.error_code.val)
        return self.check_path(result.trajectory.joint_trajectory, local=local)

    def cartesian_plan(self, target):
        from moveit_msgs.srv import GetCartesianPath
        from robot_ros import BASE, TOOL, GROUP

        req = GetCartesianPath.Request()
        req.header.frame_id, req.link_name, req.group_name = BASE, TOOL, GROUP
        req.start_state, req.waypoints = self.state(), [self.r._pose(target).pose]
        req.max_step, req.avoid_collisions = 0.001, True
        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor, req.max_acceleration_scaling_factor = 0.1, 0.1
        response = self.wait(self.r.cart_srv.call_async(req), timeout=15)
        if response.error_code.val != 1 or not np.isfinite(response.fraction) or response.fraction < 1 - 1e-6:
            raise RuntimeError("complete collision-checked Cartesian step is unavailable")
        return self.check_path(response.solution.joint_trajectory, local=True)

    def move(self, path, min_duration=0.0):
        from builtin_interfaces.msg import Duration
        from control_msgs.msg import JointTolerance
        from trajectory_msgs.msg import JointTrajectoryPoint
        from robot_ros import JOINTS

        if not self.args.live:
            raise RuntimeError("preview mode cannot send a joint trajectory")
        self.fresh(camera=self.phase in ("ALIGN", "CALIBRATE"))
        if self.is_marker and self.phase == "ALIGN":
            deadline = time.monotonic()+self.c["tracking_loss_tolerance_s"]
            while self.detection is None or not self.detection[2][0]:
                if time.monotonic() > deadline:
                    raise TrackingLost("no current marker; refusing to send an alignment step")
                self.pump()
                self.fresh(camera=True)
        drift = float(np.degrees(np.abs(self.r.q - path[0])).max())
        if drift > 0.5:
            raise RuntimeError("arm moved during planning; refusing the stale plan")
        planned_points=len(path)
        precise = self.is_marker and self.phase == "ALIGN"
        profile='segmented_quintic'
        if precise and self.c.get('smooth_joint_segments',False):
            from ibvs.alignment import straight_joint_path
            if straight_joint_path(path):
                # The complete planned curve has already passed all geometry checks.
                # Two endpoints describe the identical line, without intermediate stops.
                path=path[[0,-1]]
                profile='continuous_quintic'
        from ibvs.alignment import smooth_velocities, trajectory_times
        # Alignment steps stop at their end (quintic). Positioning and advance moves flow through their waypoints (cubic, no stops).
        times=trajectory_times(path,self.c['joint_speed_deg_s'],min_duration,
                               max(.1,1.875*drift/self.c['joint_speed_deg_s']),peak_factor=1.875 if precise else 1.25)
        velocities=np.zeros_like(path) if precise else smooth_velocities(path,times)
        request = self.action_type.Goal()
        request.trajectory.joint_names = JOINTS
        if precise:
            request.goal_tolerance = [JointTolerance(name=j, position=float(np.radians(self.c["arrival_tolerance_deg"])))
                                      for j in JOINTS]
            request.goal_time_tolerance = Duration(sec=int(self.c["arrival_timeout_s"]))
        for q, v, t in zip(path, velocities, times):
            ns = int(round(t * 1e9))
            point = JointTrajectoryPoint(positions=q.tolist(), velocities=v.tolist(),
                                         time_from_start=Duration(sec=ns//1000000000, nanosec=ns%1000000000))
            if precise:
                point.accelerations = [0.0]*6
            request.trajectory.points.append(point)
        self.pending = self.r.arm.send_goal_async(request)
        result_code = None
        try:
            self.goal = self.wait(self.pending, monitor=True)
            if self.goal is None or not self.goal.accepted:
                raise RuntimeError("joint trajectory was rejected")
            result = self.wait(self.goal.get_result_async(), timeout=float(times[-1] + 10), monitor=True)
            result_code = result.result.error_code
            if result.result.error_code != 0:
                raise RuntimeError("joint trajectory failed: " + result.result.error_string)
            finished = time.monotonic()
            if self.is_marker:
                settle_s=self.c["arrival_settle_s"] if precise else self.c.get('position_settle_s',self.c['arrival_settle_s'])
                settle_until = finished + settle_s
                while time.monotonic() < settle_until:
                    self.pump()
                    self.fresh(camera=self.phase in ("ALIGN", "CALIBRATE"))
                finished = time.monotonic()
            deadline = finished + 2
            while self.r.joint_rx <= finished and time.monotonic() < deadline:
                self.pump()
            self.fresh(camera=self.phase in ("ALIGN", "CALIBRATE"))
            error = np.degrees(np.abs(self.r.q - path[-1]))
            tolerance = self.c["arrival_tolerance_deg"] if precise else 1.0
            if self.is_marker:
                self.after_motion_stamp = self.r.get_clock().now().nanoseconds * 1e-9
            if self.r.joint_rx <= finished or error.max() > tolerance:
                raise RuntimeError("fresh joint feedback did not confirm arrival")
        except BaseException:
            self.cancel()
            raise
        finally:
            if precise:
                with (self.output / "motion.jsonl").open("a") as record:
                    record.write(json.dumps(dict(target_joints_deg=np.degrees(path[-1]).tolist(),
                                                 measured_joints_deg=np.degrees(self.r.q).tolist(),
                                                 error_deg=np.degrees(np.abs(self.r.q-path[-1])).tolist(),
                                                 tolerance_deg=self.c["arrival_tolerance_deg"],
                                                 planned_waypoints=planned_points,sent_waypoints=len(path),
                                                 profile=profile,command_duration_s=float(times[-1]),
                                                 movement_duration_s=float(times[-1]-times[0]),
                                                 controller_result=result_code)) + "\n")
            self.pending = None

    def cancel(self):
        def cancel_accepted(future):
            try:
                handle = future.result()
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
            except Exception:
                pass
        if self.pending is not None:
            if self.pending.done():
                cancel_accepted(self.pending)
            else:
                self.pending.add_done_callback(cancel_accepted)
                deadline = time.monotonic() + 2
                while not self.pending.done() and time.monotonic() < deadline:
                    self.pump()
        if self.goal is not None and self.goal.accepted:
            future = self.goal.cancel_goal_async()
            deadline = time.monotonic() + 2
            while not future.done() and time.monotonic() < deadline:
                self.pump()

    def capture(self):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            self.pump()
            self.fresh(camera=True)
            color, depth = self.r._img["color"], self.r._img["depth"]
            stamp = self.stamp(color)
            if (stamp == self.last_image_stamp or stamp <= self.after_motion_stamp
                    or abs(stamp - self.stamp(depth)) > self.c["camera_sync_s"]):
                continue
            self.last_image_stamp = stamp
            image = self.bgr(color)
            if self.initial_observation is None:
                cv2.imwrite(str(self.output / "start_alignment.png"), image)
            if self.is_marker:
                if self.detection is None or self.detection[0] != stamp:
                    continue
                pixels, valid = self.detection[1:]
            else:
                pixels, valid = self.tracker.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
            if not valid.all():
                continue
            if depth.encoding != "16UC1" or (depth.width, depth.height) != (color.width, color.height):
                raise RuntimeError("expected aligned 16UC1 depth at color resolution")
            metres = np.frombuffer(depth.data, np.uint16).reshape(depth.height, depth.step//2)[:, :depth.width] * 0.001
            observation = dict(color=image, depth_m=metres, corners_px=pixels[0], K=np.asarray(self.camera["K"]),
                               distortion=np.asarray(self.camera["distortion"]))
            self.last_observation = observation
            if self.initial_observation is None:
                self.initial_observation = observation
                np.savez_compressed(self.output / "initial_observation.npz", **observation)
            if self.is_marker:
                raw = corner_depths(metres, pixels[0], self.c["depth_min_m"], self.c["depth_max_m"])
                try:
                    Z, support = mean_marker_depth(metres,pixels[0],self.c["depth_min_m"],self.c["depth_max_m"])
                except ValueError as error:
                    raise TrackingLost(str(error)) from error
            else:
                raw = corner_depths(metres, pixels[0], self.c["depth_min_m"], self.c["depth_max_m"])
                Z, support = marker_plane_depths(metres, pixels[0], self.camera["K"], self.camera["distortion"],
                                                 self.c["depth_min_m"], self.c["depth_max_m"], self.c["depth_plane_inlier_m"],
                                                 self.c["min_depth_points"], self.c["min_depth_inlier_fraction"])
            xy = self.tracker.normalize(pixels[0])
            error = np.linalg.norm(pixels[0] - self.tracker.target_px[0], axis=1)
            rms, maximum = float(np.sqrt(np.mean(error**2))), float(error.max())
            self.csv.writerow([time.time_ns(), self.phase, json.dumps(np.degrees(self.r.q).tolist()),
                               json.dumps(pixels[0].tolist()), json.dumps(Z.tolist()), rms, maximum,
                               json.dumps([None if not np.isfinite(z) else float(z) for z in raw]), json.dumps(support)])
            self.csv_file.flush()
            return xy, Z, rms, maximum
        if self.is_marker:
            raise TrackingLost('no valid marker in fresh synchronized camera/depth pairs: %s' % getattr(self.tracker, 'status', ''))
        raise RuntimeError('no stable reference marker in fresh synchronized camera/depth pairs')

    def average_observation(self):
        for _ in range(4):
            observations = [self.capture() for _ in range(int(self.c["observation_frames"]))]
            features = np.array([item[0] for item in observations])
            focal = np.diag(np.asarray(self.camera["K"]))[:2]
            if np.max(np.std(features, axis=0) * focal) <= 0.4:
                return np.median(features, axis=0), np.median([item[1] for item in observations], axis=0)
        raise RuntimeError("marker observations are not stable enough for local motion calibration")

    def probe_to(self, baseline, delta, rotation=False):
        current = self.fk(self.r.q)
        target = baseline.copy()
        if rotation:
            target[:3, :3] = baseline[:3, :3] @ Rotation.from_rotvec(delta).as_matrix()
        else:
            target[:3, 3] += baseline[:3, :3] @ delta
        angle = Rotation.from_matrix(current[:3, :3].T @ target[:3, :3]).magnitude()
        distance = np.linalg.norm(target[:3, 3] - current[:3, 3])
        maximum_fraction = min(1.0, np.radians(0.2) / max(angle, 1e-12),
                               self.c["max_step_m"] / max(distance, 1e-12))
        relative = logm(np.linalg.inv(current) @ target).real
        from scipy.linalg import expm
        progress, moves = 0.0, 0
        while progress < 1 - 1e-10:
            fraction = min(maximum_fraction, 1 - progress)
            for backoff in range(8):
                waypoint = current @ expm(relative * (progress + fraction))
                try:
                    path = self.cartesian_plan(waypoint)
                    break
                except RuntimeError:
                    if backoff == 7:
                        raise
                    fraction *= 0.5
            self.move(path, self.c["min_move_s"])
            progress += fraction
            moves += 1
            if moves > 128:
                raise RuntimeError("probe subdivision limit reached")

    def calibrate_mapping(self):
        self.phase = "CALIBRATE"
        xy, depths = self.average_observation()
        baseline = self.fk(self.r.q)
        twists, changes, records = [], [], []
        for index in range(6):
            for sign in (-1, 1):
                delta = np.zeros(3)
                amount = self.c["calibration_translation_m"] if index < 3 else np.radians(self.c["calibration_rotation_deg"])
                delta[index % 3] = sign * amount
                print("Camera/tool probe: tool axis %d, sign %+d" % (index+1, sign), flush=True)
                self.probe_to(baseline, delta, rotation=index >= 3)
                measured, _ = self.average_observation()
                actual = self.fk(self.r.q)
                generator = logm(np.linalg.inv(baseline) @ actual).real
                twist = np.r_[generator[:3, 3], [generator[2,1], generator[0,2], generator[1,0]]]
                twists.append(twist)
                changes.append((measured - xy).ravel())
                records.append(dict(axis=index, sign=sign, tool_twist=twist.tolist(), corners_normalized=measured.tolist()))
                partial = dict(baseline_tool_pose=baseline.tolist(), baseline_normalized=xy.tolist(),
                               baseline_depths_m=depths.tolist(), probes=records, status="MEASURING")
                (self.output / "calibration_probes.json").write_text(json.dumps(partial, indent=2) + "\n")
                self.probe_to(baseline, np.zeros(3))
        T_EC, validation = fit_camera_tool_mapping(twists, changes, xy, depths)
        result = dict(T_EC=T_EC.tolist(), validation=validation, probes=records, baseline_tool_pose=baseline.tolist(),
                      baseline_normalized=xy.tolist(), baseline_depths_m=depths.tolist(),
                      status="LOCAL_MOTION_FIT", note="Free-space local motion validation; not an insertion calibration certificate")
        (self.output / "local_hand_eye.json").write_text(json.dumps(result, indent=2) + "\n")
        self.T_EC = T_EC
        self.origin = self.fk(self.r.q) @ self.T_EC
        print("Local camera/tool mapping fitted: residual %.3f; tool-to-camera Euler deg %s; offset m %s" %
              (validation["relative_residual"], np.round(Rotation.from_matrix(T_EC[:3,:3]).as_euler('xyz',degrees=True), 2),
               np.round(T_EC[:3,3], 4)), flush=True)

    def run(self):
        self.poses_completed = []
        self.event('SEQUENCE_START')
        self.startup()
        if getattr(self.args, "close_gripper", False):
            self.close_gripper()
        home = (np.array(self.grasp["scan"]["home_joints_deg"]) + 180) % 360 - 180
        waypoints = [home] + [np.asarray(q, float) for q in self.grasp["post_grasp"]["joint_waypoints_deg"]]
        if not self.args.live:
            print("PREVIEW: home -> post-grasp joint targets:", np.round(waypoints, 4).tolist(), flush=True)
            path = self.joint_plan(home)
            print("Home plan checked: %d joint points. No motion sent." % len(path), flush=True)
            return dict(status="PREVIEW", note="Post-grasp and servo plans require actual feedback after reaching their starting pose")
        if not self.args.from_post_grasp:
            scene = self.positioning_scene() if getattr(self,'is_marker',False) else None
            try:
                for index, q in enumerate(waypoints):
                    self.phase = "HOME" if index == 0 else "POST_GRASP"
                    self.visual_stage = self.phase
                    self.event(self.phase+'_START')
                    print("Moving to %s: %s" % (self.phase, np.round(q, 4)), flush=True)
                    self.move(self.joint_plan(q))
                    self.poses_completed.append(self.phase)
                    self.event(self.phase+'_REACHED',joints_deg=np.degrees(self.r.q).tolist())
                    print("Reached " + self.phase, flush=True)
            finally:
                if scene is not None:
                    scene.close()
        elif np.degrees(np.abs(self.r.q - np.radians(waypoints[-1]))).max() > 2:
            raise RuntimeError("--from-post-grasp requires the recorded post-grasp pose")
        self.origin = self.fk(self.r.q) @ self.T_EC
        self.phase = "ALIGN"
        self.visual_stage = 'ACQUIRE MARKER'
        if getattr(self, 'is_marker', False) and not self.args.position_only:
            from types import SimpleNamespace
            from joint2_priority_servo import servo
            self.visual_stage = 'STATIONARY reach/clearance check'
            result = dict(status='INCOMPLETE')
            servo(self, SimpleNamespace(live=self.args.live, steps=int(self.c['max_steps']), wait_target_s=30.,
                                        advance_m=self.args.advance_m, advance_step_m=self.args.advance_step_m,
                                        advance_speed_mps=self.args.advance_speed_mps, advance_margin_m=self.args.advance_margin_m,
                                        allow_contact=self.args.allow_contact, all_markers=self.args.all_markers,
                                        marker_order=self.args.marker_order), result)
            return result
        xy, Z, rms, maximum = self.capture()
        initial = rms
        print("Feature found: RMS %.2f px, maximum corner error %.2f px; depths %s m" % (rms, maximum, np.round(Z, 3)), flush=True)
        if self.args.position_only:
            return dict(status="POSITIONED_ONLY", initial_rms_px=initial)
        if self.args.hand_eye:
            calibrated = json.loads(self.args.hand_eye.read_text())
            self.T_EC = np.asarray(calibrated["T_EC"], float)
            if self.T_EC.shape != (4,4) or not np.isfinite(self.T_EC).all() or calibrated.get("status") != "LOCAL_MOTION_FIT":
                raise ValueError("invalid local hand-eye record")
            self.origin = self.fk(self.r.q) @ self.T_EC
            (self.output / "local_hand_eye.json").write_text(json.dumps(calibrated, indent=2) + "\n")
        else:
            self.calibrate_mapping()
        if self.args.calibration_only:
            return dict(status="LOCAL_MAPPING_READY", initial_rms_px=initial)
        self.phase = "ALIGN"
        xy, Z, rms, maximum = self.capture()
        initial = rms
        convergence = Convergence(self.c["tolerance_px"], self.c["converge_frames"])
        started = time.monotonic()
        motion_steps = 0
        best_rms, growth = rms, 0
        for step in range(int(self.c["max_steps"])):
            if convergence.update(maximum):
                cv2.imwrite(str(self.output / "converged.png"), self.bgr(self.r._img["color"]))
                return dict(status="CONVERGED" if motion_steps else "ALREADY_ALIGNED", initial_rms_px=initial,
                            final_rms_px=rms, max_corner_error_px=maximum, steps=motion_steps)
            if time.monotonic() - started > self.c["timeout_s"]:
                raise RuntimeError("alignment timeout")
            if maximum <= self.c["tolerance_px"]:
                xy, Z, rms, maximum = self.capture()
                continue
            current = self.fk(self.r.q)
            for backoff in range(5):
                transform, _ = camera_step(xy, self.tracker.star[0], Z, self.c["gain"], self.c["damping"],
                                           self.c["max_step_m"] * 0.5**backoff, np.radians(self.c["max_step_deg"]) * 0.5**backoff)
                target = current @ self.T_EC @ transform @ np.linalg.inv(self.T_EC)
                try:
                    path = self.cartesian_plan(target)
                    break
                except RuntimeError:
                    if backoff == 4:
                        raise
            distance = np.linalg.norm(transform[:3, 3])
            angle = Rotation.from_matrix(transform[:3, :3]).magnitude()
            duration = max(self.c["min_move_s"], 1.875*distance/self.c["linear_speed_m_s"],
                           1.875*np.degrees(angle)/self.c["angular_speed_deg_s"])
            old_rms = rms
            self.move(path, duration)
            motion_steps += 1
            xy, Z, rms, maximum = self.capture()
            print("Step %d: RMS %.2f px, max %.2f px" % (step+1, rms, maximum), flush=True)
            growth = growth + 1 if rms > old_rms + 0.15 else 0
            best_rms = min(best_rms, rms)
            growth_limit = self.c.get("error_growth_frames", 3)
            growth_margin = self.c.get("error_growth_px", 1.0)
            if growth >= growth_limit or rms > best_rms + max(growth_margin, best_rms*0.02):
                raise RuntimeError("image error increased after a bounded step; validate hand-eye/model before continuing")
        raise RuntimeError("alignment step limit reached")

    def close(self):
        import signal

        self.cancel()
        if self.video is not None:
            self.video.release()
        if self.overlay_video is not None:
            self.overlay_video.release()
        if hasattr(self, "latest_annotated"):
            cv2.imwrite(str(self.output / "latest_features.png"), self.latest_annotated)
        if self.last_observation is not None:
            np.savez_compressed(self.output / "last_observation.npz", **self.last_observation)
        if self.window:
            cv2.destroyAllWindows()
        self.csv_file.close()
        self.r.destroy_node()
        if self.correction_video is not None:
            self.correction_video.release()
        if self.args.shutdown:
            for process in reversed(self.children):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=8)
                    except subprocess.TimeoutExpired:
                        process.terminate()
        for log in self.log_files:
            log.close()


def scale_speeds(settings, scale):
    """Multiply the software speed limits; shorter minimum move and settling so the faster moves are not eaten by waiting."""
    for key in ("joint_speed_deg_s", "linear_speed_m_s", "angular_speed_deg_s"):
        settings[key] *= scale
    for key, floor in (("min_move_s", .2), ("arrival_settle_s", .15)):
        settings[key] = max(floor, settings[key] / scale) if scale > 1 else settings[key]
    for key in ("max_step_m", "max_step_deg"):          # a lateral move of several cm would otherwise take dozens of 2 mm steps
        if key in settings:
            settings[key] *= scale
    for key, default, cap in (("approach_limits_deg", [1., 5., 1., 1., 1., 1.], [2.5, 5., 2.5, 2.5, 2.5, 2.5]),
                              ("fine_limits_deg", [.25, 1., .25, .25, .25, .25], [.6, 1.5, .6, .6, .6, .6])):
        settings[key] = np.minimum(np.array(settings.get(key, default)) * scale, cap).tolist()


def add_advance_arguments(parser):
    parser.add_argument("--speed-scale", type=float, default=1., help="multiply joint, linear and angular speed limits (1..3)")
    parser.add_argument("--advance-m", type=float, default=0., help="after convergence, move this far forward along the measured wall normal (0 = off)")
    parser.add_argument("--advance-step-m", type=float, default=.03, help="chunk length of the forward move; the arm must follow each chunk or the move stops")
    parser.add_argument("--advance-speed-mps", type=float, default=.01, help="forward speed")
    parser.add_argument("--advance-margin-m", type=float, default=.015, help="stay this far from the wall unless --allow-contact")
    parser.add_argument("--allow-contact", action="store_true", help="do the full forward distance even if the gripper/plug reaches the wall (insertion stroke)")
    parser.add_argument("--all-markers", action="store_true", help="visit every marker in view, left to right: align, go forward, back out, next")
    parser.add_argument("--marker-order", choices=("left", "nearest"), default="left", help="which marker comes first in --all-markers mode")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="execute joint positioning and image-based alignment")
    parser.add_argument("--close-gripper", action="store_true", help="close the gripper before moving")
    add_advance_arguments(parser)
    parser.add_argument("--bringup", action="store_true", help="start missing driver, MoveIt and camera processes")
    parser.add_argument("--shutdown", action="store_true", help="stop ROS processes started by this run when it finishes")
    parser.add_argument("--position-only", action="store_true", help="stop after positioning and checking marker/depth")
    parser.add_argument("--calibration-only", action="store_true", help="stop after measuring and checking the local camera/tool mapping")
    parser.add_argument("--from-post-grasp", action="store_true", help="start alignment at the already reached post-grasp pose")
    parser.add_argument("--show-image", action="store_true", help="open the camera pane with the marker corners used by IBVS; Q stops")
    parser.add_argument('--record-sequence',action='store_true',help='also record positioning; default records only IBVS')
    parser.add_argument("--settings", type=Path, default=ROOT / "config" / "alignment.yaml")
    parser.add_argument("--reference", type=Path, default=ROOT / "config" / "marker_goal.yaml")
    parser.add_argument('--socket', help='optional: force the marker of this socket (usb_a, audio, ac) instead of choosing live')
    parser.add_argument('--marker-id', type=int, help='optional: force this ArUco id instead of choosing live')
    parser.add_argument("--hand-eye", type=Path, help="reuse a local_hand_eye.json from this unchanged camera mount; default: measure local mapping")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if args.show_image:
        for name in ("GTK_PATH", "GTK_EXE_PREFIX", "GTK_IM_MODULE_FILE", "GIO_MODULE_DIR", "GIO_EXTRA_MODULES",
                     "GDK_PIXBUF_MODULEDIR", "GDK_PIXBUF_MODULE_FILE", "GTK_MODULES"):
            os.environ.pop(name, None)
        os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
    if args.position_only and args.calibration_only:
        parser.error("choose either --position-only or --calibration-only")
    if not (0. <= args.advance_m <= .30 and 0. < args.advance_step_m <= .05 and 0. < args.advance_speed_mps <= .02 and args.advance_margin_m >= 0.):
        parser.error("advance: distance 0..0.30 m, step 0..0.05 m, speed 0..0.02 m/s, margin >= 0")
    if not 1. <= args.speed_scale <= 3.:
        parser.error("--speed-scale must be between 1 and 3")
    settings, reference, grasp = load_inputs(args.settings, args.reference, args.socket, args.marker_id)
    if args.all_markers and (args.socket or args.marker_id is not None):
        parser.error("--all-markers visits every marker: do not force one with --socket/--marker-id")
    scale_speeds(settings, args.speed_scale)
    args.advance_speed_mps = min(.03, args.advance_speed_mps * args.speed_scale)
    marker = reference.get("feature_type") == "marker"
    if marker and (args.calibration_only or args.hand_eye):
        parser.error("marker mode uses standard IBVS and the existing transform; calibration probes/fitted mappings are disabled")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
    output = args.out or PROJECT / "Media" / "IBVS" / ("run_" + stamp + "_free_space")
    output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault("ROS_LOG_DIR", str(output / "ros_logs"))
    report = dict(status="INCOMPLETE", started_utc=stamp, process_id=os.getpid(), live=args.live, from_post_grasp=args.from_post_grasp,
                  recording_scope='full sequence and IBVS' if args.record_sequence else 'IBVS only',
                  settings=settings, reference=reference,
                  home_deg=grasp["scan"]["home_joints_deg"], post_grasp=grasp["post_grasp"],
                  hand_eye=grasp["camera"], hand_eye_status="existing estimate; bounded free-space test only",
                  method=("Undamped center/size IBVS then four-corner IBVS; weighted joint mobility; measured mean depth"
                          if marker else "6-DoF image-error control with measured depth and collision-checked position steps"))
    (output / "config.json").write_text(json.dumps(report, indent=2) + "\n")
    for relative in ["ros2_iface/alignment_test.py", "ibvs/alignment.py", "ibvs/markers.py", "ibvs/control.py",
                     "ibvs/depth.py", "ibvs/frames.py", "ibvs/overlay.py", "ibvs/shape_control.py",
                     "ros2_iface/joint2_priority_servo.py", "ros2_iface/observed_scene.py",
                     "ibvs/recording.py", "ibvs/kinematics.py",
                     "config/alignment.yaml", "config/marker_goal.yaml"]:
        snapshot = output / "source" / relative
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT / relative, snapshot)
    shutil.copyfile(args.reference, output/'source'/('selected_reference'+args.reference.suffix))
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    experiment = None
    def interrupt(number, _frame):
        raise KeyboardInterrupt(signal.Signals(number).name)
    previous_terminate = signal.signal(signal.SIGTERM, interrupt)
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        signal.signal(signal.SIGINT, interrupt)
        experiment = Experiment(args, settings, reference, grasp, output)
        report.update(experiment.run())
    except KeyboardInterrupt as error:
        report.update(status="INTERRUPTED",interrupt_reason=str(error))
    except Exception as error:
        report.update(status="FAILED", error=str(error))
        print("STOPPED: " + str(error), file=sys.stderr, flush=True)
    finally:
        if experiment is not None:
            report.update({k:v for k,v in getattr(experiment,'servo_report',{}).items() if k!='status'})
            report['poses_completed'] = experiment.poses_completed
            report["started_processes"] = [process.pid for process in experiment.children]
            report["started_processes_left_running"] = not args.shutdown
            report["last_phase"] = experiment.phase
            if experiment.r.q is not None:
                report["final_joints_deg"] = np.degrees(experiment.r.q).tolist()
            try:
                experiment.close()
            except Exception as error:
                report["cleanup_error"] = str(error)
            report["video_frames"] = experiment.frames
            report['correction_video'] = str(output/'ibvs_correction.mp4') if (output/'ibvs_correction.mp4').exists() else None
            report['full_sequence_video'] = str(output/'features.mp4') if (output/'features.mp4').exists() else None
        rclpy.try_shutdown()
        signal.signal(signal.SIGTERM, previous_terminate)
        (output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        print("Run evidence: " + str(output), flush=True)
    return 0 if report["status"] in ("PREVIEW", "POSITIONED_ONLY", "LOCAL_MAPPING_READY", "CONVERGED", "ALREADY_ALIGNED") else 1


if __name__ == "__main__":
    sys.exit(main())
