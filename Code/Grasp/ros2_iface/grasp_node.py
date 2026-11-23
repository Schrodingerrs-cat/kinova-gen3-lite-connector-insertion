#!/usr/bin/env python3
"""ROS 2 node, main pipeline: colour scan -> colour marker -> camera over it -> head grasp (grid search) -> pick.
Motion path = HW7's (arm.py): MoveIt only answers IK / line paths, trajectories go to the controller.

Services (std_srvs/Trigger):
  /grasp/pick          the whole pipeline: scan -> color_target -> view -> head_grasp (or cloud_grasp) -> execute
  /grasp/scan_gripper  once, arm over empty table: sweep the gripper, scan the fingers -> config/gripper_scan.yaml
  /grasp/home          joint move to scan.home_joints_deg
  /grasp/open_gripper  fully open the gripper if it is not (run start check)
  /grasp/scan          7 camera poses over the workspace (colour + depth) -> merged cloud + marker cloud (ICP, voxel)
  /grasp/color_target  merged scan: table RANSAC, above-table clusters, marker clusters -> coarse target (marker centroid; with param connector: the sticker at that connector's at_mm)
  /grasp/view          camera straight over that target at `view_height_m`
  /grasp/head_grasp    DEFAULT. IMAGE MODE: head segmented in the image, only those pixels lifted to a head cloud, grid search of grasp candidates on it, ranked
                       (param grasp_mode:=head is the default of /grasp/pick; the sticker mode is archived in archive/sticker_mode_20261110)
  /grasp/cad_scan      CAD MODE (grasp_mode:=cad): find the sticker, move to 3 close tilted views around it, save frames (param connector)
  /grasp/cad_grasp     CAD MODE: fit the connector's STEP model to the 3 views (panel plane + leave-one-view-out gate), place its candidates, plan
  /grasp/strip_scan    STRIP MODE (grasp_mode:=strip): white strip + stickers from home, pick one cable, 3 close views across the strip (param connector)
  /grasp/strip_grasp   STRIP MODE: rectangle of the strip-given yaw on the plug top, gates, grasp straight down with measured depth
  /grasp/strip_refine  STRIP MODE: hover at the pre-grasp, refit the rectangle from straight above, update the grasp (refuses a big shift)
  /grasp/model_grasp   MODEL MODE (own mode, grasp_mode:=model): ArUco tag on the panel -> plug pose (config/panel_map.yaml) -> candidates of the STEP-model search placed in the base frame
  /grasp/cloud_grasp   CLOUD MODE: marker region only (sticker axes from the image edges, position from the marker 3D points), candidate search + ranking
                       (grasp_mode:=cloud)
  /grasp/allow_grasp   explicit enable; until called, /grasp/execute only checks the motion
  /grasp/execute       pre-grasp -> straight descent -> close -> lift -> held check (finger reading OR wrist camera)
  /grasp/hold_check    wrist camera hold check on its own (valid with an object lifted between the fingertips)
  /grasp/execute_proposal  same motion for a grasp from ANY algorithm published as JSON on /grasp/proposal
With dry_run:=true (default) the services that move check every IK / line with MoveIt but move nothing.
"""
import json
import pathlib
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
import yaml
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

ROOT = pathlib.Path(__file__).resolve().parents[1]
WORK = ROOT.parents[1] / "Media" / "Grasp" / "work"  # scratch output of the node (scans, plan frames); the record of a run is its folder in Media/Grasp
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "ros2_iface"))
from housing_grasp.config import load_config  # noqa: E402
from housing_grasp import cad_fit, cad_place, connectors, strip_config, strip_fit  # noqa: E402
from housing_grasp.strip_report import report_strip_fit  # noqa: E402
from housing_grasp.cad_report import report_cad_fit  # noqa: E402
from housing_grasp.grasp_overlay import overlay as grasp_overlay  # noqa: E402
from housing_grasp.model_mode import detect_tag_pose, load_panel_map, place_candidates, plug_pose, plug_pose_from_sticker, tag_plane_height  # noqa: E402
from housing_grasp.color_marker import wide_face_offset, analyze_scan, marker_points, head_points, housing_footprint, merge_views, sticker_candidates, sticker_rect, grid_grasp_search, oriented_normals, outline_normals, scene_points  # noqa: E402
from housing_grasp.color_marker_report import report_cloud_candidates, report_color_target, report_grid_search, report_model_overlay  # noqa: E402
from housing_grasp.gripper import finger_points, grasp_held, measure_fingers, summarize  # noqa: E402
from housing_grasp.scan import fuse_clouds, view_cloud, view_poses  # noqa: E402
from arm import Arm  # noqa: E402
from robot_ros import Robot  # noqa: E402

BASE, TOOL = "base_link", "tool_frame"
CAM = "/camera/camera"
# the D405 driver only delivers to reliable subscribers here, and aligned depth only while raw depth is subscribed
CAM_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def pose_msg(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = Rot.from_matrix(T[:3, :3]).as_quat()
    return p


def wrap180(d):
    return (np.asarray(d, float) + 180.0) % 360.0 - 180.0


def to_T(xyz, quat):
    T = np.eye(4)
    T[:3, :3] = Rot.from_quat(quat).as_matrix()
    T[:3, 3] = xyz
    return T


class GraspNode(Node):
    def __init__(self):
        super().__init__("grasp_node")
        for name, default in (("config", str(ROOT / "config" / "grasp.yaml")), ("dry_run", True),
                              ("max_candidates", 4), ("allow_grasp", False), ("grasp_mode", "head"),
                              ("max_plan_age_s", 60.0), ("view_height_m", 0.18), ("connector", "")):
            self.declare_parameter(name, default)
        # candidate_search: "config" (use color_marker.candidate_search) | on | off; ROS reads on/off as booleans, so the parameter must accept both types
        self.declare_parameter("candidate_search", "config", ParameterDescriptor(dynamic_typing=True))
        self.cfg = load_config(self.get_parameter("config").value, allow_unscanned=True)  # refuses unmeasured values
        gp, cam = self.cfg["gripper"], self.cfg["camera"]
        self.T_tool_tcp = to_T(gp["tcp_in_tool_xyz_m"], gp["tcp_in_tool_quat_xyzw"])
        self.T_tool_cam = to_T(cam["hand_eye_xyz_m"], cam["hand_eye_quat_xyzw"])
        self._load_gripper_tables()

        cb = ReentrantCallbackGroup()
        self.depth = self.info = self.color = self.last = self.target = self.fused = self.home_T = self.home_q = None
        for topic, typ, attr in ((f"{CAM}/aligned_depth_to_color/image_raw", Image, "depth"),
                                 (f"{CAM}/aligned_depth_to_color/camera_info", CameraInfo, "info"),
                                 (f"{CAM}/color/image_raw", Image, "color")):
            self.create_subscription(typ, topic, lambda m, a=attr: setattr(self, a, m), CAM_QOS, callback_group=cb)
        self.create_subscription(Image, f"{CAM}/depth/image_rect_raw", lambda m: None, CAM_QOS, callback_group=cb)
        self.pub_scene = self.create_publisher(PointCloud2, "/grasp/scene", LATCHED)
        self.pub_cluster = self.create_publisher(PointCloud2, "/grasp/housing_cloud", LATCHED)
        self.pub_marks = self.create_publisher(MarkerArray, "/grasp/markers", LATCHED)
        self.pub_fused = self.create_publisher(PointCloud2, "/grasp/scan_cloud", LATCHED)
        self.pub_marker = self.create_publisher(PointCloud2, "/grasp/marker_cloud", LATCHED)
        self.scan_dir = self.marker_pts = self.last_run = None
        self.tfbuf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tfbuf, self)
        # tcp must stay above the table floor (measured table height minus 5 mm)
        self.arm = Arm(Robot("grasp_robot", background=True), self.cfg["motion"], self.T_tool_tcp,
                       self.cfg["scan"]["table_z_m"] - 0.005, gp["close_cmd"])
        self.proposal = None
        self.create_subscription(String, "/grasp/proposal", self._on_proposal, LATCHED, callback_group=cb)
        for name, fn in (("pick", self.pick), ("scan_gripper", self.scan_gripper), ("scan", self.scan),
                         ("view", self.view), ("execute", self.execute), ("home", self.home),
                         ("execute_proposal", self.execute_proposal), ("color_target", self.color_target),
                         ("allow_grasp", self.allow_grasp), ("hold_check", self.hold_check), ("head_grasp", self.head_grasp), ("cloud_grasp", self.cloud_grasp), ("model_grasp", self.model_grasp), ("open_gripper", self.open_gripper), ("cad_scan", self.cad_scan), ("cad_grasp", self.cad_grasp),
                         ("strip_scan", self.strip_scan), ("strip_grasp", self.strip_grasp), ("strip_refine", self.strip_refine)):
            self.create_service(Trigger, f"/grasp/{name}", self._srv(fn), callback_group=cb)
        self.lock = threading.Lock()
        self.get_logger().info("grasp_node ready (dry_run=%s)" % self.get_parameter("dry_run").value)

    # ---------- inputs ----------
    def _frame(self):
        """Latest aligned depth (m), intrinsics K, camera pose in base T_BC (tool pose from TF x hand-eye)."""
        if self.depth is None or self.info is None:
            raise RuntimeError("no depth/camera_info yet (is realsense2_camera running with align_depth?)")
        m = self.depth
        depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 1e-3
        t = self.tfbuf.lookup_transform(BASE, TOOL, m.header.stamp, timeout=rclpy.duration.Duration(seconds=0.5)).transform
        T_BT = to_T([t.translation.x, t.translation.y, t.translation.z],
                    [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])
        return depth, np.array(self.info.k).reshape(3, 3), T_BT, T_BT @ self.T_tool_cam

    def _close_frame(self):
        """Close-view frame for the grasp: depth = median of scan.close_frames fresh frames (zeros ignored; the arm is still, and the glossy sticker leaves
        holes in a single frame), K, tool pose and camera pose at the last frame."""
        n = self.cfg["scan"]["close_frames"]
        frames, stamp = self._depth_frames(n)
        depth = np.nanmedian(np.where(np.stack(frames) > 0, np.stack(frames), np.nan), 0)
        T_BT = self._tool_pose(stamp)
        return np.nan_to_num(depth).astype(np.float32), np.array(self.info.k).reshape(3, 3), T_BT, T_BT @ self.T_tool_cam

    def _srv(self, fn):
        def handler(_, resp):
            if not self.lock.acquire(blocking=False):
                resp.success, resp.message = False, "busy"
                return resp
            try:
                resp.message, resp.success = fn(), True
            except Exception as e:  # report to the caller; never leave a motion half-specified
                self.get_logger().error("%s: %s" % (fn.__name__, e))
                resp.success, resp.message = False, str(e)
            finally:
                self.lock.release()
            return resp
        return handler

    def _load_gripper_tables(self):
        tbl = self.cfg["gripper"]["position_to_width_m"]
        self.scanned = tbl is not None
        if self.scanned:
            tbl = np.array(tbl, float)  # columns: gripper position, jaw gap
            self.pos_tab, self.wid_tab = tbl[:, 0], tbl[:, 1]

    def _need_scan(self):
        if not self.scanned:
            raise RuntimeError("finger geometry unknown: run /grasp/scan_gripper first")

    def _depth_frames(self, n):
        """n fresh aligned-depth frames (m) and the tool pose in base at the last one."""
        frames, last = [], None
        t0 = time.time()
        while len(frames) < n:
            m = self.depth
            if m is not None and m.header.stamp != last:
                last = m.header.stamp
                frames.append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(np.float32) * 1e-3)
            elif time.time() - t0 > 10:
                raise RuntimeError("no new depth frames")
            time.sleep(0.05)
        return frames, last

    def _color_bgr(self):
        m = self.color
        if m is None:
            raise RuntimeError("no colour image: is the realsense colour stream on?")
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        return np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img)

    def _tool_pose(self, stamp):
        t = self.tfbuf.lookup_transform(BASE, TOOL, stamp, timeout=rclpy.duration.Duration(seconds=0.5)).transform
        return to_T([t.translation.x, t.translation.y, t.translation.z], [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    # ---------- services ----------
    def scan_gripper(self):
        """Gripper sweep in free air; the wrist camera measures the fingers at each opening."""
        sc, dry = self.cfg["scan"], self.get_parameter("dry_run").value
        if dry:
            return "DRY RUN, gripper not moved. Would sweep positions %s with empty space between the fingers" % sc["gripper_positions"]
        self._need_arm()
        scans = []
        for pos in sc["gripper_positions"]:
            got, _ = self.arm.grip_to(pos, 10.0, rate=0.3)
            time.sleep(0.8)
            frames, _ = self._depth_frames(3)
            K = np.array(self.info.k).reshape(3, 3)
            try:
                m = measure_fingers(np.vstack([finger_points(f, K, self.T_tool_cam) for f in frames]))
            except ValueError as e:
                self.get_logger().warn("gripper position %.2f: %s" % (pos, e))
                continue
            scans.append((got, m))
            self.get_logger().info("position %.3f: gap %.1f mm, pad %.1f x %.1f mm, tip z %.1f mm" % (
                got, m["gap_m"] * 1e3, m["thickness_m"] * 1e3, m["width_m"] * 1e3, m["tip_z_m"] * 1e3))
        self.arm.grip_to(0.0, 10.0, rate=0.3)
        if len(scans) < 3:
            raise RuntimeError("fingers visible at only %d openings; move the camera so the fingertips are in view" % len(scans))
        out = summarize(scans)
        tip = out.pop("tip_z_in_tool_m")
        path = pathlib.Path(self.get_parameter("config").value).parent / "gripper_scan.yaml"
        path.write_text("# written by /grasp/scan_gripper %s from %d openings; tip z in tool frame %.4f m (HW7 touch value %.4f)\n%s" % (
            time.strftime("%Y-%m-%d %H:%M"), len(scans), tip, self.cfg["gripper"]["tcp_in_tool_xyz_m"][2], yaml.safe_dump(out)))
        self.cfg = load_config(self.get_parameter("config").value, allow_unscanned=True)
        self._load_gripper_tables()
        return "fingers %.1f x %.1f mm (visible length %.1f mm), fingertip at tool z %.1f mm (HW7 touch: %.1f mm); saved %s" % (
            out["finger_thickness_m"] * 1e3, out["finger_width_m"] * 1e3, out["finger_length_m"] * 1e3, tip * 1e3,
            self.cfg["gripper"]["tcp_in_tool_xyz_m"][2] * 1e3, path.name)

    def scan(self):
        """Visit the predefined camera poses over the workspace, capture, fuse into one dense cloud."""
        sc, dry = self.cfg["scan"], self.get_parameter("dry_run").value
        self._need_arm()
        _, stamp = self._depth_frames(1)
        T0 = self._tool_pose(stamp)  # tool orientation at scan start is kept for every view
        self.home_T, self.home_q = T0, self.arm.q_deg()
        out = WORK / "scans" / time.strftime("%Y%m%d_%H%M%S")
        views, skipped = [], []
        last = self.home_q
        jv = sc.get("views_joints_deg")  # recorded joint poses: visited as they are, no IK
        poses = view_poses(sc, T0, self.T_tool_cam) if jv is None else [None] * len(jv)
        n_views = len(poses)
        for k, T in enumerate(poses):
            q = (np.array(jv[k], float) + 180) % 360 - 180 if jv is not None else self.arm.ik_near(T, [self.home_q, last], sc["max_joint_dev_deg"])  # else: near home or near the previous view
            if q is None:
                skipped.append(k)
                continue
            bad = self.arm.joint_limit_error(q)
            if bad:
                self.get_logger().warn("scan pose %d skipped, outside the joint limits: %s" % (k + 1, bad))
                skipped.append(k)
                continue
            last = q
            if dry:
                views.append(k)
                continue
            self.arm.go_joints(q)
            time.sleep(0.7)
            frames, stamp = self._depth_frames(sc["frames_per_view"])
            depth = np.nanmedian(np.where(np.stack(frames) > 0, np.stack(frames), np.nan), 0)
            depth = np.nan_to_num(depth).astype(np.float32)
            T_BT = self._tool_pose(stamp)
            out.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(out / ("view_%02d.npz" % k), depth=depth, color=self._color_bgr(), K=np.array(self.info.k).reshape(3, 3), T_BC=T_BT @ self.T_tool_cam, T_BT=T_BT)
            views.append(k)
            self.get_logger().info("scan view %d/%d captured" % (len(views), n_views))
        if dry:
            return "DRY RUN, nothing moved: %d/%d view poses plannable, skipped %s" % (len(views), n_views, skipped)
        self.arm.go_joints(self.home_q)  # back to the start pose
        if len(views) < 2:
            raise RuntimeError("fewer than 2 views captured, skipped %s" % skipped)
        clouds, markers = [], []
        for k in views:
            v = np.load(out / ("view_%02d.npz" % k))
            clouds.append(view_cloud(v["depth"], v["K"], v["T_BC"], self.cfg["cloud"]))
            markers.append(marker_points(v["depth"], v["color"], v["K"], v["T_BC"], self.cfg["color_marker"]))
        fused, info = fuse_clouds(clouds, voxel_m=self.cfg["cloud"]["voxel_m"], use_icp=self.cfg["scan"].get("icp", False))
        marker = merge_views(markers, info, self.cfg["color_marker"]["voxel_m"])  # same ICP corrections as the full cloud
        np.save(out / "fused_cloud.npy", fused.astype(np.float32))
        np.save(out / "marker_cloud.npy", marker.astype(np.float32))
        self.scan_dir, self.marker_pts = out, marker
        self.pub_marker.publish(self._cloud(marker))
        (out / "fusion.json").write_text(json.dumps(dict(views=views, skipped=skipped, per_view=info), indent=1))
        self.fused = fused
        self.pub_fused.publish(self._cloud(fused))
        shifts = [i["shift_mm"] for i in info[1:]]
        return "scan %s: %d views (skipped %s), %d fused points, %d marker points; %s" % (
            out.name, len(views), skipped, len(fused), len(marker),
            "ICP corrections vs earlier views: median %.1f mm, max %.1f mm" % (np.median(shifts), np.max(shifts)) if self.cfg["scan"].get("icp", False) else "no ICP (views stacked as the kinematics place them)")

    def view(self):
        """Camera straight over the colour-marker target set by /grasp/color_target, view_height_m above it (home orientation)."""
        if self.target is None:
            raise RuntimeError("no target: run /grasp/color_target first")
        _, _, T_BT, T_BC = self._frame()
        return self._camera_over(self.target, T_BT, T_BC)

    def _camera_over(self, P, T_BT, T_BC):
        """Camera view_height_m above P, looking straight down (home orientation), via a near-home joint solution."""
        T0 = self.home_T if self.home_T is not None else T_BT
        seed = self.home_q if self.home_q is not None else self.arm.q_deg()
        goal = T0.copy()
        goal[:3, 3] = P + [0, 0, self.get_parameter("view_height_m").value] - T0[:3, :3] @ self.T_tool_cam[:3, 3]
        self._need_arm()
        q = self.arm.ik_near(goal, seed, self.cfg["scan"]["max_joint_dev_deg"])
        if q is None:
            raise RuntimeError("no near-home joint solution for the view over %s" % np.round(P, 3))
        if self.get_parameter("dry_run").value:
            return "DRY RUN, nothing moved. View move is plannable (target point %s)" % np.round(P, 3)
        self.arm.go_joints(q)
        time.sleep(1.0)
        return "camera over %s at %.0f mm" % (np.round(P, 3), self.get_parameter("view_height_m").value * 1e3)

    def pick(self):
        """MAIN PIPELINE: scan (colour) -> merge/ICP + table RANSAC + marker clusters (coarse target) -> camera over it
        -> marker-sticker grasp -> execute. Motion needs dry_run:=false AND /grasp/allow_grasp (otherwise execute only checks)."""
        grasp = dict(head=self.head_grasp, cloud=self.cloud_grasp, model=self.model_grasp, cad=None, strip=None)[self.get_parameter("grasp_mode").value]  # param grasp_mode: sticker (default) | head | cloud
        if self.get_parameter("grasp_mode").value == "cad":
            return " | ".join([self.cad_scan(), self.cad_grasp(), self.execute()])
        if self.get_parameter("grasp_mode").value == "strip":
            return " | ".join([self.strip_scan(), self.strip_grasp(), self.strip_refine(), self.execute()])
        return " | ".join([self.scan(), self.color_target(), self.view(), grasp(), self.execute()])

    def _object_in_hand(self):
        """Wrist camera hold check (HW7 rule, same camera mount), valid after the lift: the held object stays close to the lens
        between the fingertips while the table in the lower corners is farther. Returns (held, depth between the tips, table depth)."""
        frames, _ = self._depth_frames(3)
        depth = np.median(np.stack(frames), 0)

        def med(r0, r1, c0, c1):
            d = depth[r0:r1, c0:c1]
            d = d[(d > 0.05) & (d < 1.5)]
            return float(np.median(d)) if len(d) > 200 else float("nan")
        tips = med(400, 480, 340, 510)
        table = float(np.nanmedian([med(320, 480, 0, 255), med(320, 480, 595, 848)]))
        h = self.cfg["hold_check"]
        return bool(tips < h["tip_max_m"] and table - tips > h["margin_m"]), tips, table

    def hold_check(self):
        seen, tips, table = self._object_in_hand()
        return "%s: depth between the fingertips %.3f m, table in the lower corners %.3f m (rule: tips < %.2f m and table - tips > %.2f m)" % (
            "OBJECT IN HAND" if seen else "nothing between the tips", tips, table, self.cfg["hold_check"]["tip_max_m"], self.cfg["hold_check"]["margin_m"])

    # ---------- colour marker stages ----------
    def color_target(self):
        """Coarse stage on the merged colour scan: table RANSAC, above-table clusters, marker clusters. Sets the view target."""
        d = self.scan_dir or sorted((WORK / "scans").glob("*/marker_cloud.npy"))[-1].parent
        P, G = np.load(d / "fused_cloud.npy").astype(float), np.load(d / "marker_cloud.npy").astype(float)
        name = self.get_parameter("connector").value
        if name:   # keep only the sticker at this connector's predefined spot, so the largest cluster is that cable
            at = connectors.load(name)[1]["at_mm"]
            G = G[np.linalg.norm(G[:, :2] * 1000 - at, axis=1) < connectors.AT_TOL_MM]
        r = analyze_scan(P, G, self.cfg)
        if r["frame"] is None:
            raise RuntimeError("no sticker-sized colour marker above the table in the merged scan %s (marker clusters: %s points; larger than marker_max_extent_m are ignored)" % (d.name, [len(c) for c in r["marker_clusters"]]))
        f = r["frame"]
        report_color_target(d / "plots", P, r)
        (d / "color_analysis.json").write_text(json.dumps(dict(
            scan=d.name, table_normal=r["table_n"], table_d=r["table_d"], table_inliers=r["table_inliers"],
            object_clusters=int(len(set(r["object_labels"]) - {-1})), marker_cluster_points=[len(c) for c in r["marker_clusters"]],
            frame=f), indent=1, default=lambda o: np.asarray(o).tolist()))
        self.target = f["centroid"]  # view() goes over this point
        self.pub_scene.publish(self._cloud(r["object_points"]))
        self.pub_cluster.publish(self._cloud(r["marker_clusters"][0]))
        return "merged scan %s: table tilt %.1f deg (%d inliers), %d above-table clusters, marker clusters %s; target (marker centroid) %s mm. Orientation of the merged patch is NOT used (registration smear)" % (
            d.name, np.degrees(np.arccos(r["table_n"][2])), r["table_inliers"], len(set(r["object_labels"]) - {-1}),
            [len(c) for c in r["marker_clusters"]], np.round(f["centroid"] * 1e3, 0).tolist())

    def _search_on(self):
        """Candidate search switch: node param candidate_search (config | on | off); 'config' = color_marker.candidate_search."""
        p = self.get_parameter("candidate_search").value
        if p == "config":
            return self.cfg["color_marker"]["candidate_search"]
        return p if isinstance(p, bool) else str(p).lower() in ("on", "true", "search")

    def head_grasp(self):
        """IMAGE MODE: the head is segmented in the image (marker + dark pixels connected to the sticker), ONLY those pixels are lifted to 3D with their
        own depth, then a GRID SEARCH runs on that head cloud (jaw centre grid x yaw, pad window width, finger-collision test, scored by outline normals,
        width evenness, centring and contact). Ranked best first; /grasp/execute tries them in order. No on/off switch: the search always runs."""
        depth, K, T_BT, T_BC = self._close_frame()
        color, g, gc, gr = self._color_bgr(), self.cfg["color_marker"], self.cfg["grasp"], self.cfg["gripper"]
        gp = marker_points(depth, color, K, T_BC, g, self.target)   # only the marker blob nearest to the scan target
        if len(gp) < g["min_marker_points"]:
            raise RuntimeError("only %d marker points in the close view: camera over the sticker? (/grasp/view)" % len(gp))
        c = np.median(gp, 0)
        P = head_points(depth, color, K, T_BC, c, g)
        if len(P) < g["min_head_points"]:
            raise RuntimeError("head cloud has only %d points (need %d): no usable depth on the segmented head" % (len(P), g["min_head_points"]))
        N2, S = outline_normals(P), scene_points(depth, color, K, T_BC, c, g)
        jaw_home = (T_BT @ self.T_tool_tcp)[:3, 1]   # jaw axis at the current (home) wrist orientation; wrist joint 6 at home +0 / +-90 deg = jaw at that yaw / across it
        yaw_ref = float(np.arctan2(jaw_home[1], jaw_home[0]) % np.pi)
        wide, az, tilt = wide_face_offset(gp, yaw_ref, g["jaw_yaw_offsets_deg"] or [0, 90])   # from the sticker plane: the sticker is on a wide face
        g = dict(g, wide_face_offset_deg=wide if g["wide_face_offset_deg"] == "auto" else g["wide_face_offset_deg"])
        allc, stats = grid_grasp_search(P, N2, S, gp, c, g, gr["finger_width_m"], gr["finger_thickness_m"], gr["max_open_m"], gc["open_margin_m"], g["grasp_drop_m"], yaw_ref)
        if not allc:
            raise RuntimeError("grid search found no candidate on the head cloud: %s" % stats)
        cands = allc[:g["max_candidates_kept"]]
        grasps = self._grasps_from(cands)
        self.last = dict(ok=True, grasps=grasps, stamp=time.time(), cloud=P, cluster=gp)
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        run.mkdir(parents=True, exist_ok=True)
        self.last_run = run
        (run / "plan.json").write_text(json.dumps(dict(
            ok=True, method="head_image_mode_grid_search", search_stats=stats, wide_face_offset_deg=g["wide_face_offset_deg"], sticker_normal_azimuth_deg=az, sticker_plane_tilt_deg=tilt, jaw_ref_yaw_deg=float(np.degrees(yaw_ref)), sticker_centroid_median_m=c, n_marker_points=len(gp), n_head_points=len(P),
            candidates=cands, grasps=[dict(flip=q["flip"], T_BG=q["T_BG"], cost=q["cost"]) for q in grasps]), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.save(run / "head_points.npy", P.astype(np.float32))
        np.save(run / "marker_close_points.npy", gp.astype(np.float32))
        np.savez_compressed(run / "frame.npz", depth=depth, K=K, T_BC=T_BC, color=color)
        report_grid_search(run / "plots", P, oriented_normals(P), N2, gp, c, allc, stats, gr["finger_width_m"], S, gr["finger_thickness_m"], g["finger_box_h_m"])
        self._publish(self.last)
        b = cands[0]
        return "image mode grid search (wider-face jaw offset %s from the sticker plane: normal %.0f deg, tilt %.0f deg from horizontal): head cloud %d points; %d candidates generated, %d valid; best yaw %.0f deg, %.1f mm from the sticker, TCP (%.1f, %.1f, %.1f) mm, jaw (%.2f, %.2f), width %.1f mm, opening %.1f mm (run %s)" % (
            g["wide_face_offset_deg"], az, tilt, len(P), stats["generated"], stats["valid"], b["yaw_deg"], b["offset_m"] * 1e3, *(b["p"] * 1e3), b["jaw"][0], b["jaw"][1], b["width_m"] * 1e3, b["open_width_m"] * 1e3, run.name)

    # ---------- CAD mode ----------
    def _cad_find(self, spec):
        """Sticker (in the connector's marker colour) nearest the connector's predefined spot at_mm in the workspace. Base-frame point (m), None if no sticker is seen."""
        depth, K, _, T_BC = self._close_frame()
        x0, x1, y0, y1 = np.array(self.cfg["workspace"]["region"]) * 1000
        g = cad_fit.marker_cfg(self.cfg["color_marker"], spec)
        blobs = [b for b in strip_fit.sticker_blobs(dict(depth=depth, color=self._color_bgr(), K=K, T_BC=T_BC), g, g["marker_max_extent_m"] * 1000)
                 if x0 < b["centre"][0] < x1 and y0 < b["centre"][1] < y1]
        if not blobs:
            return None
        d = [np.linalg.norm(b["centre"][:2] - np.array(spec["at_mm"])) for b in blobs]
        if min(d) > connectors.AT_TOL_MM:
            raise RuntimeError("no sticker within %g mm of %s's spot %s mm; seen at %s" % (connectors.AT_TOL_MM, spec["name"], spec["at_mm"], [np.round(b["centre"][:2]).astype(int).tolist() for b in blobs]))
        return blobs[int(np.argmin(d))]["centre"] / 1000

    def _cad_target(self, spec, vc, dry):
        """Sticker position (base frame, m) and how it was found. (None, why) in a dry run that would need the 7-pose scan."""
        tgt, how = self._cad_find(spec), "seen from home"
        if (tgt is None and vc["coarse"] != "never") or (vc["coarse"] == "always" and not dry):
            if dry:
                return None, "sticker not seen from home; a real run would use the 7-pose scan"
            self.scan()
            self.color_target()
            tgt, how = np.asarray(self.target, float), "7-pose scan"
        if tgt is None:
            raise RuntimeError("sticker not seen from home and views.coarse is 'never'")
        return tgt, how

    def _view_plans(self, tgt, specs, tip_clear_mm):
        """[(azimuth deg, joints deg)]: per view the first (azimuth, distance, tilt) alternative whose fingertips stay clear and that has an IK solution."""
        Rh, plans, last = self.home_T[:3, :3] @ self.T_tool_cam[:3, :3], [], self.home_q
        for alts in specs:
            for az, dist, tilt in alts:
                T_BT = cad_fit.close_view_pose(tgt, az, dist, tilt, Rh) @ np.linalg.inv(self.T_tool_cam)
                if (T_BT @ self.T_tool_tcp)[2, 3] - tgt[2] < tip_clear_mm * 1e-3:   # fingertips would hang next to the connector
                    continue
                q = self.arm.ik_near(T_BT, [self.home_q, last], self.cfg["scan"]["max_joint_dev_deg"])
                if q is not None:
                    plans.append((az, q))
                    last = q
                    break
        if len(plans) < len(specs):
            raise RuntimeError("only %d of %d close views are reachable around %s (IK, floor or fingertip clearance %d mm): move the connector closer to the arm's centre "
                               "or change views distance_m / tilt_deg / tip_clear_mm" % (len(plans), len(specs), np.round(tgt, 3), tip_clear_mm))
        return plans

    def _capture_views(self, plans, vc, kind):
        """Move to each view, wait, save the median-depth frame as view_NN.npz in work/<kind>/<time>/, then go home. Returns the folder."""
        out = WORK / kind / time.strftime("%Y%m%d_%H%M%S")
        out.mkdir(parents=True, exist_ok=True)
        for k, (az, q) in enumerate(plans):
            self.arm.go_joints(q)
            time.sleep(vc["settle_s"])
            frames, stamp = self._depth_frames(vc["frames"])
            depth = np.nan_to_num(np.nanmedian(np.where(np.stack(frames) > 0, np.stack(frames), np.nan), 0)).astype(np.float32)
            T_BT = self._tool_pose(stamp)
            np.savez_compressed(out / ("view_%02d.npz" % k), depth=depth, color=self._color_bgr(), K=np.array(self.info.k).reshape(3, 3), T_BC=T_BT @ self.T_tool_cam, T_BT=T_BT)
            self.get_logger().info("%s view %d/%d captured (azimuth %d deg)" % (kind, k + 1, len(plans), az))
        self.arm.go_joints(self.home_q)   # close views done: back to home, the grasp is planned and run from there
        return out

    def cad_scan(self):
        """CAD mode, step 1: from home find the sticker (else the 7-pose scan), then 3 close tilted views around it; the frames are saved."""
        ccfg, spec = cad_fit.load_config(self.get_parameter("connector").value or None)
        vc, dry = ccfg["views"], self.get_parameter("dry_run").value
        self.last = None                                    # a plan of an earlier cable must never be executable after this
        self._need_arm()
        _, stamp = self._depth_frames(1)
        self.home_T, self.home_q = self._tool_pose(stamp), self.arm.q_deg()
        tgt, how = self._cad_target(spec, vc, dry)
        if tgt is None:
            return "DRY RUN: " + how
        plans = self._view_plans(tgt, [[(az0 + dz, vc["distance_m"], vc["tilt_deg"]) for dz in (0, 30, -30, 60, -60)] for az0 in vc["azimuths_deg"][: vc["n"]]], vc["tip_clear_mm"])
        azimuths = [p[0] for p in plans]
        if dry:
            return "DRY RUN, nothing moved: %s sticker %s at %s; %d close views plannable at azimuths %s deg" % (spec["name"], how, np.round(tgt, 3), len(plans), azimuths)
        t0 = time.time()
        out = self._capture_views(plans, vc, "cad")
        (out / "target.json").write_text(json.dumps(dict(connector=spec["name"], target_m=tgt.tolist(), how=how, azimuths_deg=azimuths)))
        self.cad_dir = out
        return "%s sticker %s at %s; %d close views in %.0f s (azimuths %s deg), frames in %s" % (spec["name"], how, np.round(tgt, 3), len(plans), time.time() - t0, azimuths, out.name)

    def _cad_fit(self, spec, ccfg, views, tgt):
        """Crib in every view -> views that see the whole crib are aligned on it (and the others dropped if 2+ are aligned), connector points, panel plane, model fit with its yaw
        locked to the crib. Returns the views actually used with their alignment."""
        fc, obs, stk = ccfg["fit"], [], []
        region = np.array(self.cfg["workspace"]["region"]) * 1000
        poses = [strip_fit.strip_pose(v, strip_config.load()[0]["strip"], region, tgt[:2] * 1000) for v in views]
        crib = strip_fit.strip_summary(poses)
        aligns = [(lambda xy, q=q: strip_fit.align_to_crib(xy, q, crib)) if crib and q and q["complete"] else None for q in poses]
        use = [i for i, a in enumerate(aligns) if a]
        if len(use) >= 2:
            views, aligns = [views[i] for i in use], [aligns[i] for i in use]
        for i, v in enumerate(views):
            pts, st = cad_fit.connector_points(v, spec, self.cfg["color_marker"], tgt)
            if len(pts) < fc["min_points_view"]:
                raise RuntimeError("view %d: only %d connector points (need %d): sticker or housing not segmented" % (i, len(pts), fc["min_points_view"]))
            if aligns[i]:
                pts[:, :2], st[:, :2] = aligns[i](pts[:, :2]), aligns[i](st[:, :2])
            obs.append(pts)
            stk.append(st)
        centre = np.median(np.vstack(stk), 0)
        plane = cad_fit.panel_plane(views, centre, ccfg["panel"], float(np.percentile(np.vstack(obs)[:, 2], 3)))
        M, _ = cad_fit.model_cloud(spec)
        prior = np.radians(crib["yaw"] + spec["yaw_rel_crib_deg"]) if crib and "yaw_rel_crib_deg" in spec else None
        return dict(views=views, aligns=aligns, crib=crib, yaw_prior_deg=None if prior is None else float(np.degrees(prior)), obs=obs, centre=centre, plane=plane, M=M,
                    rep=cad_fit.fit_views(obs, M, spec, plane, ccfg, prior))

    def _cad_save_plan(self, spec, ccfg, d, views, fit, cands, stats):
        """plan.json, last view frame, observed + fitted points and the figures of this plan; returns the folder."""
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        (run / "plots").mkdir(parents=True, exist_ok=True)
        rep, T = fit["rep"], fit["rep"]["T_base_model_mm"]
        ok = bool(cands) and (rep["gate_pass"] or not ccfg["fit"]["gate"]["require"])
        (run / "plan.json").write_text(json.dumps(dict(
            ok=ok, method="cad_fit", connector=spec["name"], sticker_centroid_median_m=fit["centre"] * 1e-3, views_dir=d.name, plane=fit["plane"], fit=rep,
            crib_yaw_deg=None if fit["crib"] is None else fit["crib"]["yaw"], yaw_prior_deg=fit["yaw_prior_deg"], views_used=len(views),
            place_stats=stats, n_candidates=len(cands), candidates=cands), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.savez_compressed(run / "frame.npz", depth=views[-1]["depth"], K=views[-1]["K"], T_BC=views[-1]["T_BC"], color=views[-1]["color"])
        np.savez_compressed(run / "cad_points.npz", observed_mm=np.vstack(fit["obs"]), model_posed_mm=fit["M"] @ T[:3, :3].T + T[:3, 3])
        report_cad_fit(run / "plots", views, fit["obs"], fit["M"], T, fit["plane"], cands, rep)
        grasp_overlay(run / "plan.json")
        return run

    def cad_grasp(self):
        """CAD mode, step 2: fit the STEP model to the 3 views, gate it, place its candidates (panel clearance + finger boxes vs the scene), plan like the other modes."""
        ccfg, spec = cad_fit.load_config(self.get_parameter("connector").value or None)
        self.last = None
        if self.home_T is None:
            raise RuntimeError("home pose unknown in this node session: run /grasp/cad_scan first")
        d = getattr(self, "cad_dir", None) or sorted((WORK / "cad").glob("*/view_00.npz"))[-1].parent
        info = json.loads((d / "target.json").read_text())
        if info.get("connector", spec["name"]) != spec["name"]:
            raise RuntimeError("the saved views are of '%s' but the connector is '%s': run /grasp/cad_scan again" % (info["connector"], spec["name"]))
        fit = self._cad_fit(spec, ccfg, [dict(np.load(p)) for p in sorted(d.glob("view_*.npz"))], np.array(info["target_m"]))
        views, rep, plane, T = fit["views"], fit["rep"], fit["plane"], fit["rep"]["T_base_model_mm"]
        gr, gc, jaw_home = self.cfg["gripper"], self.cfg["grasp"], (self.home_T @ self.T_tool_tcp)[:3, 1]
        cands, stats = cad_place.place(T, cad_place.load_candidates(spec, cad_fit.ROOT), spec, ccfg["place"], plane, cad_place.scene_points(views, fit["centre"], T, fit["M"], aligns=fit["aligns"]),
                                       float(np.arctan2(jaw_home[1], jaw_home[0]) % np.pi), gr["finger_width_m"], gr["finger_thickness_m"], gc["open_margin_m"], gr["max_open_m"])
        run = self._cad_save_plan(spec, ccfg, d, views, fit, cands, stats)
        self.last_run, summary = run, cad_fit.fit_summary(spec, rep, plane)
        if not rep["gate_pass"] and ccfg["fit"]["gate"]["require"]:
            raise RuntimeError("CAD fit rejected, no grasp planned (run %s): %s | %s" % (run.name, "; ".join(rep["gate_failures"]), summary))
        if not cands:
            raise RuntimeError("no CAD candidate survives the panel clearance and finger-box checks %s | %s" % (stats, summary))
        grasps = self._grasps_from(cands)
        self.last = dict(ok=True, grasps=grasps, tries=len(grasps), stamp=time.time(), cloud=np.vstack(fit["obs"]) * 1e-3, cluster=None)
        self._publish(self.last)
        b = cands[0]
        return "%s%s | crib yaw %s, model yaw locked to %s, %d views aligned | %d candidates (of %d), best: jaw yaw %.0f deg, tip %.1f mm above the panel, gap %.1f mm, opening %.1f mm, TCP (%.1f, %.1f, %.1f) mm (run %s)" % (
            summary, "" if rep["gate_pass"] else " | GATE FAILED but not required: " + "; ".join(rep["gate_failures"]),
            "n/a" if fit["crib"] is None else "%.0f deg" % fit["crib"]["yaw"], "nothing (crib not seen)" if fit["yaw_prior_deg"] is None else "%.0f deg" % fit["yaw_prior_deg"], len(views), len(cands), stats["considered"],
            float(np.degrees(np.arctan2(b["jaw"][1], b["jaw"][0])) % 180), b["tip_above_panel_mm"], b["width_m"] * 1e3, b["open_width_m"] * 1e3, *(b["p"] * 1e3), run.name)

    # ---------- strip mode ----------
    def _strip_frames(self, d):
        return [dict(np.load(p)) for p in [d / "home.npz", *sorted(d.glob("view_*.npz"))]]

    def strip_scan(self):
        """STRIP MODE step 1: from home find the white strip and the stickers on it, pick one cable, take the close views across the strip."""
        sc, spec = strip_config.load(self.get_parameter("connector").value or None)
        vc, dry = sc["views"], self.get_parameter("dry_run").value
        self.last = None   # an old plan must never be executable after a new scan
        self._need_arm()
        depth, K, T_BT, T_BC = self._close_frame()
        self.home_T, self.home_q = T_BT, self.arm.q_deg()
        home = dict(depth=depth, color=self._color_bgr(), K=K, T_BC=T_BC, T_BT=T_BT)
        g = cad_fit.marker_cfg(self.cfg["color_marker"], spec)
        x0, x1, y0, y1 = np.array(self.cfg["workspace"]["region"]) * 1000
        seen = [b for b in strip_fit.sticker_blobs(home, g, self.cfg["color_marker"]["marker_max_extent_m"] * 1000) if x0 < b["centre"][0] < x1 and y0 < b["centre"][1] < y1]
        if not seen:
            raise RuntimeError("no sticker seen from home")
        seed = np.mean([b["centre"][:2] for b in seen], 0)   # the stickers sit on the strip: search for it around them
        strip = strip_fit.strip_pose(home, sc["strip"], np.array(self.cfg["workspace"]["region"]) * 1000, seed)
        if strip is None or not strip["complete"]:
            raise RuntimeError("white strip not seen complete from home (%s): check the light, strip.white and that the whole strip is in view" % (
                "nothing white" if strip is None else "size %.0f x %.0f mm, expected %s" % (*strip["size"], sc["strip"]["size_mm"])))
        blobs = strip_fit.find_stickers(home, strip, g, sc["strip"], self.cfg["color_marker"]["marker_max_extent_m"] * 1000)
        if not blobs:
            raise RuntimeError("no sticker on the strip")
        pick = blobs[strip_fit.choose_sticker(blobs, strip, spec["slot_mm"], sc["select"]["slot_tol_mm"])]["centre"]
        stickers = [b["centre"] for b in blobs]
        specs = [[(strip["yaw"] + e["across_deg"], vc["distance_m"] * s, e["tilt_deg"] * t) for s, t in ((1, 1), (1, 0.7), (1.2, 1), (1.2, 0.7))] for e in vc["list"]]
        plans = self._view_plans(pick / 1000, specs, vc["tip_clear_mm"])
        what = "%s: strip yaw %.1f deg, %d stickers (%s mm), cable at %s mm" % (spec["name"], strip["yaw"], len(blobs), ", ".join("%.0fx%.0f" % (b["long"], b["short"]) for b in blobs), np.round(pick[:2]))
        if dry:
            return "DRY RUN, nothing moved: %s; %d close views plannable" % (what, len(plans))
        out = self._capture_views(plans, vc, "strip")
        np.savez_compressed(out / "home.npz", **home)
        (out / "target.json").write_text(json.dumps(dict(connector=spec["name"], sticker_mm=pick.tolist(), stickers_mm=[s.tolist() for s in stickers])))
        self.strip_dir = out
        return "%s; %d close views, frames in %s" % (what, len(plans), out.name)

    def strip_grasp(self):
        """STRIP MODE step 2: strip yaw -> plug yaw, rectangle on the plug top band (views aligned on the crib), grasp straight down. Nothing blocks it; the numbers are in the message."""
        sc, spec = strip_config.load(self.get_parameter("connector").value or None)
        self.last = None
        if self.home_T is None:
            raise RuntimeError("home pose unknown in this node session: run /grasp/strip_scan first")
        d = getattr(self, "strip_dir", None) or sorted((WORK / "strip").glob("*/home.npz"))[-1].parent
        info = json.loads((d / "target.json").read_text())
        if info["connector"] != spec["name"]:
            raise RuntimeError("the saved views are of '%s' but the connector is '%s': run /grasp/strip_scan again" % (info["connector"], spec["name"]))
        frames = self._strip_frames(d)
        sticker = np.array(info["sticker_mm"])
        poses = [strip_fit.strip_pose(f, sc["strip"], np.array(self.cfg["workspace"]["region"]) * 1000, sticker) for f in frames]
        strip = strip_fit.strip_summary(poses)
        if strip is None:
            raise RuntimeError("white strip not seen complete in any saved view")
        obs = [strip_fit.view_points(f, spec, self.cfg["color_marker"], sticker, strip["top_z"]) for f in frames]   # home frame first: it always sees the whole crib
        fit = strip_fit.evaluate(obs, strip, spec, sc, poses)
        run = self._strip_save_plan(spec, d, frames, strip, obs, fit)
        self.last_run = run
        grasps = self._grasps_from([fit["candidate"]])
        self.last = dict(ok=True, grasps=grasps, tries=len(grasps), stamp=time.time(), cloud=np.vstack([o["pts"] for o in obs]) * 1e-3, cluster=None,
                         strip=dict(centre_mm=fit["centre_mm"], top_mm=fit["top_mm"], strip_top_mm=strip["top_z"], yaw_deg=fit["yaw_deg"], tip_z_mm=fit["tip_z_mm"], cand=fit["candidate"]))
        self._publish(self.last)
        return "%s cable at (%.1f, %.1f) mm, rectangle %.1f x %.1f mm (%d views), yaw %.1f deg, top %.1f mm above the strip, jaws at %.0f deg (run %s)" % (
            spec["name"], *fit["centre_mm"], *fit["extent_mm"], fit["views_used"], fit["yaw_deg"], fit["top_mm"] - strip["top_z"], fit["jaw_yaw_deg"], run.name)

    def _strip_save_plan(self, spec, d, frames, strip, obs, fit):
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        (run / "plots").mkdir(parents=True, exist_ok=True)
        keep = ("centre_mm", "extent_mm", "yaw_deg", "jaw_yaw_deg", "top_mm", "tip_z_mm", "views_used", "candidate")
        (run / "plan.json").write_text(json.dumps(dict(ok=True, method="strip_fit", connector=spec["name"], views_dir=d.name, strip=strip, sticker_centroid_median_m=np.median(np.vstack([o["sticker"] for o in obs]), 0) * 1e-3,
                                                       candidates=[fit["candidate"]], fit={k: fit[k] for k in keep}), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.savez_compressed(run / "frame.npz", depth=frames[-1]["depth"], K=frames[-1]["K"], T_BC=frames[-1]["T_BC"], color=frames[-1]["color"])
        np.savez_compressed(run / "cad_points.npz", observed_mm=np.vstack([o["pts"] for o in obs]))
        report_strip_fit(run / "plots", frames, strip, obs, fit, self.cfg["gripper"]["finger_width_m"] * 1000, self.cfg["gripper"]["finger_thickness_m"] * 1000)
        grasp_overlay(run / "plan.json")
        return run

    def strip_refine(self):
        """STRIP MODE step 3: hover at the pre-grasp, refit the rectangle from straight above, and use its centre for the grasp."""
        sc, spec = strip_config.load(self.get_parameter("connector").value or None)
        res = self.last
        if not res or not res["ok"] or "strip" not in res:
            raise RuntimeError("run /grasp/strip_grasp first")
        self._need_arm()
        to_tool, dev = np.linalg.inv(self.T_tool_tcp), self.cfg["grasp"]["max_joint_dev_deg"]
        q = next((q for q in (self.arm.ik_near(g["T_pre"] @ to_tool, [self.arm.q_deg(), self.home_q], dev) for g in res["grasps"]) if q is not None), None)
        if q is None:
            raise RuntimeError("pre-grasp hover pose has no IK solution")
        if self.get_parameter("dry_run").value:
            return "DRY RUN, nothing moved: hover at the pre-grasp is reachable"
        self.arm.go_joints(q)
        time.sleep(sc["views"]["settle_s"])
        depth, K, T_BT, T_BC = self._close_frame()
        plan = res["strip"]
        r = strip_fit.refine(dict(depth=depth, color=self._color_bgr(), K=K, T_BC=T_BC), spec, self.cfg["color_marker"], plan, sc)
        cand = dict(plan["cand"], p=np.r_[r["centre_mm"], plan["tip_z_mm"]] / 1000)
        grasps = self._grasps_from([cand])
        self.last = dict(res, grasps=grasps, tries=len(grasps), stamp=time.time(), strip=dict(plan, centre_mm=r["centre_mm"], cand=cand))
        return "hover refit: centre moved %.1f mm to (%.1f, %.1f), rectangle %.1f x %.1f mm" % (r["shift_mm"], *r["centre_mm"], *r["extent_mm"])

    def model_grasp(self):
        """MODEL MODE (STEP/STL + panel ArUco tag), separate from the head and cloud modes: tag pose in the close view -> plug pose (stored offset, config/panel_map.yaml)
        -> candidates of the offline model search (plug frame, CAD_files/*_grasp_candidates_*.json) placed in the base frame. The sticker, if seen, is only compared."""
        depth, K, _, T_BC = self._frame()
        color, gr, gc, g = self._color_bgr(), self.cfg["gripper"], self.cfg["grasp"], self.cfg["color_marker"]
        pm = load_panel_map(ROOT / "config" / "panel_map.yaml")
        entry = pm["plugs"][pm["plug"]]
        tag = detect_tag_pose(color, K, T_BC, pm["tag"])
        zd = tag_plane_height(depth, K, T_BC, tag["corners_px"])   # panel height from the depth over the tag (more accurate than the tag's scale)
        z_note = "" if zd is None else "; panel height %.1f mm from depth (tag scale: %.1f mm)" % (zd * 1e3, tag["T"][2, 3] * 1e3)
        if zd is not None:
            tag["T"][2, 3] = zd
        plug = plug_pose(tag["T"], entry["T_tag_plug"])
        cands = place_candidates(json.load(open(ROOT.parents[1] / entry["candidates"])), plug, gc["open_margin_m"], gr["max_open_m"], pm["max_candidates"])
        if not cands:
            raise RuntimeError("no model candidate fits the gripper opening")
        note, gp = "", marker_points(depth, color, K, T_BC, g, self.target)
        if len(gp) >= g["min_marker_points"]:   # the sticker only checks the tag-based plug pose
            st = plug_pose_from_sticker(gp, T_BC[:3, 3])
            d, dyaw = float(np.linalg.norm(st["xy"] - plug["xy"])), float(np.degrees(abs(((st["yaw"] - plug["yaw"] + np.pi / 2) % np.pi) - np.pi / 2)))
            note = "; sticker check: plug %.1f mm and %.0f deg from the tag prediction" % (d * 1e3, dyaw)
            if pm["verify_tol_m"] is not None and d > pm["verify_tol_m"]:
                raise RuntimeError("tag and sticker disagree on the plug position (%.1f mm > %.1f mm): wrong tag, moved panel or plug not seated" % (d * 1e3, pm["verify_tol_m"] * 1e3))
        grasps = self._grasps_from(cands)
        box = np.array([[plug["xy"][0], plug["xy"][1], plug["z"] + h] for h in (0.0, 0.02)])
        self.last = dict(ok=True, grasps=grasps, stamp=time.time(), cloud=box, cluster=None)
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        run.mkdir(parents=True, exist_ok=True)
        self.last_run = run
        (run / "plan.json").write_text(json.dumps(dict(
            ok=True, method="model_mode", plug=pm["plug"], tag=dict(id=pm["tag"]["id"], side_px=tag["side_px"], reproj_px=tag["reproj_px"], T_base_tag=tag["T"]),
            plug_pose=plug, sticker_centroid_median_m=np.r_[plug["xy"], plug["z"]], n_candidates=len(cands), candidates=cands,
            grasps=[dict(flip=q["flip"], T_BG=q["T_BG"], cost=q["cost"]) for q in grasps]), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.save(run / "marker_close_points.npy", gp.astype(np.float32))
        np.savez_compressed(run / "frame.npz", depth=depth, K=K, T_BC=T_BC, color=color)
        report_model_overlay(run / "plots", color, K, T_BC, tag, plug, cands)
        self._publish(self.last)
        b = cands[0]
        return "model mode (%s, tag %d px wide, reprojection %.2f px): plug (%.1f, %.1f) mm yaw %.1f deg; %d candidates; best yaw %.0f deg, tip %.0f mm above the panel, gap %.1f mm, opening %.1f mm, TCP (%.1f, %.1f, %.1f) mm%s %s (run %s)" % (
            pm["plug"], tag["side_px"], tag["reproj_px"], *(plug["xy"] * 1e3), np.degrees(plug["yaw"]), len(cands), b["yaw_deg"], b["u0_mm"], b["width_m"] * 1e3, b["open_width_m"] * 1e3, *(b["p"] * 1e3), note, z_note, run.name)

    def cloud_grasp(self):
        """CLOUD MODE (marker region only, no black body): the marker pixels are the region of interest. Sticker position and height from the marker 3D points
        (median); sticker axes and size from the marker blob's edges in the image; then a candidate SEARCH (jaw edge x yaw offsets x pad positions x flips),
        ranked. /grasp/execute tries them in rank order (first one with a feasible IK chain)."""
        depth, K, _, T_BC = self._close_frame()
        color, g, gc, gr = self._color_bgr(), self.cfg["color_marker"], self.cfg["grasp"], self.cfg["gripper"]
        gp = marker_points(depth, color, K, T_BC, g, self.target)   # only the marker blob nearest to the scan target
        if len(gp) < g["min_marker_points"]:
            raise RuntimeError("only %d marker points in the close view: camera over the sticker? (/grasp/view)" % len(gp))
        c = np.median(gp, 0)
        gp = gp[np.linalg.norm(gp[:, :2] - c[:2], axis=1) < g["footprint_radius_m"]]  # stray marker specks elsewhere on the table are not the sticker
        rect = sticker_rect(color, K, T_BC, c, g)
        search, eff = self._search_on(), rect
        if g["extent_source"] == "footprint":  # axes and sizes from the head outline in the image (uses the dark body), centre from the sticker
            fp = housing_footprint(color, depth, K, T_BC, c, g)
            eff = dict(long_axis=fp["long_axis"], short_axis=fp["jaw_axis"], long_m=fp["length_m"], short_m=fp["width_m"], center_xy=rect["center_xy"])
        cands = sticker_candidates(eff, c, g, gr["finger_width_m"], gr["max_open_m"], gc["open_margin_m"], g["grasp_drop_m"], search)
        if not cands:
            raise RuntimeError("no candidate: the sticker is wider than the gripper opening allows")
        grasps = self._grasps_from(cands)
        self.last = dict(ok=True, grasps=grasps, stamp=time.time(), cloud=gp, cluster=gp)
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        run.mkdir(parents=True, exist_ok=True)
        self.last_run = run
        (run / "plan.json").write_text(json.dumps(dict(
            ok=True, method="marker_cloud_candidates", candidate_search=search, contact_faces=g["contact_faces"], extent_source=g["extent_source"], sticker_centroid_median_m=c,
            n_marker_points=len(gp), sticker_rect=rect, extents_used=eff, jaw_edge=g["jaw_edge"],
            n_candidates=len(cands), candidates=cands[:30]), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.save(run / "marker_close_points.npy", gp.astype(np.float32))
        np.savez_compressed(run / "frame.npz", depth=depth, K=K, T_BC=T_BC, color=color)
        report_cloud_candidates(run / "plots", gp, rect, cands)
        self._publish(self.last)
        b = cands[0]
        return "cloud mode (search %s, faces %s, extents from %s): sticker %.1f x %.1f mm; %d candidates; best: jaw along the %s edge, yaw %+.0f deg, offset %+.1f mm, pads touch faces %.1f mm wide, TCP (%.1f, %.1f, %.1f) mm, jaw (%.2f, %.2f), opening %.1f mm (run %s)" % (
            "ON" if search else "OFF", g["contact_faces"], g["extent_source"], rect["long_m"] * 1e3, rect["short_m"] * 1e3, len(cands), b["edge"], b["yaw_deg"], b["offset_m"] * 1e3, b["face_width_m"] * 1e3,
            *(b["p"] * 1e3), b["jaw"][0], b["jaw"][1], b["open_width_m"] * 1e3, run.name)

    def _grasps_from(self, cands):
        """Candidates -> grasp dicts (both jaw directions each, approach straight down, pre-grasp and lift along the approach), ranked."""
        gc, out = self.cfg["grasp"], []
        for cd in cands:
            z_G = np.asarray(cd.get("approach", [0.0, 0.0, -1.0]), float)   # CAD mode: down the fitted plug axis
            for flip in (1, -1):
                y = flip * np.asarray(cd["jaw"], float)
                y = (y - (y @ z_G) * z_G) / np.linalg.norm(y - (y @ z_G) * z_G)
                T = np.eye(4)
                T[:3, :3], T[:3, 3] = np.stack([np.cross(y, z_G), y, z_G], 1), cd["p"]
                Tp, Tl = T.copy(), T.copy()
                Tp[:3, 3] -= z_G * gc["standoff_m"]
                Tl[:3, 3] -= z_G * gc["lift_m"]
                out.append(dict(T_BG=T, T_pre=Tp, T_lift=Tl, open_width_m=cd["open_width_m"], expected_closed_width_m=cd["width_m"],
                                zone_offset_m=cd["offset_m"], flip=flip, cost=cd["cost"]))
        return out

    def allow_grasp(self):
        """Explicit enable: until this is called, /grasp/execute only checks the motion."""
        self.set_parameters([Parameter("allow_grasp", Parameter.Type.BOOL, True)])
        return "grasp motion ENABLED (needs dry_run:=false too)"

    def execute(self):
        res = self.last
        if not res or not res["ok"]:
            raise RuntimeError("run /grasp/plan first")
        if time.time() - res["stamp"] > self.get_parameter("max_plan_age_s").value:
            raise RuntimeError("plan is stale, run /grasp/plan again")
        return self._run(res["grasps"][: res.get("tries") or self.get_parameter("max_candidates").value])

    def home(self):
        q = wrap180(self.cfg["scan"]["home_joints_deg"])
        self._need_arm()
        d = float(np.abs(q - self.arm.q_deg()).max())
        if self.get_parameter("dry_run").value:
            return "DRY RUN, nothing moved. Home is %.0f deg away (largest joint)" % d
        self.arm.go_joints(q)
        return "at home (was %.0f deg away)" % d

    def open_gripper(self):
        """Fully open the gripper if it is not (position > 0.05 rad = part or fully closed). Run start check; nothing moves in a dry run."""
        self._need_arm()
        pos = self.arm.grip_pos()
        if pos <= 0.05:
            return "gripper already open (%.3f rad)" % pos
        if self.get_parameter("dry_run").value:
            return "DRY RUN, gripper not moved: it is at %.3f rad (0 = open)" % pos
        self.arm.grip_to(0.0, self.cfg["gripper"]["close_effort"], rate=0.3)
        return "gripper was at %.3f rad, opened to %.3f rad" % (pos, self.arm.grip_pos())

    # ---------- any grasp algorithm ----------
    def _on_proposal(self, m):
        """/grasp/proposal JSON: {"T_BG": 4x4 TCP pose in base_link (z approach, y jaw axis), "open_width_m": w, "source": "name"}"""
        try:
            d = json.loads(m.data)
            T, w = np.array(d["T_BG"], float), float(d["open_width_m"])
            self._check_proposal(T, w)
            self.proposal = dict(T_BG=T, open_width_m=w, source=d.get("source", "?"), stamp=time.time())
            self.get_logger().info("proposal from '%s' accepted at %s, opening %.0f mm" % (self.proposal["source"], np.round(T[:3, 3], 3), w * 1e3))
        except Exception as e:
            self.proposal = None
            self.get_logger().error("proposal rejected: %s" % e)

    def _check_proposal(self, T, w):
        """Same gate for every algorithm: valid frame, sane opening, inside the workspace, above the table."""
        R = T[:3, :3]
        if T.shape != (4, 4) or np.abs(R.T @ R - np.eye(3)).max() > 1e-3 or np.linalg.det(R) < 0.99:
            raise RuntimeError("T_BG is not a rigid 4x4 pose")
        if not 0.0 < w <= self.cfg["gripper"]["max_open_m"]:
            raise RuntimeError("opening %.0f mm outside (0, %.0f] mm" % (w * 1e3, self.cfg["gripper"]["max_open_m"] * 1e3))
        x0, x1, y0, y1 = self.cfg["workspace"]["region"]
        x, y, z = T[:3, 3]
        if not (x0 <= x <= x1 and y0 <= y <= y1 and z >= self.arm.floor_z):
            raise RuntimeError("grasp point %s outside the workspace / below the table" % np.round(T[:3, 3], 3))

    def execute_proposal(self):
        p = self.proposal
        if p is None:
            raise RuntimeError("no valid /grasp/proposal received")
        if time.time() - p["stamp"] > self.get_parameter("max_plan_age_s").value:
            raise RuntimeError("proposal is stale, publish it again")
        g = self.cfg["grasp"]
        z_G = p["T_BG"][:3, 2]
        T_pre, T_lift = p["T_BG"].copy(), p["T_BG"].copy()
        T_pre[:3, 3] -= z_G * g["standoff_m"]
        T_lift[:3, 3] -= z_G * g["lift_m"]
        return self._run([dict(T_BG=p["T_BG"], T_pre=T_pre, T_lift=T_lift, open_width_m=p["open_width_m"],
                               expected_closed_width_m=None, zone_offset_m=0.0, flip=0)])

    # ---------- the one motion path (analytic or learned grasp) ----------
    def _need_arm(self):
        if not self.arm.ready():
            raise RuntimeError("no joint states / controller actions / MoveIt IK: driver and move_group up? same ros_env.sh in every terminal?")

    def _run(self, grasps):
        self._need_scan()
        self._need_arm()
        dry, gp, log = self.get_parameter("dry_run").value, self.cfg["gripper"], []
        to_tool, dev = np.linalg.inv(self.T_tool_tcp), self.cfg["grasp"].get("max_joint_dev_deg", self.cfg["scan"]["max_joint_dev_deg"])
        chosen = None
        for i, g in enumerate(grasps):  # check the whole pick of a candidate before moving: pre-grasp IK, descent, lift
            plan, why = self.arm.plan_chain(g["T_pre"] @ to_tool, g["T_BG"] @ to_tool, g["T_lift"] @ to_tool, dev)
            log.append("candidate %d (%.0f mm, flip %+d): %s" % (i, g["zone_offset_m"] * 1e3, g["flip"], why))
            if plan:
                chosen = (g, plan)
                break
        self.get_logger().info("; ".join(log))
        if chosen is None:
            raise RuntimeError("no candidate passed the motion check: " + "; ".join(log))
        g, plan = chosen
        if dry or not self.get_parameter("allow_grasp").value:  # grasp motion needs dry_run:=false AND allow_grasp:=true
            return "NOT EXECUTED (dry_run or allow_grasp:=false), motion checked only. " + log[-1]
        self.last = None   # a plan moves the arm once; the next grasp needs a fresh plan

        if self.arm.grip_pos() > 0.05:   # left closed or part-closed: open fully first (opening only, nothing closes before the grasp)
            self.arm.grip_to(0.0, gp["close_effort"], rate=0.3)
        open_pos = float(np.interp(g["open_width_m"], self.wid_tab[::-1], self.pos_tab[::-1]))
        self.arm.approach(plan, at_pre_grasp=lambda: self.arm.grip_to(open_pos, gp["close_effort"], rate=0.3))  # the jaw opening is set up at the pre-grasp pose, just before the descent, not at the start of the long move
        pos, contact = self.arm.grip_to(gp["close_cmd"], gp["close_effort"], rate=0.2)
        held = contact or grasp_held(pos, gp)
        width = float(np.interp(pos, self.pos_tab, self.wid_tab))
        exp = g["expected_closed_width_m"]
        self.get_logger().info("gripper stopped at %.3f (empty >= %.3f), ~%.1f mm, expected %s mm: %s" % (
            pos, gp["empty_frac"] * gp["close_cmd"], width * 1e3, "?" if exp is None else "%.1f" % (exp * 1e3),
            "held" if held else "closed on nothing"))
        self.arm.retreat(plan)  # retreat either way; a failed grasp ends the cycle for a human to reset
        seen, tips, table = self._object_in_hand()  # the finger reading cannot see thin objects (HW7 docs 3.7), so the camera decides too
        self.get_logger().info("wrist camera after the lift: depth between the tips %.3f m, table %.3f m -> %s" % (tips, table, "object in hand" if seen else "nothing between the tips"))
        if not (held or seen):
            raise RuntimeError("grasp NOT held: fingers closed on nothing and the wrist camera sees nothing between the tips")
        return "held (finger check: %s, wrist camera: %s); fingers stopped at ~%.1f mm; camera depth between the tips %.3f m, table %.3f m" % (
            "yes" if held else "no", "yes" if seen else "no", width * 1e3, tips, table)

    # ---------- RViz ----------
    def _cloud(self, pts):
        h = Header(frame_id=BASE, stamp=self.get_clock().now().to_msg())
        return point_cloud2.create_cloud_xyz32(h, np.asarray(pts, np.float32)[:: max(1, len(pts) // 300000)])

    def _publish(self, res):
        self.pub_scene.publish(self._cloud(res["cloud"]))
        if res["cluster"] is not None:
            self.pub_cluster.publish(self._cloud(res["cluster"]))
        gp = self.cfg["gripper"]
        ms = MarkerArray()
        ms.markers.append(Marker(action=Marker.DELETEALL))

        def cube(mid, T, size, rgba):
            m = Marker(type=Marker.CUBE, id=mid, ns="grasp")
            m.header.frame_id = BASE
            m.pose = pose_msg(T)
            m.scale.x, m.scale.y, m.scale.z = (float(s) for s in size)
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            return m

        fw, ft, fl, below = (gp["finger_width_m"], gp["finger_thickness_m"], gp["finger_length_m"], gp["finger_tip_below_tcp_m"])
        for k, g in enumerate(res["grasps"]):
            for j, sgn in enumerate((1, -1)):
                T = np.eye(4)
                T[:3, 3] = [0, sgn * (g["open_width_m"] / 2 + ft / 2), -(fl - below) / 2]
                rgba = (0.1, 0.9, 0.2, 0.9) if k == 0 else (0.9, 0.9, 0.2, 0.25)
                ms.markers.append(cube(10 + 2 * k + j, g["T_BG"] @ T, (fw, ft, fl + below), rgba))
        self.pub_marks.publish(ms)


def main():
    rclpy.init()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(GraspNode())
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
