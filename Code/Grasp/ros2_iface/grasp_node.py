#!/usr/bin/env python3
"""ROS 2 node, main pipeline: colour scan -> colour marker -> camera over it -> marker-sticker grasp -> pick.
Motion path = HW7's (arm.py): MoveIt only answers IK / line paths, trajectories go to the controller.

Services (std_srvs/Trigger):
  /grasp/pick          the whole pipeline: scan -> color_target -> view -> sticker_grasp -> execute
  /grasp/scan_gripper  once, arm over empty table: sweep the gripper, scan the fingers -> config/gripper_scan.yaml
  /grasp/home          joint move to scan.home_joints_deg
  /grasp/scan          15 camera poses over the workspace (colour + depth) -> merged cloud + marker cloud (ICP, voxel)
  /grasp/color_target  merged scan: table RANSAC, above-table clusters, marker clusters -> coarse target (marker centroid)
  /grasp/view          camera straight over that target at `view_height_m`
  /grasp/sticker_grasp   close view: marker sticker + colour footprint of the housing -> grasp (jaw across the body, approach down)
  /grasp/head_grasp    IMAGE MODE: head segmented in the image, only those pixels lifted to a head cloud, ranked candidates synthesized on it
                       (param grasp_mode:=head makes /grasp/pick use it instead of sticker_grasp)
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
from housing_grasp.color_marker import analyze_scan, marker_mask, marker_points, head_points, housing_footprint, merge_views, sticker_candidates, sticker_rect, grid_grasp_search, oriented_normals, outline_normals, scene_points  # noqa: E402
from housing_grasp.color_marker_report import report_cloud_candidates, report_color_target, report_sticker_grasp, report_grid_search  # noqa: E402
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
                              ("max_candidates", 4), ("allow_grasp", False), ("grasp_mode", "sticker"),
                              ("max_plan_age_s", 60.0), ("view_height_m", 0.18)):
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
                         ("allow_grasp", self.allow_grasp), ("sticker_grasp", self.sticker_grasp), ("hold_check", self.hold_check), ("head_grasp", self.head_grasp), ("cloud_grasp", self.cloud_grasp)):
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
        for k, T in enumerate(view_poses(sc, T0, self.T_tool_cam)):
            q = self.arm.ik_near(T, [self.home_q, last], sc["max_joint_dev_deg"])  # near home or near the previous view
            if q is None:
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
            self.get_logger().info("scan view %d/%d captured" % (len(views), len(sc["views"])))
        if dry:
            return "DRY RUN, nothing moved: %d/%d view poses plannable, skipped %s" % (len(views), len(sc["views"]), skipped)
        self.arm.go_joints(self.home_q)  # back to the start pose
        if len(views) < 2:
            raise RuntimeError("fewer than 2 views captured, skipped %s" % skipped)
        clouds, markers = [], []
        for k in views:
            v = np.load(out / ("view_%02d.npz" % k))
            clouds.append(view_cloud(v["depth"], v["K"], v["T_BC"], self.cfg["cloud"]))
            markers.append(marker_points(v["depth"], v["color"], v["K"], v["T_BC"], self.cfg["color_marker"]))
        fused, info = fuse_clouds(clouds, voxel_m=self.cfg["cloud"]["voxel_m"])
        marker = merge_views(markers, info, self.cfg["color_marker"]["voxel_m"])  # same ICP corrections as the full cloud
        np.save(out / "fused_cloud.npy", fused.astype(np.float32))
        np.save(out / "marker_cloud.npy", marker.astype(np.float32))
        self.scan_dir, self.marker_pts = out, marker
        self.pub_marker.publish(self._cloud(marker))
        (out / "fusion.json").write_text(json.dumps(dict(views=views, skipped=skipped, per_view=info), indent=1))
        self.fused = fused
        self.pub_fused.publish(self._cloud(fused))
        shifts = [i["shift_mm"] for i in info[1:]]
        return "scan %s: %d views (skipped %s), %d fused points, %d marker points; ICP corrections vs earlier views: median %.1f mm, max %.1f mm" % (
            out.name, len(views), skipped, len(fused), len(marker), np.median(shifts), np.max(shifts))

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
        grasp = dict(sticker=self.sticker_grasp, head=self.head_grasp, cloud=self.cloud_grasp)[self.get_parameter("grasp_mode").value]  # param grasp_mode: sticker (default) | head | cloud
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
        r = analyze_scan(P, G, self.cfg)
        if r["frame"] is None:
            raise RuntimeError("no colour marker above the table in the merged scan %s" % d.name)
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

    def sticker_grasp(self):
        """Grasp straight from the marker sticker (user decision 2026-11-07): no depth-measured housing, no collision / optimizer checks.
        Jaw axis across the housing footprint seen in the colour image, TCP `grasp_drop_m` below the sticker top, approach straight down."""
        depth, K, _, T_BC = self._frame()
        color, g, gc = self._color_bgr(), self.cfg["color_marker"], self.cfg["grasp"]
        gp = marker_points(depth, color, K, T_BC, g)
        if len(gp) < 100:
            raise RuntimeError("only %d marker points in the close view: camera over the sticker? (/grasp/view)" % len(gp))
        c = np.median(gp, 0)  # median: depth streaks on the glossy head would pull a mean
        fp = housing_footprint(color, depth, K, T_BC, c, g)
        jaw = np.r_[fp["jaw_axis"], 0.0]
        jaw /= np.linalg.norm(jaw)
        z_G = np.array([0.0, 0.0, -1.0])
        open_w = min(fp["width_m"] + 2 * gc["open_margin_m"], self.cfg["gripper"]["max_open_m"])
        p = np.array([c[0], c[1], c[2] - g["grasp_drop_m"]])
        grasps = []
        for flip in (1, -1):
            y = flip * jaw
            T = np.eye(4)
            T[:3, :3], T[:3, 3] = np.stack([np.cross(y, z_G), y, z_G], 1), p
            Tp, Tl = T.copy(), T.copy()
            Tp[:3, 3] -= z_G * gc["standoff_m"]
            Tl[:3, 3] -= z_G * gc["lift_m"]
            grasps.append(dict(T_BG=T, T_pre=Tp, T_lift=Tl, open_width_m=open_w, expected_closed_width_m=fp["width_m"], zone_offset_m=0.0, flip=flip, cost=0.0))
        self.last = dict(ok=True, grasps=grasps, stamp=time.time(), cloud=gp, cluster=gp)
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        run.mkdir(parents=True, exist_ok=True)
        self.last_run = run
        (run / "plan.json").write_text(json.dumps(dict(
            ok=True, method="sticker", sticker_centroid_median_m=c, n_marker_points=len(gp), footprint=fp, jaw_axis=jaw, open_width_m=open_w,
            tcp_m=p, grasp_drop_m=g["grasp_drop_m"], grasps=[dict(flip=q["flip"], T_BG=q["T_BG"]) for q in grasps]), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.save(run / "marker_close_points.npy", gp.astype(np.float32))
        np.savez_compressed(run / "frame.npz", depth=depth, K=K, T_BC=T_BC, color=color)
        report_sticker_grasp(run / "plots", gp, fp, c, p, jaw, open_w)
        self._publish(self.last)
        return "sticker median (%.1f, %.1f, %.1f) mm, %d pts; housing footprint %.1f x %.1f mm; jaw axis (%.2f, %.2f); opening %.1f mm; TCP (%.1f, %.1f, %.1f) mm (run %s)" % (
            *(c * 1e3), len(gp), fp["length_m"] * 1e3, fp["width_m"] * 1e3, jaw[0], jaw[1], open_w * 1e3, *(p * 1e3), run.name)

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
        depth, K, _, T_BC = self._frame()
        color, g, gc, gr = self._color_bgr(), self.cfg["color_marker"], self.cfg["grasp"], self.cfg["gripper"]
        gp = marker_points(depth, color, K, T_BC, g)
        if len(gp) < 100:
            raise RuntimeError("only %d marker points in the close view: camera over the sticker? (/grasp/view)" % len(gp))
        c = np.median(gp, 0)
        P = head_points(depth, color, K, T_BC, c, g)
        if len(P) < g["min_head_points"]:
            raise RuntimeError("head cloud has only %d points (need %d): no usable depth on the segmented head" % (len(P), g["min_head_points"]))
        N2, S = outline_normals(P), scene_points(depth, color, K, T_BC, c, g)
        allc, stats = grid_grasp_search(P, N2, S, gp, c, g, gr["finger_width_m"], gr["finger_thickness_m"], gr["max_open_m"], gc["open_margin_m"], g["grasp_drop_m"])
        if not allc:
            raise RuntimeError("grid search found no candidate on the head cloud: %s" % stats)
        cands = allc[:g["max_candidates_kept"]]
        grasps = self._grasps_from(cands)
        self.last = dict(ok=True, grasps=grasps, stamp=time.time(), cloud=P, cluster=gp)
        run = WORK / "runs" / time.strftime("%Y%m%d_%H%M%S")
        run.mkdir(parents=True, exist_ok=True)
        self.last_run = run
        (run / "plan.json").write_text(json.dumps(dict(
            ok=True, method="head_image_mode_grid_search", search_stats=stats, sticker_centroid_median_m=c, n_marker_points=len(gp), n_head_points=len(P),
            candidates=cands, grasps=[dict(flip=q["flip"], T_BG=q["T_BG"], cost=q["cost"]) for q in grasps]), indent=1, default=lambda o: np.asarray(o).tolist()))
        np.save(run / "head_points.npy", P.astype(np.float32))
        np.save(run / "marker_close_points.npy", gp.astype(np.float32))
        np.savez_compressed(run / "frame.npz", depth=depth, K=K, T_BC=T_BC, color=color)
        report_grid_search(run / "plots", P, oriented_normals(P), N2, gp, c, allc, stats, gr["finger_width_m"])
        self._publish(self.last)
        b = cands[0]
        return "image mode grid search: head cloud %d points; %d candidates generated, %d valid; best yaw %.0f deg, %.1f mm from the sticker, TCP (%.1f, %.1f, %.1f) mm, jaw (%.2f, %.2f), width %.1f mm, opening %.1f mm (run %s)" % (
            len(P), stats["generated"], stats["valid"], b["yaw_deg"], b["offset_m"] * 1e3, *(b["p"] * 1e3), b["jaw"][0], b["jaw"][1], b["width_m"] * 1e3, b["open_width_m"] * 1e3, run.name)

    def cloud_grasp(self):
        """CLOUD MODE (marker region only, no black body): the marker pixels are the region of interest. Sticker position and height from the marker 3D points
        (median); sticker axes and size from the marker blob's edges in the image; then a candidate SEARCH (jaw edge x yaw offsets x pad positions x flips),
        ranked. /grasp/execute tries them in rank order (first one with a feasible IK chain)."""
        depth, K, _, T_BC = self._frame()
        color, g, gc, gr = self._color_bgr(), self.cfg["color_marker"], self.cfg["grasp"], self.cfg["gripper"]
        gp = marker_points(depth, color, K, T_BC, g)
        if len(gp) < 100:
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
        gc, z_G, out = self.cfg["grasp"], np.array([0.0, 0.0, -1.0]), []
        for cd in cands:
            for flip in (1, -1):
                y = flip * cd["jaw"]
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
        return self._run(res["grasps"][: self.get_parameter("max_candidates").value])

    def home(self):
        q = wrap180(self.cfg["scan"]["home_joints_deg"])
        self._need_arm()
        d = float(np.abs(q - self.arm.q_deg()).max())
        if self.get_parameter("dry_run").value:
            return "DRY RUN, nothing moved. Home is %.0f deg away (largest joint)" % d
        self.arm.go_joints(q)
        return "at home (was %.0f deg away)" % d

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
        gp, g = self.cfg["gripper"], self.cfg["grasp"]
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

        self.arm.grip_to(float(np.interp(g["open_width_m"], self.wid_tab[::-1], self.pos_tab[::-1])), gp["close_effort"], rate=0.3)
        self.arm.approach(plan)
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
