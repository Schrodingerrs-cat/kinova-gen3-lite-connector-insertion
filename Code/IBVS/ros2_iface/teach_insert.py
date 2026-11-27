#!/usr/bin/env python3
"""Teach one wire's goal from the real arm, no motion. Put the arm by hand where it should be (plug at the socket entrance for `insert`,
the plug fully inserted for `seated`, the standoff for `pre`), then run this: it looks at the wire's marker for ~1 s and writes its centre and size into config/insertion_refs.yaml.
  python3 ros2_iface/teach_insert.py --wire usb --stage insert"""
import argparse, datetime, json, pathlib, sys, time
import cv2, numpy as np, yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ibvs.plug import finger_pair_x  # noqa: E402
from refs_file import save_wire  # noqa: E402
JOINTS = [f"joint_{i}" for i in range(1, 7)]
DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
if hasattr(cv2.aruco, "ArucoDetector"):
    detect = cv2.aruco.ArucoDetector(DICTIONARY, cv2.aruco.DetectorParameters()).detectMarkers
else:                                                   # OpenCV < 4.7
    _params = cv2.aruco.DetectorParameters_create()
    detect = lambda gray: cv2.aruco.detectMarkers(gray, DICTIONARY, parameters=_params)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--wire", required=True, choices=("aux", "usb", "power"))
    ap.add_argument("--stage", required=True, choices=("pre", "insert", "seated"))
    ap.add_argument("--frames", type=int, default=20)
    a = ap.parse_args()
    refs = yaml.safe_load((ROOT / "config" / "insertion_refs.yaml").read_text())
    wire = refs["wires"][a.wire]

    rclpy.init(); node = Node("teach_insert"); got = {}
    node.create_subscription(Image, "/camera/camera/color/image_raw", lambda m: got.__setitem__("color", m), qos_profile_sensor_data)
    node.create_subscription(JointState, "/joint_states", lambda m: got.__setitem__("joints", dict(zip(m.name, m.position))), 10)
    seen, plug, last, t0 = [], [], None, time.time()
    while len(seen) < a.frames and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.1)
        m = got.pop("color", None)
        if m is None:
            continue
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        bgr = np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img)
        corners, ids, _ = detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        if ids is not None and wire["marker_id"] in ids.ravel():
            seen.append(corners[list(ids.ravel()).index(wire["marker_id"])].reshape(4, 2)); last = bgr
            x = finger_pair_x(bgr)
            if x is not None:
                plug.append(x)
    if len(seen) < a.frames:
        raise SystemExit("marker %d seen in only %d of %d frames: nothing written" % (wire["marker_id"], len(seen), a.frames))
    c = np.median(np.array(seen), axis=0)
    centre = c.mean(0)
    side = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
    spread = float(np.std(np.array(seen).reshape(len(seen), -1), axis=0).max())
    wire[a.stage] = dict(c=[round(float(centre[0]), 1), round(float(centre[1]), 1)], side=round(side, 1))
    if a.stage == "insert":
        wire["nudge_px"] = [0, 0]                       # a taught goal needs no hand nudge
        if len(plug) >= a.frames // 2:
            wire["plug_x"] = round(float(np.median(plug)), 1)      # where the plug sat in the grasp when this goal was taught

    save_wire(a.wire, wire)

    out = ROOT.parents[1] / "Media" / "IBVS" / "poses" / ("taught_%s_%s_%s" % (a.wire, a.stage, datetime.datetime.now().strftime("%Y%m%d_%H%M%S")))
    out.mkdir(parents=True)
    cv2.polylines(last, [c.astype(np.int32).reshape(-1, 1, 2)], True, (0, 255, 0), 2)
    cv2.imwrite(str(out / "color.png"), last)
    joints = got.get("joints", {})
    (out / "taught.json").write_text(json.dumps(dict(wire=a.wire, stage=a.stage, centre_px=wire[a.stage]["c"], side_px=wire[a.stage]["side"], frames=len(seen),
                                                    pixel_spread=round(spread, 2), joints_deg=[round(float(np.degrees(joints.get(j, np.nan))), 2) for j in JOINTS]), indent=2) + "\n")
    print("taught %s %s: marker %d centre %s side %s px (%d frames, spread %.2f px)\nwritten to config/insertion_refs.yaml; photo and joints in %s" % (
        a.wire, a.stage, wire["marker_id"], wire[a.stage]["c"], wire[a.stage]["side"], len(seen), spread, out))


if __name__ == "__main__":
    main()
