import sys, json, time, pathlib
import numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, JointState
out = pathlib.Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
rclpy.init(); n = Node("snapshot"); got = {}
q = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
p = "/eih_cam"
n.create_subscription(Image, p + "/color/image_raw", lambda m: got.__setitem__("color", m), q)
n.create_subscription(Image, p + "/aligned_depth_to_color/image_raw", lambda m: got.__setitem__("depth", m), q)
n.create_subscription(CameraInfo, p + "/color/camera_info", lambda m: got.__setitem__("info", m), q)
n.create_subscription(JointState, "/joint_states", lambda m: got.__setitem__("js", m), 10)
t0 = time.time()
while len(got) < 4 and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.2)
print("received:", sorted(got))
c = got.get("color")
if c:
    img = np.frombuffer(c.data, np.uint8).reshape(c.height, c.step)[:, :c.width*3].reshape(c.height, c.width, 3)
    cv2.imwrite(str(out/"color.png"), img[:, :, ::-1] if c.encoding == "rgb8" else img)
    print("color", c.width, c.height, c.encoding, "frame", c.header.frame_id)
d = got.get("depth")
if d:
    dep = np.frombuffer(d.data, np.uint16).reshape(d.height, d.step//2)[:, :d.width]
    np.save(out/"depth_mm.npy", dep); v = dep[dep > 0]
    print("depth valid %.0f%%, median %.0f mm, 5-95%% %s mm" % (100*v.size/dep.size, np.median(v) if v.size else 0, np.percentile(v, [5, 95]).round(0).tolist() if v.size else None))
i = got.get("info")
if i: json.dump(dict(K=list(i.k), D=list(i.d), width=i.width, height=i.height, frame_id=i.header.frame_id), open(out/"camera_info.json", "w"), indent=1); print("K", np.round(i.k, 1).tolist()[:9], "dist", np.round(i.d, 4).tolist())
js = got.get("js")
if js: json.dump(dict(zip(js.name, js.position)), open(out/"joints.json", "w"), indent=1)
n.destroy_node(); rclpy.shutdown()
