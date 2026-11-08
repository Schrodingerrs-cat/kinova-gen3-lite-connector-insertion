"""Record the run: camera video, and joints / gripper / tool pose at 10 Hz. Stops on Ctrl-C or SIGTERM.
python3 ros2_iface/record.py OUT_DIR   ->  OUT_DIR/videos/camera.mp4, OUT_DIR/trajectory.csv
"""
import csv
import os
import signal
import sys
import time

import cv2
import numpy as np
import rclpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robot_ros import Robot

out = sys.argv[1]
os.makedirs(out + "/videos", exist_ok=True)
rclpy.init()
r = Robot("grasp_recorder", background=True)
assert r.wait_ready(10), "no driver"
stop = []
signal.signal(signal.SIGTERM, lambda *a: stop.append(1))
f = open(out + "/trajectory.csv", "w", newline="")
w = csv.writer(f)
w.writerow(["t_s"] + ["j%d_deg" % i for i in range(1, 7)] + ["gripper_rad", "tcp_x_m", "tcp_y_m", "tcp_z_m"])
vid, t0, seen = None, time.time(), None
try:
    while not stop:
        t = time.time() - t0
        try:
            T = r.tool_matrix()
        except Exception:
            T = np.full((4, 4), np.nan)
        w.writerow(["%.2f" % t] + list(np.round(np.degrees(r.q), 3)) + [r.grip_pos_msr] + list(np.round(T[:3, 3], 4)))
        m = r._img.get("color")
        if m is not None and m is not seen:
            seen = m
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            img = np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img)
            if vid is None:
                vid = cv2.VideoWriter(out + "/videos/camera.mp4", cv2.VideoWriter_fourcc(*"mp4v"), 10, (m.width, m.height))
            vid.write(img)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
f.close()
if vid is not None:
    vid.release()
os._exit(0)
