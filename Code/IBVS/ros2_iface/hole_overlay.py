#!/usr/bin/env python3
"""Camera-only preview and optional 12-hole click setup. Keys 1/2/3 select a rectangle."""
import argparse
import copy
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ibvs.hole_rectangles import HoleRectangleTracker, number_rectangles  # noqa: E402
from ibvs.hole_templates import template_data  # noqa: E402
from ibvs.overlay import FeatureOverlay  # noqa: E402


class HolePreview:
    def __init__(self, reference, path, calibrate=False):
        self.reference, self.path, self.calibrating = reference, path, calibrate
        self.display = FeatureOverlay(reference)
        self.tracker = self.camera_key = None
        self.image = self.frozen = None
        self.valid_frame = False
        self.clicks = []
        self.message = ''
        self.observation = (np.full((1, 4, 2), np.nan), np.array([False]))
        self.last_stamp = None

    def mouse(self, event, x, y, _flags, _parameter):
        if not self.calibrating or not self.valid_frame or self.image is None:
            return
        if event == cv2.EVENT_RBUTTONDOWN:
            self.clicks = self.clicks[:-1]
        elif event == cv2.EVENT_LBUTTONDOWN and len(self.clicks) < 12:
            if self.frozen is None:
                self.frozen = self.image.copy()
            holes = [] if self.tracker.templates is not None else self.tracker.find_holes(self.frozen)
            close = sorted(holes, key=lambda hole: np.linalg.norm(hole[1]-[x, y]))
            point = close[0][1] if close and np.linalg.norm(close[0][1]-[x,y])<=5 else np.array([x,y],float)
            if any(np.linalg.norm(point-previous) < 5 for previous in self.clicks):
                self.message = 'That hole is already selected.'
                return
            self.clicks.append(point.copy())
            self.message = f'{len(self.clicks)}/12 holes selected'

    def select(self, number):
        self.reference['feature_tracking']['selected_rectangle'] = number
        self.display.select_rectangle(number)
        if self.tracker is not None:
            self.tracker.select_rectangle(number)
        self.last_stamp = None

    def save(self):
        if len(self.clicks) != 12:
            self.message = 'Select all 12 holes: four per rectangle, then press S.'
            return
        try:
            groups = number_rectangles(np.asarray(self.clicks).reshape(3, 4, 2))
            updated = copy.deepcopy(self.reference)
            updated['feature_tracking']['seed_rectangles_image_px'] = groups.tolist()
            template_path=self.path.with_name(self.path.stem+'_templates.npz')
            np.savez_compressed(template_path,**template_data(self.frozen,groups))
            updated['feature_tracking']['template_file']=str(template_path.resolve())
            FeatureOverlay(updated)
            self.path.write_text(yaml.safe_dump(updated, sort_keys=False))
        except (OSError, ValueError) as error:
            self.message = str(error)
            return
        print('Saved 12 measured hole centers, numbered left to right:', self.path, flush=True)
        self.reference = updated
        self.display = FeatureOverlay(updated)
        self.tracker = self.camera_key = None
        self.last_stamp = None
        self.frozen = None
        self.calibrating = False
        self.message = 'Saved. Preview: 1/2/3 select, Q quits.'

    def render(self, image, camera, stamp, age, valid):
        self.image, self.valid_frame = image, valid and age <= .5 and camera is not None
        if camera is not None:
            key = (tuple(np.ravel(camera['K'])), tuple(camera['distortion']))
            if key != self.camera_key:
                self.tracker = HoleRectangleTracker(camera, self.reference['feature_tracking'],
                                                     self.reference['corners_image_px'])
                self.camera_key = key
                self.last_stamp = None
        if self.calibrating:
            result = (self.frozen if self.frozen is not None else image).copy()
            for index, point in enumerate(self.clicks):
                anchor = tuple(np.rint(point).astype(int))
                cv2.circle(result, anchor, 5, (0, 255, 180), 2)
                cv2.putText(result, f'{index//4+1}.{index%4+1}', (anchor[0]+7, anchor[1]-7),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 180), 1)
            caption = 'Click 4 holes per rectangle (any group order); S save, R reset, right-click undo, Q quit'
            cv2.putText(result, caption, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 255, 180), 1)
            if not self.valid_frame:
                self.message = 'Waiting for a fresh camera frame at the configured size and frame ID.'
        else:
            if self.valid_frame and stamp != self.last_stamp:
                self.observation = self.tracker.detect(image)
                self.last_stamp = stamp
            result = self.display.render(image, None if camera is None else camera['K'],
                [] if camera is None else camera['distortion'], stamp, age_s=age, frame_valid=valid,
                observation=self.observation, draw_target=True, phase='CAMERA PREVIEW', feature_tracker=self.tracker)
            caption = '1 / 2 / 3: select rectangle | Q / Esc: quit'
            cv2.putText(result, caption, (10, result.shape[0]-30), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 180), 1)
        if self.message:
            cv2.putText(result, self.message[:115], (10, result.shape[0]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, .43, (0, 200, 255), 1)
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, default=ROOT/'config/hole_rectangles.yaml')
    parser.add_argument('--rectangle', type=int, choices=(1, 2, 3))
    parser.add_argument('--calibrate', action='store_true', help='click four holes per rectangle, then S saves the initial groups')
    parser.add_argument('--bringup', action='store_true', help='start the D405 if no camera frames arrive')
    args = parser.parse_args()
    reference = yaml.safe_load(args.reference.read_text())
    if reference.get('feature_tracking', {}).get('type') != 'hole_rectangles':
        parser.error('use a hole_rectangles reference')
    if args.rectangle is not None:
        reference['feature_tracking']['selected_rectangle'] = args.rectangle
    preview = HolePreview(reference, args.reference, args.calibrate)
    for name in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE', 'GIO_MODULE_DIR',
                 'GIO_EXTRA_MODULES', 'GDK_PIXBUF_MODULEDIR', 'GDK_PIXBUF_MODULE_FILE', 'GTK_MODULES'):
        os.environ.pop(name, None)
    os.environ.setdefault('QT_QPA_PLATFORM', 'xcb')
    os.environ.setdefault('ROS_LOG_DIR', tempfile.mkdtemp(prefix='ibvs_hole_overlay_logs_'))

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = rclpy.create_node('ibvs_hole_overlay')
    latest, bridge = {}, CvBridge()
    prefix = reference['camera_topic_prefix'].rstrip('/')
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(Image, prefix+'/color/image_raw', lambda msg: latest.update(image=msg), qos)
    node.create_subscription(CameraInfo, prefix+'/color/camera_info', lambda msg: latest.update(info=msg), qos)
    publisher = node.create_publisher(Image, '/ibvs/feature_image', 1)
    camera_process = camera_log = None
    started = time.monotonic()
    last_published = None
    width, height = reference['image_size_px']
    cv2.namedWindow('IBVS holes', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('IBVS holes', width, height)
    cv2.setMouseCallback('IBVS holes', preview.mouse)
    print('Camera-only hole overlay. No robot motion is commanded.', flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.03)
            if args.bringup and 'image' not in latest and camera_process is None and time.monotonic()-started > 2:
                log_path = Path(os.environ['ROS_LOG_DIR'])/'camera.log'
                camera_log = log_path.open('w')
                camera_process = subprocess.Popen(['ros2', 'launch', 'realsense2_camera', 'rs_launch.py',
                    'camera_name:=eih_cam', 'camera_namespace:=/', 'tf_prefix:=eih_cam',
                    f'depth_module.color_profile:={width}x{height}x30',
                    f'depth_module.depth_profile:={width}x{height}x30', 'align_depth.enable:=true'],
                    stdout=camera_log, stderr=subprocess.STDOUT, start_new_session=True)
                print('Starting D405; log:', log_path, flush=True)
            if camera_process is not None and camera_process.poll() is not None:
                raise RuntimeError('D405 launch exited; inspect '+str(log_path))
            if 'image' in latest:
                message, info = latest['image'], latest.get('info')
                stamp = message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                age = node.get_clock().now().nanoseconds*1e-9-stamp
                camera = None if info is None else dict(K=np.asarray(info.k).reshape(3, 3), distortion=list(info.d))
                valid = (info is not None and [info.width, info.height] == [width, height]
                    and [message.width, message.height] == [width, height]
                    and message.header.frame_id == info.header.frame_id == reference['camera_frame_id']
                    and age >= -.1)
                image = bridge.imgmsg_to_cv2(message, 'bgr8')
                result = preview.render(image, camera, stamp, max(0., age), valid)
                if last_published != stamp:
                    output = bridge.cv2_to_imgmsg(result, 'bgr8')
                    output.header = message.header
                    publisher.publish(output)
                    last_published = stamp
            else:
                result = np.zeros((height, width, 3), np.uint8)
                cv2.putText(result, 'Waiting for D405 color and camera info...', (15, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 180), 1)
            cv2.imshow('IBVS holes', result)
            key = cv2.waitKey(1) & 0xff
            if key in (ord('q'), 27):
                break
            if key in (ord('1'), ord('2'), ord('3')) and not preview.calibrating:
                preview.select(int(chr(key)))
            if preview.calibrating and key == ord('r'):
                preview.clicks, preview.frozen, preview.message = [], None, ''
            if preview.calibrating and key == ord('s'):
                preview.save()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if camera_process is not None and camera_process.poll() is None:
            os.killpg(camera_process.pid, signal.SIGINT)
            try:
                camera_process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(camera_process.pid, signal.SIGTERM)
        if camera_log is not None:
            camera_log.close()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
