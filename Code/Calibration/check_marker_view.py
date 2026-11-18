#!/usr/bin/env python3
"""Save a camera/tag diagnostic. No robot motion or calibration samples."""
from pathlib import Path
import cv2
import rclpy
import yaml
from robot_ros import Robot

rclpy.init()
robot=Robot('check_calibration_marker',background=False)
try:
    color,_,K=robot.grab()
    cfg=yaml.safe_load((Path(__file__).parent/'calibration.yaml').read_text())['aruco']
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,cfg['dictionary'])),cv2.aruco.DetectorParameters())
    corners,ids,rejected=detector.detectMarkers(color)
    preview=color.copy()
    if ids is not None:cv2.aruco.drawDetectedMarkers(preview,corners,ids)
    folder=Path(__file__).parent/'results/visibility_check';folder.mkdir(parents=True,exist_ok=True)
    cv2.imwrite(str(folder/'color.png'),color)
    cv2.imwrite(str(folder/'detection.png'),preview)
    print('Detected IDs:',None if ids is None else ids.ravel().tolist(),flush=True)
    print('Rejected candidate squares:',len(rejected),flush=True)
    print('Image:',(folder/'detection.png').resolve(),flush=True)
finally:
    robot.destroy_node();rclpy.shutdown()
