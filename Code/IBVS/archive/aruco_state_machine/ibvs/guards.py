"""Hard stop checks. Each returns a stop reason string or None."""
import numpy as np


def check_servo(now, cam_stamp, joint_stamp, tool_xyz, g, box, t_start, timeout, missing_frames, max_missing):
    if now - cam_stamp > g["camera_stale_s"]:
        return "CAMERA_STALE"
    if now - joint_stamp > g["joint_stale_s"]:
        return "JOINTS_STALE"
    x, y, z = tool_xyz
    if not (box[0] <= x <= box[1] and box[2] <= y <= box[3] and box[4] <= z <= box[5]):
        return "OUT_OF_BOUNDS"
    if now - t_start > timeout:
        return "TIMEOUT"
    if missing_frames > max_missing:
        return "MARKERS_LOST"
    return None


def check_axial(commanded_m, measured_m, total_m, ax):
    """Per step: robot must follow the command; total travel capped. No force sensor: this is not a contact measure."""
    if total_m + measured_m > ax["max_travel_m"]:
        return "MAX_TRAVEL"
    if measured_m < ax["min_progress_frac"] * commanded_m:
        return "NO_PROGRESS"
    return None
