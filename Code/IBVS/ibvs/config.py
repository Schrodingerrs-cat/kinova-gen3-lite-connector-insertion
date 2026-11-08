import hashlib
import pathlib

import yaml

SERVO = ("robot", "camera", "markers", "ibvs", "guards")      # non-contact alignment
CONTACT = SERVO + ("axial", "search", "motion", "verification", "waypoints")
FLAGS = ("verification.positive_control_passed", "verification.negative_control_passed",
         "safety.supervisor_approved", "safety.e_stop_tested", "safety.clear_retract_path_tested")


def _nulls(d, path):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _nulls(v, f"{path}{k}.")
        elif v is None:
            yield f"{path}{k}"


def load_config(path, contact=False):
    """Config dict + sha256. Refuses unmeasured values; contact=True also refuses UNCALIBRATED and unset safety flags."""
    text = pathlib.Path(path).read_text()
    cfg = yaml.safe_load(text)
    sections = CONTACT if contact else SERVO
    bad = [m for s in sections for m in _nulls(cfg[s], s + ".")]
    if contact:
        if cfg["status"] != "CALIBRATED":
            bad.append("status is " + str(cfg["status"]))
        bad += [f for f in FLAGS if not cfg[f.split(".")[0]][f.split(".")[1]]]
    if bad:
        raise ValueError("not ready: " + ", ".join(bad))
    cfg["config_hash"] = hashlib.sha256(text.encode()).hexdigest()[:12]
    return cfg
