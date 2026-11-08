import pathlib

import yaml

SCAN = "gripper_scan.yaml"  # written by /grasp/scan_gripper: finger size and position->width table
SCAN_KEYS = ("finger_width_m", "finger_thickness_m", "finger_length_m", "position_to_width_m")


def _nulls(d, path=""):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _nulls(v, f"{path}{k}.")
        elif v is None:
            yield f"{path}{k}"


def load_config(path, allow_unscanned=False):
    path = pathlib.Path(path)
    cfg = yaml.safe_load(open(path))
    scan = path.parent / SCAN
    if scan.exists():
        cfg["gripper"].update({k: v for k, v in yaml.safe_load(open(scan)).items() if k in cfg["gripper"]})
    missing = [m for m in _nulls(cfg) if not (allow_unscanned and m.split(".")[-1] in SCAN_KEYS)]
    if missing:
        raise ValueError("unmeasured config values: " + ", ".join(missing) + " (gripper ones: run /grasp/scan_gripper)")
    return cfg
