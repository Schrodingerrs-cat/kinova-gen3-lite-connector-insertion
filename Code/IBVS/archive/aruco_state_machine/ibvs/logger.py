import csv
import json
import time

COLUMNS = ["run_id", "timestamp_ns", "controller", "config_hash", "connector_id", "phase", "q", "ee_pose", "camera_pose",
           "features_px", "desired_features_px", "feature_confidence", "depth", "u_command", "u_executed",
           "contact_proxy_valid", "verification_signal", "verified_success", "stop_reason", "trial_seed/perturbation",
           "attempt_index"]


class TrialLogger:
    def __init__(self, path, **fixed):
        self.f = open(path, "w", newline="")
        self.w = csv.DictWriter(self.f, COLUMNS)
        self.w.writeheader()
        self.fixed = fixed

    def row(self, **kw):
        r = {**self.fixed, "timestamp_ns": time.time_ns(), **kw}
        self.w.writerow({k: (json.dumps(v.tolist() if hasattr(v, "tolist") else v) if isinstance(v, (list, tuple)) or hasattr(v, "tolist") else v)
                         for k, v in r.items()})
        self.f.flush()
