#!/usr/bin/env python3
"""Pick, align (IBVS), insert for several wires with retries and resume.
python3 harness/run_task.py [--wire NAME ...] [--live] [--resume RUN_DIR]
Default is dry: perception + plans only, IBVS preview, nothing grips or inserts. Needs Grasp/run_ros.sh (dry_run:=false for --live)."""
import argparse
import pathlib
import sys
import time

import yaml

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from core import Policy, Task, Wire  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=pathlib.Path, default=HERE / "config.yaml")
    ap.add_argument("--wire", nargs="+", help="subset of the configured wires, in this order")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--resume", type=pathlib.Path, help="run folder to continue; finished wires are skipped")
    a = ap.parse_args()
    cfg = yaml.safe_load(open(a.config))
    wires = [Wire(w["name"], w["socket"], w.get("hint", "")) for w in cfg["wires"]]
    if a.wire:
        by = {w.name: w for w in wires}
        unknown = [n for n in a.wire if n not in by]
        if unknown:
            sys.exit("unknown wire(s) %s, configured: %s" % (unknown, sorted(by)))
        wires = [by[n] for n in a.wire]
    run_dir = a.resume or HERE.parents[1] / "Media" / "Task" / time.strftime("run_%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "config.yaml").write_text(yaml.safe_dump(cfg))
    policy = Policy(dict(pick=cfg["pick"]["max_attempts"], align=cfg["align"]["max_attempts"], insert=cfg["insert"]["max_attempts"]),
                    max_consecutive_wire_failures=cfg["max_consecutive_wire_failures"], **cfg["retry"])
    from stages import RosBackend      # ROS only needed here
    ok = Task(wires, RosBackend(cfg, a.live, run_dir), run_dir, policy).run(resume=bool(a.resume))
    sys.stdout.flush()
    import os
    os._exit(0 if ok else 1)


if __name__ == "__main__":
    main()
