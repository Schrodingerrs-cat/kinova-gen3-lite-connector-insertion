#!/usr/bin/env python3
"""Grasp pipeline end to end for one or several cables, with data collection into a Media/Grasp run folder. Needs the stack and the grasp node running.

python3 ros2_iface/run_pipeline.py RUN_DIR [--grasp] [--skip-scan] [--mode head|cloud|model|cad]
                                   [--connector NAME [NAME ...]] [--hint X,Y [X,Y ...]] [--between pause|release|stop]

head / cloud / model: home -> 7-pose scan -> colour target -> close view -> grasp plan -> [--grasp] execute.
cad: home -> find the sticker -> 3 close views -> fit the connector's STEP model -> grasp plan -> [--grasp] execute.
Every run starts with the arm at home and the gripper open. --connector (cad mode) picks the named connectors of config/connectors.yaml one after the
other, each in its own sub-folder of RUN_DIR; see ros2_iface/pipeline.py for what happens between two cables. Exit status 0 = every cable planned / picked.
"""
import os
import pathlib
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
WORK = ROOT.parents[1] / "Media" / "Grasp" / "work"  # where the node writes its scans / plan frames (same as grasp_node.py)
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
import pipeline  # noqa: E402
from housing_grasp import connectors  # noqa: E402
from robot_ros import Robot  # noqa: E402

OK_OUTCOMES = ("GRASP_REPORTED_HELD", "PERCEPTION_ONLY")


class Recorder:
    def __init__(self, out_dir):
        self.proc = subprocess.Popen([sys.executable, str(HERE / "record.py"), str(out_dir)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def stop(self):
        self.proc.send_signal(signal.SIGTERM)
        self.proc.wait(timeout=20)


class RosIo(pipeline.Io):
    def __init__(self, robot):
        self.robot, self.clients = robot, {}

    def call(self, service, timeout):
        cli = self.clients.setdefault(service, self.robot.create_client(Trigger, "/grasp/" + service))
        if not cli.wait_for_service(10.0):
            raise RuntimeError("service /grasp/%s not available (is the grasp node running?)" % service)
        fut, t = cli.call_async(Trigger.Request()), time.time()
        while not fut.done() and time.time() - t < timeout:
            time.sleep(0.2)
        if not fut.done():
            raise TimeoutError("/grasp/%s took longer than %d s" % (service, timeout))
        r = fut.result()
        return r.success, r.message

    def set_node_params(self, **values):
        cli = self.clients.setdefault("_set_parameters", self.robot.create_client(SetParameters, "/grasp_node/set_parameters"))
        if not cli.wait_for_service(10.0):
            raise RuntimeError("/grasp_node/set_parameters not available (is the grasp node running?)")
        req = SetParameters.Request()
        req.parameters = [Parameter(name=k, value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)) for k, v in values.items()]
        fut = cli.call_async(req)
        t = time.time()
        while not fut.done() and time.time() - t < 10:
            time.sleep(0.05)
        if not fut.done() or not all(r.successful for r in fut.result().results):
            raise RuntimeError("could not set the grasp node parameters %s" % sorted(values))

    def joints_deg(self):
        return np.degrees(self.robot.q)

    def snap(self, path):
        m = self.robot._img.get("color")
        if m is not None:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            cv2.imwrite(str(path), np.ascontiguousarray(img[:, :, ::-1] if m.encoding == "rgb8" else img))

    def start_recorder(self, out_dir):
        return Recorder(out_dir)

    def ask(self, prompt):
        return input(prompt)


def main():
    opts = pipeline.parse_args()
    for name in opts.connectors or ():
        try:
            connectors.load(name)           # validates config/connectors.yaml and the name before anything moves
        except (KeyError, ValueError) as e:
            sys.exit("connectors: %s" % (e.args[0] if e.args else e))
    home = (np.array(yaml.safe_load(open(ROOT / "config" / "grasp.yaml"))["scan"]["home_joints_deg"]) + 180) % 360 - 180
    rclpy.init()
    robot = Robot("pipeline", background=True)
    if not robot.wait_ready(10):
        sys.exit("no driver / controller")
    results = pipeline.Pipeline(opts, RosIo(robot), home, WORK, ROOT).run()
    code = 0 if results and all(r.outcome.startswith(OK_OUTCOMES) for r in results) and len(results) == max(1, len(opts.connectors)) else 1
    sys.stdout.flush()
    os._exit(code)      # background rclpy threads would keep the process alive


if __name__ == "__main__":
    main()
