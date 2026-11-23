"""Gen3 Lite for MuJoCo: Kinova limits, finger linkage (mimic) as joint equalities, position servos.
Limits from kortex_description (URDF + MoveIt joint_limits.yaml). Gripper force/stroke from the vendor are NOT in the
description files: GRIP_EFFORT is a TBD parameter, swept in the grasp gate."""
import os, numpy as np, mujoco
HERE = os.path.dirname(os.path.abspath(__file__))
ARM = [f"joint_{i}" for i in range(1, 7)]
EFFORT = [10, 14, 10, 7, 7, 7]                 # Nm, URDF
VEL = [1.6, 1.6, 1.6, 1.6, 1.6, 3.2]           # rad/s, URDF
ACC = [0.86, 0.43, 0.34, 0.86, 8.6, 8.6]       # rad/s^2, MoveIt joint_limits.yaml
KP, KV = [200, 300, 200, 100, 100, 100], [20, 30, 20, 10, 10, 10]
GRIP_EFFORT = 5.0                              # Nm at finger_bottom joint, TBD (URDF 50, Gazebo plugin 5, ros2_control 100 unitless)
TIP_MIMIC = (0.149, -0.676)                    # tip = 0.149 - 0.676 * right_bottom


def load(spec=None):
    spec = spec or mujoco.MjSpec.from_file(f"{HERE}/assets/robot/gen3_lite.urdf")
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST  # stable finger linkage
    for j in spec.joints:  # armature/damping verified by open-close sweep (monotonic, symmetric, within limits)
        if "finger" in j.name: j.armature, j.damping = 0.002, [0.05, 0, 0]
        elif j.name.startswith("joint_"): j.armature, j.damping = 0.01, [1.0, 0, 0]
    for n, j in zip(ARM, range(6)):
        spec.add_actuator(name=f"a_{n}", target=n, trntype=mujoco.mjtTrn.mjTRN_JOINT, gainprm=[KP[j]] + [0] * 9,
                          biastype=mujoco.mjtBias.mjBIAS_AFFINE, biasprm=[0, -KP[j], -KV[j]] + [0] * 7,
                          forcelimited=1, forcerange=[-EFFORT[j], EFFORT[j]])
    spec.add_actuator(name="a_grip", target="right_finger_bottom_joint", trntype=mujoco.mjtTrn.mjTRN_JOINT, gainprm=[20.0] + [0] * 9,
                      biastype=mujoco.mjtBias.mjBIAS_AFFINE, biasprm=[0, -20.0, -0.5] + [0] * 7, forcelimited=1, forcerange=[-GRIP_EFFORT, GRIP_EFFORT])
    eq = lambda n, j1, j2, c: spec.add_equality(name=n, type=mujoco.mjtEq.mjEQ_JOINT, name1=j1, name2=j2, data=[*c, 0, 0, 0, 0, 0, 0][:11],
                                                 solref=[0.002, 1])
    eq("left_bottom", "left_finger_bottom_joint", "right_finger_bottom_joint", [0, 1, 0, 0, 0])
    eq("right_tip", "right_finger_tip_joint", "right_finger_bottom_joint", [TIP_MIMIC[0], TIP_MIMIC[1], 0, 0, 0])
    eq("left_tip", "left_finger_tip_joint", "right_finger_bottom_joint", [TIP_MIMIC[0], TIP_MIMIC[1], 0, 0, 0])
    return spec
