import json
from pathlib import Path
import numpy as np

root = Path("../../Media/Grasp/work/runs")
files = sorted(root.glob("*/plan.json"), reverse=True)

for path in files:
    plan = json.loads(path.read_text())
    if plan.get("method") == "model_mode":
        break
else:
    raise RuntimeError("No saved model plan found")

print("Plan:", path)
print("Confirm this is the plan used for your pregrasp move.")

for index, grasp in enumerate(plan["grasps"][:4]):
    T = np.array(grasp["T_BG"])
    print(
        "Candidate:", index,
        "flip:", grasp["flip"],
        "fingertip XYZ (m):", np.round(T[:3, 3], 4)
    )
    print("Orientation:\n", np.round(T[:3, :3], 3))
