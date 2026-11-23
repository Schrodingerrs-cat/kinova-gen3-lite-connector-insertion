import pathlib
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]


def test_3d_projection_works_whichever_report_module_imports_pyplot_first():
    for first in ("housing_grasp.cad_report", "housing_grasp.strip_report", "housing_grasp.color_marker_report"):
        code = "import %s, matplotlib.pyplot as plt; plt.figure().add_subplot(projection='3d')" % first
        r = subprocess.run([sys.executable, "-c", code], cwd=ROOT, capture_output=True, text=True)
        assert r.returncode == 0, "%s: %s" % (first, r.stderr[-300:])
