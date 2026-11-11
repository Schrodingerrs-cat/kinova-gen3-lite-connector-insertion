"""Printable ArUco tags for the panel (offline). Writes tags/panel_tags_aruco4x4_25mm.png + .pdf at true size (the black square is SIDE_MM wide).
Stick one flat on the panel top near the plug, matte paper, no gloss; measure the printed black square with a ruler and put it in the config.
python3 tools/make_panel_tag.py [side_mm]"""
import pathlib
import sys

import cv2
import numpy as np
from PIL import Image

SIDE_MM = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
IDS = (0, 1)                     # 0 = the one to use, 1 = a spare
CELL_PX = 100                    # 6 x 6 cells (4 x 4 bits + a 1-cell black border) -> 600 px; scale chosen so the black square is exactly SIDE_MM
DPI = 6 * CELL_PX / (SIDE_MM / 25.4)
mm = lambda v: int(round(v / 25.4 * DPI))
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
W, H, quiet = mm(2 * SIDE_MM + 36), mm(SIDE_MM + 26), mm(5)
page = np.full((H, W), 255, np.uint8)
for i, tid in enumerate(IDS):
    tag = cv2.aruco.generateImageMarker(d, tid, 6 * CELL_PX, borderBits=1)
    x0, y0 = quiet + i * (mm(SIDE_MM) + mm(10)), quiet
    page[y0:y0 + tag.shape[0], x0:x0 + tag.shape[1]] = tag
    cv2.putText(page, "ID %d" % tid, (x0, y0 + tag.shape[0] + mm(5)), cv2.FONT_HERSHEY_SIMPLEX, DPI / 300, 0, max(2, int(DPI / 150)))
y = H - mm(6)
cv2.line(page, (quiet, y), (quiet + mm(10), y), 0, max(3, int(DPI / 100)))   # 10 mm scale bar: check it with a ruler after printing
cv2.putText(page, "10 mm | ArUco DICT_4X4_50 | black square %g mm" % SIDE_MM, (quiet + mm(13), y + mm(1.2)), cv2.FONT_HERSHEY_SIMPLEX, DPI / 400, 0, max(2, int(DPI / 200)))
out = pathlib.Path(__file__).resolve().parents[1] / "tags"
out.mkdir(exist_ok=True)
img = Image.fromarray(page)
img.save(out / ("panel_tags_aruco4x4_%gmm.png" % SIDE_MM), dpi=(DPI, DPI))
img.convert("1").save(out / ("panel_tags_aruco4x4_%gmm.pdf" % SIDE_MM), resolution=DPI)   # 1-bit page: PIL has no JPEG writer here
print("wrote", out, "| page %.0f x %.0f mm at %.1f dpi, black square %g mm, ids %s" % (W / DPI * 25.4, H / DPI * 25.4, DPI, SIDE_MM, IDS))
# detection check on the rendered page (a clean print should be found at once)
det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
_, ids, _ = det.detectMarkers(page)
print("detected on the page image:", sorted(int(i) for i in ids.ravel()) if ids is not None else None)
