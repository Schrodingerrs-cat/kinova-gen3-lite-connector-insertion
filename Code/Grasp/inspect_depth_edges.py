import cv2
import numpy as np

p = "../../Media/Grasp/work/failed_grasps/20261115_033009/"
f = np.load(p + "diagnostic.npz")
depth = f["depth"]
valid = np.isfinite(depth) & (depth > 0)

edges = np.zeros(depth.shape, dtype=bool)
edges[:, 1:] |= (
    valid[:, 1:] & valid[:, :-1]
    & (np.abs(depth[:, 1:] - depth[:, :-1]) > 0.005)
)
edges[1:, :] |= (
    valid[1:, :] & valid[:-1, :]
    & (np.abs(depth[1:, :] - depth[:-1, :]) > 0.005)
)

image = f["color"].copy()
image[edges] = (0, 0, 255)
crop = image[130:290, 350:510]
crop = cv2.resize(crop, None, fx=4, fy=4,
                  interpolation=cv2.INTER_NEAREST)
cv2.imwrite(p + "depth_edges.png", crop)
