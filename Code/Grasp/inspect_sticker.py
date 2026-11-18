import cv2
import yaml
from housing_grasp.color_marker import marker_mask

folder = "../../diagnostics/camera/"
image = cv2.imread(folder + "color.png")
if image is None:
    raise RuntimeError("Run check_depth.py first")

with open("config/grasp.yaml") as f:
    settings = yaml.safe_load(f)["color_marker"]

mask = marker_mask(image, settings)
image[mask] = (0, 0, 255)

print("Detected sticker pixels:", int(mask.sum()))
cv2.imwrite(folder + "sticker_mask.png", image)
