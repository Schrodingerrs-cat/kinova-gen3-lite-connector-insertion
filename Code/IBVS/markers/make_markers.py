"""Print sheets of ArUco markers at true scale (DICT_4X4_50). Run: python3 markers/make_markers.py
Prints on US Letter or A4: content stays inside 190 x 255 mm. Print at 100 % / actual size, never 'fit to page'."""
import pathlib

import cv2
import numpy as np
from PIL import Image

DPI = 600
MM = DPI / 25.4
PAGE_W, PAGE_H = round(215.9 * MM), round(279.4 * MM)      # Letter; A4 is narrower and taller, content fits both
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
OUT = pathlib.Path(__file__).resolve().parent
# page title, black square size (mm), [(id, name)], columns
PAGES = [("Wall markers 30 mm", 30, [(10, "usb_a"), (11, "audio"), (12, "ac")], 3),
         ("Hand-eye calibration marker 100 mm", 100, [(0, "hand_eye")], 1)]


def text(page, s, x, y, scale=1.6):
    cv2.putText(page, s, (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, scale, 0, 3, cv2.LINE_AA)


def make_page(title, size_mm, items, columns):
    page = np.full((PAGE_H, PAGE_W), 255, np.uint8)
    cell = round(size_mm / 6 * MM)                        # whole pixels per marker cell: sharp edges
    side = 6 * cell
    margin = round(side / 4)                              # white quiet zone around the black square
    box = side + 2 * margin
    gap, top = round(8 * MM), round(30 * MM)
    left = (PAGE_W - (columns * box + (columns - 1) * gap)) // 2
    text(page, title, left, round(18 * MM))
    placed = []
    for n, (marker_id, name) in enumerate(items):
        x = left + (n % columns) * (box + gap)
        y = top + (n // columns) * (box + round(16 * MM))
        page[y + margin:y + margin + side, x + margin:x + margin + side] = cv2.aruco.generateImageMarker(DICT, marker_id, side)
        cv2.rectangle(page, (x, y), (x + box, y + box), 170, 2)           # light cut line around the quiet zone
        actual = side / MM
        text(page, "ID %d  %s  black square %.1f mm" % (marker_id, name, actual), x, y + box + round(7 * MM), 1.2)
        placed.append((marker_id, x + margin, y + margin, side))
    bar_y, bar_x = PAGE_H - round(25 * MM), left
    cv2.line(page, (bar_x, bar_y), (bar_x + round(50 * MM), bar_y), 0, 4)
    for dx in (0, round(50 * MM)):
        cv2.line(page, (bar_x + dx, bar_y - 25), (bar_x + dx, bar_y + 25), 0, 4)
    text(page, "Scale check: this bar must measure 50 mm. Print at 100 percent.", bar_x, bar_y + round(9 * MM), 1.1)
    return page, placed


def main():
    pages = []
    for number, (title, size_mm, items, columns) in enumerate(PAGES, 1):
        page, placed = make_page(title, size_mm, items, columns)
        cv2.imwrite(str(OUT / ("page%d_%dmm.png" % (number, size_mm))), page)
        pages.append(Image.fromarray(page))
        print("page %d: %s" % (number, title), [(i, "%.2f mm" % (s / MM)) for i, _, _, s in placed])
    pages[0].save(OUT / "aruco_markers.pdf", "PDF", resolution=DPI, save_all=True, append_images=pages[1:])


if __name__ == "__main__":
    main()
