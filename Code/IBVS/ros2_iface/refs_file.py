"""Write one wire's entry back into config/insertion_refs.yaml (only its line changes, the comments of the file stay)."""
import pathlib
import re

PATH = pathlib.Path(__file__).resolve().parents[1] / "config" / "insertion_refs.yaml"


def save_wire(name, wire, path=PATH):
    line = "  %s: {marker_id: %d, pre: {c: %s, side: %s}, insert: {c: %s, side: %s}, nudge_px: %s%s%s}" % (
        name, wire["marker_id"], wire["pre"]["c"], wire["pre"]["side"], wire["insert"]["c"], wire["insert"]["side"], wire["nudge_px"],
        ", plug_x: %s" % wire["plug_x"] if "plug_x" in wire else "",
        ", seated: {c: %s, side: %s}" % (wire["seated"]["c"], wire["seated"]["side"]) if "seated" in wire else "")
    new, n = re.subn(r"^  %s:.*$" % name, line.replace("\\", "\\\\"), path.read_text(), flags=re.M)
    assert n == 1, "wire line not found in insertion_refs.yaml"
    path.write_text(new)
