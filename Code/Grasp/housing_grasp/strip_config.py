"""config/strip.yaml: loading and validation of the strip mode settings. YAML only."""
import pathlib

import yaml

from .connectors import MARKER_KEYS

CONFIG = pathlib.Path(__file__).resolve().parents[1] / "config" / "strip.yaml"
CONNECTOR_KEYS = ("long_axis_rel_strip_deg", "footprint_mm", "footprint_tol_mm", "jaw_rel_strip_deg", "open_width_m", "clear_mm", "band_mm", "top_margin_mm", "dark_v", "roi_radius_mm")


def _need(ok, msg):
    if not ok:
        raise ValueError(msg)


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def validate_connector(name, c):
    where = "strip connector '%s': " % name
    for k in CONNECTOR_KEYS:
        _need(k in c, where + "missing key '%s'" % k)
    _need(c["long_axis_rel_strip_deg"] is None or _num(c["long_axis_rel_strip_deg"]), where + "long_axis_rel_strip_deg must be a number or null (round: orientation does not matter)")
    _need(isinstance(c["footprint_mm"], (list, tuple)) and len(c["footprint_mm"]) == 2 and all(_num(x) and x > 0 for x in c["footprint_mm"]), where + "footprint_mm must be [along strip, across strip] in mm")
    _need(_num(c["footprint_tol_mm"]) and c["footprint_tol_mm"] > 0 and _num(c["jaw_rel_strip_deg"]), where + "footprint_tol_mm > 0 and jaw_rel_strip_deg a number")
    _need(all(_num(c[k]) and c[k] > 0 for k in ("open_width_m", "clear_mm", "band_mm", "top_margin_mm", "roi_radius_mm")) and 0 < c["dark_v"] <= 255, where + "open_width_m, clear_mm, band_mm, top_margin_mm, roi_radius_mm must be > 0 and dark_v in (0, 255]")
    _need(c["open_width_m"] > min(c["footprint_mm"]) * 1e-3, where + "open_width_m must exceed the footprint")
    if "marker" in c:
        _need(set(c["marker"]) <= set(MARKER_KEYS), where + "marker may only override %s" % (MARKER_KEYS,))


def validate(cfg):
    s, v, p, sel, r = cfg["strip"], cfg["views"], cfg["plug"], cfg["select"], cfg["refine"]
    _need(len(s["size_mm"]) == 2 and s["size_mm"][0] >= s["size_mm"][1] > 0 and 0 < s["size_tol"] < 0.5, "strip.size_mm must be [long, short] and size_tol in (0, 0.5)")
    _need(s["yaw_spread_deg"] > 0 and s["min_points"] > 0 and s["search_radius_m"] > 0, "strip: yaw_spread_deg, min_points, search_radius_m must be > 0")
    _need(len(v["list"]) >= 2 and all({"across_deg", "tilt_deg"} <= set(e) and 0 <= e["tilt_deg"] <= 70 for e in v["list"]), "views.list needs >= 2 entries with across_deg and tilt_deg (0-70)")
    _need(v["distance_m"] >= 0.07 and v["frames"] >= 1 and v["tip_clear_mm"] >= 0, "views: distance_m >= 0.07 (D405 minimum range), frames >= 1, tip_clear_mm >= 0")
    _need(len(p["top_band_mm"]) == 2 and p["top_band_mm"][0] < p["top_band_mm"][1] and 0 < p["edge_pct"] < 20, "plug: top_band_mm must be [low, high], edge_pct in (0, 20)")
    _need(all(p[k] > 0 for k in ("min_points_view", "loo_mm", "top_spread_mm", "yaw_check_deg", "yaw_check_aspect")), "plug: gate values must be > 0")
    _need(sel["neighbour_radius_mm"] > 0 and sel["finger_margin_mm"] >= 0, "select: neighbour_radius_mm > 0, finger_margin_mm >= 0")
    _need(r["max_shift_mm"] > 0 and r["frames"] >= 1 and isinstance(r["require"], bool), "refine: max_shift_mm > 0, frames >= 1, require true/false")
    _need(cfg["active"] in cfg["connectors"], "strip.yaml: active '%s' is not a connector (%s)" % (cfg["active"], ", ".join(cfg["connectors"])))
    for name, c in cfg["connectors"].items():
        if c.get("status") != "pending":
            validate_connector(name, c)


def load(name=None, path=None):
    """(cfg, spec) of the strip connector `name` (default `active`). Unknown names list the available ones; a connector marked `status: pending` is refused."""
    cfg = yaml.safe_load(open(path or CONFIG))
    validate(cfg)
    name = name or cfg["active"]
    _need(name in cfg["connectors"], "unknown strip connector '%s' (available: %s)" % (name, ", ".join(cfg["connectors"])))
    _need(cfg["connectors"][name].get("status") != "pending", "strip connector '%s' is not set up yet (status: pending in config/strip.yaml)" % name)
    return cfg, dict(cfg["connectors"][name], name=name)


def names(path=None, ready_only=False):
    c = yaml.safe_load(open(path or CONFIG))["connectors"]
    return [n for n, v in c.items() if not (ready_only and v.get("status") == "pending")]
