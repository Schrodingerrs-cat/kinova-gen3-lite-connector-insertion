"""Connector registry of config/connectors.yaml: loading and validation. YAML and NumPy only, so the runner can use it without the heavy CAD imports."""
import pathlib

import numpy as np
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]
CONFIG = ROOT / "Code" / "Grasp" / "config" / "connectors.yaml"
SYMMETRIES = ("flip", "axial", "none")
COARSE = ("never", "fallback", "always")
MARKER_KEYS = ("h", "s_min", "v_min")


def _need(ok, msg):
    if not ok:
        raise ValueError(msg)


def _pair(v, name):
    _need(isinstance(v, (list, tuple)) and len(v) == 2 and all(isinstance(x, (int, float)) for x in v) and v[0] < v[1], "%s must be [low, high] with low < high, got %r" % (name, v))


def validate_connector(name, c, root=ROOT):
    where = "connector '%s': " % name
    for k in ("step", "mesh", "candidates", "mesh_to_model", "seat_u_mm", "fit_u_mm", "body_u_mm", "symmetry", "roi_radius_mm", "dark_v"):
        _need(k in c, where + "missing key '%s'" % k)
    _need((root / c["step"]).exists(), where + "STEP file not found: %s" % c["step"])
    A, b = np.array(c["mesh_to_model"].get("A", []), float), np.array(c["mesh_to_model"].get("b_mm", []), float)
    _need(A.shape == (3, 3) and b.shape == (3,), where + "mesh_to_model needs A (3x3) and b_mm (3)")
    _need(np.allclose(A @ A.T, np.eye(3)) and np.isclose(np.linalg.det(A), 1.0), where + "mesh_to_model.A must be a proper rotation (orthonormal, determinant +1)")
    _pair(c["fit_u_mm"], where + "fit_u_mm")
    _pair(c["body_u_mm"], where + "body_u_mm")
    _need(c["fit_u_mm"][0] <= c["seat_u_mm"] <= c["fit_u_mm"][1], where + "seat_u_mm %s is outside fit_u_mm %s" % (c["seat_u_mm"], c["fit_u_mm"]))
    _need(c["symmetry"] in SYMMETRIES, where + "symmetry must be one of %s" % (SYMMETRIES,))
    _need(c["roi_radius_mm"] > 0 and 0 < c["dark_v"] <= 255, where + "roi_radius_mm must be > 0 and dark_v in (0, 255]")
    if "marker" in c:
        m = c["marker"]
        _need(set(m) <= set(MARKER_KEYS), where + "marker may only override %s" % (MARKER_KEYS,))
        _need(all(isinstance(x, (int, float)) for x in m.get("h", [0, 0])) and len(m.get("h", [0, 0])) == 2, where + "marker.h must be [h0, h1] (OpenCV hue 0-179; h0 > h1 wraps, e.g. red)")


def validate(cfg, root=ROOT):
    _need(cfg.get("connectors"), "connectors.yaml: no connectors")
    _need(cfg["active"] in cfg["connectors"], "connectors.yaml: active '%s' is not a connector (%s)" % (cfg["active"], ", ".join(cfg["connectors"])))
    v, f, p, pl = cfg["views"], cfg["fit"], cfg["panel"], cfg["place"]
    _need(isinstance(v["n"], int) and 2 <= v["n"] <= len(v["azimuths_deg"]), "views.n must be an integer >= 2 and at most the number of azimuths_deg")
    _need(v["distance_m"] >= 0.07 and 0 <= v["tilt_deg"] <= 70 and v["frames"] >= 1 and v["tip_clear_mm"] >= 0, "views: distance_m >= 0.07 (D405 minimum range), tilt_deg 0-70, frames >= 1, tip_clear_mm >= 0")
    _need(v["coarse"] in COARSE, "views.coarse must be one of %s" % (COARSE,))
    _need(0.3 < f["trim"] <= 1.0 and f["train_per_view"] > f["min_points_view"] > 0 and f["tilt_max_deg"] >= 0, "fit: trim in (0.3, 1], train_per_view > min_points_view > 0, tilt_max_deg >= 0")
    g = f["gate"]
    for k in ("p80_mm", "within_mm", "min_within_pct", "pos_spread_mm", "axis_spread_deg", "seat_tol_mm"):
        _need(isinstance(g.get(k), (int, float)) and g[k] > 0, "fit.gate.%s must be a positive number" % k)
    _need(isinstance(g.get("require"), bool), "fit.gate.require must be true or false")
    _pair(p["ring_mm"], "panel.ring_mm")
    _need(p["min_inliers"] <= p["min_points"] and 0 < p["max_tilt_deg"] <= 20, "panel: min_inliers <= min_points and 0 < max_tilt_deg <= 20")
    _need(pl["clearance_mm"] >= 0 and pl["max_hits"] >= 0 and pl["max_candidates"] >= 1 and len(pl["jaw_yaw_offsets_deg"]) >= 1, "place: clearance_mm >= 0, max_hits >= 0, max_candidates >= 1, jaw_yaw_offsets_deg not empty")
    for name, c in cfg["connectors"].items():
        validate_connector(name, c, root)


def load(name=None, path=None):
    """(cfg, spec) of the connector `name` (default: `active`). Validates the whole file; unknown names list the available ones."""
    cfg = yaml.safe_load(open(path or CONFIG))
    validate(cfg)
    name = name or cfg["active"]
    if name not in cfg["connectors"]:
        raise KeyError("unknown connector '%s' (available: %s)" % (name, ", ".join(cfg["connectors"])))
    return cfg, dict(cfg["connectors"][name], name=name)


def names(path=None):
    return list(yaml.safe_load(open(path or CONFIG))["connectors"])


def marker_cfg(g, spec):
    """The colour_marker settings with the connector's own marker colour, if it has one."""
    return dict(g, **spec.get("marker", {}))
