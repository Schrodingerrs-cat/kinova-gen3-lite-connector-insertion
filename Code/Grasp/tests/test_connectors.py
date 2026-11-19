import copy
import pathlib
import sys

import pytest
import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from housing_grasp import connectors as C  # noqa: E402


@pytest.fixture
def cfg():
    return yaml.safe_load(open(C.CONFIG))


def test_the_shipped_config_is_valid_and_lists_its_connectors(cfg):
    C.validate(cfg)
    assert C.names() == ["usb_a", "micro_jack"]
    c, spec = C.load("micro_jack")
    assert spec["name"] == "micro_jack" and spec["symmetry"] == "axial"
    assert C.load()[1]["name"] == cfg["active"]


def test_an_unknown_connector_names_the_available_ones():
    with pytest.raises(KeyError, match="unknown connector 'iec'.*usb_a, micro_jack"):
        C.load("iec")


@pytest.mark.parametrize("edit,message", [
    (lambda c: c["connectors"]["usb_a"].pop("seat_u_mm"), "connector 'usb_a': missing key 'seat_u_mm'"),
    (lambda c: c["connectors"]["usb_a"].update(seat_u_mm=99.0), "seat_u_mm 99.0 is outside fit_u_mm"),
    (lambda c: c["connectors"]["usb_a"].update(symmetry="round"), "symmetry must be one of"),
    (lambda c: c["connectors"]["usb_a"].update(fit_u_mm=[28.0, 0.0]), "fit_u_mm must be"),
    (lambda c: c["connectors"]["usb_a"]["mesh_to_model"].update(A=[[1, 0, 0], [0, 1, 0], [0, 0, 1.0], ]) or c["connectors"]["usb_a"]["mesh_to_model"].update(A=[[1, 0, 0], [0, 1, 0], [0, 0, -1.0]]), "proper rotation"),
    (lambda c: c["connectors"]["usb_a"].update(step="CAD_files/missing.step"), "STEP file not found"),
    (lambda c: c["connectors"]["usb_a"].update(marker=dict(hue=[1, 2])), "marker may only override"),
    (lambda c: c.update(active="nope"), "active 'nope' is not a connector"),
    (lambda c: c["views"].update(n=1), "views.n must be"),
    (lambda c: c["views"].update(distance_m=0.03), "D405 minimum range"),
    (lambda c: c["views"].update(coarse="sometimes"), "views.coarse must be one of"),
    (lambda c: c["fit"]["gate"].update(require="yes"), "fit.gate.require must be true or false"),
    (lambda c: c["fit"].update(trim=0.1), "fit: trim"),
    (lambda c: c["panel"].update(ring_mm=[40, 16]), "panel.ring_mm must be"),
])
def test_a_bad_config_is_rejected_with_a_clear_message(cfg, edit, message):
    bad = copy.deepcopy(cfg)
    edit(bad)
    with pytest.raises(ValueError, match=message):
        C.validate(bad)


def test_a_connector_can_have_its_own_marker_colour(cfg):
    g = dict(h=[80, 105], s_min=140, v_min=60, edge_jump_m=0.004)
    assert C.marker_cfg(g, dict(name="x")) == g
    red = C.marker_cfg(g, dict(marker=dict(h=[170, 10], s_min=120)))
    assert red["h"] == [170, 10] and red["s_min"] == 120 and red["v_min"] == 60 and red["edge_jump_m"] == 0.004
    cfg["connectors"]["usb_a"]["marker"] = dict(h=[170, 10])
    C.validate(cfg)                                                                    # a wrapping hue range is fine
