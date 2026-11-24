"""Connector profiles: ~/kinova/work/connectors/<name>.yaml (usb, aux)."""
import pathlib

import yaml

DIR = pathlib.Path.home() / "kinova/work/connectors"


def load(name="usb"):
    return yaml.safe_load(open(DIR / (name + ".yaml")))
