import yaml


def _nulls(d, path=""):
    for k, v in d.items():
        if isinstance(v, dict):
            yield from _nulls(v, f"{path}{k}.")
        elif v is None:
            yield f"{path}{k}"


def load_config(path):
    cfg = yaml.safe_load(open(path))
    missing = list(_nulls(cfg))
    if missing:
        raise ValueError("unmeasured config values: " + ", ".join(missing))
    return cfg
