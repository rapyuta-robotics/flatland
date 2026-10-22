"""Parameter-file schema, load-time validation and writer.

A parameter file is YAML:

    schema_version: 1
    name: example
    calibrated_step_s: 0.02          # or null: valid at any step size
    parameters: {delay_s: ..., ...}  # exactly stepper.PARAM_NAMES; `.inf` means unlimited
    provenance: {...}                # free-form mapping, logged and published by the plugin

The plugin applies the same checks at load.
"""
from __future__ import annotations

import math
from pathlib import Path

import yaml

from . import stepper

SCHEMA_VERSION = 1
TOP_KEYS = ("schema_version", "name", "calibrated_step_s", "parameters", "provenance")


class ParamFileError(ValueError):
    """The parameter file is missing, malformed, or of an unknown schema version."""


def validate(doc) -> dict:
    """Check a loaded document and return it; raise ParamFileError otherwise."""
    if not isinstance(doc, dict):
        raise ParamFileError("parameter file must be a mapping")
    version = doc.get("schema_version")
    if version != SCHEMA_VERSION or isinstance(version, bool):
        raise ParamFileError(f"unsupported schema_version {version!r}; this version reads {SCHEMA_VERSION}")
    unknown = sorted(set(doc) - set(TOP_KEYS))
    if unknown:
        raise ParamFileError(f"unknown top-level key(s): {', '.join(map(str, unknown))}")
    for key in ("name", "calibrated_step_s", "parameters", "provenance"):
        if key not in doc:
            raise ParamFileError(f"missing key {key!r}")
    if not isinstance(doc["name"], str) or not doc["name"]:
        raise ParamFileError("name must be a non-empty string")
    step = doc["calibrated_step_s"]
    if step is not None and (isinstance(step, bool) or not isinstance(step, (int, float))
                             or not math.isfinite(step) or step <= 0):
        raise ParamFileError("calibrated_step_s must be a positive number or null")
    if not isinstance(doc["provenance"], dict):
        raise ParamFileError("provenance must be a mapping")
    values = doc["parameters"]
    if not isinstance(values, dict):
        raise ParamFileError("parameters must be a mapping")
    for k, x in values.items():
        if isinstance(x, bool) or not isinstance(x, (int, float)):
            raise ParamFileError(f"parameter {k!r} must be a number")
    try:
        stepper.validate_params(values)
    except ValueError as e:
        raise ParamFileError(str(e)) from None
    return doc


def load(path) -> dict:
    p = Path(path)
    try:
        text = p.read_text()
    except OSError as e:
        raise ParamFileError(f"cannot read parameter file {p}: {e.strerror}") from None
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise ParamFileError(f"{p}: invalid YAML: {e}") from None
    return validate(doc)


def make(name: str, values: dict, calibrated_step_s, provenance: dict) -> dict:
    doc = {
        "schema_version": SCHEMA_VERSION,
        "name": name,
        "calibrated_step_s": calibrated_step_s,
        "parameters": {k: float(values[k]) for k in stepper.PARAM_NAMES + tuple(
            k for k in stepper.OPTIONAL_NAMES if values.get(k, 0.0) != 0.0)},
        "provenance": provenance,
    }
    return validate(doc)


def dump(doc: dict) -> str:
    validate(doc)
    return yaml.safe_dump(doc, sort_keys=False, default_flow_style=False, width=100)


def save(doc: dict, path) -> None:
    Path(path).write_text(dump(doc))
