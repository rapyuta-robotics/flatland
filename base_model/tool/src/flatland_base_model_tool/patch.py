"""Model patch: swap a flatland model's DiffDrive block for BaseModelDrive.

The change is one edited line and one added line in the drive plugin entry:

    - type: DiffDrive             ->   - type: BaseModelDrive
                                         params_name: example

Everything else in the model file, including comments and other plugins, stays byte for byte. The result is
parsed and compared with the original to prove that nothing but that plugin entry changed.
"""
from __future__ import annotations

import copy
import difflib
import re

import yaml

TYPE_LINE = re.compile(r"^(?P<indent>[ \t]*(?:-[ \t]+)?)type:[ \t]*['\"]?(?:flatland_plugins::)?DiffDrive['\"]?[ \t]*(?:#.*)?$")
FORBIDDEN = ("linear_dynamics", "angular_dynamics")


class PatchError(ValueError):
    """The model file cannot be patched, or the patched result did not verify."""


def _plugins(doc) -> list:
    if not isinstance(doc, dict) or not isinstance(doc.get("plugins"), list):
        raise PatchError("model file has no plugins list")
    return doc["plugins"]


def _of_type(doc, names) -> list:
    return [p for p in _plugins(doc) if isinstance(p, dict) and p.get("type") in names]


def patch_text(text: str, params_name: str | None = None, params_file: str | None = None) -> str:
    """Return the patched model text. Exactly one of `params_name` (a shipped file) and `params_file`."""
    if (params_name is None) == (params_file is None):
        raise PatchError("give exactly one of params_name and params_file")
    doc = yaml.safe_load(text)
    drives = _of_type(doc, ("DiffDrive", "flatland_plugins::DiffDrive"))
    if not drives:
        if _of_type(doc, ("BaseModelDrive", "flatland_plugins::BaseModelDrive")):
            raise PatchError("model already uses BaseModelDrive")
        raise PatchError("model has no DiffDrive plugin")
    if len(drives) > 1:
        raise PatchError("model has more than one DiffDrive plugin; patch it by hand")
    bad = [k for k in FORBIDDEN if k in drives[0]]
    if bad:
        raise PatchError(f"the DiffDrive block has {', '.join(bad)}; BaseModelDrive refuses these. "
                         "Remove them first: the parameter file carries the dynamics")
    lines = text.splitlines(keepends=True)
    hits = [i for i, line in enumerate(lines) if TYPE_LINE.match(line.rstrip("\r\n"))]
    if len(hits) != 1:
        raise PatchError("could not find the DiffDrive `type:` line (flow-style or unusual layout); patch it by hand")
    i = hits[0]
    m = TYPE_LINE.match(lines[i].rstrip("\r\n"))
    eol = lines[i][len(lines[i].rstrip("\r\n")):] or "\n"
    key_indent = " " * len(m.group("indent").expandtabs(2))
    lines[i] = f"{m.group('indent')}type: BaseModelDrive{eol}"
    key, value = ("params_name", params_name) if params_name is not None else ("params_file", params_file)
    lines.insert(i + 1, f"{key_indent}{key}: {value}{eol}")
    out = "".join(lines)
    verify(text, out, key, value)
    return out


def verify(original: str, patched: str, key: str, value: str) -> None:
    """Prove the patched text differs from the original only by the drive entry."""
    before, after = yaml.safe_load(original), yaml.safe_load(patched)
    if _of_type(after, ("DiffDrive", "flatland_plugins::DiffDrive")):
        raise PatchError("verification failed: a DiffDrive plugin remains")
    new = _of_type(after, ("BaseModelDrive",))
    if len(new) != 1 or new[0].get(key) != value:
        raise PatchError("verification failed: the BaseModelDrive entry is missing or wrong")
    expected = copy.deepcopy(before)
    for p in _plugins(expected):
        if isinstance(p, dict) and p.get("type") in ("DiffDrive", "flatland_plugins::DiffDrive"):
            p["type"] = "BaseModelDrive"
            p[key] = value
    if expected != after:
        raise PatchError("verification failed: the patch changed more than the drive entry")
    if patched.count("\n") - original.count("\n") != 1:
        raise PatchError("verification failed: the patch changed the line count by more than one")


def unified_diff(original: str, patched: str, name: str = "model.yaml") -> str:
    return "".join(difflib.unified_diff(original.splitlines(keepends=True), patched.splitlines(keepends=True),
                                        f"a/{name}", f"b/{name}"))
