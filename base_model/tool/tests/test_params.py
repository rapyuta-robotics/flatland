import math

import pytest
import yaml

from flatland_base_model_tool import params, stepper
from limits import LIMITED


def good_doc(**over):
    doc = {"schema_version": 1, "name": "t", "calibrated_step_s": 0.02,
           "parameters": dict(LIMITED), "provenance": {"robot": "x"}}
    doc.update(over)
    return doc


def test_roundtrip_with_inf(tmp_path):
    doc = params.make("identity", stepper.IDENTITY, None, {"note": "no shaping"})
    path = tmp_path / "identity.yaml"
    params.save(doc, path)
    assert ".inf" in path.read_text()
    back = params.load(path)
    assert back["parameters"]["a_lin_up"] == math.inf and back["calibrated_step_s"] is None


def test_missing_file(tmp_path):
    with pytest.raises(params.ParamFileError, match="cannot read"):
        params.load(tmp_path / "absent.yaml")


@pytest.mark.parametrize("doc, text", [
    (good_doc(schema_version=2), "schema_version"),
    (good_doc(schema_version="1"), "schema_version"),
    (good_doc(calibrated_step_s=0.0), "calibrated_step_s"),
    (good_doc(name=""), "name"),
    (good_doc(provenance=[1]), "provenance"),
    (good_doc(extra=1), "unknown top-level"),
    (good_doc(parameters={**LIMITED, "tau_lin_s": -1.0}), "tau_lin_s"),
    (good_doc(parameters={**LIMITED, "yaw_bias": 0.0}), "yaw_bias"),
    (good_doc(parameters={**LIMITED, "a_ang": "fast"}), "a_ang"),
    ([1, 2], "mapping"),
])
def test_rejects(doc, text):
    with pytest.raises(params.ParamFileError, match=text):
        params.validate(doc)


def test_missing_top_key():
    doc = good_doc()
    del doc["provenance"]
    with pytest.raises(params.ParamFileError, match="provenance"):
        params.validate(doc)


def test_invalid_yaml(tmp_path):
    p = tmp_path / "bad.yaml"
    p.write_text("a: [unclosed")
    with pytest.raises(params.ParamFileError, match="invalid YAML"):
        params.load(p)


def test_shipped_files_load():
    from pathlib import Path
    root = Path(__file__).resolve().parents[2] / "params"
    names = {p.stem for p in root.glob("*.yaml")}
    assert {"identity", "example"} <= names
    for p in root.glob("*.yaml"):
        assert yaml.safe_load(p.read_text())["name"] == p.stem
        params.load(p)
