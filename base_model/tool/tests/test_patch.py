from pathlib import Path

import pytest
import yaml

from flatland_base_model_tool import cli, patch

DATA = Path(__file__).parent / "data"
MODEL = DATA / "generic_diffdrive.model.yaml"
EXAMPLE_DIFF = Path(__file__).resolve().parents[2] / "example" / "generic_diffdrive.base_model.diff"


def test_model_patch_changes_only_the_drive_entry():
    text = MODEL.read_text()
    out = patch.patch_text(text, params_name="example")
    before, after = yaml.safe_load(text), yaml.safe_load(out)
    drive = [p for p in after["plugins"] if p["type"] == "BaseModelDrive"]
    assert len(drive) == 1 and drive[0]["params_name"] == "example"
    assert drive[0]["body"] == "base" and drive[0]["pub_rate"] == 100
    assert after["bodies"] == before["bodies"] and after["joints"] == before["joints"]
    others_before = [p for p in before["plugins"] if p["type"] != "DiffDrive"]
    assert [p for p in after["plugins"] if p["type"] != "BaseModelDrive"] == others_before
    # the text diff is exactly one removed and two added lines
    changed = [l for l in patch.unified_diff(text, out).splitlines() if l[:1] in "+-" and l[:3] not in ("+++", "---")]
    assert changed == ["-  - type: DiffDrive", "+  - type: BaseModelDrive", "+    params_name: example"]


def test_example_diff_is_current():
    text = MODEL.read_text()
    out = patch.patch_text(text, params_name="example")
    assert EXAMPLE_DIFF.read_text() == patch.unified_diff(text, out, "generic_diffdrive.model.yaml")


def test_params_file_variant():
    out = patch.patch_text(MODEL.read_text(), params_file="$BASE_MODEL_PARAMS")
    assert "  params_file: $BASE_MODEL_PARAMS\n" in out


def test_refuses_dynamics_block():
    text = MODEL.read_text().replace("    pub_rate: 100", "    pub_rate: 100\n    linear_dynamics: {acceleration_limit: 1.0}")
    with pytest.raises(patch.PatchError, match="linear_dynamics"):
        patch.patch_text(text, params_name="x")


def test_refuses_already_patched_and_missing_and_double():
    text = MODEL.read_text()
    once = patch.patch_text(text, params_name="x")
    with pytest.raises(patch.PatchError, match="already"):
        patch.patch_text(once, params_name="x")
    with pytest.raises(patch.PatchError, match="no DiffDrive"):
        patch.patch_text("plugins:\n- type: Laser\n  name: l\n", params_name="x")
    with pytest.raises(patch.PatchError, match="more than one"):
        patch.patch_text("plugins:\n- type: DiffDrive\n  name: a\n- type: DiffDrive\n  name: b\n", params_name="x")


def test_needs_exactly_one_params_source():
    with pytest.raises(patch.PatchError):
        patch.patch_text(MODEL.read_text())
    with pytest.raises(patch.PatchError):
        patch.patch_text(MODEL.read_text(), params_name="a", params_file="b")


def test_type_not_first_key_and_qualified_name():
    text = "plugins:\n- name: d\n  type: flatland_plugins::DiffDrive  # drive\n  body: b\n"
    out = patch.patch_text(text, params_name="p")
    assert out == "plugins:\n- name: d\n  type: BaseModelDrive\n  params_name: p\n  body: b\n"


def test_cli_prints_diff_and_writes(tmp_path, capsys):
    assert cli.main(["patch-model", str(MODEL), "--params-name", "identity"]) == 0
    assert "+  - type: BaseModelDrive" in capsys.readouterr().out
    out = tmp_path / "m.yaml"
    assert cli.main(["patch-model", str(MODEL), "--params-name", "identity", "--out", str(out)]) == 0
    assert "BaseModelDrive" in out.read_text()
    assert cli.main(["patch-model", str(out), "--params-name", "identity"]) == 1
