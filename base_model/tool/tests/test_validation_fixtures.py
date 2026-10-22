"""The validation world and models stay consistent with the generic test model and the model patch."""
from pathlib import Path

import yaml

from flatland_base_model_tool import patch

ROOT = Path(__file__).resolve().parents[2] / "validation"
GENERIC_TEXT = (Path(__file__).parent / "data" / "generic_diffdrive.model.yaml").read_text()
GENERIC = yaml.safe_load(GENERIC_TEXT)
NO_DRIVE_KEYS = {"enable_odom_pub", "enable_twist_pub", "enable_ground_truth_pub", "enable_tf_pub", "pub_rate",
                 "odom_pub", "odom_frame_id", "body", "name"}


def model(name):
    return yaml.safe_load((ROOT / "models" / f"{name}.model.yaml").read_text())


def drive_entry(doc):
    (entry,) = [p for p in doc["plugins"] if p["type"] in ("DiffDrive", "BaseModelDrive")]
    return entry


def test_bodies_and_joints_are_the_generic_ones():
    for name in ("generic_base_model", "generic_stock"):
        doc = model(name)
        assert doc["bodies"] == GENERIC["bodies"] and doc["joints"] == GENERIC["joints"]
        assert [p["type"] for p in doc["plugins"]] == [drive_entry(doc)["type"], "CommandPlayback"]
        playback = doc["plugins"][1]
        assert playback["playback_file"] == '$eval env("PLAYBACK_FILE")' and playback["playback_out"] == '$eval env("PLAYBACK_OUT")'


def test_base_model_drive_is_the_patched_stock_drive():
    stock = drive_entry(GENERIC)
    patched = yaml.safe_load(patch.patch_text(GENERIC_TEXT, params_file="$eval env(\"BASE_MODEL_PARAMS_FILE\")"))
    assert drive_entry(model("generic_base_model")) == drive_entry(patched)
    assert drive_entry(model("generic_stock")) == stock


def test_worlds_reference_existing_files():
    for kind, mdl in (("base_model", "generic_base_model"), ("stock", "generic_stock")):
        world = yaml.safe_load((ROOT / "world" / f"{kind}.world.yaml").read_text())
        (entry,) = world["models"]
        assert (ROOT / "models" / entry["model"]).name == f"{mdl}.model.yaml"
        assert (ROOT / "models" / entry["model"]).exists()
        assert entry["namespace"] == "robot" and entry["pose"] == [0, 0, 0]
        for layer in world["layers"]:
            if "map" in layer:
                assert (ROOT / "world" / layer["map"]).exists()


def test_map_is_blank_and_large():
    import struct
    import zlib

    m = yaml.safe_load((ROOT / "world" / "map.yaml").read_text())
    data = (ROOT / "world" / m["image"]).read_bytes()
    width, height, depth, color = struct.unpack(">IIBB", data[16:26])
    assert (width, height, depth, color) == (3200, 3200, 8, 0)          # 8-bit grey
    pos, idat = 8, b""
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        if data[pos + 4:pos + 8] == b"IDAT":
            idat += data[pos + 8:pos + 8 + length]
        pos += 12 + length
    raw = zlib.decompress(idat)
    assert len(raw) == height * (width + 1)
    import numpy as np

    rows = np.frombuffer(raw, np.uint8).reshape(height, width + 1)
    prev = np.zeros(width, np.uint8)
    for r in range(height):
        kind, row = rows[r, 0], rows[r, 1:]
        assert kind in (0, 1, 2)                 # the filters a flat image uses
        if kind == 1:
            row = np.cumsum(row, dtype=np.uint64).astype(np.uint8)
        elif kind == 2:
            row = (row + prev).astype(np.uint8)
        assert row.min() == 255
        prev = row
    half = width * m["resolution"] / 2
    assert m["origin"][:2] == [-half, -half]
    assert half >= 40      # a drive can start in the middle of the map and stay inside it for 40 m in any direction
