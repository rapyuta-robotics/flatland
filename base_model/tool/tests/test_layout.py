"""The base model keeps working from its place in the flatland repository: the vectors, the parameter files and the
plugin sources are where the gtests, the plugin and the tool expect them."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from flatland_base_model_tool import params, stepper, vectors

BASE = Path(__file__).resolve().parents[2]          # base_model/
PLUGINS = BASE.parent / "flatland_plugins"


def test_vectors_load_from_this_directory_and_match_the_stepper():
    path = BASE / "test_vectors" / "stepper.json"
    doc = json.loads(path.read_text())
    assert len(doc["cases"]) == 60 and doc["tolerance"] == 1e-9
    case = doc["cases"][7]
    p = stepper.vector(vectors.decode_params(case["params"]))
    ov, ow = stepper.run(p, case["cmd_v"], case["cmd_w"], case["dt"])
    assert np.max(np.abs(ov - case["out_v"])) <= doc["tolerance"] and np.max(np.abs(ow - case["out_w"])) <= doc["tolerance"]


def test_the_plugin_package_reaches_the_parameter_files_through_its_link():
    link = PLUGINS / "base_model_params"
    assert link.is_symlink() and link.resolve() == (BASE / "params").resolve()
    for name in ("example", "identity"):
        assert params.load(link / f"{name}.yaml")["name"] == name


def test_the_gtests_find_their_inputs():
    """CMake passes these paths to the gtests as compile definitions."""
    cmake = (PLUGINS / "CMakeLists.txt").read_text()
    assert "../base_model/test_vectors/stepper.json" in cmake and "../base_model/params" in cmake
    assert (BASE / "test_vectors" / "stepper.json").exists()
    for t in ("base_model_shaper_test", "base_model_params_test", "command_playback_schedule_test"):
        assert (PLUGINS / "test" / f"{t}.cpp").exists() and t in cmake


def test_plugins_are_registered_and_diff_drive_is_still_there():
    root = ET.parse(PLUGINS / "flatland_plugins.xml").getroot()
    types = {c.get("type") for c in root.iter("class")}
    assert {"flatland_plugins::BaseModelDrive", "flatland_plugins::CommandPlayback", "flatland_plugins::DiffDrive"} <= types
    for src in ("base_model_drive", "command_playback"):
        assert f"src/{src}.cpp" in (PLUGINS / "CMakeLists.txt").read_text()
        assert (PLUGINS / "src" / f"{src}.cpp").exists()


def test_catkin_does_not_see_this_directory_as_a_package():
    assert not list(BASE.rglob("package.xml"))
