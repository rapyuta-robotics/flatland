#include "flatland_plugins/base_model_params.h"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <fstream>

using flatland_plugins::base_model::LoadParamFile;
using flatland_plugins::base_model::ParamFileError;
using flatland_plugins::base_model::ParseParamFile;

namespace {

const char* kGood = R"(
schema_version: 1
name: t
calibrated_step_s: 0.02
parameters:
  delay_s: 0.04
  tau_lin_s: 0.1
  tau_ang_s: 0.2
  a_lin_up: 0.8
  a_lin_dn: 1.2
  a_ang: 1.5
  jerk_lin: 20.0
  jerk_ang: 200.0
  yaw_scale: 0.9
  v_min: -0.5
  v_max: 1.5
  w_max: 2.0
provenance:
  robot: x
)";

YAML::Node Good() { return YAML::Load(kGood); }

void ExpectRefused(YAML::Node doc, const std::string& text) {
  try {
    ParseParamFile(doc);
    ADD_FAILURE() << "accepted a file that should be refused: " << text;
  } catch (const ParamFileError& e) {
    EXPECT_NE(std::string(e.what()).find(text), std::string::npos) << e.what();
  }
}

}  // namespace

TEST(ParamsIo, ParsesAGoodFile) {
  const auto f = ParseParamFile(Good());
  EXPECT_EQ(f.name, "t");
  EXPECT_DOUBLE_EQ(f.calibrated_step_s, 0.02);
  EXPECT_DOUBLE_EQ(f.params.a_lin_dn, 1.2);
  EXPECT_EQ(f.provenance["robot"].as<std::string>(), "x");
  EXPECT_NE(flatland_plugins::base_model::Describe(f).find("robot: x"),
            std::string::npos);
}

TEST(ParamsIo, InfAndNullCalibratedStep) {
  YAML::Node d = Good();
  d["calibrated_step_s"] = YAML::Node(YAML::NodeType::Null);
  d["parameters"]["a_lin_up"] = ".inf";
  d["parameters"]["v_min"] = "-.inf";
  const auto f = ParseParamFile(d);
  EXPECT_EQ(f.calibrated_step_s, 0.0);
  EXPECT_TRUE(std::isinf(f.params.a_lin_up));
  EXPECT_TRUE(std::isinf(f.params.v_min) && f.params.v_min < 0);
}

TEST(ParamsIo, RefusesUnknownSchemaVersion) {
  YAML::Node d = Good();
  d["schema_version"] = 2;
  ExpectRefused(d, "schema_version");
  d["schema_version"] = "one";
  ExpectRefused(d, "schema_version");
  YAML::Node none = Good();
  none.remove("schema_version");
  ExpectRefused(none, "schema_version");
}

TEST(ParamsIo, RefusesBadStructure) {
  YAML::Node d = Good();
  d["extra"] = 1;
  ExpectRefused(d, "unknown top-level");
  d = Good();
  d["calibrated_step_s"] = 0.0;
  ExpectRefused(d, "calibrated_step_s");
  d = Good();
  d["parameters"]["yaw_bias"] = 0.1;
  ExpectRefused(d, "yaw_bias");
  d = Good();
  d["parameters"].remove("a_ang");
  ExpectRefused(d, "a_ang");
  d = Good();
  d["parameters"]["a_ang"] = "fast";
  ExpectRefused(d, "a_ang");
  d = Good();
  d["parameters"]["tau_lin_s"] = -1.0;
  ExpectRefused(d, "tau_lin_s");
  d = Good();
  d["provenance"] = 3;
  ExpectRefused(d, "provenance");
  ExpectRefused(YAML::Load("[1, 2]"), "mapping");
}

TEST(ParamsIo, MissingFileAndBadYaml) {
  try {
    LoadParamFile("/nonexistent/base_model.yaml");
    FAIL();
  } catch (const ParamFileError& e) {
    EXPECT_NE(std::string(e.what()).find("cannot read"), std::string::npos);
  }
  const std::string path = ::testing::TempDir() + "bad.yaml";
  std::ofstream(path) << "a: [unclosed";
  try {
    LoadParamFile(path);
    FAIL();
  } catch (const ParamFileError& e) {
    EXPECT_NE(std::string(e.what()).find("invalid YAML"), std::string::npos);
  }
}

TEST(ParamsIo, ShippedFilesLoad) {
  for (const char* name :
       {"identity", "example"}) {
    const auto f =
        LoadParamFile(std::string(FBM_PARAMS_DIR) + "/" + name + ".yaml");
    EXPECT_EQ(f.name, name);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
