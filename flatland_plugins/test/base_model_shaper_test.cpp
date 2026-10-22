#include "flatland_plugins/base_model_shaper.h"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <vector>

#include "flatland_plugins/base_model_params.h"

using flatland_plugins::base_model::kInf;
using flatland_plugins::base_model::Params;
using flatland_plugins::base_model::Shaper;

namespace {

// Made-up limits for the tests; the model has no built-in limits.
Params Limited() {
  Params p;
  p.a_lin_up = 0.8;
  p.a_lin_dn = 1.2;
  p.a_ang = 1.5;
  p.jerk_lin = 20.0;
  p.jerk_ang = 200.0;
  p.v_min = -0.5;
  p.v_max = 1.5;
  p.w_max = 2.0;
  return p;
}

Params Unlimited() { return Params(); }

std::vector<double> RunShaper(const Params& p, const std::vector<double>& cv,
                        const std::vector<double>& cw, double dt,
                        std::vector<double>* out_w = nullptr) {
  Shaper s(p);
  std::vector<double> ov(cv.size()), ow(cv.size());
  for (size_t k = 0; k < cv.size(); ++k) s.Step(cv[k], cw[k], dt, ov[k], ow[k]);
  if (out_w) *out_w = ow;
  return ov;
}

std::vector<double> Seq(const YAML::Node& n) {
  std::vector<double> v;
  v.reserve(n.size());
  for (const auto& x : n) v.push_back(x.as<double>());
  return v;
}

Params ParamsOf(const YAML::Node& n) {
  flatland_plugins::base_model::ParamFile f;
  YAML::Node doc;
  doc["schema_version"] = 1;
  doc["name"] = "vector";
  doc["calibrated_step_s"] = YAML::Node(YAML::NodeType::Null);
  doc["parameters"] = n;
  doc["provenance"] = YAML::Node(YAML::NodeType::Map);
  return flatland_plugins::base_model::ParseParamFile(doc).params;
}

}  // namespace

TEST(TestVectors, MatchReferenceStepper) {
  const YAML::Node doc = YAML::LoadFile(FBM_TEST_VECTORS);
  const double tol = doc["tolerance"].as<double>();
  ASSERT_EQ(doc["cases"].size(), 60u);
  for (const auto& c : doc["cases"]) {
    const std::string name = c["name"].as<std::string>();
    const double dt = c["dt"].as<double>();
    const Params p = ParamsOf(c["params"]);
    const std::vector<double> cv = Seq(c["cmd_v"]), cw = Seq(c["cmd_w"]);
    const std::vector<double> ev = Seq(c["out_v"]), ew = Seq(c["out_w"]);
    std::vector<double> ow;
    const std::vector<double> ov = RunShaper(p, cv, cw, dt, &ow);
    ASSERT_EQ(ov.size(), ev.size()) << name;
    double worst = 0.0;
    for (size_t k = 0; k < ov.size(); ++k) {
      worst = std::max(
          worst, std::max(std::fabs(ov[k] - ev[k]), std::fabs(ow[k] - ew[k])));
    }
    EXPECT_LE(worst, tol) << name;
  }
}

TEST(Shaper, IdentityReproducesCommandExactly) {
  const std::vector<double> cv = {0.0, 1.0, -2.5, 3.0, 0.3},
                            cw = {0.0, 0.5, -7.0, 0.0, 1e-4};
  std::vector<double> ow;
  const std::vector<double> ov = RunShaper(Unlimited(), cv, cw, 0.02, &ow);
  for (size_t k = 0; k < cv.size(); ++k) {
    EXPECT_EQ(ov[k], cv[k]);
    EXPECT_EQ(ow[k], cw[k]);
  }
}

TEST(Shaper, RateLimitRamp) {
  Params p = Unlimited();
  p.a_lin_up = 1.0;
  const std::vector<double> ov = RunShaper(p, std::vector<double>(100, 1.0),
                                     std::vector<double>(100, 0.0), 0.02);
  EXPECT_NEAR(ov[0], 0.02, 1e-12);
  EXPECT_NEAR(ov[24], 0.5, 1e-12);
  EXPECT_DOUBLE_EQ(ov[99], 1.0);
}

TEST(Shaper, ExactZeroAtRestAfterLagTail) {
  Params p = Unlimited();
  p.tau_lin_s = 0.3;
  p.tau_ang_s = 0.3;
  std::vector<double> cv(1000, 0.0);
  for (int k = 0; k < 50; ++k) cv[k] = 1.0;
  std::vector<double> ow;
  const std::vector<double> ov = RunShaper(p, cv, cv, 0.02, &ow);
  EXPECT_EQ(ov.back(), 0.0);
  EXPECT_EQ(ow.back(), 0.0);
  size_t first_zero = 0;
  for (size_t k = 51; k < ov.size(); ++k) {
    if (ov[k] == 0.0) {
      first_zero = k;
      break;
    }
  }
  ASSERT_GT(first_zero, 0u);
  for (size_t k = first_zero; k < ov.size(); ++k) EXPECT_EQ(ov[k], 0.0);
  EXPECT_GE(ov[first_zero - 1], 1e-3);
}

TEST(Shaper, NoSnapWhileCommandNonZero) {
  const std::vector<double> ov =
      RunShaper(Unlimited(), std::vector<double>(300, 1e-4),
          std::vector<double>(300, 0.0), 0.02);
  EXPECT_EQ(ov.back(), 1e-4);
}

TEST(Shaper, DelayRoundsToWholeSteps) {
  struct Case {
    double delay, dt;
    int steps;
  };
  for (const Case& c :
       {Case{0.1, 0.02, 5}, Case{0.11, 0.02, 6}, Case{0.03, 0.04, 1},
        Case{0.019, 0.04, 0}, Case{0.1, 0.08, 1}}) {
    EXPECT_EQ(flatland_plugins::base_model::DelaySteps(c.delay, c.dt), c.steps);
    Params p = Unlimited();
    p.delay_s = c.delay;
    std::vector<double> cv(65, 1.0);
    for (int k = 0; k < 5; ++k) cv[k] = 0.0;
    const std::vector<double> ov =
        RunShaper(p, cv, std::vector<double>(65, 0.0), c.dt);
    size_t first = 0;
    while (ov[first] == 0.0) ++first;
    EXPECT_EQ(first, static_cast<size_t>(5 + c.steps));
  }
}

TEST(Shaper, YawScaleAppliesToOutputOnly) {
  Params p = Unlimited();
  p.yaw_scale = 0.9;
  std::vector<double> ow;
  RunShaper(p, std::vector<double>(200, 0.0), std::vector<double>(200, 2.0), 0.02,
      &ow);
  EXPECT_NEAR(ow.back(), 1.8, 1e-12);
}

TEST(Shaper, SpeedCaps) {
  Params p = Unlimited();
  p.v_max = 0.5;
  p.v_min = -0.2;
  p.w_max = 1.0;
  std::vector<double> ow;
  std::vector<double> ov = RunShaper(p, std::vector<double>(400, 3.0),
                               std::vector<double>(400, -7.0), 0.02, &ow);
  EXPECT_EQ(ov.back(), 0.5);
  EXPECT_EQ(ow.back(), -1.0);
  ov = RunShaper(p, std::vector<double>(400, -3.0), std::vector<double>(400, 0.0),
           0.02);
  EXPECT_EQ(ov.back(), -0.2);
}

TEST(Shaper, ResetClearsStateAndHistory) {
  Params p = Limited();
  p.delay_s = 0.1;
  p.tau_lin_s = 0.2;
  Shaper s(p);
  double v = 0, w = 0;
  for (int k = 0; k < 100; ++k) s.Step(1.0, 1.0, 0.02, v, w);
  EXPECT_GT(v, 0.5);
  s.Reset();
  s.Step(0.0, 0.0, 0.02, v, w);
  EXPECT_EQ(v, 0.0);
  EXPECT_EQ(w, 0.0);
  // the old command must not come back through the delay line
  for (int k = 0; k < 10; ++k) s.Step(0.0, 0.0, 0.02, v, w);
  EXPECT_EQ(v, 0.0);
}

TEST(Shaper, StepSizeIndependenceOfAPureLag) {
  Params p = Unlimited();
  p.tau_lin_s = 0.2;
  double at[3];
  int i = 0;
  for (double dt : {0.02, 0.04, 0.08}) {
    const int n = static_cast<int>(std::lround(1.0 / dt));
    const std::vector<double> ov =
        RunShaper(p, std::vector<double>(n, 1.0), std::vector<double>(n, 0.0), dt);
    at[i++] = ov[static_cast<int>(std::lround(0.4 / dt)) - 1];
  }
  EXPECT_NEAR(at[0], at[1], 1e-12);
  EXPECT_NEAR(at[0], at[2], 1e-12);
  EXPECT_NEAR(at[0], 1.0 - std::exp(-0.4 / 0.2), 1e-12);
}

TEST(Validate, RejectsWhatPythonRejects) {
  struct Case {
    void (*mutate)(Params&);
    const char* text;
  };
  const Case cases[] = {
      {[](Params& p) { p.tau_lin_s = -0.1; }, "tau_lin_s"},
      {[](Params& p) { p.delay_s = std::nan(""); }, "NaN"},
      {[](Params& p) { p.a_lin_up = 0.0; }, "a_lin_up"},
      {[](Params& p) { p.yaw_scale = 0.0; }, "yaw_scale"},
      {[](Params& p) { p.v_min = 0.5; }, "v_min"},
      {[](Params& p) { p.v_max = -0.5; }, "v_max"},
      {[](Params& p) { p.w_max = 0.0; }, "w_max"},
      {[](Params& p) { p.delay_s = kInf; }, "delay_s"},
  };
  for (const Case& c : cases) {
    Params p = Limited();
    c.mutate(p);
    try {
      flatland_plugins::base_model::Validate(p);
      ADD_FAILURE() << "accepted a bad value for " << c.text;
    } catch (const std::invalid_argument& e) {
      EXPECT_NE(std::string(e.what()).find(c.text), std::string::npos)
          << e.what();
    }
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
