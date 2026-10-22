// Parameter-file loading and validation. Schema and rules are those of
// base_model/tool/src/flatland_base_model_tool/params.py.
//
//   schema_version: 1
//   name: example
//   calibrated_step_s: 0.02           # or null
//   parameters: {delay_s: ..., ...}   # exactly the twelve parameters; .inf is
//   allowed where unlimited provenance: {...}                 # any mapping
#ifndef FLATLAND_PLUGINS_BASE_MODEL_PARAMS_H
#define FLATLAND_PLUGINS_BASE_MODEL_PARAMS_H

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

#include "flatland_plugins/base_model_shaper.h"

namespace flatland_plugins {
namespace base_model {

constexpr int kSchemaVersion = 1;

class ParamFileError : public std::runtime_error {
 public:
  explicit ParamFileError(const std::string& msg) : std::runtime_error(msg) {}
};

struct ParamFile {
  std::string name;
  double calibrated_step_s = 0.0;  // 0 when the file is valid at any step size
  Params params;
  YAML::Node provenance;
};

namespace detail {

inline double Number(const YAML::Node& n, const std::string& what) {
  if (!n.IsScalar()) throw ParamFileError(what + " must be a number");
  const std::string s = n.Scalar();
  if (s == "true" || s == "false")
    throw ParamFileError(what + " must be a number");
  try {
    return n.as<double>();
  } catch (const YAML::Exception&) {
    throw ParamFileError(what + " must be a number");
  }
}

}  // namespace detail

inline ParamFile ParseParamFile(const YAML::Node& doc) {
  if (!doc.IsMap()) throw ParamFileError("parameter file must be a mapping");
  const YAML::Node version = doc["schema_version"];
  int v = 0;
  bool ok = version.IsDefined() && version.IsScalar();
  if (ok) {
    try {
      v = version.as<int>();
    } catch (const YAML::Exception&) {
      ok = false;
    }
  }
  if (!ok || v != kSchemaVersion) {
    throw ParamFileError(
        "unsupported schema_version " +
        (version.IsDefined() && version.IsScalar() ? version.Scalar()
                                                   : std::string("(none)")) +
        "; this version reads " + std::to_string(kSchemaVersion));
  }
  const std::set<std::string> top = {"schema_version", "name",
                                     "calibrated_step_s", "parameters",
                                     "provenance"};
  for (const auto& kv : doc) {
    const std::string k = kv.first.as<std::string>();
    if (!top.count(k)) throw ParamFileError("unknown top-level key(s): " + k);
  }
  for (const char* k :
       {"name", "calibrated_step_s", "parameters", "provenance"}) {
    if (!doc[k].IsDefined())
      throw ParamFileError(std::string("missing key '") + k + "'");
  }
  ParamFile out;
  if (!doc["name"].IsScalar() || doc["name"].Scalar().empty())
    throw ParamFileError("name must be a non-empty string");
  out.name = doc["name"].Scalar();
  const YAML::Node step = doc["calibrated_step_s"];
  if (step.IsNull()) {
    out.calibrated_step_s = 0.0;
  } else {
    const double s = detail::Number(step, "calibrated_step_s");
    if (!std::isfinite(s) || s <= 0.0)
      throw ParamFileError(
          "calibrated_step_s must be a positive number or null");
    out.calibrated_step_s = s;
  }
  if (!doc["provenance"].IsMap())
    throw ParamFileError("provenance must be a mapping");
  out.provenance = doc["provenance"];
  const YAML::Node values = doc["parameters"];
  if (!values.IsMap()) throw ParamFileError("parameters must be a mapping");

  struct Field {
    const char* name;
    double Params::* member;
  };
  const Field fields[] = {
      {"delay_s", &Params::delay_s},     {"tau_lin_s", &Params::tau_lin_s},
      {"tau_ang_s", &Params::tau_ang_s}, {"a_lin_up", &Params::a_lin_up},
      {"a_lin_dn", &Params::a_lin_dn},   {"a_ang", &Params::a_ang},
      {"jerk_lin", &Params::jerk_lin},   {"jerk_ang", &Params::jerk_ang},
      {"yaw_scale", &Params::yaw_scale}, {"v_min", &Params::v_min},
      {"v_max", &Params::v_max},         {"w_max", &Params::w_max}};
  std::set<std::string> known;
  for (const Field& f : fields) known.insert(f.name);
  for (const auto& kv : values) {
    const std::string k = kv.first.as<std::string>();
    detail::Number(kv.second, "parameter '" + k + "'");
    if (!known.count(k)) throw ParamFileError("unknown parameter(s): " + k);
  }
  for (const Field& f : fields) {
    if (!values[f.name].IsDefined())
      throw ParamFileError(std::string("missing parameter(s): ") + f.name);
    out.params.*(f.member) = detail::Number(values[f.name], f.name);
  }
  try {
    Validate(out.params);
  } catch (const std::invalid_argument& e) {
    throw ParamFileError(e.what());
  }
  return out;
}

inline ParamFile LoadParamFile(const std::string& path) {
  YAML::Node doc;
  try {
    doc = YAML::LoadFile(path);
  } catch (const YAML::BadFile&) {
    throw ParamFileError("cannot read parameter file " + path);
  } catch (const YAML::Exception& e) {
    throw ParamFileError(path + ": invalid YAML: " + e.what());
  }
  return ParseParamFile(doc);
}

// Name, step size, values and provenance as one YAML string: logged at load and
// published latched.
inline std::string Describe(const ParamFile& f) {
  YAML::Emitter e;
  e << YAML::BeginMap;
  e << YAML::Key << "name" << YAML::Value << f.name;
  e << YAML::Key << "calibrated_step_s" << YAML::Value << f.calibrated_step_s;
  e << YAML::Key << "provenance" << YAML::Value << f.provenance;
  e << YAML::EndMap;
  return e.c_str();
}

}  // namespace base_model
}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_BASE_MODEL_PARAMS_H
