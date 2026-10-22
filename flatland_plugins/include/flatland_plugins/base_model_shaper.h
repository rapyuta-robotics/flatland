// The discrete maths of the base model. No ROS, no flatland: it builds and runs
// under plain gtest.
//
// This is the C++ twin of
// base_model/tool/src/flatland_base_model_tool/stepper.py. Both must reproduce
// base_model/test_vectors/stepper.json within 1e-9; keep the order of
// operations identical when changing either.
//
// Per step of length dt, from the command current at the step:
//  1. Delay: the model sees the command from n = floor(delay_s / dt + 0.5)
//  steps ago (zero before the start).
//  2. Lag, per axis: lag += a * (u - lag), a = 1 - exp(-dt / tau); lag = u if
//  tau <= 0.
//  3. Rate limits, per axis: clip the lagged target to the speed caps, move
//  toward it by an acceleration
//     clipped by the signed acceleration limits and then by the jerk limit; a
//     step that reaches the target ends on it exactly.
//  4. Zero snap: an axis whose delayed input is exactly 0 and whose speed is
//  below 1e-3 is set to exactly 0
//     (with its lag and acceleration state).
//  5. Output (v, yaw_scale * w).
#ifndef FLATLAND_PLUGINS_BASE_MODEL_SHAPER_H
#define FLATLAND_PLUGINS_BASE_MODEL_SHAPER_H

#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace flatland_plugins {
namespace base_model {

constexpr double kSnapV = 1e-3;  // m/s
constexpr double kSnapW = 1e-3;  // rad/s
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Params {
  double delay_s = 0.0;
  double tau_lin_s = 0.0;
  double tau_ang_s = 0.0;
  double a_lin_up = kInf;  // limit on dv/dt > 0
  double a_lin_dn = kInf;  // limit on -dv/dt > 0
  double a_ang = kInf;
  double jerk_lin = kInf;
  double jerk_ang = kInf;
  double yaw_scale = 1.0;
  double v_min = -kInf;
  double v_max = kInf;
  double w_max = kInf;
};

// Same rules as stepper.validate_params. Throws std::invalid_argument.
inline void Validate(const Params& p) {
  struct Named {
    const char* name;
    double value;
  };
  const Named all[] = {{"delay_s", p.delay_s},     {"tau_lin_s", p.tau_lin_s},
                       {"tau_ang_s", p.tau_ang_s}, {"a_lin_up", p.a_lin_up},
                       {"a_lin_dn", p.a_lin_dn},   {"a_ang", p.a_ang},
                       {"jerk_lin", p.jerk_lin},   {"jerk_ang", p.jerk_ang},
                       {"yaw_scale", p.yaw_scale}, {"v_min", p.v_min},
                       {"v_max", p.v_max},         {"w_max", p.w_max}};
  for (const Named& n : all) {
    if (std::isnan(n.value))
      throw std::invalid_argument(std::string(n.name) + " is NaN");
  }
  const Named finite[] = {{"delay_s", p.delay_s},
                          {"tau_lin_s", p.tau_lin_s},
                          {"tau_ang_s", p.tau_ang_s},
                          {"yaw_scale", p.yaw_scale}};
  for (const Named& n : finite) {
    if (!std::isfinite(n.value))
      throw std::invalid_argument(std::string(n.name) + " must be finite");
  }
  const Named nonneg[] = {{"delay_s", p.delay_s},
                          {"tau_lin_s", p.tau_lin_s},
                          {"tau_ang_s", p.tau_ang_s}};
  for (const Named& n : nonneg) {
    if (n.value < 0.0)
      throw std::invalid_argument(std::string(n.name) + " must be >= 0");
  }
  const Named positive[] = {
      {"a_lin_up", p.a_lin_up}, {"a_lin_dn", p.a_lin_dn},
      {"a_ang", p.a_ang},       {"jerk_lin", p.jerk_lin},
      {"jerk_ang", p.jerk_ang}, {"yaw_scale", p.yaw_scale},
      {"w_max", p.w_max}};
  for (const Named& n : positive) {
    if (!(n.value > 0.0))
      throw std::invalid_argument(std::string(n.name) + " must be > 0");
  }
  if (p.v_min > 0.0) throw std::invalid_argument("v_min must be <= 0");
  if (p.v_max < 0.0) throw std::invalid_argument("v_max must be >= 0");
}

inline long DelaySteps(double delay_s, double dt) {
  return static_cast<long>(std::floor(delay_s / dt + 0.5));
}

// One rate-limited axis step.
inline void AxisStep(double& x, double& a, double target, double dt,
                     double acc_pos, double acc_neg, double jerk, double lo,
                     double hi) {
  if (target < lo) {
    target = lo;
  } else if (target > hi) {
    target = hi;
  }
  const double a_raw = (target - x) / dt;
  double a_des = a_raw;
  if (a_des > acc_pos) {
    a_des = acc_pos;
  } else if (a_des < -acc_neg) {
    a_des = -acc_neg;
  }
  double a_new;
  if (std::isinf(jerk)) {
    a_new = a_des;
  } else {
    double da = a_des - a;
    const double lim = jerk * dt;
    if (da > lim) {
      da = lim;
    } else if (da < -lim) {
      da = -lim;
    }
    a_new = a + da;
  }
  double x_new = x + a_new * dt;
  if ((x < target && target <= x_new) || (x_new <= target && target < x) ||
      a_new == a_raw) {
    x_new = target;
  }
  if (x_new < lo) {
    x_new = lo;
  } else if (x_new > hi) {
    x_new = hi;
  }
  a = (x_new - x) / dt;
  x = x_new;
}

class Shaper {
 public:
  explicit Shaper(const Params& p) : p_(p) { Validate(p_); }

  // Back to rest, with an empty command history.
  void Reset() {
    hist_.clear();
    lag_v_ = lag_w_ = v_ = av_ = w_ = aw_ = 0.0;
  }

  // `cmd_*` is the command current at this step. Returns the (v, w) to apply
  // and to report.
  void Step(double cmd_v, double cmd_w, double dt, double& out_v,
            double& out_w) {
    const long nd = DelaySteps(p_.delay_s, dt);
    hist_.emplace_back(cmd_v, cmd_w);
    while (static_cast<long>(hist_.size()) > nd + 1) hist_.pop_front();
    double u_v = 0.0, u_w = 0.0;
    if (static_cast<long>(hist_.size()) > nd) {
      const std::pair<double, double>& u = hist_.front();
      u_v = u.first;
      u_w = u.second;
    }
    if (p_.tau_lin_s <= 0.0) {
      lag_v_ = u_v;
    } else {
      lag_v_ += (1.0 - std::exp(-dt / p_.tau_lin_s)) * (u_v - lag_v_);
    }
    if (p_.tau_ang_s <= 0.0) {
      lag_w_ = u_w;
    } else {
      lag_w_ += (1.0 - std::exp(-dt / p_.tau_ang_s)) * (u_w - lag_w_);
    }
    AxisStep(v_, av_, lag_v_, dt, p_.a_lin_up, p_.a_lin_dn, p_.jerk_lin,
             p_.v_min, p_.v_max);
    AxisStep(w_, aw_, lag_w_, dt, p_.a_ang, p_.a_ang, p_.jerk_ang, -p_.w_max,
             p_.w_max);
    if (u_v == 0.0 && std::fabs(v_) < kSnapV) lag_v_ = v_ = av_ = 0.0;
    if (u_w == 0.0 && std::fabs(w_) < kSnapW) lag_w_ = w_ = aw_ = 0.0;
    out_v = v_;
    out_w = p_.yaw_scale * w_;
  }

  const Params& params() const { return p_; }

 private:
  Params p_;
  std::deque<std::pair<double, double>> hist_;
  double lag_v_ = 0.0, lag_w_ = 0.0, v_ = 0.0, av_ = 0.0, w_ = 0.0, aw_ = 0.0;
};

}  // namespace base_model
}  // namespace flatland_plugins

#endif  // FLATLAND_PLUGINS_BASE_MODEL_SHAPER_H
