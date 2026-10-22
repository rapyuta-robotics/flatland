"""Made-up, round parameter values for the tests (no robot's values)."""
from flatland_base_model_tool import stepper

# a model with rate limits and speed caps, no delay or lag
LIMITED = {
    "delay_s": 0.0, "tau_lin_s": 0.0, "tau_ang_s": 0.0,
    "a_lin_up": 0.8, "a_lin_dn": 1.2, "a_ang": 1.5,
    "jerk_lin": 20.0, "jerk_ang": 200.0,
    "yaw_scale": 1.0, "v_min": -0.5, "v_max": 1.5, "w_max": 2.0,
}
# a model with delay, lags, a yaw scale and a different pair of acceleration limits: the "true" model of the synthetic drives
TRUE_PARAMS = {**LIMITED, "delay_s": 0.06, "tau_ang_s": 0.2, "yaw_scale": 0.9, "a_lin_up": 1.1, "a_lin_dn": 2.0}
WHEEL_BASE = 0.2
assert set(LIMITED) == set(stepper.PARAM_NAMES)
