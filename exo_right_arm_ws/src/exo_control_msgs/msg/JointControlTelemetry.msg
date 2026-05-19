# Generic joint-space control telemetry for loggers, observers, and downstream tools.
std_msgs/Header header
string[] joint_names
float64[] position
float64[] velocity
float64[] position_reference
float64[] velocity_reference
float64[] effort_command
# Model- or feedforward contribution (e.g. gravity, friction model); empty if unknown
float64[] effort_feedforward
# Reported/estimated joint effort from hardware/state; empty if unavailable
float64[] effort_measured
# Optional Cartesian operational point (e.g. EE), length 0 or 3 each
float64[] operational_position_actual
float64[] operational_position_desired

# --- Sliding-mode / adaptive control diagnostics (all empty/0 when N/A) ---
# Per-joint sliding variable s used by the control law.
float64[] sliding_surface
# Per-joint integral-of-error state ei (integral SMC only); empty otherwise.
float64[] integral_error
# Sliding-surface slope lambda, so the s=0 line can be drawn in the phase plane.
float64 sliding_lambda
# Online adaptive parameter estimate theta_hat (adaptive controllers).
float64[] adaptive_parameters
# Seed/initial value theta_hat0 (same length as adaptive_parameters).
float64[] adaptive_parameters_initial
