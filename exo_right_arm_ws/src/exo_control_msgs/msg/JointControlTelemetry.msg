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
