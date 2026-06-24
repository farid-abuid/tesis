"""Paths and defaults for the EMG + encoder acquisition app."""
import os

# Delsys vendor repo (Example-Applications). Not copied into this repo: the
# DelsysAPI.dll, native .so libs, and key/license are loaded from here at runtime.
DELSYS_PYTHON_DIR = os.environ.get(
    "DELSYS_PYTHON_DIR",
    "/home/utec/Documents/farid/Example-Applications-main_vf/Example-Applications-main/Python",
)

# ROS 2 workspace with the exo stack.
EXO_WS = os.environ.get("EXO_WS", "/home/utec/tesis/exo_right_arm_ws")

# Target EMG sample rate [Hz]; the sensor mode whose EMG rate is closest to this
# is auto-selected after scanning (Avanti modes are fixed: 1259.259, 2148.148, ...).
TARGET_EMG_RATE_HZ = 2000.0

# Encoder stream (one arm at a time).
JOINT_STATE_TOPIC_FMT = "/{side}_joint_state_broadcaster/joint_states"
ENCODER_RATE_HZ = 500.0  # controller_manager update_rate (informational, for CSV header)

# Output directory for recordings (created on demand).
DEFAULT_OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")

# Windowed datasets (.npy) built from each recording on stop (see windowing.py).
DATASET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "datasets")

# Windowing params, replicating Sedighi et al. (RAL 2023) CNN-LSTM 30x6 input.
# Both EMG (2148 Hz) and joints (~500 Hz) are resampled to WINDOW_GRID_HZ, then
# sliced into WINDOW_MS windows. The model target is the joint-angle vector
# PREDICTION_HORIZON_MS into the future (paper uses 450 ms for its slow pneumatic
# plant; ours is far faster, so the default here is shorter).
WINDOW_GRID_HZ = 500.0           # common grid (paper: 2 ms / 500 Hz)
WINDOW_MS = 60.0                 # window length (paper: 60 ms -> 30 rows)
PREDICTION_HORIZON_MS = 150.0    # target horizon (0 = input-only, no target)
AUTO_WINDOW_ON_STOP = True       # build the dataset automatically when recording stops
