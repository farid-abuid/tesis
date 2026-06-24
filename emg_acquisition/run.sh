#!/bin/bash
# Launch the EMG + encoder acquisition GUI.
#
# sg dialout gives serial-port group access for the Trigno base USB, but:
#  - sg runs its command with sh (not bash), so `source` is unavailable, and
#  - sg is setgid, so the loader strips LD_LIBRARY_PATH from its environment.
# Hence the two-stage layout: the outer run re-execs itself under
# `sg dialout -c "bash run.sh"`, and the inner run (marker env var set, which
# does survive sg) sources ROS and sets LD_LIBRARY_PATH before starting python.
cd "$(dirname "$0")"

export DELSYS_PYTHON_DIR="${DELSYS_PYTHON_DIR:-/home/utec/Documents/farid/Example-Applications-main_vf/Example-Applications-main/Python}"
export EXO_WS="${EXO_WS:-/home/utec/tesis/exo_right_arm_ws}"

if [ -z "$EMG_ACQ_INNER" ]; then
    if [ ! -d venv ]; then
        echo "venv not found - running setup_venv.sh first"
        ./setup_venv.sh
    fi
    EMG_ACQ_INNER=1 exec sg dialout -c "bash '$0'"
fi

source /opt/ros/jazzy/setup.bash
if [ -f "$EXO_WS/install/setup.bash" ]; then
    source "$EXO_WS/install/setup.bash"
fi
export LD_LIBRARY_PATH="$DELSYS_PYTHON_DIR:$LD_LIBRARY_PATH"
exec ./venv/bin/python main.py
