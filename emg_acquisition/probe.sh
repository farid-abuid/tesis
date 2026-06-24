#!/bin/bash
# Wrapper for probe_api.py with the same env as run.sh (sg dialout, LD_LIBRARY_PATH).
# Only strictly needed for `./probe.sh --connect`; the offline reflection mode
# also works as `./venv/bin/python probe_api.py` with no extra setup.
cd "$(dirname "$0")"

export DELSYS_PYTHON_DIR="${DELSYS_PYTHON_DIR:-/home/utec/Documents/farid/Example-Applications-main_vf/Example-Applications-main/Python}"

if [ -z "$EMG_ACQ_INNER" ]; then
    EMG_ACQ_INNER=1 exec sg dialout -c "bash '$0' $*"
fi

export LD_LIBRARY_PATH="$DELSYS_PYTHON_DIR:$LD_LIBRARY_PATH"
exec ./venv/bin/python probe_api.py "$@"
