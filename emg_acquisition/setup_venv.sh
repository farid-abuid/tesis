#!/bin/bash
# One-time venv setup. --system-site-packages so rclpy (from the sourced ROS
# Jazzy environment) and the system numpy remain importable inside the venv.
set -e
cd "$(dirname "$0")"
python3 -m venv --system-site-packages venv
./venv/bin/pip install --upgrade pip
./venv/bin/pip install -r requirements.txt
echo "venv ready. Launch the app with ./run.sh"
