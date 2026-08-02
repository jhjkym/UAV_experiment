#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="/home/tom/UAV_experiment"
cd "$REPO_DIR"

source scripts/env/ros_noetic_wsl.bash
source devel/setup.bash

exec /usr/bin/python3 scripts/experiments/m0_c5b2b_sitl_circle.py "$@"
