#!/usr/bin/env bash
set -euo pipefail

if [[ "${UAV_ALLOW_SITL_FLIGHT:-}" != "YES" ]]; then
  echo "Refusing M0-C5B1 SITL flight: UAV_ALLOW_SITL_FLIGHT must be exactly YES." >&2
  exit 2
fi

REPO_DIR="/home/tom/UAV_experiment"
cd "${REPO_DIR}"

export UAV_ROS_HOME="${UAV_ROS_HOME:-/tmp/uav_m0_c5b1/ros_home}"

source scripts/env/ros_noetic_wsl.bash
source devel/setup.bash

exec /usr/bin/python3 scripts/experiments/m0_c5b1_sitl_line.py
