#!/usr/bin/env bash
set -euo pipefail

if [[ "${UAV_ALLOW_SITL_FLIGHT:-}" != "YES" ]]; then
  echo "Refusing M0-C5B1-R1A handoff rehearsal: UAV_ALLOW_SITL_FLIGHT must be exactly YES" >&2
  exit 2
fi

cd /home/tom/UAV_experiment

source scripts/env/ros_noetic_wsl.bash
source devel/setup.bash

exec /usr/bin/python3 scripts/experiments/m0_c5b1_r1a_handoff_ground.py
