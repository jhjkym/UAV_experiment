#!/usr/bin/env bash

# Source this file from a ROS development shell:
#   source scripts/env/ros_noetic_wsl.bash

_uav_ros_env_fail() {
  echo "[ros_noetic_wsl] ERROR: $*" >&2
  return 1
}

if [ -n "${CONDA_PREFIX:-}" ] && command -v conda >/dev/null 2>&1; then
  conda deactivate >/dev/null 2>&1 || true
fi

_uav_clean_path=""
IFS=':' read -r -a _uav_path_parts <<< "${PATH:-}"
for _uav_path_part in "${_uav_path_parts[@]}"; do
  case "${_uav_path_part}" in
    *anaconda*/bin|*Anaconda*/bin|*miniconda*/bin|*Miniconda*/bin|*conda*/bin)
      ;;
    *)
      if [ -z "${_uav_clean_path}" ]; then
        _uav_clean_path="${_uav_path_part}"
      else
        _uav_clean_path="${_uav_clean_path}:${_uav_path_part}"
      fi
      ;;
  esac
done
export PATH="${_uav_clean_path}"
unset _uav_clean_path _uav_path_part _uav_path_parts

unset PYTHONHOME
if [ -n "${UAV_ROS_HOME:-}" ]; then
  export ROS_HOME="${UAV_ROS_HOME}"
  mkdir -p "${ROS_HOME}" || {
    _uav_ros_env_fail "failed to create ROS_HOME at ${ROS_HOME}"
    return $?
  }
fi

if [ ! -f /opt/ros/noetic/setup.bash ]; then
  _uav_ros_env_fail "/opt/ros/noetic/setup.bash was not found"
  return $?
fi

source /opt/ros/noetic/setup.bash

if [ -f devel/setup.bash ]; then
  source devel/setup.bash
fi

_uav_python="$(command -v python3 || true)"
if [ "${_uav_python}" != "/usr/bin/python3" ]; then
  _uav_ros_env_fail "python3 must resolve to /usr/bin/python3, got '${_uav_python}'"
  return $?
fi

python3 - <<'PY'
import sys
if sys.version_info[:2] != (3, 8):
    raise SystemExit("Python must be 3.8.x, got %s" % sys.version.split()[0])
import rospy
import rospkg
print("[ros_noetic_wsl] Python OK:", sys.executable, sys.version.split()[0])
PY
_uav_status=$?
unset _uav_python
if [ "${_uav_status}" -ne 0 ]; then
  _uav_ros_env_fail "failed to validate rospy/rospkg with /usr/bin/python3"
  return $?
fi
unset _uav_status

echo "[ros_noetic_wsl] ROS_DISTRO=${ROS_DISTRO:-unset}"
