#!/usr/bin/env bash
set -eo pipefail
agri_gz_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
agri_sim_dir="$(dirname -- "$agri_gz_dir")"
agri_project_dir="$(dirname -- "$agri_sim_dir")"
mkdir -p "$agri_sim_dir/.cache/tmp" "$agri_sim_dir/.cache/ros" "$agri_sim_dir/.cache/matplotlib" "$agri_sim_dir/.cache/gazebo_build"
export TMPDIR="$agri_sim_dir/.cache/tmp" ROS_LOG_DIR="$agri_sim_dir/.cache/ros"
export MPLCONFIGDIR="$agri_sim_dir/.cache/matplotlib" PYTHONDONTWRITEBYTECODE=1
# 只加载 ROS，不加载实车 bringup，不继承实车 Domain。
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1
if ! cmake -S "$agri_gz_dir" -B "$agri_sim_dir/.cache/gazebo_build" >"$agri_sim_dir/.cache/gazebo_build/configure.log" 2>&1 ||
   ! cmake --build "$agri_sim_dir/.cache/gazebo_build" -j2 >"$agri_sim_dir/.cache/gazebo_build/build.log" 2>&1; then
  echo "Gazebo本地支持库构建失败，见 tracking_simulation/.cache/gazebo_build/" >&2
  exit 1
fi
exec "$agri_project_dir/agri_backend/.venv/bin/python" -B "$agri_gz_dir/runner.py" "$@"
