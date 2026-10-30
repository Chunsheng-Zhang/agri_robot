#!/usr/bin/env bash
# One terminal: bash /home/jetson/agri_robot/scripts/project.bash [start|status|stop]
set -eo pipefail
agri_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$agri_root"
mkdir -p "$agri_root/.runtime/tmp" "$agri_root/.runtime/project"
export TMPDIR="$agri_root/.runtime/tmp"
export ROS_LOG_DIR="$agri_root/.runtime/project/ros"
source "$agri_root/scripts/setup_environment.bash"
if [[ ! -x "$agri_root/agri_backend/.venv/bin/python" ]]; then
  echo '缺少项目后端虚拟环境：agri_backend/.venv/bin/python' >&2
  exit 1
fi
exec "$agri_root/agri_backend/.venv/bin/python" -B "$agri_root/scripts/project.py" "$@"
