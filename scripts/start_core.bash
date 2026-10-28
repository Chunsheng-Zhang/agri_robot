#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/setup_environment.bash"
exec ros2 launch agri_bringup system.launch.py "$@"
