#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/setup_environment.bash"
cd "$AGRI_PROJECT_ROOT/mgx23_ws"
colcon build --symlink-install --packages-select mgx23_driver
source install/local_setup.bash
cd "$AGRI_PROJECT_ROOT/agri_robot_ws"
colcon build --symlink-install
