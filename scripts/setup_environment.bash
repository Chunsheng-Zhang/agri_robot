#!/usr/bin/env bash
# Source this file; use local_setup to avoid importing historical overlay chains.
AGRI_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
for agri_underlay in \
  "$AGRI_PROJECT_ROOT/yahboomcar_ros2_ws/software/library_ws/install/local_setup.bash" \
  "$AGRI_PROJECT_ROOT/yahboomcar_ros2_ws/yahboomcar_ws/install/local_setup.bash" \
  "$AGRI_PROJECT_ROOT/mgx23_ws/install/local_setup.bash" \
  "$AGRI_PROJECT_ROOT/open_navigation_ws/install/local_setup.bash" \
  "$AGRI_PROJECT_ROOT/agri_robot_ws/install/local_setup.bash"; do
  if [[ -f "$agri_underlay" ]]; then
    source "$agri_underlay"
  fi
done
unset agri_underlay
