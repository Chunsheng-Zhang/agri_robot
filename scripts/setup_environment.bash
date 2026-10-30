#!/usr/bin/env bash
# Source this file; use local_setup to avoid importing historical overlay chains.
AGRI_PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
# Vendor workspaces may be installed beside this repository on the Jetson.
# Source their local setup only; never rebuild or modify the vendor installation.
for agri_vendor in software/library_ws yahboomcar_ws; do
  agri_vendor_setup="$AGRI_PROJECT_ROOT/yahboomcar_ros2_ws/$agri_vendor/install/local_setup.bash"
  if [[ ! -f "$agri_vendor_setup" ]]; then
    agri_vendor_setup="$(dirname -- "$AGRI_PROJECT_ROOT")/yahboomcar_ros2_ws/$agri_vendor/install/local_setup.bash"
  fi
  if [[ -f "$agri_vendor_setup" ]]; then
    source "$agri_vendor_setup"
  fi
done
for agri_underlay in \
  "$AGRI_PROJECT_ROOT/mgx23_ws/install/local_setup.bash" \
  "$AGRI_PROJECT_ROOT/open_navigation_ws/install/local_setup.bash" \
  "$AGRI_PROJECT_ROOT/agri_robot_ws/install/local_setup.bash"; do
  if [[ -f "$agri_underlay" ]]; then
    source "$agri_underlay"
  fi
done
# Fields2Cover's bundled OR-Tools is not installed into its overlay lib directory.
# Keep this process-local so no system linker configuration needs changing.
agri_ortools_lib="$AGRI_PROJECT_ROOT/open_navigation_ws/build/fields2cover/_deps/ortools-src/lib"
if [[ -f "$agri_ortools_lib/libortools.so.9" ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":$agri_ortools_lib:"*) ;;
    *) export LD_LIBRARY_PATH="$agri_ortools_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
  esac
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-$AGRI_PROJECT_ROOT/.runtime/ros}"
export PYTHONDONTWRITEBYTECODE=1
unset agri_underlay agri_vendor agri_vendor_setup agri_ortools_lib
