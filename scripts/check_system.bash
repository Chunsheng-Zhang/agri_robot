#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/setup_environment.bash"
ros2 node list
for agri_topic in /mgx23/position /mgx23/attitude_stamped /odom /localization/valid /cmd_vel /base/driver_cmd_vel /safety/status; do
  ros2 topic info "$agri_topic" --verbose
done
# Inspect publisher count/identity: /odom=localization, /cmd_vel=safety,
# /base/driver_cmd_vel=chassis_adapter. Inspect TF with view_frames or tf2_echo.
