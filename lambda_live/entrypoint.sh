#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /rslidar_ws/install/setup.bash
GLIM_WS="$(cat /etc/glim_ws_path 2>/dev/null || echo none)"
if [[ "${GLIM_WS}" != "none" && -f "${GLIM_WS}/install/setup.bash" ]]; then
  source "${GLIM_WS}/install/setup.bash"
fi
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

LIVE_GLIM_CONFIG=/tmp/glim_config_live
rm -rf "${LIVE_GLIM_CONFIG}"
cp -r /glim_config "${LIVE_GLIM_CONFIG}"
if ! grep -q "librviz_viewer.so" "${LIVE_GLIM_CONFIG}/config_ros.json"; then
  sed -i 's#\("libmemory_monitor.so"\)#\1,\n        "librviz_viewer.so"#' \
    "${LIVE_GLIM_CONFIG}/config_ros.json"
fi

ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/rslidar_live.yaml &
sleep 4
exec ros2 run glim_ros glim_rosnode --ros-args -p config_path:="${LIVE_GLIM_CONFIG}"
