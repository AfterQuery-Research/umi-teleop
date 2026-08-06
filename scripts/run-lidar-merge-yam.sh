#!/usr/bin/env bash
# Run on the YAM Pi, beside umi-yam. It merges Lambda LiDAR poses and UMI gripper/button
# telemetry into the existing umi-yam UDP protocol on localhost:9140.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,6p' "$0"
  exit 0
fi

exec "${REPO_ROOT}/.venv/bin/umi-lidar-merge-source" \
  --target-host "${YAM_TELEOP_HOST:-127.0.0.1}" \
  --target-port "${YAM_TELEOP_PORT:-9140}" \
  --pose-port "${LIDAR_POSE_PORT:-9142}" \
  --arm-port "${LIDAR_ARM_PORT:-9143}"
