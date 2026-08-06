#!/usr/bin/env bash
# Run on the right-hand LiDAR UMI Pi: umi1@10.103.0.80.
#
# This stops umi-recorder.service for this runtime session and starts the LiDAR teleop
# gripper/button source. It does not disable recorder autostart on boot.
#
# Usage:
#   scripts/run-lidar-teleop-umi1-right.sh <MERGER_HOST>
# or:
#   MERGER_HOST=<host> scripts/run-lidar-teleop-umi1-right.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,11p' "$0"
  exit 0
fi

MERGER_HOST="${1:-${MERGER_HOST:-}}"

if [[ -z "${MERGER_HOST}" ]]; then
  echo "usage: $0 <MERGER_HOST>" >&2
  exit 64
fi

EXTRA_FLAGS=()
if [[ "${RESTART_RECORDER_ON_EXIT:-0}" == "1" ]]; then
  EXTRA_FLAGS+=(--restart-recorder-on-exit)
fi

exec env GPIOZERO_PIN_FACTORY="${GPIOZERO_PIN_FACTORY:-lgpio}" \
  "${REPO_ROOT}/.venv/bin/umi-lidar-teleop-mode" \
    --target-host "${MERGER_HOST}" \
    --arm right \
    --source-id "${SOURCE_ID:-10.103.0.80}" \
    --camera-device "${CAMERA_DEVICE:-/dev/v4l/by-id/usb-Global_Shutter_Camera_Global_Shutter_Camera_01.00.00-video-index0}" \
    --calib "${REPO_ROOT}/src/umi_teleop/data/umi1_calib.npz" \
    --tag-ids 2 3 \
    --width 800 \
    --height 600 \
    --buttons gpio \
    --record-pin 12 \
    --quit-pin 16 \
    --confirm-stop-recorder \
    "${EXTRA_FLAGS[@]}"
