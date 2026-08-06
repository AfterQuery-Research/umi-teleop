#!/usr/bin/env bash
# Run on the YAM Pi. Starts umi-yam for LiDAR/SLAM UMI teleop with the current
# adapter-side mapping and LiDAR->TCP lever.
#
# Safety: this energizes both arms and holds their current positions. Automatic
# startup motion is not requested. Clear the complete workspace before running.
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,8p' "$0"
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

LEFT_CAN="${LEFT_CAN:-209D378045465008}"
RIGHT_CAN="${RIGHT_CAN:-206237A845465006}"

exec "${REPO_ROOT}/.venv/bin/umi-yam" \
  --bind "${YAM_TELEOP_BIND:-127.0.0.1}" \
  --left-adapter-serial "${LEFT_CAN}" \
  --right-adapter-serial "${RIGHT_CAN}" \
  --left-tip-offset -0.10479 0.0 0.22244 \
  --right-tip-offset -0.10479 0.0 0.22244 \
  --confirm-clear
