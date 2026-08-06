#!/usr/bin/env bash
# Run on the Lambda box after the per-UMI live GLIM containers are already running.
#
# Usage:
#   scripts/run-lidar-lambda-pose-sender.sh <MERGER_HOST>
#
# Override container names with LEFT_CONTAINER / RIGHT_CONTAINER if needed.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,12p' "$0"
  exit 0
fi

MERGER_HOST="${1:-${MERGER_HOST:-}}"

if [[ -z "${MERGER_HOST}" ]]; then
  echo "usage: $0 <MERGER_HOST>" >&2
  exit 64
fi

exec "${REPO_ROOT}/.venv/bin/umi-lidar-lambda" \
  --target-host "${MERGER_HOST}" \
  --left-container "${LEFT_CONTAINER:-glim-10-103-0-84}" \
  --right-container "${RIGHT_CONTAINER:-glim-10-103-0-80}" \
  --left-serial "${LEFT_SERIAL:-10.103.0.84}" \
  --right-serial "${RIGHT_SERIAL:-10.103.0.80}"
