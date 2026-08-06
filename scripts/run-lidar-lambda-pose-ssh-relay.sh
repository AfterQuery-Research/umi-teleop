#!/usr/bin/env bash
set -euo pipefail

LAMBDA_HOST="${1:-ubuntu@192.222.52.98}"
TARGET_HOST="${2:-10.103.3.85}"
TARGET_PORT="${3:-9142}"
LAMBDA_REPO="${LAMBDA_REPO:-/home/ubuntu/umi-teleop}"
LEFT_CONTAINER="${LEFT_CONTAINER:-glim-10-103-0-84}"
RIGHT_CONTAINER="${RIGHT_CONTAINER:-glim-10-103-0-80}"
LEFT_SERIAL="${LEFT_SERIAL:-10.103.0.84}"
RIGHT_SERIAL="${RIGHT_SERIAL:-10.103.0.80}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

PYTHONPATH="${REPO_ROOT}/src" python3 -m umi_teleop.lidar_ssh_relay \
  --lambda-host "${LAMBDA_HOST}" \
  --lambda-repo "${LAMBDA_REPO}" \
  --target-host "${TARGET_HOST}" \
  --target-port "${TARGET_PORT}" \
  --left-container "${LEFT_CONTAINER}" \
  --right-container "${RIGHT_CONTAINER}" \
  --left-serial "${LEFT_SERIAL}" \
  --right-serial "${RIGHT_SERIAL}"
