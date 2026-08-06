#!/usr/bin/env bash
# Run on Lambda. Builds/starts two live GLIM containers and their resilient Pi UDP feeds.
#
# Use BUILD_ONLY=1 to build the Docker image without starting containers/feeds.
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  sed -n '2,8p' "$0"
  exit 0
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
IMG="${LIVE_GLIM_IMAGE:-umi-teleop-glim-live}"

docker build -f "${REPO_ROOT}/lambda_live/Dockerfile" -t "${IMG}" "${REPO_ROOT}/lambda_live"

if [[ "${BUILD_ONLY:-0}" == "1" ]]; then
  echo "Built ${IMG}; BUILD_ONLY=1 so not starting live containers."
  exit 0
fi

LEFT_PI="${LEFT_PI:-umi2@10.103.0.84}"
RIGHT_PI="${RIGHT_PI:-umi1@10.103.0.80}"
LEFT_CTR="${LEFT_CONTAINER:-glim-10-103-0-84}"
RIGHT_CTR="${RIGHT_CONTAINER:-glim-10-103-0-80}"

for ctr in "${LEFT_CTR}" "${RIGHT_CTR}"; do
  docker rm -f "${ctr}" >/dev/null 2>&1 || true
  docker run -d --gpus all --name "${ctr}" "${IMG}" >/dev/null
done

pkill -f "feed-lidar-pi-to-container.sh ${LEFT_PI} ${LEFT_CTR}" 2>/dev/null || true
pkill -f "feed-lidar-pi-to-container.sh ${RIGHT_PI} ${RIGHT_CTR}" 2>/dev/null || true
nohup "${SCRIPT_DIR}/feed-lidar-pi-to-container.sh" "${LEFT_PI}" "${LEFT_CTR}" \
  >"/tmp/feed-${LEFT_CTR}.log" 2>&1 &
nohup "${SCRIPT_DIR}/feed-lidar-pi-to-container.sh" "${RIGHT_PI}" "${RIGHT_CTR}" \
  >"/tmp/feed-${RIGHT_CTR}.log" 2>&1 &

echo "Started ${LEFT_CTR} fed from ${LEFT_PI}"
echo "Started ${RIGHT_CTR} fed from ${RIGHT_PI}"
echo "Feed logs: /tmp/feed-${LEFT_CTR}.log /tmp/feed-${RIGHT_CTR}.log"
