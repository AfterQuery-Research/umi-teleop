#!/usr/bin/env bash
# Run on Lambda. Taps one UMI Pi's raw LiDAR UDP ports over SSH and pipes packets into
# a live GLIM container's /box_unpack.py.
set -euo pipefail

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || $# -lt 2 ]]; then
  echo "usage: $0 <user@pi-ip> <glim-container> [ports...]" >&2
  exit 64
fi

PI="$1"
CTR="$2"
shift 2
PORTS=("$@")
if [[ ${#PORTS[@]} -eq 0 ]]; then
  PORTS=(6699 7788 6688)
fi

SSH=(ssh -o ConnectTimeout=10 -o ServerAliveInterval=15 -o StrictHostKeyChecking=no)

while true; do
  "${SSH[@]}" "${PI}" "python3 ~/robot-umi/device/scripts/pi_tap.py ${PORTS[*]} 2>/dev/null" \
    | docker exec -i "${CTR}" python3 /box_unpack.py 2>/dev/null
  echo "[feed ${CTR}] dropped; reconnecting in 3s..." >&2
  sleep 3
done
