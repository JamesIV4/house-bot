#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <pi-host-or-ip> [port] [MASt3R-SLAM arguments]" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI_HOST="$1"
PORT="${2:-5600}"
CODEC="${C920_CODEC:-h264}"
if [[ $# -ge 2 ]]; then
  shift 2
else
  shift
fi

if [[ "$CODEC" == "h264" ]]; then
  CONTAINER_FORMAT="mpegts"
else
  CONTAINER_FORMAT="mjpeg"
fi

exec "$ROOT_DIR/scripts/run_mast3r_slam.sh" \
  "tcp-connect://${PI_HOST}:${PORT}?format=${CONTAINER_FORMAT}" \
  --config "$ROOT_DIR/config/c920-live.yaml" \
  --save-as "pi-c920-live" \
  "$@"
