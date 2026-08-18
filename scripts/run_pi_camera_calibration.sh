#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <pi-host-or-ip> [port] [max-views]" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$ROOT_DIR/envs/dpvo"
PI_HOST="$1"
PORT="${2:-5600}"
MAX_VIEWS="${3:-32}"
SSH_KEY="${HOUSEBOT_SSH_KEY:-$HOME/.ssh/housebot_ed25519}"
REMOTE_DIR="/home/james/house-bot"
OUTPUT_DIR="$ROOT_DIR/data/output/c920-calibration"
RUN_NAME="c920-1280x720-$(date +%Y%m%d-%H%M%S)"
REMOTE_PID=""

if [[ ! -x "$ENV_PREFIX/bin/python" ]]; then
  echo "DPVO environment is missing. Run scripts/bootstrap_dpvo.sh first." >&2
  exit 1
fi

if [[ ! -f "$SSH_KEY" ]]; then
  echo "House Bot SSH key is missing: $SSH_KEY" >&2
  exit 1
fi

SSH=(ssh -i "$SSH_KEY" -o BatchMode=yes james@"$PI_HOST")
SCP=(scp -i "$SSH_KEY" -o BatchMode=yes)

cleanup() {
  if [[ -n "$REMOTE_PID" ]]; then
    "${SSH[@]}" "if kill -0 '$REMOTE_PID' 2>/dev/null; then kill '$REMOTE_PID'; fi" \
      >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

report_power_state() {
  local state hex value
  state="$("${SSH[@]}" "vcgencmd get_throttled")"
  hex="${state#*=}"
  value=$((hex))
  echo "Pi power state: $state"
  if (( value & 0x5 )); then
    echo "Warning: Pi is currently undervolted or throttled." >&2
    echo "The native C920 stream was verified at 30.013 FPS in this state; continuing." >&2
  fi
  if (( value & 0x50000 )); then
    echo "Warning: undervoltage/throttling occurred earlier in this boot." >&2
  fi
}

report_power_state
"${SSH[@]}" "mkdir -p '$REMOTE_DIR'"
"${SCP[@]}" "$ROOT_DIR/scripts/pi_stream_c920.sh" \
  "james@${PI_HOST}:${REMOTE_DIR}/pi_stream_c920.sh" >/dev/null
"${SSH[@]}" "chmod +x '$REMOTE_DIR/pi_stream_c920.sh'"

REMOTE_PID="$("${SSH[@]}" \
  "nohup env C920_AUTOFOCUS=0 C920_FOCUS_ABSOLUTE=0 '$REMOTE_DIR/pi_stream_c920.sh' /dev/video0 '$PORT' > /tmp/house-bot-camera.log 2>&1 & echo \$!")"
echo "Started fixed-focus Pi camera stream as PID $REMOTE_PID"
sleep 3
report_power_state

mkdir -p "$OUTPUT_DIR"
"$ENV_PREFIX/bin/python" "$ROOT_DIR/scripts/calibrate_c920.py" \
  --host "$PI_HOST" \
  --port "$PORT" \
  --format mpegts \
  --pattern-cols 9 \
  --pattern-rows 6 \
  --min-views 20 \
  --max-views "$MAX_VIEWS" \
  --duration 120 \
  --name "$RUN_NAME" \
  --output-dir "$OUTPUT_DIR"
