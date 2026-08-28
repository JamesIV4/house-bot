#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <pi-host-or-ip> [port] [max-poses]" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$ROOT_DIR/envs/dpvo"
PI_HOST="$1"
PORT="${2:-5600}"
MAX_POSES="${3:-300}"
SSH_KEY="${HOUSEBOT_SSH_KEY:-$HOME/.ssh/housebot_ed25519}"
REMOTE_DIR="/home/james/house-bot"
OUTPUT_DIR="$ROOT_DIR/data/output/dpvo-pi-live"
RUN_NAME="pi-c920-dpvo-$(date +%Y%m%d-%H%M%S)"
CALIBRATION="${DPVO_CALIBRATION:-$ROOT_DIR/config/c920-dpvo-measured.txt}"
REMOTE_PID=""
EXTRA_ARGS=()

if [[ "${DPVO_ALLOW_UNINITIALIZED:-0}" == "1" ]]; then
  EXTRA_ARGS+=(--allow-uninitialized)
fi

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

"${SSH[@]}" "mkdir -p '$REMOTE_DIR'"
"${SCP[@]}" "$ROOT_DIR/scripts/pi_stream_c920.sh" \
  "james@${PI_HOST}:${REMOTE_DIR}/pi_stream_c920.sh" >/dev/null
"${SSH[@]}" "chmod +x '$REMOTE_DIR/pi_stream_c920.sh'"

# Wait for the listener rather than guessing at it. ffmpeg binds the port about
# 1.8 s after launch on this Pi, against what used to be a fixed 2 s sleep, so
# any jitter in SSH round-trip or camera warm-up raced the client into
# ECONNREFUSED. Polled on the Pi so one SSH round trip covers the whole wait,
# and because probing the port from here would consume ffmpeg's single accept.
REMOTE_PID="$("${SSH[@]}" \
  "nohup env C920_AUTOFOCUS=0 C920_FOCUS_ABSOLUTE=0 '$REMOTE_DIR/pi_stream_c920.sh' /dev/video0 '$PORT' > /tmp/house-bot-camera.log 2>&1 & \
   pid=\$!; \
   for _ in \$(seq 1 200); do \
     if ss -ltn 2>/dev/null | grep -q ':$PORT '; then echo \$pid; exit 0; fi; \
     if ! kill -0 \$pid 2>/dev/null; then echo 'camera stream exited during start-up' >&2; exit 1; fi; \
     sleep 0.1; \
   done; \
   echo 'camera stream did not listen within 20s' >&2; exit 1")"

if [[ -z "$REMOTE_PID" ]]; then
  echo "Pi camera stream failed to start; see /tmp/house-bot-camera.log on $PI_HOST" >&2
  exit 1
fi
echo "Started Pi camera stream as PID $REMOTE_PID (listening on $PORT)"

export CUDA_HOME="$ENV_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_TARGET="$CUDA_HOME/targets/x86_64-linux"
export LD_LIBRARY_PATH="$CUDA_TARGET/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST="12.0"

mkdir -p "$OUTPUT_DIR"
"$ENV_PREFIX/bin/python" "$ROOT_DIR/scripts/run_live_dpvo.py" \
  --dpvo-root "$ROOT_DIR/external/DPVO" \
  --network "$ROOT_DIR/external/DPVO/dpvo.pth" \
  --config "$ROOT_DIR/config/dpvo-navigation.yaml" \
  --calib "$CALIBRATION" \
  --host "$PI_HOST" \
  --port "$PORT" \
  --format mpegts \
  --source-fps 30 \
  --pose-rate-target 15 \
  --max-poses "$MAX_POSES" \
  --name "$RUN_NAME" \
  --output-dir "$OUTPUT_DIR" \
  "${EXTRA_ARGS[@]}"
