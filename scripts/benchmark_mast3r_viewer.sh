#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
INPUT="${1:-$ROOT_DIR/data/input/c920-room-loop-20260817-121954.mp4}"
CONFIG="${2:-$ROOT_DIR/config/c920-navigation.yaml}"
MAX_FRAMES="${3:-300}"
RUN_NAME="${4:-c920-navigation-viz-bench}"
STDOUT_LOG="$ROOT_DIR/data/output/$RUN_NAME.stdout.log"
STDERR_LOG="$ROOT_DIR/data/output/$RUN_NAME.stderr.log"

mkdir -p "$ROOT_DIR/data/output"

benchmark_pid=""
cleanup() {
  if [[ -n "$benchmark_pid" ]] && kill -0 "$benchmark_pid" 2>/dev/null; then
    kill "$benchmark_pid"
    wait "$benchmark_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

PYTHONUNBUFFERED=1 "$ROOT_DIR/scripts/run_mast3r_slam.sh" \
  "$INPUT" \
  --config "$CONFIG" \
  --max-frames "$MAX_FRAMES" \
  --save-as "$RUN_NAME" \
  >"$STDOUT_LOG" \
  2>"$STDERR_LOG" &
benchmark_pid=$!

completed=0
SECONDS=0
for _ in $(seq 1 150); do
  if grep -qx "done" "$STDOUT_LOG" 2>/dev/null; then
    completed=1
    break
  fi
  if ! kill -0 "$benchmark_pid" 2>/dev/null; then
    break
  fi
  sleep 1
done

if [[ "$completed" -ne 1 ]]; then
  echo "Visual benchmark did not complete; see $STDERR_LOG" >&2
  exit 1
fi

echo "Visual benchmark completed and saved as $RUN_NAME"
echo "elapsed_seconds: $SECONDS"
echo "stdout: $STDOUT_LOG"
echo "stderr: $STDERR_LOG"
