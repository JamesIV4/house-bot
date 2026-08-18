#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$ROOT_DIR/envs/dpvo"
UPSTREAM_DIR="$ROOT_DIR/external/DPVO"
PYTHON="$ENV_PREFIX/bin/python"
VIDEO="${DPVO_VIDEO:-$ROOT_DIR/data/input/c920-room-loop-20260817-121954.mp4}"
CALIB="${DPVO_CALIB:-$ROOT_DIR/config/c920-dpvo-approx.txt}"
MODE="${1:-all}"
OUTPUT_DIR="${DPVO_OUTPUT_DIR:-$ROOT_DIR/data/output/dpvo-c920-room-20260817}"

if [[ ! -x "$PYTHON" || ! -f "$UPSTREAM_DIR/dpvo.pth" ]]; then
  echo "DPVO is not ready. Run scripts/bootstrap_dpvo.sh first." >&2
  exit 1
fi

if [[ ! -f "$VIDEO" ]]; then
  echo "Input video is missing: $VIDEO" >&2
  exit 1
fi

export CUDA_HOME="$ENV_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_TARGET="$CUDA_HOME/targets/x86_64-linux"
export CPATH="$CUDA_TARGET/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CUDA_TARGET/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CUDA_TARGET/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST="12.0"

mkdir -p "$OUTPUT_DIR"

run_case() {
  local name="$1"
  local config="$2"
  shift 2
  "$PYTHON" "$ROOT_DIR/scripts/benchmark_dpvo.py" \
    --dpvo-root "$UPSTREAM_DIR" \
    --network "$UPSTREAM_DIR/dpvo.pth" \
    --video "$VIDEO" \
    --calib "$CALIB" \
    --config "$config" \
    --stride 2 \
    --name "$name" \
    --output-dir "$OUTPUT_DIR" \
    "$@" 2>&1 | tee "$OUTPUT_DIR/$name.log"
}

case "$MODE" in
  pose)
    run_case dpvo-default-stride2 "$UPSTREAM_DIR/config/default.yaml"
    ;;
  loop)
    run_case dpv-slam-default-stride2 "$UPSTREAM_DIR/config/default.yaml" --loop-closure
    ;;
  fast-loop)
    run_case dpv-slam-fast-stride2 "$UPSTREAM_DIR/config/fast.yaml" --loop-closure
    ;;
  navigation)
    run_case dpv-slam-navigation-stride2 "$ROOT_DIR/config/dpvo-navigation.yaml" --loop-closure
    ;;
  all)
    run_case dpvo-default-stride2 "$UPSTREAM_DIR/config/default.yaml"
    run_case dpv-slam-default-stride2 "$UPSTREAM_DIR/config/default.yaml" --loop-closure
    ;;
  *)
    echo "Usage: $0 [all|pose|loop|fast-loop|navigation]" >&2
    exit 2
    ;;
esac
