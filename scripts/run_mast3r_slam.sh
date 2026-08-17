#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <video-or-image-folder> [MASt3R-SLAM arguments]" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$ROOT_DIR/envs/mast3r-slam"
UPSTREAM_DIR="$ROOT_DIR/external/MASt3R-SLAM"
INPUT="$1"
shift

export CUDA_HOME="$ENV_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_TARGET="$CUDA_HOME/targets/x86_64-linux"
export CPATH="$CUDA_TARGET/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CUDA_TARGET/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CUDA_TARGET/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST="12.0"

cd "$UPSTREAM_DIR"

ARGS=(main.py --dataset "$INPUT" --config config/base.yaml)
ARGS+=("$@")

exec "$ENV_PREFIX/bin/python" "${ARGS[@]}"
