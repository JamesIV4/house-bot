#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Usage: $0 <groundtruth.txt> <estimated-trajectory.txt> [evo_ape arguments]" >&2
  exit 2
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$ROOT_DIR/envs/mast3r-slam"
GROUND_TRUTH="$1"
ESTIMATE="$2"
shift 2

exec "$ENV_PREFIX/bin/evo_ape" tum "$GROUND_TRUTH" "$ESTIMATE" --align "$@"
