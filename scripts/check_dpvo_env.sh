#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$ROOT_DIR/envs/dpvo"
UPSTREAM_DIR="$ROOT_DIR/external/DPVO"
PYTHON="$ENV_PREFIX/bin/python"

if [[ ! -x "$PYTHON" ]]; then
  echo "DPVO environment is missing. Run scripts/bootstrap_dpvo.sh first." >&2
  exit 1
fi

export CUDA_HOME="$ENV_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_TARGET="$CUDA_HOME/targets/x86_64-linux"
export CPATH="$CUDA_TARGET/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CUDA_TARGET/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CUDA_TARGET/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST="12.0"

cd "$UPSTREAM_DIR"
"$PYTHON" - <<'PY'
import pathlib
import av
import torch
import cv2
import cuda_ba
import cuda_corr
import lietorch_backends
import torch_scatter
from dpvo.net import VONet

checkpoint = pathlib.Path("dpvo.pth")
assert checkpoint.is_file(), checkpoint
assert torch.cuda.is_available()
assert torch.cuda.get_device_capability(0) == (12, 0)

print(f"torch={torch.__version__}")
print(f"cuda_runtime={torch.version.cuda}")
print(f"gpu={torch.cuda.get_device_name(0)}")
print(f"compute_capability={torch.cuda.get_device_capability(0)}")
print(f"opencv={cv2.__version__}")
print(f"pyav={av.__version__}")
print(f"checkpoint={checkpoint.resolve()} ({checkpoint.stat().st_size} bytes)")
print("DPVO CUDA extensions: ready")
PY

echo "upstream_commit=$(git rev-parse HEAD)"
sha256sum dpvo.pth
