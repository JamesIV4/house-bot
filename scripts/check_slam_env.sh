#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_PREFIX="$ROOT_DIR/envs/mast3r-slam"
UPSTREAM_DIR="$ROOT_DIR/external/MASt3R-SLAM"

export CUDA_HOME="$ENV_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_TARGET="$CUDA_HOME/targets/x86_64-linux"
export CPATH="$CUDA_TARGET/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CUDA_TARGET/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CUDA_TARGET/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST="12.0"

"$ENV_PREFIX/bin/python" - <<'PY'
import torch

print(f"torch={torch.__version__}")
print(f"torch_cuda={torch.version.cuda}")
print(f"cuda_available={torch.cuda.is_available()}")
print(f"device={torch.cuda.get_device_name(0)}")
print(f"capability={torch.cuda.get_device_capability(0)}")

x = torch.rand((2048, 2048), device="cuda")
y = x @ x
torch.cuda.synchronize()
print(f"cuda_smoke_sum={y.sum().item():.3f}")

import mast3r_slam_backends
print(f"mast3r_slam_backends={mast3r_slam_backends.__file__}")

import lietorch_backends
print(f"lietorch_backends={lietorch_backends.__file__}")

import lietorch_extras
print(f"lietorch_extras={lietorch_extras.__file__}")

import curope
print(f"curope={curope.__file__}")
PY

git -C "$UPSTREAM_DIR" status --short --branch
