#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="$ROOT_DIR/.tools"
MAMBA_ROOT_PREFIX="$ROOT_DIR/.mamba"
MICROMAMBA="$TOOLS_DIR/micromamba"
ENV_PREFIX="$ROOT_DIR/envs/mast3r-slam"
UPSTREAM_DIR="$ROOT_DIR/external/MASt3R-SLAM"
UPSTREAM_URL="https://github.com/rmurai0610/MASt3R-SLAM.git"
UPSTREAM_REF="6717231a2daf55d501a5824bbec43314d4fb77d9"
LIETORCH_DIR="$ROOT_DIR/external/lietorch"
LIETORCH_URL="https://github.com/princeton-vl/lietorch.git"
LIETORCH_REF="e7df86554156b36846008d8ddbcc4d8521a16554"
PATCH_FILES=(
  "$ROOT_DIR/patches/mast3r-slam/blackwell-sm120.patch"
  "$ROOT_DIR/patches/mast3r-slam/video-input.patch"
)

mkdir -p "$TOOLS_DIR" "$ROOT_DIR/envs" "$ROOT_DIR/external"

if [[ ! -x "$MICROMAMBA" ]]; then
  echo "Downloading micromamba"
  MICROMAMBA_VERSION="2.8.1-0"
  MICROMAMBA_SHA256="9689782d863c05a1bf5d2d371ba527104e7a4eb4310c1637d8653b751aed9c82"
  curl -L --fail --retry 3 \
    "https://github.com/mamba-org/micromamba-releases/releases/download/$MICROMAMBA_VERSION/micromamba-linux-64" \
    --output "$MICROMAMBA"
  echo "$MICROMAMBA_SHA256  $MICROMAMBA" | sha256sum --check
  chmod +x "$MICROMAMBA"
fi

export MAMBA_ROOT_PREFIX

if [[ ! -x "$ENV_PREFIX/bin/python" ]]; then
  echo "Creating Python/CUDA build environment"
  "$MICROMAMBA" create -y -p "$ENV_PREFIX" \
    -c conda-forge -c nvidia \
    python=3.11 pip cmake ninja pkg-config cuda-toolkit=12.8
fi

if [[ ! -d "$UPSTREAM_DIR/.git" ]]; then
  git clone --recursive "$UPSTREAM_URL" "$UPSTREAM_DIR"
fi

git -C "$UPSTREAM_DIR" fetch origin "$UPSTREAM_REF"
git -C "$UPSTREAM_DIR" checkout --detach "$UPSTREAM_REF"
git -C "$UPSTREAM_DIR" submodule update --init --recursive

if [[ ! -d "$LIETORCH_DIR/.git" ]]; then
  git clone "$LIETORCH_URL" "$LIETORCH_DIR"
fi

git -C "$LIETORCH_DIR" fetch origin "$LIETORCH_REF"
git -C "$LIETORCH_DIR" checkout --detach "$LIETORCH_REF"
git -C "$LIETORCH_DIR" submodule update --init --recursive

for patch_file in "${PATCH_FILES[@]}"; do
  if git -C "$UPSTREAM_DIR" apply --check "$patch_file" 2>/dev/null; then
    git -C "$UPSTREAM_DIR" apply "$patch_file"
  elif git -C "$UPSTREAM_DIR" apply --reverse --check "$patch_file" 2>/dev/null; then
    echo "Patch already applied: $(basename "$patch_file")"
  else
    echo "Patch does not apply cleanly: $patch_file" >&2
    exit 1
  fi
done

export CUDA_HOME="$ENV_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_TARGET="$CUDA_HOME/targets/x86_64-linux"
export CPATH="$CUDA_TARGET/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CUDA_TARGET/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CUDA_TARGET/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST="12.0"
export MAX_JOBS="4"

PYTHON="$ENV_PREFIX/bin/python"

if "$PYTHON" -c \
  'import torch; import cv2, curope, in3d, lietorch, lietorch_backends, lietorch_extras, mast3r, mast3r_slam, mast3r_slam_backends; assert torch.cuda.is_available(); assert torch.cuda.get_device_capability(0) == (12, 0)' \
  >/dev/null 2>&1; then
  echo "MASt3R-SLAM environment is already ready"
  exit 0
fi

"$PYTHON" -m pip install --upgrade pip setuptools wheel
"$PYTHON" -m pip install \
  torch==2.8.0 torchvision==0.23.0 \
  --index-url https://download.pytorch.org/whl/cu128
"$PYTHON" -m pip install \
  numpy==1.26.4 opencv-python==4.11.0.86 plyfile==1.0.3

cd "$UPSTREAM_DIR"
"$PYTHON" -m pip install --no-build-isolation -e thirdparty/mast3r
"$PYTHON" -m pip install \
  imgui==2.0.0 moderngl==5.12.0 moderngl-window==2.4.6 \
  glfw pyglm msgpack matplotlib 'trimesh[easy]'
"$PYTHON" -m pip install --no-deps --no-build-isolation -e thirdparty/in3d
"$PYTHON" -m pip install pyrealsense2 evo natsort
"$PYTHON" -m pip install \
  numpy==1.26.4 opencv-python==4.11.0.86 plyfile==1.0.3
"$PYTHON" -m pip install --no-build-isolation -e "$LIETORCH_DIR"
"$PYTHON" -m pip install --no-deps --no-build-isolation -e .

echo
echo "MASt3R-SLAM environment installed at $ENV_PREFIX"
echo "Run scripts/check_slam_env.sh next."
