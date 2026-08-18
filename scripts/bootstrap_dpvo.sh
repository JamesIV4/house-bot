#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="$ROOT_DIR/.tools"
MAMBA_ROOT_PREFIX="$ROOT_DIR/.mamba"
MICROMAMBA="$TOOLS_DIR/micromamba"
ENV_PREFIX="$ROOT_DIR/envs/dpvo"
UPSTREAM_DIR="$ROOT_DIR/external/DPVO"
UPSTREAM_URL="https://github.com/princeton-vl/DPVO.git"
UPSTREAM_REF="859bbbfdac6c6185f345003b3c473901fcd13ace"
EIGEN_VERSION="3.4.0"
EIGEN_SHA256="eba3f3d414d2f8cba2919c78ec6daab08fc71ba2ba4ae502b7e5d4d99fc02cda"
EIGEN_ARCHIVE="$ROOT_DIR/.tools/eigen-$EIGEN_VERSION.zip"
EIGEN_DIR="$UPSTREAM_DIR/thirdparty/eigen-$EIGEN_VERSION"
MODEL_ARCHIVE="$UPSTREAM_DIR/models.zip"
MODEL_GDRIVE_ID="1dRqftpImtHbbIPNBIseCv9EvrlHEnjhX"
MODEL_ARCHIVE_SHA256="89f43de2c92676ddcf7f49e8ae3f8940b7af73f549a7a5b8308850b67de78479"
MODEL_SHA256="30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12"
PATCH_FILE="$ROOT_DIR/patches/dpvo/blackwell-torch28.patch"

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
  echo "Creating DPVO Python/CUDA build environment"
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

if [[ ! -d "$EIGEN_DIR/Eigen" ]]; then
  echo "Downloading Eigen $EIGEN_VERSION"
  curl -L --fail --retry 3 \
    "https://gitlab.com/libeigen/eigen/-/archive/$EIGEN_VERSION/eigen-$EIGEN_VERSION.zip" \
    --output "$EIGEN_ARCHIVE"
  echo "$EIGEN_SHA256  $EIGEN_ARCHIVE" | sha256sum --check
  "$ENV_PREFIX/bin/python" -m zipfile -e "$EIGEN_ARCHIVE" "$UPSTREAM_DIR/thirdparty"
fi

if git -C "$UPSTREAM_DIR" apply --check "$PATCH_FILE" 2>/dev/null; then
  git -C "$UPSTREAM_DIR" apply "$PATCH_FILE"
elif git -C "$UPSTREAM_DIR" apply --reverse --check "$PATCH_FILE" 2>/dev/null; then
  echo "Patch already applied: $(basename "$PATCH_FILE")"
else
  echo "Patch does not apply cleanly: $PATCH_FILE" >&2
  exit 1
fi

export CUDA_HOME="$ENV_PREFIX"
export PATH="$CUDA_HOME/bin:$PATH"
CUDA_TARGET="$CUDA_HOME/targets/x86_64-linux"
export CPATH="$CUDA_TARGET/include${CPATH:+:$CPATH}"
export LIBRARY_PATH="$CUDA_TARGET/lib${LIBRARY_PATH:+:$LIBRARY_PATH}"
export LD_LIBRARY_PATH="$CUDA_TARGET/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_CUDA_ARCH_LIST="12.0"
export MAX_JOBS="2"

PYTHON="$ENV_PREFIX/bin/python"

if ! "$PYTHON" -c 'import av; assert av.__version__ == "18.1.0"' >/dev/null 2>&1; then
  "$PYTHON" -m pip install av==18.1.0
fi

if ! "$PYTHON" -c \
  'import av, torch; import cv2, cuda_ba, cuda_corr, lietorch_backends, torch_scatter; assert av.__version__ == "18.1.0"; assert torch.__version__.startswith("2.8.0"); assert torch.cuda.is_available(); assert torch.cuda.get_device_capability(0) == (12, 0)' \
  >/dev/null 2>&1; then
  "$PYTHON" -m pip install --upgrade pip setuptools wheel
  "$PYTHON" -m pip install \
    torch==2.8.0 torchvision==0.23.0 \
    --index-url https://download.pytorch.org/whl/cu128
  "$PYTHON" -m pip install \
    torch-scatter==2.1.2 \
    -f https://data.pyg.org/whl/torch-2.8.0+cu128.html
  "$PYTHON" -m pip install \
    tensorboard==2.21.0 numba==0.67.0 tqdm==4.70.0 einops==0.8.2 \
    pypose==0.9.5 kornia==0.8.3 numpy==1.26.4 plyfile==1.1.3 \
    evo==1.37.0 opencv-python==4.11.0.86 yacs==0.1.8 gdown==6.1.0
  cd "$UPSTREAM_DIR"
  "$PYTHON" -m pip install --no-build-isolation -e .
fi

if [[ ! -f "$UPSTREAM_DIR/dpvo.pth" ]]; then
  echo "Downloading official DPVO checkpoints"
  "$PYTHON" -m gdown "$MODEL_GDRIVE_ID" --output "$MODEL_ARCHIVE"
  echo "$MODEL_ARCHIVE_SHA256  $MODEL_ARCHIVE" | sha256sum --check
  "$PYTHON" -m zipfile -l "$MODEL_ARCHIVE"
  "$PYTHON" -m zipfile -e "$MODEL_ARCHIVE" "$UPSTREAM_DIR"
fi

if [[ ! -f "$UPSTREAM_DIR/dpvo.pth" ]]; then
  echo "Expected checkpoint was not found after extracting $MODEL_ARCHIVE" >&2
  exit 1
fi

echo "$MODEL_SHA256  $UPSTREAM_DIR/dpvo.pth" | sha256sum --check

echo
echo "DPVO environment installed at $ENV_PREFIX"
echo "Run scripts/check_dpvo_env.sh next."
