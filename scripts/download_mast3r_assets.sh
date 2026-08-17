#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UPSTREAM_DIR="$ROOT_DIR/external/MASt3R-SLAM"
CHECKPOINT_DIR="$UPSTREAM_DIR/checkpoints"

mkdir -p "$CHECKPOINT_DIR"

download() {
  local url="$1"
  local output="$2"
  if [[ -s "$output" ]]; then
    echo "Already present: $output"
    return
  fi
  curl -L --fail --retry 3 --continue-at - "$url" --output "$output"
}

BASE_URL="https://download.europe.naverlabs.com/ComputerVision/MASt3R"
download "$BASE_URL/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth" \
  "$CHECKPOINT_DIR/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric.pth"
download "$BASE_URL/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_trainingfree.pth" \
  "$CHECKPOINT_DIR/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_trainingfree.pth"
download "$BASE_URL/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_codebook.pkl" \
  "$CHECKPOINT_DIR/MASt3R_ViTLarge_BaseDecoder_512_catmlpdpt_metric_retrieval_codebook.pkl"

echo "MASt3R checkpoints are ready in $CHECKPOINT_DIR"
