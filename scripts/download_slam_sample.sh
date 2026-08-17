#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_DIR="$ROOT_DIR/external/MASt3R-SLAM/datasets/tum"
ARCHIVE="$DATASET_DIR/rgbd_dataset_freiburg1_room.tgz"
EXTRACTED="$DATASET_DIR/rgbd_dataset_freiburg1_room"
URL="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_room.tgz"

mkdir -p "$DATASET_DIR"

if [[ ! -s "$ARCHIVE" ]]; then
  curl -L --fail --retry 3 --continue-at - "$URL" --output "$ARCHIVE"
fi

if [[ ! -d "$EXTRACTED/rgb" ]]; then
  tar -xzf "$ARCHIVE" -C "$DATASET_DIR"
fi

echo "$EXTRACTED"
