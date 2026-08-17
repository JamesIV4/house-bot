# SLAM Bring-up

This is the shortest reproducible path from a fresh checkout to a MASt3R-SLAM
map on the development PC.

## Current verified stack

- Ubuntu 24.04 under WSL2
- NVIDIA GeForce RTX 5060 Ti, compute capability 12.0
- Python 3.11
- PyTorch 2.8.0 with CUDA 12.8
- MASt3R-SLAM pinned at `6717231a2daf55d501a5824bbec43314d4fb77d9`
- local CUDA extension builds targeting `sm_120`

The bootstrap keeps upstream source, checkpoints, the Python environment, and
generated maps outside Git. The compatibility changes themselves are stored in
`patches/mast3r-slam/`.

## Build and verify

Run from `/home/james/Repos/house-bot` in Ubuntu WSL:

```bash
./scripts/bootstrap_mast3r_slam.sh
./scripts/check_slam_env.sh
./scripts/download_mast3r_assets.sh
```

The environment check must name the RTX 5060 Ti, report capability `(12, 0)`,
complete the CUDA matrix multiplication, and import all four compiled extension
modules.

## Reproduce the reference run

```bash
./scripts/download_slam_sample.sh
./scripts/run_mast3r_slam.sh \
  external/MASt3R-SLAM/datasets/tum/rgbd_dataset_freiburg1_room \
  --config config/calib.yaml \
  --no-viz \
  --save-as house-bot-tum-room
```

Evaluate the saved trajectory:

```bash
./scripts/evaluate_tum_run.sh \
  external/MASt3R-SLAM/datasets/tum/rgbd_dataset_freiburg1_room/groundtruth.txt \
  external/MASt3R-SLAM/logs/house-bot-tum-room/rgbd_dataset_freiburg1_room.txt
```

Verified result on 2026-08-17:

- exit code 0 in 138.7 seconds;
- 51 keyframes;
- approximately 5.5 processing FPS after startup;
- 134,410,876-byte PLY reconstruction;
- 6.34 cm translation APE RMSE after SE(3) alignment.

Outputs are under `external/MASt3R-SLAM/logs/house-bot-tum-room/`.

## Record the first house loop

The C920 is attached to Windows, while SLAM runs in WSL. Windows FFmpeg bridges
that gap by recording directly into the repo's ignored `data/input/` directory.

From Windows PowerShell:

```powershell
& '\\wsl.localhost\Ubuntu\home\james\Repos\house-bot\scripts\capture_webcam.ps1' `
  -DurationSeconds 60
```

Walk a smooth loop through a textured room or two and finish facing the same
area where the recording began. The script prints the exact WSL SLAM command
for the resulting file. Run it first with `--no-viz` so the result is saved even
if the viewer has a separate display problem.

Camera calibration is intentionally not a prerequisite for the first house
map. MASt3R-SLAM can estimate monocular geometry without it. Calibrate the C920
after a recognizable loop exists, then measure whether calibration improves
the reconstruction.

The capture/decoder path was smoke-tested with a 2.03-second, 31-frame C920
recording. MASt3R-SLAM processed it successfully in 16.1 seconds and reported
about 8 processing FPS after startup. That stationary clip verified plumbing;
the completed moving loop is documented below.

## First verified house loop

The 60-second C920 loop from 2026-08-17 completed with 73 keyframes and
7,384,818 colored reconstruction points. It used
`config/c920-fast.yaml`, which samples every second frame.

See [the experiment record](experiments/2026-08-17-c920-room-loop.md) for the
exact command, metrics, artifacts, and the sequential-decoder fix discovered
during the run.
