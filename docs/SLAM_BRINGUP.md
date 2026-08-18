# SLAM Bring-up

This is the shortest reproducible path from a fresh checkout to a MASt3R-SLAM
map on the development PC.

For the selected sparse production pose path, build and run DPV-SLAM instead:

```bash
./scripts/bootstrap_dpvo.sh
./scripts/check_dpvo_env.sh
./scripts/run_dpvo_benchmark.sh navigation
```

The selected `config/dpvo-navigation.yaml` profile sustained 17.59 FPS on the
60-second C920 loop and 16.43 FPS on a five-minute graph-growth test. See the
[DPVO/DPV-SLAM benchmark](experiments/2026-08-17-dpvo-dpv-slam-benchmark.md).
The calibration used by that benchmark is explicitly approximate; measure the
fixed robot camera before treating its monocular trajectory as navigation
geometry.

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

## Run live C920 SLAM

From the repository root in Ubuntu WSL:

```bash
./run_3d_visualizer.sh
```

The Windows desktop shortcut named `House Bot 3D Visualizer` enters WSL and
runs this same Bash entry point. The Bash launcher uses Windows only for the
C920 DirectShow capture, then launches the WSL listener and interactive 3D
viewer. Close the MASt3R-SLAM viewer to finish the session and save its map.

The live configuration keeps MASt3R's 512-pixel neural input, reduces dense
pointmaps to quarter density, and refreshes the full diagnostic map at 2 Hz.
The 2 Hz viewer is intentional: it keeps tracking near the headless rate
instead of consuming the same GPU and locks at display refresh rate.

For an automatic headless check:

```powershell
& '\\wsl.localhost\Ubuntu\home\james\Repos\house-bot\scripts\run_live_c920.ps1' `
  -Headless -MaxFrames 60 -SaveAs live-check
```

See [the live C920 experiment](experiments/2026-08-17-live-c920.md) for the
transport design and verified results. See the
[navigation performance experiment](experiments/2026-08-17-mast3r-navigation-performance.md)
for the pointmap and viewer benchmarks.

To repeat the bounded visual performance test:

```bash
./scripts/benchmark_mast3r_viewer.sh
```

## Move the camera stream to the Pi

On the Raspberry Pi, with the C920 attached and FFmpeg installed:

```bash
./scripts/pi_stream_c920.sh /dev/video0 5600
```

The sender serves the C920's native compressed video without transcoding. On
the WSL development PC, connect MASt3R-SLAM to it:

```bash
./scripts/run_pi_camera_slam.sh <pi-host-or-ip>
```

Start the Pi sender first; it waits for the WSL mapper to connect. The same
latest-frame adapter, viewer, mapping code, and output format are used for the
Windows and Pi camera sources.

The verified Pi path now defaults to the C920's native H.264 rather than MJPEG.
It disables exposure-driven dynamic framerate, supplies timestamps, and wraps
the untouched bitstream in MPEG-TS. For the selected DPV-SLAM pose path, one
WSL command deploys the sender, starts it over SSH, runs a bounded newest-frame
session, and cleans up the exact remote process:

```bash
./scripts/run_pi_camera_dpvo.sh 192.168.0.241 5600 300
```

The address is the Pi's current DHCP address and may change. The command needs
physical camera motion to initialize DPV-SLAM. See the
[Pi transport record](experiments/2026-08-17-pi-c920-transport.md) for the
stationary smoke metrics and SSH key location.

## Calibrate the Pi-mounted C920

Open the reusable 9x6-inner-corner target full-screen:

```text
docs/calibration/c920-checkerboard-9x6.svg
```

Then collect 32 automatically selected views:

```bash
./scripts/run_pi_camera_calibration.sh 192.168.0.241 5600 32
```

Move the camera left, right, high, low, close, far, and through tilted views
while keeping the whole board visible. The launcher fixes focus at zero and
rejects insufficiently sharp or redundant frames. Calibration candidates and
quality metrics are written under `data/output/c920-calibration/`.

The selected result is `config/c920-dpvo-measured.txt`: 29 retained views,
0.5706 px RMS reprojection error, and full coverage-gate success. Pi DPV-SLAM
launches use that file by default and apply the same fixed-focus setting.

To repeat the verified one-minute pose route:

```bash
./scripts/run_pi_camera_dpvo.sh 192.168.0.241 5600 1000
```

See the [calibration and longer-route record](experiments/2026-08-17-c920-calibration-long-route.md)
for the measured lens values, portable-power frame-rate check, tracking
latencies, and loop interpretation.
