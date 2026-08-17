# Session Log

Keep entries concise and evidence based. Link detailed experiment records when
they exist.

## 2026-08-17 - First C920 room map completed

### Verified

- Captured 60.033 seconds / 1,593 frames from the C920 at 1280x720.
- Processed 796 samples using every second frame in 138.7 seconds.
- Saved 73 keyframes, a 110,772,451-byte PLY, and a 73-pose trajectory.
- Reconstruction contains 7,384,818 colored points.
- Tracker successfully relocalized four times after skipped frames.
- Estimated path length is 12.332 m; final position is 0.558 m from the initial
  position. No external ground truth is available for an accuracy score.

### Fixed

- Replaced unreliable per-frame OpenCV seeking with sequential video decoding.
- Corrected video timestamps when input subsampling is enabled.
- Verified all 796 sampled positions before the successful rerun.

Detailed record:
`docs/experiments/2026-08-17-c920-room-loop.md`.

## 2026-08-17 - First SLAM result completed

### Verified

- Built PyTorch 2.8.0/CUDA 12.8 plus the MASt3R-SLAM, lietorch, and RoPE CUDA
  extensions for the RTX 5060 Ti's `sm_120` target.
- Environment GPU matrix-multiply and compiled-module import checks pass.
- The Freiburg room sequence completed headlessly in 138.7 seconds at roughly
  5.5 processing FPS after startup.
- It saved 51 keyframes, a 134,410,876-byte PLY reconstruction, and a
  TUM-format pose trajectory.
- `evo_ape` measured 0.063373 m translation RMSE after SE(3) alignment.
- Windows FFmpeg detects the Logitech as `HD Pro Webcam C920`; a 1280x720 MP4
  capture into WSL-visible `data/input/` completed and was readable by
  `ffprobe`.
- MASt3R-SLAM processed the resulting 2.03-second, 31-frame C920 smoke clip with
  exit code 0 in 16.1 seconds, proving the Windows capture-to-WSL decoder path.

### Changed

- Added reproducible bootstrap, asset download, sample download, run,
  environment-check, reference-evaluation, and C920-capture scripts.
- Added `docs/SLAM_BRINGUP.md` with the exact known-good commands and results.
- Patched upstream Blackwell compatibility, current PyTorch extension APIs, and
  optional RealSense loading without modifying the pinned upstream revision.

### Next action

- Record a deliberate 60-90 second handheld C920 room loop, run it through the
  working pipeline, and inspect the saved reconstruction and loop closure.

## 2026-08-17 - Direction changed to SLAM first

### Decision

- Removed the hardware-first and teleoperation-first sequence.
- Made standalone MASt3R-SLAM the first implementation target.
- ROS, Pi transport, mobile-base integration, navigation, and interaction now
  follow a demonstrated live mapper.

### Environment findings

- The RTX 5060 Ti reports compute capability 12.0 (`sm_120`).
- WSL can see the GPU through the Windows driver.
- WSL currently has no CUDA toolkit/compiler, PyTorch, Conda/Mamba, FFmpeg, or
  exposed `/dev/video*` device.
- Upstream MASt3R-SLAM documents PyTorch 2.5.1 with CUDA up to 12.4; RTX 50-series
  compatibility work requires a newer Blackwell-capable toolchain and extension
  API changes.

### Next action

- Add and run the reproducible MASt3R-SLAM bootstrap for PyTorch/CUDA with
  `sm_120` support.

## 2026-08-17 - Repository and MVP plan established

### Verified

- Repository exists at `/home/james/Repos/house-bot` in Ubuntu WSL.
- WSL is Ubuntu 24.04.4 LTS on WSL2, x86_64.
- Host GPU reports NVIDIA GeForce RTX 5060 Ti, 16,311 MiB VRAM, Windows driver
  596.49.
- Available robot hardware currently described as a wheeled battery platform,
  Raspberry Pi 3B, Logitech 1080p webcam, and Wyze Cam Pan v2.

### Changed

- Reworked the initial research-heavy roadmap into measurable MVP phases.
- Selected the Logitech as the fixed baseline camera.
- Added a non-destructive, time-boxed Wyze evaluation path.
- Added durable repository instructions, hardware inventory, decisions,
  references, and ignore rules.

### Unknown / next evidence needed

- Wheel platform, motor, motor driver, encoder, battery, regulator, charger, and
  disconnect details.
- Raspberry Pi OS, storage, and current software state.
- Robot-side microphone/speaker availability.
- Stable local Wyze stream, PTZ/IR control, position feedback, latency, and power
  behavior.
- ROS 2, CUDA Python, Foxglove, rosbag, and WSL-to-LAN discovery are planned but
  not yet installed or validated for this repo.
