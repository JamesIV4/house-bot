# Session Log

Keep entries concise and evidence based. Link detailed experiment records when
they exist.

## 2026-08-17 - Hardware-free Nav2 and browser UI implemented

### Verified

- The container built from the digest-pinned Jazzy base and revision-pinned
  Vizanti source, then all Nav2 lifecycle nodes reached active state.
- Four navigation-package unit tests passed.
- The end-to-end named Kitchen route succeeded at `(2.506, -0.059)`, 0.111 m
  from its `(2.6, 0.0)` target and inside the 0.30 m acceptance gate.
- The rendered browser UI showed the mock occupancy map, transforms, live
  planned path, and named buttons; its browser console reported no warnings or
  errors. Clicking Home in the browser invoked Nav2 and the return route also
  succeeded.

### Changed

- Added a pinned ROS 2 Jazzy Docker/Compose runtime with Nav2, RTAB-Map ROS,
  rosbridge, and Vizanti revision `ab43643b`.
- Added a deterministic mock house and the official Nav2 loopback simulator as
  the temporary `/cmd_vel`/`/odom` mobile base.
- Configured a small differential-base Nav2 stack using NavFn, regulated pure
  pursuit, velocity smoothing, static costmaps, behaviors, and waypoint
  actions.
- Added a House Bot adapter for point goals, waypoint lists, named-destination
  services, cancellation, and latched JSON state.
- Added a project Vizanti layout for map, TF, odometry trail, active path,
  point goals, waypoints, four named mock rooms, and navigation state.
- Added one-command launch and an end-to-end test that must reach the mock
  Kitchen within 0.30 m.

### Hardware handoff

- The real Pi/base adapter must replace loopback by consuming `/cmd_vel` and
  publishing `/odom` plus `odom -> base_link`.
- Metric localization/mapping must publish `/map` and `map -> odom`.
- Provisional 0.16 m radius and velocity limits must be replaced with measured
  assembled-base values.

Detailed guide: `docs/NAVIGATION.md`.

## 2026-08-17 - Measured C920 calibration and longer Pi route passed

### Verified

- The portable-power-bank C920 stream delivered 451 frames over 14.9934 source
  seconds, or 30.013 FPS, despite current throttle/undervoltage bits in 10 of
  12 simultaneous Pi samples.
- A fixed-focus 9x6 checkerboard capture retained 29 of 32 diverse views and
  solved 1280x720 intrinsics at 0.5706 px RMS reprojection error; the maximum
  retained-view error was 1.0503 px.
- The measured-calibration larger-space run stayed initialized for 1,000 poses
  over 60.2667 source seconds, produced poses at 16.576 Hz, and sustained 16.887
  synchronized tracking FPS.
- The run retained 154 keyframes, formed 132 long-range frame pairs, kept
  newest-frame queue age below 54.82 ms, and left no Pi camera process running.
- Relative to the closest initialized early-route pose, the final pose had a
  1.150% path-normalized translation residual and 6.10 degree orientation
  residual. Scale remains ambiguous and the checkerboard made this loop easier
  than a natural-scene return.

### Changed

- Added a reusable screen checkerboard, automatic diverse-view collector,
  calibration quality gates, and one-command Pi calibration launcher.
- Added `config/c920-dpvo-measured.txt` and selected it for Pi DPV-SLAM runs.
- Locked calibration and navigation capture to manual focus value zero and
  precomputed the undistortion maps used by the live runner.
- Added a post-initialization loop metric so DPV-SLAM's motion-probe origin no
  longer distorts route closure reporting.

Detailed record:
`docs/experiments/2026-08-17-c920-calibration-long-route.md`.

## 2026-08-17 - Raspberry Pi C920 stream reached DPV-SLAM

### Verified

- Discovered `housebot` at `192.168.0.241`: Raspberry Pi OS Lite based on
  Debian 13, aarch64, with 905 MiB RAM, 904 MiB swap, and no throttle flags.
- The attached C920 is `/dev/video0` and exposes native H.264 at
  1280x720/30 FPS.
- After disabling exposure-driven dynamic framerate, a 10.004-second native
  H.264/MPEG-TS test contained 301 frames and used 3.9 MiB.
- A bounded Pi-to-WSL DPV-SLAM smoke decoded 215 frames, processed 150, and
  deliberately dropped 65 stale frames. P95 newest-frame queue age was
  0.169 ms and p95 decoder-to-processing completion was 14.21 ms.
- The stationary camera correctly remained in DPVO's motion-probe stage; a
  physical moving-camera pass was then used for trajectory evidence.
- The launcher left no camera/FFmpeg process running on the Pi after exit.
- The moving run initialized DPV-SLAM, saved 300 poses over a 17.63-second
  source span, retained 54 keyframes, and produced seven long-range frame
  pairs. Effective live output was 16.96 Hz and synchronized compute throughput
  was 18.15 FPS.
- P95 pose processing was 59.27 ms, only 2 of 290 steady-state frames missed
  the 15 Hz compute deadline, and newest-frame age never exceeded 31.88 ms.

### Changed

- Installed FFmpeg on the fresh Pi and deployed a passwordless project SSH
  key outside the repository; the supplied password was not stored.
- Changed the Pi C920 sender to native H.264 passthrough in MPEG-TS and disabled
  dynamic exposure framerate before capture.
- Added PyAV to the DPVO environment, a newest-frame live DPV-SLAM runner, and
  a one-command Pi deploy/start/run/cleanup launcher.
- Marked the Pi newest-frame live pose gate complete; measured C920 calibration
  and a longer robot-mounted route are next.

Detailed record:
`docs/experiments/2026-08-17-pi-c920-transport.md`.

## 2026-08-17 - DPVO / DPV-SLAM production pose selected

### Verified

- Built pinned DPVO commit `859bbbfd` with PyTorch 2.8.0, CUDA 12.8, and native
  RTX 5060 Ti `sm_120` extensions.
- DPVO pose-only processed the 796-sample C920 loop at 17.33 FPS.
- Stock DPV-SLAM processed the one-minute loop at 16.07 FPS, but fell to 11.90
  FPS on a five-minute graph-growth test and no longer kept up with input.
- The upstream 48-patch fast profile reached 34.05 FPS but its normalized loop
  residual worsened to 19.67%, so it was rejected.
- The 96-patch navigation profile processed one minute at 17.59 FPS and five
  minutes at 16.43 FPS. Its final 481-frame window remained at 15.78 FPS, with
  a 4.70% five-loop residual and 2.52% steady-state deadline misses.

### Changed

- Added reproducible DPVO bootstrap, environment check, benchmark runner,
  official model checksum, C920 approximate calibration, and Blackwell patch.
- Added `config/dpvo-navigation.yaml`, reducing synchronous loop-optimization
  frequency from every 15 retained keyframes to every 60 without reducing the
  default 96 tracking patches.
- Selected this DPV-SLAM profile for the next live C920/Pi pose test while
  retaining MASt3R-SLAM for dense diagnostic maps.

Detailed record:
`docs/experiments/2026-08-17-dpvo-dpv-slam-benchmark.md`.

## 2026-08-17 - Navigation-oriented SLAM performance decision

### Verified

- The user's moving live dense-viewer run processed only about 1.9-2.1 FPS;
  the viewer was a separate bottleneck from headless MASt3R inference.
- Quarter-density pointmaps completed the same 796-sample room recording in
  117.68 seconds versus the 138.7-second baseline.
- The tuned run retained 71 versus 73 keyframes, estimated a 12.297 versus
  12.332 m path, and reduced the PLY from 7.38 million to 1.86 million points.
- A 300-sample visualized comparison reached 7.21 cumulative FPS with a 2 Hz
  full-map refresh versus 3.70 FPS with a 10 Hz refresh.
- A 60-sample live C920 headless smoke test completed with exit code 0 and
  reported 9.21 FPS at sample 30.

### Changed

- Repaired upstream `img_downsample` shared-buffer sizing.
- Added `config/c920-navigation.yaml` and applied the same pointmap/viewer
  limits to the live C920 configuration.
- Added a reproducible bounded viewer benchmark.
- Reframed MASt3R-SLAM as the dense mapping baseline, with DPVO/DPV-SLAM next
  for the production pose benchmark.

Detailed record:
`docs/experiments/2026-08-17-mast3r-navigation-performance.md`.

## 2026-08-17 - Live C920 SLAM and viewer completed

### Verified

- Windows FFmpeg connected to a WSL TCP listener and streamed live C920 H.264
  MPEG-TS at 960x540.
- PyAV 18.1.0 decoded continuously while exposing only the newest frame.
- A bounded 60-sample headless run completed at approximately 8.39 FPS and
  saved a PLY and trajectory with exit code 0.
- A second 60-sample run opened the interactive `MASt3R-SLAM (Ubuntu)` WSLg
  viewer, completed mapping, and saved artifacts while the viewer remained
  open.

### Fixed

- Replaced spawned visualization with a thread to avoid invalid CUDA IPC
  resource handles under WSL/Blackwell.
- Added environment-local `libGL.so` and `libEGL.so` loader links.
- Selected TCP after UDP failed across the Windows/WSL boundary.
- Added `--max-frames` for bounded live automation.

Detailed record: `docs/experiments/2026-08-17-live-c920.md`.

## 2026-08-17 - Pi camera transport prepared

### Verified

- No Pi responded as `raspberrypi.local`, `house-bot.local`, or
  `housebot.local`.
- Added WSL outbound `tcp-connect://` support so the Pi can serve the stream
  without exposing WSL through Windows NAT.
- Added raw MJPEG decoding for a Pi sender that does not spend Pi 3B CPU on
  video transcoding.
- A Windows-hosted stand-in MJPEG server connected to WSL, supplied 30 live
  samples to MASt3R-SLAM, and completed with exit code 0.

### Added

- `scripts/pi_stream_c920.sh`: V4L2 C920 MJPEG TCP server for the Pi.
- `scripts/run_pi_camera_slam.sh`: WSL viewer/mapper connection to a Pi host.

### Next evidence needed

- Pi hostname or IP, Raspberry Pi OS version, and SSH availability.
- C920 V4L2 device path and supported MJPEG mode on the Pi.
- Actual Wi-Fi throughput, delay, dropped-frame behavior, and reconnect result.

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
