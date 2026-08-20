# Decision Log

## D-001 - SLAM is the first subsystem

- **Status:** accepted, 2026-08-17
- **Decision:** begin with standalone GPU dense SLAM using a camera attached to
  the development PC. Do not make teleoperation, motor integration, or ROS
  prerequisites.
- **Reason:** a working map and live pose are the core result. Every later robot
  subsystem can integrate against that concrete pipeline.

## D-002 - MASt3R-SLAM is the first target

- **Status:** accepted, 2026-08-17
- **Decision:** start with MASt3R-SLAM for monocular dense mapping.
- **Reason:** it accepts ordinary video and provides camera pose plus dense
  geometry, matching the available cameras and GPU-heavy architecture.

## D-003 - Standalone before ROS

- **Status:** accepted, 2026-08-17
- **Decision:** run MASt3R-SLAM directly on a sample and Logitech video before
  wrapping it in ROS 2.
- **Reason:** ROS integration does not help prove the first map and would add a
  dependency layer to the initial debugging path.

## D-004 - Logitech is the baseline camera

- **Status:** accepted, 2026-08-17
- **Decision:** use the Logitech for the first recorded and live SLAM runs.
  Evaluate the Wyze Cam Pan v2 immediately after the Logitech works.
- **Reason:** the fixed UVC source is the shortest path to real house video. The
  Wyze then adds active look-around and night vision without blocking SLAM.

## D-005 - Patch for Blackwell rather than changing GPUs

- **Status:** accepted, 2026-08-17
- **Decision:** use a PyTorch/CUDA toolchain that supports compute capability
  12.0 and patch/build MASt3R-SLAM's CUDA extensions for `sm_120`.
- **Reason:** upstream's documented PyTorch 2.5.1/CUDA 12.4 maximum predates RTX
  50-series support. The available RTX 5060 Ti is otherwise the intended GPU.

## D-006 - PC/Pi responsibility boundary

- **Status:** accepted, 2026-08-17
- **Decision:** run SLAM, perception, navigation, speech, and AI on the PC. Use
  the Raspberry Pi for camera transport and mobile-base hardware I/O.
- **Reason:** it keeps heavy computation on the RTX GPU and the robot-side code
  small.

## D-007 - TCP latest-frame camera interface

- **Status:** accepted, 2026-08-17
- **Decision:** transport H.264 MPEG-TS over TCP into a PyAV decoder that exposes
  only the newest frame to SLAM.
- **Reason:** SLAM processes fewer frames per second than the camera produces.
  Dropping stale frames preserves live behavior, while TCP works across the
  Windows/WSL boundary and maps directly to the future Pi sender.

## D-008 - Threaded viewer under WSL

- **Status:** accepted, 2026-08-17
- **Decision:** run MASt3R-SLAM visualization in a thread rather than a spawned
  process on this WSL/Blackwell environment.
- **Reason:** spawned visualization failed while rebuilding CUDA IPC storage
  with an invalid resource handle. The threaded viewer shares CUDA state
  directly and opens a stable WSLg window.

## D-009 - Dense visualization is diagnostic and runs at 2 Hz

- **Status:** accepted, 2026-08-17
- **Decision:** keep the WSLg dense viewer off the control path, downsample its
  MASt3R pointmaps by two in each dimension, and refresh the full map at 2 Hz.
- **Reason:** a moving live run dropped to about 2 FPS when the viewer redrew
  every dense keyframe at display rate. On the same 300-frame recording, a
  2 Hz full-map viewer sustained 7.21 cumulative FPS near completion versus
  3.70 FPS at 10 Hz.

## D-010 - Separate production pose, depth, and persistent mapping

- **Status:** accepted, 2026-08-17
- **Decision:** retain MASt3R-SLAM as a dense mapping baseline, but benchmark
  DPVO/DPV-SLAM as the production pose tracker. Run depth inference and person
  perception independently, and persist only bounded navigation-scale
  geometry.
- **Reason:** quarter-density MASt3R preserved nearly identical trajectory
  statistics on the recorded room loop and reduced the PLY by 75%, but the
  full headless run still slowed from roughly 9.4 to 7.6 FPS as its graph grew.
  The robot needs timely pose and local obstacles more than a continuously
  growing photorealistic point cloud.

## D-011 - Delay depth-backend selection until pose is proven

- **Status:** accepted, 2026-08-17
- **Decision:** benchmark DPVO/DPV-SLAM first. Evaluate Online Video Depth
  Anything only after the pose path meets its latency target; do not use DA3
  Streaming as the primary SLAM runtime.
- **Reason:** oVDA is promising and lightweight but produces relative,
  scale-and-shift-invariant depth that needs a metric scale source. DA3
  Streaming describes itself as not a SLAM system and its published runtime
  and VRAM figures do not improve our current control-path constraints.

## D-012 - DPV-SLAM navigation profile owns production pose

- **Status:** accepted, 2026-08-17
- **Decision:** use DPV-SLAM with 96 patches per frame and
  `GLOBAL_OPT_FREQ: 60` as the production pose candidate. Retain MASt3R-SLAM
  for dense diagnostic mapping. Reject the 48-patch fast DPV-SLAM preset and
  the stock 15-keyframe global-optimization schedule for continuous use.
- **Reason:** the selected profile sustained 16.43 FPS over a five-minute,
  3,981-sample graph-growth test, including 15.78 FPS in the final partial
  window. Stock DPV-SLAM decayed to 11.90 FPS, while the fast preset's
  one-loop normalized endpoint residual worsened to 19.67%. The selected
  profile retained a 4.70% five-loop residual and missed 2.52% of steady-state
  pose deadlines.

## D-013 - Use the C920 native H.264 stream from the Pi

- **Status:** accepted, 2026-08-17
- **Decision:** configure the Pi-attached C920 for native 1280x720/30 FPS H.264,
  disable exposure-driven dynamic framerate, copy the bitstream into MPEG-TS
  over TCP, and expose only the newest decoded frame to DPV-SLAM.
- **Reason:** the camera generated 301 frames in a 10.004-second test without
  encoding on the Pi or any throttle flags. The first live WSL smoke kept
  newest-frame queue age below 0.169 ms at p95 while deliberately discarding
  30.23% of frames that the consumer did not need. The subsequent moving run
  initialized DPV-SLAM, produced poses at an effective 16.96 Hz, and kept
  newest-frame queue age below 31.88 ms while tracking.

## D-014 - Use measured fixed-focus C920 intrinsics for navigation

- **Status:** accepted, 2026-08-17
- **Decision:** disable C920 continuous autofocus, set `focus_absolute=0`,
  undistort 1280x720 source frames with `config/c920-dpvo-measured.txt`, and use
  the measured camera matrix for subsequent Pi DPV-SLAM navigation runs.
- **Reason:** a 29-view checkerboard solution passed all coverage gates at
  0.5706 px RMS reprojection error. The first 1,000-pose measured-calibration
  route sustained 16.887 tracking FPS, kept the newest-frame queue bounded,
  and returned within a 1.150% post-initialization path residual.

## D-015 - Implement Nav2 against its loopback base before motors arrive

- **Status:** accepted, 2026-08-17
- **Decision:** run ROS 2 Jazzy in a pinned Docker image, use Nav2 for planning
  and control, and substitute the official `nav2_loopback_sim` node for the
  unfinished Pi base. Preserve `/cmd_vel`, `/odom`, and TF as the hardware
  replacement boundary.
- **Reason:** navigation actions, controller behavior, named destinations, and
  the operator UI can be integrated and tested now. The motor bring-up then
  replaces one defined adapter instead of starting a new navigation stack.

## D-016 - Use Vizanti as the MVP navigation UI

- **Status:** accepted, 2026-08-17
- **Decision:** use pinned Vizanti ROS 2 for map, pose, path, point-goal,
  waypoint, and status views. Add only a thin House Bot adapter for named Nav2
  goals and UI topic conversion.
- **Reason:** Vizanti already implements the required browser/RViz-style
  workflow. A custom dashboard would duplicate mapping and goal tools without
  improving the first autonomous navigation result.

## D-017 - Use GPIO-gated original-remote matrix control for the first base

- **Status:** accepted, 2026-08-19
- **Decision:** keep the original `GT004TX-V01` remote as the paired 2.4 GHz
  transmitter. Observe each selected button's scan row with an input-only Pi
  GPIO and pull its shared input column low only inside that row's low window.
- **Reason:** directly holding the shared column low moved both motors. BCM5
  then exposed a repeatable 215.8 us scan window every 40.15 ms, and the Pi
  mirrored 49 windows over two seconds through BCM4 while moving only the
  intended motor. This provides a verified no-new-parts command path while the
  receiver remains the motor power stage.
