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
