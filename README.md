# House Bot

House Bot is a small indoor robot built around a wheeled battery platform, a
Raspberry Pi 3B, and remote PC compute. The project starts with GPU dense SLAM,
then puts the working mapper on the robot and adds navigation and interaction.

Start with [the SLAM-first development plan](robot_development_plan.md).

## Current status

The standalone mapper is working on the RTX 5060 Ti. The pinned Blackwell CUDA
environment passes its GPU/import checks, and the TUM RGB-D reference run
measured 6.34 cm translation APE RMSE after alignment.

The first 60-second handheld C920 room loop also completed: 73 keyframes and
7.38 million colored reconstruction points. See [SLAM bring-up](docs/SLAM_BRINGUP.md)
and the [first C920 experiment](docs/experiments/2026-08-17-c920-room-loop.md).

Continuous C920 input and the interactive WSLg map viewer are now working. The
[live C920 experiment](docs/experiments/2026-08-17-live-c920.md) documents the
one-command launcher and verified transport.

The moving viewer exposed a performance split: unthrottled dense visualization
reduced live processing to about 2 FPS, while headless MASt3R-SLAM remained
limited to roughly 7-9 FPS and slowed as its graph grew. Quarter-density
pointmaps plus a 2 Hz full-map viewer now preserve 7-8 FPS with visualization
and reduce saved geometry by 75%. See the
[navigation performance experiment](docs/experiments/2026-08-17-mast3r-navigation-performance.md).

MASt3R-SLAM is now the dense mapping baseline rather than the robot control
loop. The [DPVO/DPV-SLAM benchmark](docs/experiments/2026-08-17-dpvo-dpv-slam-benchmark.md)
selected the 96-patch House Bot DPV-SLAM navigation profile for production
pose: 17.59 FPS on the 60-second C920 loop and 16.43 FPS over a five-minute
graph-growth stress test. Stock DPV-SLAM decayed to 11.90 FPS on the long run;
the 48-patch fast profile was rejected for substantially worse loop drift.

The Pi is now online with Raspberry Pi OS Lite, and its C920 supplies native
1280x720/30 H.264 without Pi-side encoding. A bounded newest-frame test reached
the WSL DPV-SLAM environment with 0.169 ms p95 queue age and cleanly discarded
unneeded stale frames. See the
[Pi C920 transport experiment](docs/experiments/2026-08-17-pi-c920-transport.md).

The C920 now has a measured fixed-focus 1280x720 calibration at 0.5706 px RMS
reprojection error. With undistortion enabled, a larger-space Pi-camera pass
saved 1,000 poses over 60.27 source seconds at 16.58 effective pose Hz, retained
154 keyframes, and closed to 1.15% of post-initialization path length. See the
[calibration and longer-route record](docs/experiments/2026-08-17-c920-calibration-long-route.md).
The base now has a verified Pi-controlled motor path, and the next gate is
[rigid camera/base and motion calibration](docs/BASE_CALIBRATION.md). The real
ROS bridge is calibration-gated, starts disarmed, retains the Pi watchdog, and
labels its initial odometry as open-loop rather than presenting it as encoder
feedback.

While the base is being assembled, the navigation and operator layers are now
implemented against Nav2's official loopback base. The browser UI exposes the
mock occupancy map, pose trail, planned path, point goals, waypoints, named
rooms, status, and cancellation. Start it with `./run_navigation_ui.sh`; see
[Navigation and Browser UI](docs/NAVIGATION.md) for the hardware handoff.

## Working principles

- Optimize for a working end-to-end result.
- Start SLAM standalone; add ROS when the mapper works.
- Keep the camera input replaceable so local video, Pi video, and the Wyze can
  feed the same pipeline.
- Run expensive perception and AI workloads on the PC GPU.
- Pin and automate every environment needed to reproduce a result.

## Repository map

```text
docs/                       Project knowledge and decisions
scripts/                    Environment and run automation
patches/                    Upstream compatibility patches
config/                     Camera and SLAM configuration
navigation/                 Pinned ROS 2 Docker/Compose runtime
ros_ws/src/                 House Bot ROS packages
data/                       Ignored local recordings and generated data
robot_development_plan.md   SLAM-first roadmap and acceptance criteria
```

Large videos, model weights, generated maps, build products, and secrets do not
belong in Git. Their expected local locations are listed in `.gitignore`.
