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
data/                       Ignored local recordings and generated data
robot_development_plan.md   SLAM-first roadmap and acceptance criteria
```

Large videos, model weights, generated maps, build products, and secrets do not
belong in Git. Their expected local locations are listed in `.gitignore`.
