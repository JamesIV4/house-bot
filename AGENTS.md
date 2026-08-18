# House Bot Project Instructions

This repository is the durable source of truth for the project.

## Priority

Get to the working end result quickly. The active order is:

1. dense SLAM from the Logitech camera;
2. live camera transport from the robot;
3. robot-mounted mapping and localization;
4. navigation;
5. person interaction.

Do not insert hardware bringup, simulation, ROS infrastructure, dashboards, or
other preliminary work before SLAM unless a direct SLAM blocker requires it.

## Before making changes

1. Read `robot_development_plan.md`.
2. Read `docs/DECISIONS.md` and preserve the active decisions.
3. Read the newest entry in `docs/SESSION_LOG.md`.

## Development practice

- Run Linux commands from `/home/james/Repos/house-bot` inside Ubuntu WSL.
- Keep reproducible project scripts under `scripts/` and local upstream patches
  under `patches/`.
- Fetch external projects under `external/`; do not vendor their Git history.
- Pin upstream commits and dependency versions.
- Keep checkpoints, environments, source video, generated maps, and credentials
  out of Git.
- Prefer the shortest live experiment that proves or disproves the current
  approach.
- Update `docs/SESSION_LOG.md` with exact results and errors.
- Update `docs/DECISIONS.md` when evidence changes the architecture.

## Current implementation target

The measured fixed-focus C920 calibration and 1,000-pose live Pi route have
passed. Identify the base motor driver and encoders, rigidly mount and measure
the C920-to-base transform, then repeat the calibrated
`config/dpvo-navigation.yaml` route from a natural starting scene. Preserve
tuned MASt3R-SLAM as the dense mapping baseline, including the Blackwell,
live-stream, pointmap-downsample, and 2 Hz WSL viewer patches. Monocular pose is
not metric until wheel motion or another scale observation is integrated.
