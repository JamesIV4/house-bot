# Technical References

These links inform experiments; they are not substitutes for recording the
exact versions and results used by House Bot.

## Dense SLAM

- [MASt3R-SLAM upstream repository](https://github.com/rmurai0610/MASt3R-SLAM)
- [MASt3R-SLAM project and paper](https://edexheim.github.io/mast3r-slam/)
- [Upstream RTX 50-series compatibility work](https://github.com/rmurai0610/MASt3R-SLAM/pull/86)

Upstream supports video files, image folders, and a RealSense live input. Its
documented PyTorch 2.5.1/CUDA combinations predate Blackwell support, so House
Bot maintains a reproducible `sm_120` setup rather than using those pins
unchanged.

## ROS 2 and platform

- [ROS 2 Jazzy installation on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 on Raspberry Pi](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html)

ROS 2 Jazzy targets Ubuntu 24.04 on x86-64 and ARM64. The Pi 3B's limited memory
still makes a minimal robot-side installation and measured load important.

## Wyze Cam Pan v2 experiments

- [docker-wyze-bridge project and supported cameras](https://github.com/mrlt8/docker-wyze-bridge)
- [docker-wyze-bridge stream URI documentation](https://github.com/mrlt8/docker-wyze-bridge/wiki/Camera-Stream-URIs)
- [docker-wyze-bridge WebUI API](https://github.com/mrlt8/docker-wyze-bridge/wiki/WebUI-API)
- [Thingino firmware](https://github.com/themactep/thingino-firmware)
- [Thingino installer tooling and recovery warning](https://github.com/wltechblog/thingino-installers)

The stock-firmware bridge documents Pan v2 video support without special
firmware. Thingino is a possible local RTSP/ONVIF/PTZ path and has community
Pan v2 evidence, but flashing is a separate risk-bearing experiment and exact
hardware/firmware compatibility must be verified first.
