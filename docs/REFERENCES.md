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

## Real-time pose and scheduled depth candidates

- [DPVO/DPV-SLAM official repository](https://github.com/princeton-vl/DPVO)
- [DPVO RTX 50-series / CUDA 12.8 build guide](https://github.com/princeton-vl/DPVO/issues/100)
- [Logitech C920e data sheet](https://www.logitech.com/content/dam/logitech/en_sg/video-collaboration/pdf/c920e-datasheet.pdf)
- [Online Video Depth Anything official repository](https://github.com/FriedFeid/OnlineVideoDepthAnything)
- [Depth Anything 3 Streaming documentation](https://github.com/ByteDance-Seed/Depth-Anything-3/blob/main/da3_streaming/README.md)

DPV-SLAM is the selected production pose candidate because it provides video
input, configurable stride, trajectory output, and long-range loop closure.
The RTX 50-series guide documents the PyTorch extension API changes and
`sm_120` build needed by the project's 5060 Ti. The Logitech data sheet's
published 78-degree diagonal field of view was used only for the explicitly
approximate benchmark calibration; a measured calibration remains required.
oVDA is a later local-depth candidate; its released small model produces relative,
scale-and-shift-invariant depth, so persistent metric fusion needs an external
scale source. DA3 Streaming is not being treated as the robot's SLAM tracker:
its official documentation explicitly separates it from SLAM and reports
runtime and memory figures that do not beat the current control-path target.

## ROS 2 and platform

- [ROS 2 Jazzy installation on Ubuntu 24.04](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [ROS 2 on Raspberry Pi](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html)
- [Navigation2 repository](https://github.com/ros-navigation/navigation2)
- [Nav2 configuration guide](https://docs.nav2.org/configuration/index.html)
- [Nav2 loopback simulator](https://docs.nav2.org/configuration/packages/configuring-loopback-sim.html)
- [RTAB-Map ROS 2 integration](https://github.com/introlab/rtabmap_ros)
- [Vizanti ROS 2 browser UI](https://github.com/MoffKalast/vizanti)

ROS 2 Jazzy targets Ubuntu 24.04 on x86-64 and ARM64. The Pi 3B's limited memory
still makes a minimal robot-side installation and measured load important.
Nav2's loopback simulator is the project's hardware-free `/cmd_vel` and
odometry stand-in. Vizanti supplies the ROS-native browser map/goal UI, and
RTAB-Map ROS remains the established occupancy-mapping integration once metric
motion and depth/point-cloud inputs exist.

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
