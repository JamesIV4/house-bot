# Hardware Inventory

Update this file from labels, datasheets, measurements, and clear photos. Use
`unknown` instead of guessing.

## Compute

| Item | Known details | Status / questions |
|---|---|---|
| Development PC | Windows + WSL2; RTX 5060 Ti, 16 GB VRAM | Verified 2026-08-17 |
| WSL distro | Ubuntu 24.04.4 LTS, x86_64 | Verified 2026-08-17 |
| Robot computer | Raspberry Pi 3B; Raspberry Pi OS Lite / Debian 13 `trixie`, aarch64; 905 MiB RAM; 904 MiB swap; hostname `housebot` | OS, SSH, Wi-Fi, and C920 access verified 2026-08-17; portable-bank test intermittently reported `0x50005`, but native camera output remained 30.013 FPS |
| Pi storage | 29 GiB root filesystem, 25 GiB available before FFmpeg install | Card model and condition unknown |

## Mobile base and power

| Item | Known details | Status / questions |
|---|---|---|
| Wheel platform | Battery-powered wheeled platform | Manufacturer/model and photos needed |
| Drive arrangement | Two independently driven sides; forward, reverse, and both pivot combinations verified | Wheel diameter, track width, and caster arrangement still needed |
| Motors | Two LEGO-compatible geared motors with four-wire proprietary connectors | Label, rated voltage/current, stall current, gear ratio, and meaning of extra conductors unknown |
| Motor driver | Original rechargeable four-port A-D receiver/battery unit; proprietary 2.4 GHz link through `GT004TX-V01` remote | Pi GPIO matrix gating is verified; receiver driver IC and peak current unknown |
| Motor-control wiring | Nine Pi-to-remote wires: common ground plus row/column pairs for four directions | Authoritative map: `docs/REMOTE_WIRING_PINOUT.md` |
| Wheel encoders | No feedback integrated | Determine whether the motors' four conductors include encoder signals |
| Battery | Rechargeable battery is integrated into the four-port receiver; USB-C charging | Chemistry, nominal/full voltage, capacity, and BMS unknown |
| Pi regulator | unknown | Separate regulated 5 V supply? Current rating and measured output? |
| Portable power bank | Model and cable unknown | Powered Pi+C920 during calibrated route; current throttle/undervoltage bits were intermittent, while camera rate remained 30 FPS |
| Charger | unknown | Model and compatibility with battery chemistry? |

## Cameras and audio

| Item | Known details | Planned role / questions |
|---|---|---|
| Logitech HD Pro Webcam C920 | Windows DirectShow and Pi V4L2 expose MJPEG/H.264; Pi `/dev/video0` supplies native H.264 at 1280x720/30 FPS; fixed focus 0; measured `fx=940.634`, `fy=936.165`, `cx=659.789`, `cy=363.855` plus five distortion coefficients | MVP fixed mapping camera; measured calibration, native transport, and a 1,000-pose Pi route verified 2026-08-17 |
| Wyze Cam Pan v2 | Pan/tilt and night vision | Bench experiment; power, latency, PTZ state, and local control unknown |
| Microphone | Webcam(s) may contain one | Confirm capture path and robot-side suitability |
| Speaker | unknown | Needed for robot-local spoken interaction |

## Measurements to capture

- total platform length, width, height, mass, and ground clearance;
- wheel diameter and distance between driven-wheel contact centers;
- battery voltage at rest and under motor load;
- regulated Pi voltage at idle and while motors start/stop;
- motor current per side with wheels raised and under controlled load;
- encoder counts for ten wheel revolutions in each direction;
- camera mount translation and rotation relative to `base_link`.

## Photo/evidence convention

Store small reference photos under `docs/hardware/photos/` with descriptive
names. Do not commit serial numbers, Wi-Fi credentials, API keys, QR setup codes,
or labels containing private account information.
