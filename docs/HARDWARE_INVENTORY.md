# Hardware Inventory

Update this file from labels, datasheets, measurements, and clear photos. Use
`unknown` instead of guessing.

## Compute

| Item | Known details | Status / questions |
|---|---|---|
| Development PC | Windows + WSL2; RTX 5060 Ti, 16 GB VRAM | Verified 2026-08-17 |
| WSL distro | Ubuntu 24.04.4 LTS, x86_64 | Verified 2026-08-17 |
| Robot computer | Raspberry Pi 3B | RAM/storage, OS, case, and cooling unknown |
| Pi storage | unknown | Card size/model and condition? |

## Mobile base and power

| Item | Known details | Status / questions |
|---|---|---|
| Wheel platform | Battery-powered wheeled platform | Manufacturer/model and photos needed |
| Drive arrangement | unknown | Differential drive? Caster count? Wheel diameter? Track width? |
| Motors | unknown | Label, rated voltage/current, stall current, gear ratio? |
| Motor driver | unknown | Board/model, logic voltage, control protocol, peak current? |
| Wheel encoders | unknown | Present? Type, counts/revolution, voltage, connector/pinout? |
| Battery | unknown | Chemistry, nominal/full voltage, capacity, connector, BMS? |
| Pi regulator | unknown | Separate regulated 5 V supply? Current rating and measured output? |
| Charger | unknown | Model and compatibility with battery chemistry? |

## Cameras and audio

| Item | Known details | Planned role / questions |
|---|---|---|
| Logitech HD Pro Webcam C920 | DirectShow exposes MJPEG/H.264 up to 1920x1080 at 30 FPS | MVP fixed mapping camera; 1280x720 MP4 capture and SLAM decode verified |
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
