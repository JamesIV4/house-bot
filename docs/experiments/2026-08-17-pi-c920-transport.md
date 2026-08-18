# Raspberry Pi C920 Transport - 2026-08-17

## Outcome

The Raspberry Pi 3B now serves the attached C920's native H.264 stream over
Wi-Fi to the selected DPV-SLAM environment in WSL. The Pi does not encode
video; FFmpeg only supplies wall-clock timestamps, wraps the camera bitstream
in MPEG-TS, and serves it over TCP.

A stationary end-to-end smoke test proved the camera, Pi, Wi-Fi, TCP stream,
PyAV decoder, newest-frame handoff, GPU input, automatic deployment, and clean
remote-process shutdown. A subsequent moving-camera run initialized DPV-SLAM,
sustained the pose target, and saved a 300-pose trajectory.

## Verified Pi state

- host: `housebot`; current IPv4 address: `192.168.0.241`
- Raspberry Pi OS Lite based on Debian 13 `trixie`, aarch64
- kernel: `6.18.34+rpt-rpi-v8`
- memory: 905 MiB RAM and 904 MiB swap
- root storage: 29 GiB total, 25 GiB available before FFmpeg installation
- Wi-Fi interface: `wlan0`; Ethernet is disconnected
- `vcgencmd get_throttled`: `0x0` before and after camera tests
- C920 device: `/dev/video0`, accessible through the `video` group
- FFmpeg 7.1.5 installed from Raspberry Pi OS packages

No login password is stored in the repository. Initial access installed a
dedicated `house-bot-dev` ED25519 public key. Its private key remains at
`/home/james/.ssh/housebot_ed25519` in WSL.

## Camera mode

The C920 advertised native H.264 and MJPEG at 1280x720/30 FPS. Native H.264 was
selected because it avoids Pi-side encoding and uses much less Wi-Fi bandwidth
than MJPEG.

The first native-H.264 capture produced only 171 frames because the webcam had
`exposure_dynamic_framerate=1`. The sender now sets that control to zero before
capture. With dynamic framerate disabled and MPEG-TS timestamps generated, a
bounded file test produced:

```text
codec: H.264
resolution: 1280x720
frames: 301
duration: 10.004 seconds
size: 3.9 MiB
camera rate: 30 FPS
Pi throttle flags: 0x0
```

The initial C920 H.264 packet has no timestamp, so FFmpeg reports and repairs
one non-monotonic DTS at stream startup. The resulting MPEG-TS decoded
continuously. A broken-pipe warning at shutdown is expected when the bounded
consumer disconnects.

## Live newest-frame smoke test

Command from Ubuntu WSL:

```bash
DPVO_ALLOW_UNINITIALIZED=1 \
  ./scripts/run_pi_camera_dpvo.sh 192.168.0.241 5600 150
```

The launcher copies the versioned sender script to the Pi, starts only that
camera process, runs the WSL consumer, and terminates its recorded remote PID
on exit.

Measured result:

| Metric | Result |
|---|---:|
| Frames decoded | 215 |
| Frames processed | 150 |
| Stale frames intentionally dropped | 65 (30.23%) |
| Synchronized processing rate | 24.03 FPS |
| P95 newest-frame queue age | 0.169 ms |
| P95 decoder-arrival to completed processing | 14.21 ms |
| Steady-state 15 Hz deadline misses | 1 / 140 (0.71%) |
| Retained keyframes | 1 |
| DPV-SLAM initialized | no; camera was stationary |
| Remote camera process after run | none |

The 24 FPS figure is not the moving SLAM rate: a stationary image remains in
DPVO's inexpensive motion-probe stage. The prior moving-file benchmark remains
the realistic compute expectation until the camera is physically moved during
a live run.

Local ignored evidence:

```text
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-144008.json
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-144008.first-frame.jpg
```

## Moving DPV-SLAM run

The camera was physically moved through the room and returned toward its
starting view while this command ran:

```bash
./scripts/run_pi_camera_dpvo.sh 192.168.0.241 5600 300
```

| Metric | Result |
|---|---:|
| DPV-SLAM initialized | yes |
| Timestamped poses | 300 |
| Source-camera span | 17.63 seconds |
| Effective live pose rate over source span | 16.96 Hz |
| Synchronized compute throughput | 18.15 FPS |
| Frames decoded at tracking end | 532 |
| Stale frames intentionally dropped | 230 (43.40%) |
| Retained keyframes | 54 |
| P95 ingest-to-pose processing | 59.27 ms |
| Steady-state 15 Hz deadline misses | 2 / 290 (0.69%) |
| P95 newest-frame queue age | 27.91 ms |
| Maximum newest-frame queue age | 31.88 ms |
| P95 decoder arrival to ready pose | 85.13 ms |
| Long-range frame pairs | 7 |
| Global-BA frame states | 2 |
| Shutdown optimization | 0.75 seconds |
| Reserved GPU memory | 2.44 GiB |
| Start/end displacement over path | 9.61% |
| Pi throttle flags after run | `0x0` |
| Remote camera process after run | none |

The newest-frame queue never exceeded one 30 FPS camera period, so the
pipeline did not accumulate stale video even while tracking and global
optimization ran. The decoder-to-pose p95 includes up to one camera period of
phase/queue age plus the synchronized GPU pose computation.

The trajectory contains real camera motion and loop edges, but it is not an
accuracy result. Translation remains monocular and scale-ambiguous, the camera
still uses approximate FOV-derived intrinsics, and this short hand-moved route
has no ground truth. The first two accepted source frames were separated by
1.97 seconds while CUDA and the tracker warmed up; keep the robot stationary
for the first two seconds of future sessions.

Moving-run evidence:

```text
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-160627.json
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-160627.tum.txt
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-160627.trajectory.png
```

## Next step

Replace the approximate C920 intrinsics with a measured calibration, keep the
robot stationary during tracker warmup, then run a longer robot-mounted room
loop. Verify:

- the calibrated trajectory remains connected over multiple rooms;
- the camera-to-base transform is fixed and measured;
- wheel motion provides metric scale when the drive hardware is available.
