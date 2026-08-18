# C920 Calibration and Longer Pi Route - 2026-08-17

## Outcome

The Pi-mounted C920 now has measured 1280x720 intrinsics and distortion for a
fixed-focus navigation mode. A subsequent 60.27-second larger-space pass kept
DPV-SLAM initialized for 1,000 poses, sustained fresh output above 15 Hz, and
returned close to the initialized start of the route.

The portable power bank intermittently asserted the Pi's current undervoltage
and throttle flags, but a simultaneous transport measurement showed that this
did not reduce the camera's native H.264 frame rate. It remains a system
reliability warning, not a demonstrated camera-throughput blocker.

## Fixed camera mode

- source: Pi-attached Logitech C920 at 1280x720/30 FPS native H.264;
- focus: continuous autofocus disabled, `focus_absolute=0`;
- exposure-driven dynamic frame rate: disabled;
- transport: unchanged H.264 in MPEG-TS over TCP;
- target: 10x7 checkerboard displayed on a flat screen, producing 9x6 inner
  corners;
- calibration implementation: OpenCV checkerboard detection and camera model.

The physical square size is not needed for intrinsic calibration. The focus
setting is now applied by both the calibration and live DPV-SLAM launchers so
the lens state matches the measured model.

## Power-bank frame-rate check

After discarding three seconds of decoder warmup, a clean 15-second interval
produced:

| Metric | Result |
|---|---:|
| Frames | 451 |
| Source span | 14.9934 s |
| Source rate | 30.013 FPS |
| Arrival rate | 29.974 FPS |
| Maximum source-frame gap | 41.22 ms |
| Power samples with current throttle/undervoltage bits | 10 / 12 |
| Observed ARM frequency range | 700-1200 MHz |

DPV-SLAM runs on the development PC, so Pi CPU frequency changes do not affect
tracking compute. This test proves the C920 path remained at its requested
rate; it does not establish that the power bank is suitable for future motor
I/O or every Pi workload.

## Calibration result

The automatic collector accepted 32 diverse views and rejected three
reprojection outliers. All quality gates passed.

| Metric | Result |
|---|---:|
| Retained views | 29 |
| RMS reprojection error | 0.5706 px |
| Mean per-view error | 0.5181 px |
| Maximum retained-view error | 1.0503 px |
| Horizontal centroid coverage | 46.49% of image width |
| Vertical centroid coverage | 37.69% of image height |
| Board area range | 4.54%-26.38% of image |

Selected DPVO calibration at source resolution:

```text
fx=940.634358268 fy=936.165045978 cx=659.789057793 cy=363.854720686
k1=0.128634665 k2=-0.329331146 p1=-0.002547356 p2=0.003340322 k3=0.200722308
```

The committed runtime file is `config/c920-dpvo-measured.txt`. The live runner
precomputes OpenCV undistortion maps and remaps each source frame before the
existing half-resolution DPV-SLAM input conversion.

## Larger-space DPV-SLAM route

Command:

```bash
./scripts/run_pi_camera_dpvo.sh 192.168.0.241 5600 1000
```

| Metric | Result |
|---|---:|
| Source span | 60.2667 s |
| Processed poses | 1,000 |
| Effective pose output over source span | 16.576 Hz |
| Synchronized tracking compute | 16.887 FPS |
| Retained keyframes | 154 |
| Long-range frame pairs, gap greater than 30 | 132 |
| Global BA states | 3 |
| P95 GPU SLAM call | 63.51 ms |
| GPU calls missing the 15 Hz deadline | 23 / 1,000 |
| P95 newest-frame queue age | 29.00 ms |
| Maximum newest-frame queue age | 54.82 ms |
| Frames decoded / deliberately dropped stale | 1,810 / 809 |

The consumer remained ahead of the freshness deadline: no newest-frame queue
sample exceeded 66.67 ms. The 44.72% stale-drop rate is intentional because a
30 FPS source feeds a roughly 16.6 Hz pose consumer.

### Loop interpretation

DPV-SLAM's first returned poses include its pre-initialization origin and two
large initialization transitions. The raw first-to-final comparison therefore
is not a useful loop metric. Searching the initialized 2.5-10.0 second route
window found the final pose nearest the pose at 3.3667 seconds:

| Metric | Result |
|---|---:|
| Translation residual, scale ambiguous | 0.17390 |
| Path length after matched early pose, scale ambiguous | 15.1170 |
| Translation residual / path | 1.150% |
| Orientation residual | 6.10 degrees |

The trajectory remained connected and the graph formed 132 long-range frame
pairs. The checkerboard was visible at the natural start/end of this run,
which made final place recognition easier than an ordinary room view. A future
robot-mounted acceptance pass should start on a natural textured scene.

## Artifacts

Calibration artifacts:

```text
data/output/c920-calibration/c920-1280x720-20260817-163509/
```

Long-route artifacts:

```text
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-163707.json
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-163707.tum.txt
data/output/dpvo-pi-live/pi-c920-dpvo-20260817-163707.trajectory.png
```

The Pi camera process was absent after launcher cleanup. Its final power state
was `0x50000`, meaning the current-condition bits had cleared while the
since-boot history remained.

## Remaining limitation

This route has no external ground truth and monocular translation is still
scale ambiguous. The result verifies timely, loop-closing live pose with the
measured lens model. Wheel odometry or another metric observation is still
required before treating the trajectory as metric navigation state.
