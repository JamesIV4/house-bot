# DPVO / DPV-SLAM C920 Benchmark - 2026-08-17

## Outcome

DPV-SLAM with the House Bot navigation profile is the production pose
candidate. It sustained the current 13.27 Hz input rate for five minutes while
preserving the upstream default 96 patches per frame. MASt3R-SLAM remains the
dense mapping and visualization baseline.

The stock DPV-SLAM configuration passed the 60-second test but failed the
longer throughput gate as its synchronous global bundle-adjustment work grew.
The upstream 48-patch fast preset was rejected because its room-loop residual
was much worse despite excellent speed.

## Reproducible environment

- GPU: NVIDIA GeForce RTX 5060 Ti, compute capability 12.0
- OS: Ubuntu 24.04 under WSL2
- DPVO commit: `859bbbfdac6c6185f345003b3c473901fcd13ace`
- Python 3.11, PyTorch 2.8.0+cu128, CUDA toolkit 12.8
- official `dpvo.pth` SHA-256:
  `30d02dc2b88a321cf99aad8e4ea1152a44d791b5b65bf95ad036922819c0ff12`
- local Blackwell/PyTorch compatibility patch:
  `patches/dpvo/blackwell-torch28.patch`

Build and verify:

```bash
./scripts/bootstrap_dpvo.sh
./scripts/check_dpvo_env.sh
```

The environment check loaded `cuda_corr`, `cuda_ba`, and
`lietorch_backends`, and confirmed `(12, 0)` GPU capability.

## Input and calibration

The matched one-minute input was:

```text
data/input/c920-room-loop-20260817-121954.mp4
1280x720, 1593 frames, 26.5353 FPS, 60.033 seconds
```

All runs used stride 2: 796 tracked images at 13.2676 Hz. DPVO's video path
resized those images to 640x352. Per-frame timings synchronize the GPU and
include video decode, resize, host-to-device transfer, tracking, and any
synchronous optimization before a pose is considered available.

No measured C920 calibration exists yet. `config/c920-dpvo-approx.txt` is an
explicit benchmark-only centered pinhole estimate derived from Logitech's
published 78-degree diagonal field of view for a 16:9 1280x720 image:

```text
fx=906.787933 fy=906.787933 cx=640 cy=360
```

It has no distortion coefficients. This is sufficient for backend selection,
but it is not the final navigation calibration.

## One-minute matched results

| Backend/profile | Tracking FPS | Real-time margin | P95 pose latency | Shutdown | Retained keyframes | Long-range pairs | Reserved VRAM | Start/end over path |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| DPVO default, no loop closure | 17.33 | 1.31x | 64.54 ms | 0.56 s | 220 | 0 | 3.44 GiB | 6.20% |
| DPV-SLAM default | 16.07 | 1.21x | 93.06 ms | 1.70 s | 218 | 830 | 3.31 GiB | 5.65% |
| DPV-SLAM fast, 48 patches | 34.05 | 2.57x | 37.81 ms | 0.76 s | 220 | 1,247 | 0.96 GiB | 19.67% |
| DPV-SLAM navigation | 17.59 | 1.33x | 64.21 ms | 2.51 s | 221 | 1,123 | 4.73 GiB | 8.68% |
| Tuned MASt3R dense baseline | 7.63 near end | 0.58x | not measured | included in 117.68 s | 71 | n/a | n/a | 4.45% |

The fast profile is not the winner. Its normalized endpoint error is 3.5 times
the stock DPV-SLAM result on a recording intended to finish near its start.
Absolute monocular path lengths are scale-ambiguous, so the comparison uses
start/end displacement divided by total path length.

The navigation profile keeps the upstream default tracking density and changes
`GLOBAL_OPT_FREQ` from 15 to 60 retained keyframes. On the one-minute run, 19
of 796 pose deadlines were missed (2.39%); after the first ten frames, 17 of
786 were missed (2.16%).

Reproduce the selected one-minute run:

```bash
./scripts/run_dpvo_benchmark.sh navigation
```

## Five-minute graph-growth stress test

The stress input repeats the room loop five times without re-encoding:

```text
data/input/c920-room-loop-5x-20260817.mp4
1280x720, 7963 frames, 300.066 seconds
```

At stride 2, both configurations tracked 3,981 images. Repetition makes loop
recognition easier than a genuinely new multi-room route, but it still grows
the retained keyframes, inactive factors, long-range graph, and bundle
adjustment workload. It is therefore a throughput and memory stress test, not
an accuracy benchmark.

| Result | Stock DPV-SLAM | Navigation profile |
|---|---:|---:|
| Tracking wall time | 334.57 s | 242.29 s |
| Tracking FPS | 11.90 | 16.43 |
| Tracking + shutdown FPS | 11.64 | 16.00 |
| P95 pose latency | 223.54 ms | 65.19 ms |
| Final 481-frame window | below 13.27 Hz | 15.78 FPS |
| Deadline misses after 10 frames | not captured in first harness version | 100 / 3,971 (2.52%) |
| Retained keyframes | 1,071 | 1,064 |
| Inactive patch factors | 3,204,768 | 2,942,592 |
| Long-range frame pairs | 11,395 | 9,489 |
| Global-BA frame states | 375 | 89 |
| Reserved VRAM | 6.49 GiB | 6.66 GiB |
| Shutdown optimization | 7.50 s | 6.46 s |
| Start/end over path | 7.84% | 4.70% |

Stock DPV-SLAM crossed below the 13.27 Hz input rate around sample 2,900 and
finished at 11.90 FPS. The navigation profile remained above the 15 Hz project
gate in every 500-frame window; its slowest window was 15.16 FPS. It reduced
the number of global-BA frame states by 76% while retaining thousands of
long-range constraints.

The one-minute and five-minute JSON metrics, TUM trajectories, logs, and the
navigation run's per-frame latency CSV are under the ignored local directory:

```text
data/output/dpvo-c920-room-20260817/
data/output/dpvo-c920-room-5x-20260817/
```

## Decision and limits

Use `config/dpvo-navigation.yaml` for the next live C920 and Pi stream work.
Do not use the 48-patch fast preset, and do not use stock DPV-SLAM's 15-keyframe
global-optimization frequency for an accumulating route.

This benchmark selects the pose architecture, not a finished localization
system:

- the C920 still needs a measured intrinsic/distortion calibration;
- there is no external trajectory ground truth for this house recording;
- monocular translation has arbitrary scale and needs wheel odometry or
  another scale observation before metric navigation;
- the upstream 4,096-keyframe buffer and accumulated inactive factors are not
  an indefinite-lifetime map. Continuous operation will need a rolling active
  graph plus saved submaps or deliberate session boundaries;
- the next gate is the existing newest-frame live/Pi transport, where stale
  frames must be dropped rather than queued.
