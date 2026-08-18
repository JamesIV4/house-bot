# MASt3R-SLAM Navigation Performance - 2026-08-17

## Outcome

MASt3R-SLAM remains useful as the dense mapping baseline, but it is not fast
enough to own the robot's production control loop on this machine. The
navigation runtime will benchmark DPVO/DPV-SLAM for pose, keep depth and person
perception on independent schedules, and fuse only navigation-scale geometry.

The dense viewer was also a separate, severe source of contention. Reducing
the pointmap to quarter density and refreshing the full reconstruction at 2 Hz
restored visualized tracking from roughly 2 FPS to 7-8 FPS.

## Evidence before tuning

- The verified 796-sample, 60-second C920 room recording took 138.7 seconds
  headless and reported approximately 6.4-8.9 FPS.
- The user's subsequent moving live viewer run reported approximately
  1.90-2.14 FPS while accumulating 26 keyframes and a 49 MB PLY.
- The viewer rendered every dense keyframe at the display refresh rate while
  sharing the mapper's CUDA context and state locks.
- The upstream `local_opt.window_size` value is currently read but not applied;
  the factor graph and optimization workload continue to grow with keyframes.

This separates two problems: dense visualization caused the immediate collapse
to about 2 FPS, while neural inference and the growing graph impose a lower
7-9 FPS headless ceiling.

## Changes tested

`config/c920-navigation.yaml` now preserves MASt3R's 512-pixel-long-edge neural
input while setting `dataset.img_downsample: 2`. That reduces pointmap,
matching, optimization, visualization, and saved geometry to one quarter of
the original sample count.

The upstream `img_downsample` path had never allocated correspondingly sized
shared buffers and crashed on the first keyframe. The House Bot runtime patch
now separates full neural-image/feature dimensions from downsampled pointmap
dimensions.

The viewer now accepts a configurable maximum refresh rate. The House Bot live
and navigation configurations use 2 Hz because the full dense map is
diagnostic output, not a robot control signal.

## Full-room headless comparison

Both runs used the same file and every second source frame:

`data/input/c920-room-loop-20260817-121954.mp4`

| Result | Original | Navigation config |
|---|---:|---:|
| Samples | 796 | 796 |
| Wall time | 138.7 s | 117.68 s |
| Keyframes | 73 | 71 |
| PLY points | 7,384,818 | 1,858,009 |
| PLY size | 110.8 MB | 27.9 MB |
| Estimated path | 12.3318 m | 12.2970 m |
| Start/end separation | 0.5581 m | 0.5470 m |
| Reported FPS near end | about 6.4 | 7.63 |

The trajectory statistics remain close, but this house recording has no
ground truth and cannot prove equal localization accuracy.

Command:

```bash
./scripts/run_mast3r_slam.sh \
  data/input/c920-room-loop-20260817-121954.mp4 \
  --config /home/james/Repos/house-bot/config/c920-navigation.yaml \
  --no-viz \
  --save-as c920-navigation-downsample2
```

## Viewer comparison

A bounded 300-sample run on the same recording measured the combined effect of
pointmap downsampling and viewer throttling:

| Full-map viewer rate | Cumulative FPS near sample 300 | Harness wall time |
|---|---:|---:|
| 10 Hz | 3.70 | 94.7 s |
| 2 Hz | 7.21 | 54.5 s |

The reproducible harness is `scripts/benchmark_mast3r_viewer.sh`. It waits for
the bounded run to save, then terminates only its own viewer process.

A separate 60-sample live C920 headless smoke test completed with exit code 0
and reported 9.21 FPS at sample 30, confirming that the repaired pointmap path
also works through the Windows-to-WSL newest-frame transport.

## Architecture consequence

Do not spend the next phase turning MASt3R-SLAM into a long-running robot
runtime. Keep it for dense mapping experiments and comparison. The production
candidate should be:

```text
30 FPS camera
  |-- DPVO / DPV-SLAM: pose and loop closure
  |-- depth model: local geometry on a slower independent schedule
  `-- person detector/tracker: independent real-time schedule

pose + selected depth frames
  -> bounded local voxel map
  -> floor/obstacle projection
  -> navigation
```

DPVO/DPV-SLAM is the next controlled benchmark on the same recording. Do not
add Online Video Depth Anything until pose throughput is proven: its released
small model predicts scale-and-shift-invariant relative depth, so metric fusion
also needs a scale source such as wheel odometry or calibrated scene geometry.

DA3 Streaming is not the next candidate. Its own documentation describes it as
not a SLAM system, reports about 8.5 FPS on an A100, and reports at least 11.5
GB peak VRAM in its published streaming configurations.
