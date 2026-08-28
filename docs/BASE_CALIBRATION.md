# Base calibration and real-base bring-up

The real base uses the established ROS differential-drive contract: Nav2 emits
`/cmd_vel`; the House Bot base bridge sends calibrated left/right commands and
publishes `/odom` plus `odom -> base_link`. The Pi remains a small GPIO and
watchdog endpoint. Nav2, rather than custom project code, continues to own path
planning, path following, behaviors, waypoint actions, and velocity smoothing.

The current receiver exposes no verified encoder feedback. The first `/odom`
source is therefore explicitly high-covariance **open-loop command
integration**. It is suitable for calibration and short supervised motion
tests. It is not sufficient for unattended navigation. Metric visual
localization and a live obstacle source remain autonomy gates -- where "obstacle
source" means a SLAM-derived obstacle topic, not a range sensor. The camera and
IMU are the complete sensor set; see D-021. There are no wheel encoders and none
are planned, so visual pose correction is the source of truth for position.

## Safety setup

- Put the robot on a clear, level floor with at least one metre of clearance.
- Keep the receiver power control or original remote within immediate reach.
- Run one bounded motion at a time. Do not start Nav2 during calibration.
- The Pi releases all controls after 350 ms without commands.
- The ROS base starts disabled, times out stale `Twist` commands after 250 ms,
  disables itself if motor acknowledgements stop, and supports a latched
  `/house_bot/estop` input.

## 1. Deploy binary wheel control

The original remote reliably exposes full-power and stop commands. Repeated
50% pulse-density routes usually activated only the left tread, even after the
remote was awake. The service therefore rejects fractional commands by default.
Pulse density remains available only behind an explicitly experimental service
flag and must not be used for calibration, ROS control, or autonomy.

```bash
cd /home/james/Repos/house-bot
./scripts/deploy_pi_motor_service.sh 192.168.0.241
```

## 2. Measure rigid geometry

Use metres and the ROS REP-103/105 convention. Define `base_link` at the floor
projection of the centre of the tread contact area: x forward, y left, z up.

Estimate:

- maximum assembled footprint length and width, including protrusions;
- C920 optical-centre x, y, and z relative to `base_link`;
- camera mount roll, pitch, and yaw. Zero means level and forward-facing.

For a tread base, do not guess a wheel separation. The solver derives an
effective skid-steer track width from observed straight and pivot motion, which
captures tread scrub better than the outside-to-outside dimension.

The base bridge publishes `base_link -> camera_link` from these measurements
and the standard fixed `camera_link -> camera_optical_frame` rotation.

## 3. Record the IMU mounting offset

The GY-521 is bolted on at whatever angle the mounting allows; this base reads
8-10 degrees off vertical. That costs nothing for yaw, which is projected onto
measured gravity, but it means raw board pitch and roll are not base pitch and
roll. Capture the fixed offset once, with the base on a surface you have
confirmed level:

```bash
python3 scripts/imu_monitor.py --record-level
python3 scripts/imu_monitor.py --attitude     # should now read about 0, 0
```

This writes `config/local/imu_mount.json`. It is not needed for the trials
below, only for any later use of base attitude.

## 4. Record full-power motion trials

Heading is measured by the IMU, not by eye. Distance still needs a tape.

```bash
python3 scripts/calibrate_base_trials.py forward --duration 3 --repeats 3
python3 scripts/calibrate_base_trials.py reverse --duration 3 --repeats 3
python3 scripts/calibrate_base_trials.py left    --duration 3 --repeats 3
python3 scripts/calibrate_base_trials.py right   --duration 3 --repeats 3
```

Each trial re-calibrates gyro bias while stationary, zeroes yaw, runs one
bounded full-power command, settles, and records the heading change. Straight
trials prompt for the measured distance; pivot trials need no manual entry at
all. Results append to `config/local/base_calibration_measurements.json`, so
`calibrate_base.py` averages the repeats and flags inconsistency.

Mark the starting tread midpoint and heading before each straight trial, and
let the robot reach a complete stop before measuring. Never estimate a
measurement that was not taken.

**These trials are deliberately open-loop.** `calibrate_base.py` derives the
per-tread speed split from the heading change during a straight run -- that
asymmetry is the signal being measured. Running them through
`drive_heading.py` would steer out the very thing being fitted. Heading control
belongs on the scale route, not here.

If three seconds would leave the clear test area, use a shorter `--duration`;
it is recorded with each trial.

### Recalibrated 2026-08-27

Redone after the rewire; `CALIBRATION_QUALITY=PASS`. See the current-status
section of `docs/experiments/2026-08-27-remote-remap-and-imu.md`.

### Previous calibration quarantined (historical)

The 2026-08-23 calibration was fitted before the remote rewire, before
`--invert-left` was removed, and before the duplicate-net wiring fault was
found. Under that fault paired commands behaved differently from single ones,
which is exactly what the solver fits, so the tread speeds and track width in
it cannot be trusted. Those files are renamed `*.pre-2026-08-27-rewire.bak` so
nothing reads them silently, and `drive_distance.py`,
`drive_straight_compensated.py`, and the scale route now fail loudly until the
calibration is redone. See
`docs/experiments/2026-08-27-remote-remap-and-imu.md`.

## Driving multi-leg routes

Use `scripts/drive_route.py`, which streams a whole route from one process,
socket and session:

```bash
python3 scripts/drive_route.py --dry-run --gap 2.5 forward:3 reverse:3
python3 scripts/drive_route.py --execute --gap 2.5 forward:3 reverse:3
```

**Do not loop `send_motor_command.py` once per leg.** Doing so latches the GT004
remote into a state where paired commands drive only one tread, clearing only on
a power cycle. See `docs/experiments/2026-08-27-remote-remap-and-imu.md`. A
single one-off command through `send_motor_command.py` is fine.

## 5. Solve and review

```bash
python3 scripts/calibrate_base.py \
  config/local/base_calibration_measurements.json
```

This produces `config/local/base_calibration.yaml` and a JSON summary. It fits
an effective skid-steer width and direction-specific full-scale tread speeds,
generates the measured camera transform, and replaces the provisional circular
Nav2 radius with a conservative rectangular footprint. A coefficient of
variation above 20% produces `CALIBRATION_QUALITY=REVIEW` instead of silently
accepting inconsistent trials.

## 6. Supervised ROS base check

Start the real base bridge in one terminal. It stays disabled.

```bash
./scripts/run_base_driver.sh
```

In another terminal, inspect status, then arm it only while the robot is in the
clear test area:

```bash
./scripts/base_control.sh status
./scripts/base_control.sh enable
```

Disable or stop it with:

```bash
./scripts/base_control.sh disable
./scripts/base_control.sh stop
```

The ROS bridge deliberately refuses to arm while
`proportional_control_verified` is false. The current original-remote path is
appropriate for supervised, bounded full-power experiments only; it cannot
reproduce Nav2's continuous velocity commands.

## Autonomy handoff

After the base calibration passes:

1. repeat a timestamped full-power `config/dpvo-navigation.yaml` route from a
   natural scene, with every segment at least 1.5 seconds and an observer
   confirming both treads engaged. `scripts/run_base_calibration_route.py` now
   drives turns closed-loop against the IMU, holds heading on the straight legs,
   and logs continuous yaw into the route file;
2. align DPV-SLAM translation scale to the measured base motion and publish the
   live metric pose through the ROS frame tree. `align_dpvo_scale.py` recovers
   capture latency by cross-correlating the route's IMU yaw against the visual
   rotation, falling back to the commanded-turn markers when the correlation is
   weak;
3. replace the binary toy-remote actuation path with a proportional motor
   driver and wheel feedback, then expose it through `ros2_control`'s
   differential-drive controller;
4. fuse metric visual pose and wheel motion using `robot_localization`, with a
   single owner for `odom -> base_link`;
5. load a real occupancy map and provide `map -> odom`;
6. add a live depth/range obstacle layer and Nav2 collision monitor;
7. only then run a low-speed, supervised Nav2 point goal.

This sequence keeps the custom work limited to the unusual remote bridge and
DPV-SLAM adapter. Standard ROS components own fusion, frames, costmaps,
planning, control, collision monitoring, and lifecycle management.
