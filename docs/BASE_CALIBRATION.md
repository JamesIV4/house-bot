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
localization and a live obstacle source remain autonomy gates.

## Safety setup

- Put the robot on a clear, level floor with at least one metre of clearance.
- Keep the receiver power control or original remote within immediate reach.
- Run one bounded motion at a time. Do not start Nav2 during calibration.
- The Pi releases all controls after 350 ms without commands.
- The ROS base starts disabled, times out stale `Twist` commands after 250 ms,
  disables itself if motor acknowledgements stop, and supports a latched
  `/house_bot/estop` input.

## 1. Deploy fractional wheel control

The tested motor service now honors command magnitude using pulse density over
the remote's 25 Hz scan windows. A magnitude of `1.0` retains the already
verified full-power behavior; lower magnitudes select a corresponding fraction
of scan windows.

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

## 3. Record full-power motion trials

Copy the worksheet; the local copy is ignored by Git.

```bash
mkdir -p config/local
cp config/base-calibration-measurements.example.json \
  config/local/base_calibration_measurements.json
```

Mark the starting tread midpoint and heading approximately. Run every command
once, letting the robot reach a complete stop before measuring. Enter positive travel
distances for forward/reverse, signed straight-run heading change (left positive,
right negative), and positive turn magnitudes for left/right.

```bash
python3 scripts/send_motor_command.py forward --duration 3 --power 1
python3 scripts/send_motor_command.py reverse --duration 3 --power 1
python3 scripts/send_motor_command.py left    --duration 3 --power 1
python3 scripts/send_motor_command.py right   --duration 3 --power 1
```

If two seconds would leave the clear test area, use the same shorter duration
in both the command and worksheet. Never estimate a measurement that was not
taken.

## 4. Solve and review

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

## 5. Supervised ROS base check

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

The next acceptance run should validate `0.5` power in each straight direction
and both pivots before Nav2 is connected. Compare observed half-power motion
with the fitted rate; if the toy receiver/motor response is materially
nonlinear, add measured duty breakpoints rather than pretending it is linear.

## Autonomy handoff

After the base calibration passes:

1. repeat the measured `config/dpvo-navigation.yaml` route from a natural scene
   with the camera rigidly mounted;
2. align DPV-SLAM translation scale to the measured base motion and publish the
   live metric pose through the ROS frame tree;
3. fuse metric visual pose and base motion using `robot_localization`, with a
   single owner for `odom -> base_link`;
4. load a real occupancy map and provide `map -> odom`;
5. add a live depth/range obstacle layer and Nav2 collision monitor;
6. only then run a low-speed, supervised Nav2 point goal.

This sequence keeps the custom work limited to the unusual remote bridge and
DPV-SLAM adapter. Standard ROS components own fusion, frames, costmaps,
planning, control, collision monitoring, and lifecycle management.
