# IMU wiring, mounting, and yaw contract

**Part:** HiLetgo GY-521 breakout carrying an InvenSense MPU-6050 (3-axis gyro,
3-axis accelerometer, 16-bit ADC, I2C).
**Purpose:** measure how far the base has actually turned. A magnetometer is
not usable on this robot; the motors and the receiver's H-bridges swamp the
earth's field. A rate gyro is immune to that.

## Wiring

| GY-521 pad | Wire | Pi physical pin | BCM |
| --- | --- | ---: | --- |
| `VCC` | white | 1 | 3.3 V |
| `SDA` | black | 3 | BCM2 / SDA1 |
| `SCL` | brown | 5 | BCM3 / SCL1 |
| `INT` | red | 7 | BCM4 (unused; wired so a future data-ready path needs no rewire) |
| `GND` | orange | 9 | GND |

`AD0`, `XDA`, and `XCL` are unconnected. `AD0` floating selects address
**0x68**; some board batches pull it high instead, giving 0x69, which
`imu_service.py --address 0x69` accepts.

Power comes from 3.3 V rather than 5 V deliberately. The board's own regulator
tolerates 5 V, but on some clone batches the SDA/SCL pull-ups tie to `VCC`
ahead of that regulator, which would put 5 V on the Pi's 3.3 V-only I2C lines.

The colour order is unconventional: **white is 3.3 V and orange is ground.**
Check continuity on those two before first power-up.

The IMU ground is its own wire to pin 9. It is not shared with the remote's
ground on the GT004 harness.

## Mounting contract

The board lies **flat**, component side up, rigidly fastened to a horizontal
part of the chassis, with:

- **+Y** pointing to the **front** of the base;
- **+X** pointing to the **right**;
- therefore **+Z up**, and gyro Z is yaw rate.

Rigid matters more than exact: any flex between the module and the treads
shows up directly as heading error. Squareness to the chassis matters less
than it looks, because `mpu6050.YawEstimator` projects the gyro vector onto
the measured gravity direction rather than reading gyro Z raw, so a mount a
few degrees off level still reports true rotation about the vertical. On a
perfectly level mount the projection reduces to gyro Z exactly.

Sign convention follows REP-103: **positive yaw is a left turn**,
counter-clockwise seen from above.

## Pi bring-up

```bash
sudo raspi-config nonint do_i2c 0
sudo apt install -y i2c-tools python3-smbus2
sudo reboot
i2cdetect -y 1          # expect 0x68
```

Nothing appearing is almost always SDA and SCL swapped. `0x69` instead of
`0x68` means this board pulls `AD0` high.

The default 100 kHz bus speed is sufficient: a 14-byte burst at 200 Hz uses a
small fraction of it. `dtparam=i2c_arm_baudrate=400000` is only needed if the
sample rate is raised substantially. The BCM2835 clock-stretching erratum does
not apply; the MPU-6050 never stretches.

## Services

```bash
./scripts/deploy_pi_imu_service.sh 192.168.0.241
```

`imu_service.py` runs as `house-bot-imu.service`, samples at 200 Hz, and
publishes integrated yaw on **UDP 8766** to subscribers.

It is a separate process from `pi_motor_service.py` on purpose. The motor
service busy-polls the remote's matrix rows for a 215 us low window every
40 ms; a 14-byte I2C burst takes longer than that window, so sharing a thread
would drop scan windows and quietly weaken every motor command. The Pi 3B has
four cores and runs both loops side by side.

| Command | Effect |
| --- | --- |
| `{"cmd":"subscribe"}` | start receiving pushed state; expires after 2 s without refresh |
| `{"cmd":"state"}` | one-shot read |
| `{"cmd":"zero"}` | set integrated yaw to 0 |
| `{"cmd":"calibrate","seconds":1.5}` | re-estimate gyro bias; rejected if the base was moving |

Every published packet carries `rate` as well as `yaw`, so a controller can
extrapolate across transport latency instead of steering on a stale angle. At
180 dps a 10 ms delay is 1.8 degrees.

## Configuration and why

| Setting | Value | Reason |
| --- | --- | --- |
| Gyro full scale | +/-500 dps | a 90 degree pivot peaks near 180 dps; +/-250 dps would clip a fast turn |
| Accel full scale | +/-2 g | only used for the gravity direction and stillness detection |
| DLPF | 42 Hz gyro bandwidth | rejects tread and gearbox vibration, far above any real turn rate |
| Internal rate | 1 kHz / 5 = 200 Hz | comfortably oversamples a sub-second turn |
| Clock source | X gyro PLL | markedly more stable than the internal 8 MHz oscillator the reset default selects |

## Bias is the whole problem

An uncorrected 1 dps gyro offset is one degree of error per second of turning,
and the MPU-6050's offset moves with die temperature. Three things address it:

1. a stationary capture at service start seeds the bias;
2. the bias is tracked slowly whenever the base is confirmed still, which
   requires both a quiet gyro *and* a steady accelerometer -- gyro alone is
   not enough, because a tread base driving straight has a near-zero yaw rate
   while very much in motion;
3. `turn_by_imu.py` re-calibrates immediately before each turn.

Measure the residual before trusting a turn:

```bash
python3 scripts/imu_monitor.py --drift 60
```

Under 0.05 deg/s is well inside a 2 degree turn tolerance.

## Closed-loop turns

```bash
python3 scripts/turn_by_imu.py 90       # left  90 degrees
python3 scripts/turn_by_imu.py -90      # right 90 degrees
```

This replaces the measured duration table in `drive_primed.py`. Turn angle is
not proportional to command duration on this base, so a stopwatch cannot be
made accurate; yaw feedback removes the need for the table entirely.

The base has no proportional speed (D-020), so the controller is bang-bang:
full power until the projected stop point, then release. Because the base
coasts, the stop is issued early by `lead_seconds` worth of the current yaw
rate. Each turn measures its own coast and blends it back into
`config/local/imu_turn_calibration.json`, so the lead converges without a
separate calibration route.

Order of operations per turn:

1. calibrate gyro bias while stationary;
2. zero yaw;
3. prime the receiver, since it sleeps after about 5 s idle and swallows a
   variable amount of the first command -- yaw is zeroed *before* the prime,
   so any creep the prime causes is counted rather than silently added;
4. stream the pivot, evaluating the stop condition every 2 ms while refreshing
   the command at 20 Hz, so the release is not quantised to the packet interval;
5. stop, settle, measure;
6. correct with up to two further closed-loop nudges, accepting any residual
   below `--min-correction` because the base cannot reliably move less.

Two aborts protect the base rather than letting it run the clock out: no
rotation within 0.45 s means the receiver is asleep, the battery is flat, or
the motor service is down; rotation in the wrong direction means the motor
service's `--invert-left` / `--swap-sides` flags disagree with the wiring.
