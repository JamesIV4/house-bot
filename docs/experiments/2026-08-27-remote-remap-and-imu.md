# Remote rewire, GPIO remap, and IMU bring-up

**Date:** 2026-08-27

## Current status (2026-08-27, end of session 2)

**Base calibration is complete and passing.** `CALIBRATION_QUALITY=PASS`, written
to `config/local/base_calibration.yaml` and `.summary.json`.

| | Forward | Reverse |
| --- | ---: | ---: |
| left tread | 0.2579 m/s | 0.2375 m/s |
| right tread | 0.2391 m/s | 0.2436 m/s |

Effective track width 0.2164 m, Nav2 radius 0.1437 m. Coefficient of variation
2.0% on both left channels, 3.9-4.5% on the right. The track width lands within
0.2% of the pre-rewire 0.216 m from independent trials.

Going forward the left tread runs 7.9% faster than the right, which is the
systematic rightward drift; in reverse the two are within 2.6% and the drift
nearly vanishes. Now measured and compensated rather than unexplained.

**ROOT CAUSE FOUND: driving a route as one process per leg corrupted the
remote.** Looping `send_motor_command.py` once per leg put the GT004 into a
latched state where paired commands drove only one tread, clearing only on a
power cycle -- and reproducible on the remote's own physical buttons, which is
what made it look like hardware. `scripts/drive_route.py` drives the identical
route from **one process, one socket, one session, one continuous 20 Hz
stream**, and the same route then ran clean with the controls fine afterwards.

Every leg boundary in the old pattern produced a traffic burst: the stop
confirmation retries at 5 ms intervals for up to 0.6 s, sends two more stops,
then the process tears down and the next starts with a fresh socket and session.
The motor service busy-polls GPIO to catch 215 us scan windows and services the
network only every 500 us, so a burst at every boundary starves the gating loop
precisely when it is releasing a column. Miss that timing and it actuates
buttons nobody asked for.

That explains the dose-response seen all day: rapid short legs (many boundaries)
broke it fastest; the 12 long, widely spaced calibration trials survived; the
slow-cadence run still broke because the camera added CPU contention on top.

**Rule: drive multi-leg routes with `scripts/drive_route.py`. Never loop
`send_motor_command.py`.** A single one-off command is fine.

**A separate, real bug was found and fixed on the way** in
`pi_motor_service.py`: a pivot's two actions share one physical column pin
(left pivot = BCM22 twice, right = BCM17 twice) but each kept its own
`sink_active` flag, so the second action released the press the first had
asserted microseconds earlier in the same poll pass. Sink state is now keyed by
column pin. Regression-tested. This was **not** the cause of the drive fault.

**The "will not drive straight" fault did not reproduce.** Twelve trials ran
clean. What was ruled out first, by measurement: the pin map, the deployed code,
the calibration script's inputs, the column release timing, and the shared-pin
pivot -- plus an on-blocks pass where all four single actions, both pairs, and
the UDP service path were correct with the treads unloaded. See "Session 2: the
Pi side is not the fault" below.

**The right tread is NOT the problem.** It carries roughly double the left's
run-to-run variation in the fit, which is worth watching, but it was wrongly
promoted to leading suspect before the driver was suspected. So were the motor
battery pack and a column release race. Do not restart there.

**The remote sleeps after 2-5 minutes idle** and its MCU stops scanning
entirely. A command sent into a slept remote actuates nothing and reads as
`+0.00 deg`, not as an error -- one calibration trial was lost this way and
discarded. Unrelated to the retired `drive_primed.py`, which addressed dropped
short commands on the old wiring.

> Everything below this section is research history, in the order it happened.
> Several of its conclusions were later disproved; those are marked inline.
> Trust this section over anything below it.

## What changed

The GT004 remote's button wires were re-soldered and the Pi header assignment
was moved off BCM4-BCM11. The remap frees BCM2/BCM3 for an MPU-6050, and
releases SPI0, the serial console, and both hardware PWM channels, none of
which the old layout had any use for.

## Row/column identification

`identify-rows` profiles both wires of each pair for 2 s and classifies each as
a scan row or an input column, rather than inferring the role from where the
wire sits on the switch. All four pairs resolved on the first run:

```
left-forward:  BCM17 -> column (steady)      BCM18 -> row (50 windows, 223.7us, 40.16ms)
left-reverse:  BCM22 -> column (steady)      BCM23 -> row (50 windows, 214.6us, 40.17ms)
right-forward: BCM19 -> row (49 windows, 214.9us, 40.17ms)   BCM16 -> column (steady)
right-reverse: BCM26 -> column (steady)      BCM20 -> row (50 windows, 216.6us, 40.18ms)
```

The widths and periods match the 2026-08-19 measurements on the old wiring,
which is a useful cross-check that the remote itself is unchanged.

The row is on the **even** header pin for three pairs and the **odd** pin for
`right-forward`. A proposed odd-is-row convention was therefore wrong, and the
left and right button clusters being mirrored on the PCB is the reason. Role is
electrical, not geometric, and is measured every time.

## Tread-direction verification

Each button was pressed alone for 0.8 s through `matrix-pulse`, below the motor
service so no software inversion sat in the path, with the physical result
observed before the next command was issued.

| Command | Row -> column | Observed |
| --- | --- | --- |
| `left-forward` | BCM18 -> BCM17 | left tread forward |
| `left-reverse` | BCM23 -> BCM22 | left tread backward |
| `right-forward` | BCM19 -> BCM16 | right tread forward |
| `right-reverse` | BCM20 -> BCM26 | right tread backward |

All four match their names. **`--invert-left` was removed** from
`deploy/house-bot-motors.service`; it had been calibrated against the
pre-rebuild drivetrain and was now wrong.

Each 0.8 s pulse mirrored 20 scan windows, against about 25 available, so a
few windows are lost to process startup.

## Straight-line travel

| Run | Prime | Receiver idle before | Duration | Distance |
| --- | --- | --- | --- | --- |
| unprimed forward | none | 12 s | 0.60 s | 8 in (203 mm) |

About 0.34 m/s. Single run; spread is not yet characterised.

## Priming retired

`drive_primed.py` existed because turn angle varied far more than command
duration could explain, and short commands sometimes produced no motion at
all. The suspected cause was the receiver swallowing a variable part of the
first command while waking.

Two things now argue against that explanation:

- the prime itself was observed moving the base forward, which is motion the
  receiver was clearly awake enough to act on;
- an unprimed 0.6 s command after 12 s of idle travelled 8 in, which is not a
  command that lost a significant part of itself.

Flaky connections on the old wiring explain the original symptom better than
receiver sleep does. Priming is off by default in `turn_by_imu.py` and remains
only as an explicit `--prime` escape hatch.


Closed-loop turning removes the need for it either way: the controller drives
until the measured angle arrives, so a slow wake costs time rather than
accuracy. The fixed-time direction check was replaced with first-motion
detection for the same reason, so that a slow start is waited out instead of
being read as a fault.

## IMU bring-up

Wiring, mounting contract, and service design are in `docs/IMU_WIRING.md`.
I2C was not enabled on the Pi initially; after `raspi-config nonint do_i2c 0`
and a reboot the MPU-6050 answered on `/dev/i2c-1` and the service came up.
`i2c-tools` is still not installed, which does not matter -- the service does
its own WHO_AM_I check.

| Measurement | Result |
| --- | --- |
| Gyro noise, Z axis | 0.034 dps |
| Gyro bias, Z axis | -0.52 dps |
| Yaw drift after calibration | -0.080 deg over 60 s (-4.8 deg/hour) |
| Drift contributed to a 1 s turn | 0.001 deg |
| Mount tilt | 8-10 deg from vertical |
| Accelerometer magnitude | 0.960 g |

The mount is about 9 deg off level. This costs nothing: the estimator projects
the gyro vector onto measured gravity, so it reports rotation about true
vertical rather than about the board's own Z axis. The 4% accelerometer scale
error likewise does not reach yaw, because the gravity vector is normalised.

Sign and scale were confirmed by hand before any motor was trusted with it: a
90 degree counter-clockwise rotation by hand read **+90.4 deg** and then held,
decaying 0.1 deg over the following 18 s.

## Closed-loop turn results

Five consecutive 90 degree left pivots, each preceded by its own stationary
bias calibration:

| Run | Stopping model | Achieved | Error | Coast | Rate at stop |
| ---: | --- | ---: | ---: | ---: | ---: |
| 1 | lead 0.100 s | +109.35 | -19.35 | 33.76 | 150.4 dps |
| 2 | lead 0.225 s | +95.33 | -5.33 | 33.27 | 124.8 dps |
| 3 | coast 33.5 deg | +87.04 | +2.96 | 30.44 | 84.5 dps |
| 4 | coast 32.4 deg | +89.14 | +0.86 | 31.52 | 134.7 dps |
| 5 | coast 32.1 deg | +95.98 | -5.98 | 37.91 | 138.4 dps |

### Coast is an angle, not a time

Runs 1 and 2 used `coast = lead_seconds * yaw_rate`. That model mispredicts:
the implied lead moved from 0.225 s to 0.267 s between two runs whose actual
coast differed by only 0.49 deg. Switching to a learned constant coast **angle**
cut the error from 19 deg to 3 deg in one step.

### The accuracy floor is coast repeatability

Over five runs the coast was mean **33.38 deg**, sd **2.86 deg**, range
30.44-37.91. Runs 4 and 5 stopped at almost the same rate, 134.7 and 138.4 dps,
and coasted 31.52 and 37.91 deg -- 6.39 deg apart. The spread is run-to-run
randomness rather than rate dependence, so fitting coast against rate would be
fitting noise.

This is what limits a turn, and it is mechanical, not sensory. The IMU
contributes 0.001 deg to a one-second turn; the coast contributes several
degrees. A single full-power bang-bang pivot therefore lands within roughly
+/-3 deg typical and +/-6 deg worst observed.

### Corrections below the coast are impossible

The base cannot rotate less than it coasts at full power, so any residual under
about 33 deg is reported and accepted rather than chased. A nudge would
overshoot further than the error it was meant to remove. Getting finer than the
current floor needs a lower-momentum release -- a short pulse that never
reaches full speed -- not a better controller.

### Bug found and fixed

The first run's two correction attempts drove nowhere. `drive_until_angle`
derived turn direction from the sign of the *target* rather than the sign of
the *remaining error*, so correcting from +109 deg toward a +90 deg target
commanded a left turn and found its stop condition already satisfied. Fixed,
with regression cover in `test_turn_by_imu.py`.

## Rotation quantum: how small a correction is possible

A full-power pivot coasts about 33 deg, so it cannot correct anything smaller.
Two levers make a rotation smaller: a pulse short enough that the base never
reaches full speed, and driving one tread instead of two.

First sweep, pivot mode only, two trials each:

| Pulse | Mean | sd | Left | Right |
| ---: | ---: | ---: | ---: | ---: |
| 0.05 s | 20.25 | 10.23 | +27.49 | -13.02 |
| 0.08 s | 23.47 | 0.09 | +23.54 | -23.41 |
| 0.12 s | 30.20 | 0.62 | +30.64 | -29.76 |
| 0.16 s | 35.55 | 4.20 | +38.52 | -32.59 |
| 0.20 s | 41.75 | 1.11 | +42.54 | -40.96 |
| 0.30 s | 57.51 | 8.03 | +63.19 | -51.83 |

0.05 s is a single command packet, so no shorter pulse exists. Fitting the
repeatable middle of the range gives roughly `angle = 11 + 154 * seconds`, where
the 11 deg intercept is the irreducible coast.

The smallest *repeatable* pivot pulse is 0.08 s at 23.5 deg +/-0.1. The 0.05 s
pulse averages less but ranges 13-27 deg, which cannot close a 20 deg residual:
it would miss by 7 deg in either direction, at a random sign. Mean alone made
that pulse look usable, so `pulse_response.recommend` now gates on spread too.

A typical post-turn residual is 1-6 deg, so pivot-mode nudging is useless for
post-turn cleanup regardless.

### All three rotation modes measured

Bench expectation, largest to smallest: pivot, one tread forward, one tread
backward. Two trials per cell, alternating direction:

| Mode | 0.05 s | 0.08 s | 0.12 s | 0.20 s |
| --- | --- | --- | --- | --- |
| pivot | 18.8 +/-6.4 | 22.5 +/-0.5 | 27.1 +/-1.1 | 39.4 +/-2.9 |
| tread-forward | 5.5 +/-0.0 | 9.5 +/-0.03 | 26.8 +/-15.4 | 33.1 +/-17.3 |
| tread-reverse | 14.8 +/-10.5 | 17.0 +/-7.1 | 22.5 +/-10.4 | 31.9 +/-14.1 |

**tread-reverse is not the finest lever.** It was expected to be, and it is
both larger than tread-forward and unrepeatable at every length tested.

### The 5.5 deg result did not reproduce

The tread-forward row above was the basis for changing the default nudge mode.
A four-trial repeat of just those two cells contradicts it:

| Pulse | First sweep, n=2 | Repeat, n=4 |
| --- | --- | --- |
| 0.05 s | 5.5 +/-0.0 | 13.9 +/-2.4 |
| 0.08 s | 9.5 +/-0.03 | 15.9 +/-5.4 |

The identical +5.49 / -5.49 pair was a coincidence, not a measurement. Two
trials per cell is not enough to establish a spread, and a default was changed
on it before the check was run. Any single sweep here is provisional.

### Achieved granularity, by strategy

| Strategy | Finest quantum | Spread | Usable for a 1-6 deg residual |
| --- | ---: | ---: | --- |
| Full-power turn | 33.4 deg | +/-2.9 | no, this is the floor it creates |
| pivot pulse, 0.08 s | 22.5 deg | +/-0.5 (n=2) | no |
| tread-reverse pulse | ~15 deg | +/-7 to 10 | no |
| tread-forward pulse, 0.05 s | 13.9 deg | +/-2.4 (n=4) | no |
| Slot steering while driving | not yet measured | | expected yes |

No stationary mode gets below about 14 degrees, and every one of them has a
run-to-run spread comparable to the residual it would be correcting. Stationary
correction is therefore not usable on this base at any pulse length or mode.
`--nudge-pulse` stays 0 by default and the mechanism is kept only so the
measurement can be repeated if the drivetrain changes.

### The left tread is stronger than the right

Across both sweeps, every outsized reading is the one driven by the left tread:

- tread-reverse turning left uses the left tread: 22.2 / 22.0 / 29.9 / 41.9,
  against the right tread's 7.4 / 12.0 / 15.2 / 21.9;
- tread-forward turning right uses the left tread: 37.7 / 45.3 at 0.12 and
  0.20 s, against the right tread's 15.9 / 20.9, and 20.3 against 11.5 at
  0.08 s in the repeat sweep.

Below about 0.08 s the two match, which suggests a common startup impulse
dominates before sustained speed does.

This is directionally consistent with the base veering right when both treads
drive forward, recorded before the IMU existed.

**Session 2: this is now the leading lead, not a curiosity.** Once the Pi side
was cleared, a weak right drivetrain became the best remaining explanation for
the whole fault. Note the asymmetry appears with a *single* tread driving, where
supply sag is mildest, which argues against the motor pack and for the right
side itself.

The 1.76:1 magnitude does **not** transfer to straight-line driving, though.
Taken literally it predicts both-treads-forward rotating at about 68 deg/s;
the measured drift after the wiring fix is 4.4 deg/s. Single-tread trials are
not a clean measurement of tread speed: the undriven tread is a geared motor
being dragged, which loads the driven side, and a 0.5 s pulse is substantially
coast. Treat the ratio as evidence of which side is stronger, not as a
calibration.

## Contaminated runs, discarded

A forward heading-hold test and a steering-authority measurement were run while
nobody was with the robot, and it crashed into something during the sequence.
Both results are discarded rather than recorded:

- forward heading hold, 2 s: reported 5.75 deg of uncorrected drift;
- steering authority, full one-tread drop for 1 s: reported 37.93 deg.

Neither is trustworthy after an impact, and gains must not be tuned on them.
Both need re-running with someone present.

## The duplicate-net wiring discovery

> **Superseded.** This section originally read "the duplicate-net wiring fault"
> and credited it with causing the drive symptoms below. It does not. The wiring
> observation is real and the one-pin-per-net map is correct, but it was not the
> cause — see "Session 2: the Pi side is not the fault" at the end of this
> document. The text is kept because the reasoning is instructive about how a
> plausible mechanism got mistaken for a confirmed one.

The harness wires each of the remote's four matrix nets to the Pi **twice**,
once at each button touching it. `MATRIX_PINS` treated the eight wires as eight
independent pins, so it drove and released the same net through two different
pins. Releasing BCM16 does nothing while BCM22 still holds net C low. That is a
genuine defect and worth fixing on its own terms.

It was credited with these symptoms:

- `forward` drove one tread and spun the base hard right rather than driving
  straight, intermittently;
- a single left-side button press moved *both* treads on a tank-style base;
- pivots ran on one tread much of the time. A one-tread pivot still rotates the
  base, so this went unnoticed, surfacing only as unexplained variance in pivot
  rate (84-150 dps) and coast (30.4-37.9 deg).

**None of those attributions hold.** The old and new maps are electrically
identical for exactly the commands whose behaviour changed, the improvement did
not persist, and every symptom returned later the same day with the corrected
map deployed. The reasoning that should have caught this at the time is already
recorded below under "Unexplained: why the single-tread results changed" — it
was written, and then not acted on.

Diagnosis was by driving each wire low and reading the others:

| Net | Wires | Pi pins | Role |
| --- | --- | --- | --- |
| A | blue, white | BCM17, BCM26 | column |
| B | brown, black | BCM18, BCM23 | row |
| C | yellow, orange | BCM22, BCM16 | column |
| D | red, purple | BCM19, BCM20 | row |

Two hypotheses were tested and rejected first, both by measurement:

- **One side geared backwards.** Rejected: single-tread yaw signs are both
  correct, and `left-forward + right-reverse` spun rather than driving straight,
  which it would not have if a side were inverted.
- **A release race into the next row's scan window.** Rejected, but on wrong
  numbers: the "428 us budget" is the row-B-fall to row-D-fall spacing, which
  includes row B's own ~210 us window, and the hold was measured with mock pins
  in a tight loop. The real budget from row B's rising edge is 216 us. The
  conclusion happened to be right — see the direct measurement in session 2 —
  but it was not established by this.

`identify-rows` now detects shared nets and rewrites its printed `MATRIX_PINS`
to one pin per net.

## Results after the wiring fix

Pivot coast tightened from a 30.4-37.9 deg spread to 28.1-29.3 deg, and a
90 deg turn landed at +91.22 deg on the second attempt once the coast was
re-learned.

Straight-line heading, 2 s runs:

| Direction | Open loop | Closed loop, old gains | Closed loop, tuned |
| --- | ---: | ---: | ---: |
| Forward | -8.74 deg | -4.77 deg | **+0.14 deg** |
| Reverse | -0.56 deg | -1.79 deg | -1.49 deg |

Forward carries a systematic ~4.4 deg/s rightward drift; reverse barely drifts.
Proportional gain 2.5 settles around 6.6 deg of standing error against that
drift, which matches the -4.77 deg result. Gain 6.0 with integral 10.0 and a
0.3 duty floor brought a 2 s forward run to 0.14 deg, 0.97 deg rms. Those are
now the defaults.

Composed turn-then-drive: the turn landed +89.97 deg (0.03 deg error) and the
following 2 s drive held the heading to +2.58 deg.

Steering authority is roughly 27 dps at a full one-tread drop, so nulling the
4.4 dps forward drift needs about a 16% duty reduction. The previous 0.45 duty
floor left little headroom above that; 0.3 leaves plenty.

## Runs discarded

Three runs hit obstacles and are excluded rather than recorded: an early
forward heading hold and steering-authority measurement (robot unattended), and
a later turn-then-drive run. A fourth, a forward run reading -128.49 deg, was
initially suspected as an electrical fault; the timing measurement ruled that
out and a repeat on a clear floor gave -8.74 deg, matching the earlier -9.86 deg.

## Rotation quantum, re-measured after the wiring fix

Four trials per cell, all three modes, repeating the earlier sweep:

| Mode | 0.05 s | 0.08 s | 0.12 s | 0.20 s |
| --- | --- | --- | --- | --- |
| pivot | 15.7 +/-4.8 | **22.7 +/-0.5** | 28.8 +/-0.7 | 40.7 +/-2.7 |
| tread-forward | 8.7 +/-4.3 | **11.2 +/-0.9** | 14.4 +/-0.7 | 20.8 +/-1.2 |
| tread-reverse | **8.3 +/-1.0** | 11.1 +/-1.3 | 13.9 +/-2.0 | 20.6 +/-1.9 |

### Repeat sweep: only 0.08 s and longer reproduce

The tread modes were swept twice, independently, four trials per cell:

| Mode / pulse | Sweep 1 | Sweep 2 |
| --- | --- | --- |
| tread-forward 0.05 | 8.7 +/-4.3 | 3.0 +/-2.4 |
| tread-forward 0.08 | 11.2 +/-0.9 | 12.0 +/-1.0 |
| tread-forward 0.12 | 14.4 +/-0.7 | 13.9 +/-1.0 |
| tread-forward 0.20 | 20.8 +/-1.2 | 20.5 +/-2.0 |
| tread-reverse 0.05 | 8.3 +/-1.0 | 7.8 +/-3.3 |
| tread-reverse 0.08 | 11.1 +/-1.3 | 11.6 +/-1.4 |
| tread-reverse 0.12 | 13.9 +/-2.0 | 13.5 +/-2.3 |
| tread-reverse 0.20 | 20.6 +/-1.9 | 19.9 +/-4.3 |

0.08 s and above reproduce within about a degree. **The 0.05 s cells do not.**
A single command packet either lands inside a scan window or does not, and both
modes swing across sweeps.

Sweep 1 alone suggested `tread-reverse` reached 8.3 deg +/-1.0 at 0.05 s and
was therefore the finest lever, vindicating the bench expectation. The repeat
does not support that: it was one lucky cluster of four trials. At 0.08 s, the
shortest reliable length, the two tread modes are equal within noise, and
`tread-forward` holds a tighter spread at longer pulses.

**The practical floor for a stationary nudge is about 12 deg**, so residuals
down to roughly 14 deg can be closed standing still. Post-turn residuals of
1-6 deg remain below that, so `--nudge-pulse` stays 0 and heading is corrected
while driving.

This is the second time a two-trial or single-sweep result here pointed the
wrong way. Nothing from one sweep of this measurement should be treated as
settled.

### Unexplained: why the single-tread results changed

This is not fully accounted for. The single-tread modes drive one action, which
touches only one pin per net, so the old and new `MATRIX_PINS` are
*electrically identical* for them: old `right-forward` was (BCM19, BCM16) and
new is (BCM19, BCM22), and BCM16 and BCM22 are the same net. The same is true
of `forward`, which is (net B, net A) + (net D, net C) under both maps.

So the duplicate-net cleanup should have been a no-op for exactly the commands
that changed behaviour, yet forward went from reliably spinning to six
consecutive clean runs immediately after that deploy, and the tread asymmetry
vanished at the same time. Either something else changed at that deploy, or the
earlier sweeps were contaminated by conditions that were not recorded.

The clean way to settle it is an A/B: redeploy the old eight-pin map, run a 2 s
forward, and see whether it spins again. That has not been done.

## Still to measure

- rotation produced by short `tread-forward` and `tread-reverse` pulses, which
  sets the finest stationary correction available;
- steering authority at full one-tread drop, re-run cleanly;
- forward and reverse heading hold against real drift, re-run cleanly;
- straight-line distance spread over repeated identical unprimed commands;
- whether right pivots coast the same as left.

## Session 1 handoff: base does not drive straight, calibration blocked

> **Partly superseded by session 2**, below. The electrical hypotheses in this
> section were all tested and none survived. The observations and the discarded
> runs stand; the diagnosis does not.

**Status at handoff.** Base calibration is blocked at forward trial 1. The base
will not drive straight: a 3 s `forward` command spun roughly two-thirds of a
turn. One forward trial was run and discarded; the worksheet
`config/local/base_calibration_measurements.json` contains no trials, so nothing
corrupt is recorded and the sequence can resume from the start.

Nothing below is settled. Today produced several reversals -- a phantom 5.5 deg
pulse result, a tread asymmetry recorded then retracted then observed again, and
a wiring change that appeared to fix forward driving and then did not hold. Treat
every item here as provisional and re-measure before relying on it.

### Observations, this session's final state

Measured with `scripts/probe_matrix.py`, which gates intersections directly on
the Pi with no motor service, no networking in the gating loop, and no reliance
on `MATRIX_PINS`. Each run is 1 s.

| Command | Yaw |
| --- | ---: |
| left tread forward alone | -154.3 deg |
| left tread reverse alone | +144.7 deg |
| right tread forward alone | +72.5 deg |
| right tread reverse alone | -78.5 deg |
| `forward`, both treads | -138.4 deg |
| `reverse`, both treads | +152.7 deg |
| pivot left, both treads | +153.0 deg |

Two things stand out, neither explained:

- the left tread turns the base about twice as fast as the right, individually;
- every paired command lands close to its left action alone. `reverse` should be
  near zero and instead reads +152.7 deg, which is left-reverse alone.

A 3 s single-tread run of the right tread fluctuated between 56 and 93 dps with
no steady decay across the run.

Earlier the same day, the same commands drove the base nearly straight: -8.74 deg
over a 2 s open-loop forward, and +0.14 deg closed-loop. Both treads must have
been engaging and closely matched then. Something changed during the session;
what, is not established.

### What appears ruled out, and how

Stated as evidence rather than conclusions. Each was checked once or twice, not
repeatedly, and today showed that single measurements here can mislead.

- **Action mapping.** Each of the four actions was verified individually and
  moved the tread its name claims, in the direction its name claims.
- **Pi-side gating.** Both actions of a pair mirror the full 25 scan windows per
  second, simultaneously.
- **Loop service order.** Reversing which action the gating loop services first
  changed the result by 6 deg out of 160, well inside run-to-run variation.
- **Motor service and its networking.** The fault reproduces in a bare tight loop
  that runs no service and does no socket work.
- **Column release timing.** With two actions active the column is held low for
  160-249 us against a 428 us gap to the next row's scan window; 0 of 148 presses
  overran. Note this was measured with dummy pins in a tight loop, not inside the
  service.

### Open hypotheses, untested

Session 2 resolved three of these four. Verdicts added inline; the reasoning is
in "Session 2: the Pi side is not the fault" at the end of this document.

- **Motor battery low.** The pack had been driving all day. A weak pack would let
  either motor run alone while sagging under two, stalling the weaker one. This
  would account for both the 2:1 imbalance and the paired-command dropout with one
  cause, which is why it is listed first -- not because it has any evidence over
  the others. **Demoted:** the 2:1 imbalance appears with a single tread running,
  where sag is mildest, so the pack does not explain it. Still worth eliminating
  with fresh cells.
- **Right tread mechanically impaired.** The base struck obstacles at least twice
  during the session. The 56-93 dps fluctuation within a single run is at least as
  consistent with slipping as with a clean motor. **Now the leading candidate**,
  by elimination of everything upstream and by the single-tread asymmetry.
- **Remote transmits only one key at a time.** Would explain the paired dropout
  but not the single-tread imbalance, and paired commands demonstrably worked
  earlier both today and before the rewire. **Rejected:** on blocks, `FORWARD`
  and `PIVOT LEFT` both drive two treads correctly.
- **The duplicate-net change was not the real fix.** Forward driving went from
  reliably spinning to six consecutive clean runs immediately after that deploy,
  yet old and new `MATRIX_PINS` are electrically identical for exactly the
  commands that changed behaviour. **Confirmed:** it was not the fix. The six
  clean runs were a coincidence. One pin per net is still correct, for its own
  reasons.

### How to re-test

```bash
python3 scripts/probe_matrix.py \
  "left fwd=18:17" "right fwd=19:22" "FORWARD=18:17,19:22"
```

Healthy looks like the two single treads roughly equal and opposite, and
`FORWARD` near zero. At handoff it reads -154 / +73 / -138.

Run this on the floor only. On blocks it measures nothing, because the base
cannot rotate — and on blocks every one of these commands is already known to be
correct.

Then resume calibration from `docs/BASE_CALIBRATION.md` step 3.

### Repo state at handoff

Working but unexercised against a healthy base:

- `scripts/calibrate_base_trials.py` -- IMU-measured trials, open-loop by design;
  distance is the only manual entry. Non-interactive shells record the heading
  and leave distance pending for `--fill-distance`.
- `scripts/run_base_calibration_route.py` -- turns closed-loop, straights hold
  heading, continuous yaw logged into the route file.
- `scripts/align_dpvo_scale.py` -- capture latency from IMU-yaw cross-correlation,
  falling back to commanded-turn markers below 0.35 correlation.
- `scripts/probe_matrix.py` and `remote_gpio_controller.py gate-pairs` -- the
  diagnostic used above.
- `imu_monitor.py --record-level` -- mounting offset captured:
  pitch +0.99 deg, roll -10.47 deg, saved to `config/local/imu_mount.json`.

The 2026-08-23 base calibration is quarantined as `*.pre-2026-08-27-rewire.bak`,
so `drive_distance.py`, `drive_straight_compensated.py` and the scale route fail
loudly rather than fitting against pre-rewire numbers.

149 tests pass; 7 need numpy and run under `envs/dpvo/bin/python`.


## Session 2: the Pi side is not the fault

Everything between the calibration script and the remote's matrix was tested and
cleared. The fault is downstream of all of it, and needs load to appear.

### What was checked, and how

| Suspect | Verdict | Evidence |
| --- | --- | --- |
| Calibration script drives with stale pins | **Clear** | `calibrate_base_trials.py` holds no pin knowledge: `MOTIONS` -> wheel floats -> UDP -> `actions_for_wheels` -> `MATRIX_PINS` |
| Deployed Pi code drifted from the repo | **Clear** | `remote_gpio_controller.py` and `pi_motor_service.py` on the Pi are md5-identical to the repo; unit file has no invert flags; service active |
| Pin map wrong after the rewire | **Clear** | `MATRIX_PINS` matches the harness net-for-net against the wire colours |
| Column release races the next row | **Clear** | measured directly; see below |
| Shared-pin pivot has a software bug | **Clear** | `PIVOT LEFT` drives both actions through BCM22 and behaves correctly |

### The timing theory, raised and killed

Every column pin is shared by one left action and one right action (BCM17 is
`left-forward` + `right-reverse`; BCM22 is `left-reverse` + `right-forward`), and
the scan is sharply asymmetric:

| Interval | Median |
| --- | ---: |
| row B rises -> row D falls (a left action's budget) | 216 us |
| row D rises -> row B falls (a right action's budget) | 39 524 us |

Two runs 45 minutes apart agree to within 0.2 us. Row B is scanned immediately
before row D, so a slow left-side release would ghost the right-side button on
the same column while a right-side release never could. That predicts exactly the
reported symptom — left controls moving both treads, right controls clean — and
it is wrong.

Measured with the duplicate harness wires used as probes, which is what they are
good for: drive net A through BCM17 gated on row B, and read BCM26 (same net) and
BCM20 (row D) at the same time.

| Gated action | Net recovery after row rise | Ghost presses |
| --- | ---: | ---: |
| `left-forward` (18 -> 17) | 20.7 us median, 49.3 us max | 0 of 49 windows |
| `right-forward` (19 -> 22) | 19.3 us median, 23.9 us max | 0 of 50 windows |

The column is back up about 20 us after the row rises, against 216 us of budget,
with a 4x margin at the worst sample and not one ghost. An isolated
`gpio.setup()` benchmark reads 51 us median and 163 us max, which would have been
marginal, but that overstates the cost inside the settled gating loop.

### On blocks: the symptom does not reproduce unloaded

The confound that made every floor test ambiguous: when one tread drives and the
base pivots, the *undriven* tread is dragged backwards across the floor. From
outside that is indistinguishable from both treads being driven in opposite
directions. Putting the base on blocks removes it entirely.

0.4 s bursts, one at a time, physical result reported before the next was sent:

| Gated | Windows | Observed |
| --- | ---: | --- |
| `18:17` left-forward | 10 of ~9 | left tread forward |
| `18:22` left-reverse | 10 | left tread backward |
| `19:22` right-forward | 10 | right tread forward |
| `19:17` right-reverse | 10 | right tread backward |
| `18:17` + `19:22` FORWARD | 10 / 10 | both treads forward |
| `18:22` + `19:22` PIVOT LEFT | 10 / 10 | counterclockwise |
| `forward` via the UDP service | 8/8 acked | both treads forward |

All seven correct, including both pairs and the production service path.
`FORWARD` — the command that spun two-thirds of a turn on the floor and blocked
calibration — is correct off the ground.

### Where the fault is

Load-dependent and left-dominant. The right drivetrain is ahead of the battery
pack as the leading candidate, because the asymmetry shows up with a **single**
tread running (left alone rotated the base 154 deg, right alone 72.5 deg), where
supply sag is mildest. A pack that cannot feed two motors does not explain a 2:1
split with one motor drawing. A right side that spins freely unloaded but cannot
hold torque under load does, and it also fits the 56-93 dps fluctuation within a
single run, which reads as slipping rather than as a clean motor. The base struck
obstacles at least twice during session 1.

### The remote sleeps

It stopped scanning entirely mid-session — both rows flat, having been clean 45
minutes earlier — and returned once its power was checked. Idle timeout is
roughly 2-5 minutes. This is the remote, not the receiver, and it is a different
component from the one the priming investigation cleared. See "Priming retired"
above, now reopened.

Any gating script should refuse to drive until both rows show a low window,
rather than reporting a sleeping remote as a dead tread.

### lgpio notes for this Pi

`RPi.GPIO` here is the lgpio shim, so `setup()` is a kernel line release and
reclaim rather than a register write, and lines are claimed exclusively — the
motor service must be stopped before any manual GPIO work.

- direct `lgpio.gpio_claim_input` fails on every harness pin with every flag
  tried; use `RPi.GPIO` for inputs;
- `lgpio.gpio_claim_output(h, lgpio.SET_OPEN_DRAIN, pin, 0)` works and
  `gpio_write` toggles in 6.5 us without reclaiming the line, never driving the
  pin high. Kept on file as an available optimisation, not as a fix — release
  speed turned out not to be the problem.

### Next

1. With the base on blocks, load each tread by hand (a folded cloth against the
   tread, not fingers) and compare the force needed to stall each side at the
   same battery state. Check the right tread's tension, sprocket engagement, free
   play, and drag with the power off.
2. If both load up equally, fresh cells in the motor pack, then a floor re-run of
   `probe_matrix.py "left fwd=18:17" "right fwd=19:22" "FORWARD=18:17,19:22"`.
   Healthy is the two singles roughly equal and opposite with `FORWARD` near
   zero; the baseline to beat is -154 / +73 / -138.
3. Base calibration stays blocked until `FORWARD` drives straight on the floor.
   The worksheet is still empty, so nothing corrupt is recorded.

### Method note

Three separate mechanisms in this document were argued convincingly and then
failed on measurement: the duplicate-net fault, the release race, and a tread
asymmetry that was recorded, retracted, and observed again. In each case the
argument was sound and the mechanism was real; what was missing was a
measurement that could have said no. The on-blocks test cost about ten minutes
and settled what a day of reasoning could not.
