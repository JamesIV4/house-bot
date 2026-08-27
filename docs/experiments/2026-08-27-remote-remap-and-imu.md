# Remote rewire, GPIO remap, and IMU bring-up

**Date:** 2026-08-27

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

## The duplicate-net wiring fault

Every measurement above involving more than one simultaneous action is
invalidated by a fault found late in the session, and the ones taken after the
fix differ substantially.

The harness wires each of the remote's four matrix nets to the Pi **twice**,
once at each button touching it. `MATRIX_PINS` treated the eight wires as eight
independent pins, so it drove and released the same net through two different
pins. Releasing BCM16 does nothing while BCM22 still holds net C low.

It hid for so long because **every action is correct in isolation**: a single
action touches only one pin per net, so all four verified perfectly one at a
time, and `identify-rows` classified every wire correctly. Only simultaneous
pairs break.

Symptoms, all from this one cause:

- `forward` drove one tread and spun the base hard right rather than driving
  straight, intermittently;
- a single left-side button press moved *both* treads on a tank-style base;
- pivots ran on one tread much of the time. A one-tread pivot still rotates the
  base, so this went unnoticed, surfacing only as unexplained variance in pivot
  rate (84-150 dps) and coast (30.4-37.9 deg).

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
- **A release race into the next row's scan window.** Rejected: with two actions
  active the column is held 160-249 us against a 428 us budget, 0 of 148 presses
  overrunning.

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

### Oddity, not being pursued

The single-tread results changed sharply across the wiring fix even though the
old and new `MATRIX_PINS` are electrically identical for single actions: old
`right-forward` was (BCM19, BCM16) and new is (BCM19, BCM22), and BCM16 and
BCM22 are the same net. A later repeat sweep confirms the post-fix numbers are
stable, so the pre-fix sweep is the anomaly rather than the current one.

Recorded as an oddity and closed. The wiring fault is fixed, forward and
reverse drive correctly, and the post-fix measurements are the real results.

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
