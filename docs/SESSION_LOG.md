# Session Log

Keep entries concise and evidence based. Link detailed experiment records when
they exist.

## 2026-08-30 (later) - Real-time scheduling; dead duty-cycle code removed

### Changed

- **The motor service now runs `SCHED_FIFO` priority 10.** Its blind time
  between polls, not the release cost, was the dominant term against the 419 us
  budget: 629 us worst case on the fair scheduler, with 20 574 involuntary
  preemptions in 44 minutes. It now takes about **2 per restart**. The service
  requests this itself and logs a warning if refused.
  - The permission cannot come from the unit file. A *user* unit cannot set
    `AmbientCapabilities` -- systemd exits `218/CAPABILITIES` and the service
    fails to start -- and the user's rtprio hard limit is 0, so `LimitRTPRIO=`
    cannot raise it. It comes from
    `/etc/security/limits.d/99-housebot-rtprio.conf` containing
    `james - rtprio 20`, then `systemctl restart user@1000`.
  - Check with `ps -o cls,rtprio`: healthy is `FF 10`, not `TS -`.
- **Removed the pulse-density machinery from `poll()`** -- duty accumulator,
  per-action enable flags, and the levels table were recomputed on every pass of
  the hottest loop in the system while doing nothing: fractional magnitudes are
  rejected by `require_binary_wheels` at the door. The
  `--allow-experimental-pulse-density` flag went with them, since nothing was
  left behind it and accepting a fractional value would now silently mean full
  power. `poll()` is now read the scan line, drive the column while it is low,
  release before assert.

### Verified

- Six paired legs, same shape and gap as the scale route, with IMU logging.
  **Total direction reversals 42 -> 16.**
  - `reverse` is now **clean: 0 reversals on all three legs**, and its yaw swing
    collapsed from about +/-100 dps to +/-25.
  - `forward` still oscillates: 4, 8, 4 reversals, with swings to -72 dps.

### Open

- **Why forward and not reverse.** The gating is symmetric between them -- same
  scan slots, same two-pass ordering, mirror-image columns -- so the asymmetry
  is not in this code. Physical drivetrain difference is the obvious candidate
  and the project has a recorded history of suspecting the right side. Not
  investigated.
- Packet acknowledgement dipped again on some legs (91.7%, 96.7%, 0% on
  tail-stop) while the service stayed up. Wi-Fi latency, not the service.

### Retracted

- **The open-drain optimisation does not work on this kernel** (6.18.34, lgpio
  0.2.2), despite the note carried since 2026-08-27. `gpio_claim_output` with
  initial level 1 is rejected outright; with level 0 it claims and `gpio_write`
  is genuinely 6.1 us, but writing 1 **never releases the line to high
  impedance** -- verified by reading the duplicate wire on the same net. And
  `gpio_free` does not restore the pin, which left BCM17 driving low and BCM16
  driving high into the remote's own scan line until it was caught and restored
  with `pinctrl set <pin> ip pn`. **Do not retry without a scope.**
- **The "camera contention" hypothesis is dropped.** It was never demonstrated,
  only asserted, and it is not a basis for anything.

## 2026-08-30 - Remote latch traced to the F key; right tread moved to channel D

### Root cause

- The remote is a **5x2 matrix, not the 2x2** this project assumed: ten buttons,
  being two four-way pads (four motor channels) plus the **E and F function
  keys**. Confirmed against the manufacturer's manual, which states *"E button :
  Merges A and B channel. F button : Merges A and C channel"*, and that channel
  **A is always the merge master**.
- The treads were on receiver ports **A and C**. F merges A and C, which is
  exactly the reported fault: *left buttons drive both treads, right buttons
  drive only the right tread*. Not corruption -- F, working as designed, latched
  until the transmitter was power-cycled.
- The Pi presses buttons by grounding a **direction line**, which is shared by
  every button on the keypad including E and F. Only release timing keeps the
  press looking like one button. A finger bridges one contact and is inert
  during every other channel's scan turn, which is why no human input could
  ever reproduce this.
- Credit: Codex "Daybreak Blue" identified the hidden channel-merge mechanism
  and the submatrix framing before any of the measurements below.

### Changed

- **Right tread moved from receiver port C to port D**, and its Pi wires from
  the right pad's up/down buttons to its left/right buttons. No merge function
  touches channel D, and channel A is only ever a master -- so F, E and E+F now
  merge the left tread into empty ports and **every latched state is a no-op**.
  The Pi can still ghost-press F; it has nowhere to land.
- `poll()` now releases every column that must go high **before** asserting any
  that must go low. A single pass applied them in pin order, so whether a frame
  was safe depended on how pin numbers happened to sort; on this harness
  `reverse` sorted the wrong way and the remote saw both directions of one
  channel pressed at once for the length of a `setup()` call, 25 times a second
  for the whole leg. Regression test verified against the old code.
- `MATRIX_PINS` re-measured and corrected. `docs/REMOTE_WIRING_PINOUT.md`
  rewritten for the 5x2 model.

### Verified

- `identify-rows` resolved all four pairs cleanly; the harness still collapses
  to four nets across eight wires. **The scan/direction roles swapped sides**
  when the right tread moved, despite the leg positions being kept the same --
  role is electrical, never geometric.
- All four intersections re-tested singly through `matrix-pulse`, base on
  blocks, physical result reported before the next. Clean bijection, no
  inversion flags. The right side's two direction labels had to be swapped.
- Scan timing: channel A and D windows both ~225 us; **A rises -> D falls
  419 us** (channel C sat one slot away at 216 us); frame 40.16 ms.
- **Gaps between route legs are not needed.** A 30-leg stress route -- 24
  pivots, the pattern that historically broke it fastest -- ran twice: once at
  a 0.4 s cadence, then with **zero gaps**, 14.3 s of unbroken motion with a
  direct reversal at every leg boundary and no stop in between. Both were
  followed by a four-leg single-tread probe. **No latch either time**, and no
  misbehaviour observed. 100% packet acknowledgement on all 30 legs of both
  runs.

### Caveats

- One clean run is weak evidence on its own. The historical latch rate was
  roughly one per few minutes of driving, so 14 s proves little by itself; the
  reason to trust the fix is the channel move, not this run.
- The no-gap probe lost acks on its last two legs (66.7%, then 0% on
  tail-stop). The service was **active with 0 restarts**, `throttled=0x0`, and
  the Wi-Fi interface showed zero errors and zero drops -- ping measured 0%
  loss but latency spikes to 235 ms against a 19 ms average. Transient Wi-Fi
  latency, not the service. Lost acknowledgements are not lost commands, and
  the 350 ms watchdog covers a genuinely missed stop.
- **The service's stdout is not being captured** -- `journalctl --user` reports
  no journal files, so watchdog messages are invisible. Worth fixing before
  relying on the log for evidence.
- Gaps still matter for a separate, unchanged reason: the receiver sleeps after
  roughly 5 s idle and drops the opening of the next command. That is about
  priming, not about protecting the remote.
- Still unrecorded: which direction line carries F and which carries E, and
  where the function row sits in the scan order.

## 2026-08-23 - Motor base rebuilt to front-drive; inputs remapped

### Changed

- Rebuilt the tread base so both motors mount at the front. Previously the left
  side was driven from the back and the right from the front, making the
  drivetrain mirror-asymmetric; that asymmetry is the most likely cause of the
  direction-dependent per-side speed differences fitted on 2026-08-20.
- The right side gained one extra gear, which reverses its rotation.

### Verified

- Re-tested all four matrix-gated raw actions individually, one action per
  trial, with the motor service stopped:

  | Raw pins (row/col) | Code label | Physical result |
  | --- | --- | --- |
  | BCM5 / BCM4 | `left-forward` | left tread backward |
  | BCM7 / BCM6 | `left-reverse` | left tread forward |
  | BCM9 / BCM8 | `right-forward` | right tread forward |
  | BCM11 / BCM10 | `right-reverse` | right tread backward |

- The map is a clean bijection with no duplicates. The code's `left-*` pins now
  drive the left tread and `right-*` drive the right tread, so the side swap
  required before the rebuild is gone. Only the left tread is inverted.
- Reduced the service flags from `--invert-left --invert-right --swap-sides` to
  `--invert-left`, redeployed, and confirmed the running unit picked them up.
- Verified all four semantic motions through `scripts/send_motor_command.py`
  against the live service: `forward` drove both treads forward, `reverse` both
  backward, `left` pivoted counterclockwise, and `right` pivoted clockwise.
  Every run acknowledged 40/40 packets with an acknowledged stop.

### Recalibrated

- Re-ran the four full-power 3-second trials against the front-drive base and
  solved with `scripts/calibrate_base.py`; result was `CALIBRATION_QUALITY=PASS`.
- Forward travelled 0.838 m and reverse 0.737 m, both with **zero heading
  drift**, against 0.58 m at -17.5 deg and 0.740 m at +5 deg on the previous
  build. Pivots reached 450 deg (left) and 390 deg (right) in three seconds.
- Fitted 0.216 m effective track width, tread speeds of 0.245-0.281 m/s, a
  0.144 m conservative radius, and at most 7.0% coefficient of variation.
- Geometry re-measured: 6 x 6 in footprint (5 in treads plus a 1 in rear body
  extension) and the camera 1 in forward of `base_link`, 6 in high, level.
- The 2026-08-22 command-slot duty trim is retired. It existed only to cancel
  the old drivetrain asymmetry, which the rebuild eliminated.

### Speed control rejected a second time

- Tested command-slot duty cycling as a *speed* control by applying equal duty
  to both treads. This is distinct from the GPIO scan-window pulse density that
  D-020 rejected: every packet remains a full-power binary command and only
  whole 50 ms slots are dropped.
- Measured over three seconds: 60/60 slots gave 0.838 m, 30/60 gave 0.610 m,
  and 10/60 gave 0.292 m. Effective speed is therefore 100%, 73%, and 35% of
  full against a linear expectation of 100%, 50%, and 17%.
- The curve is heavily compressed because tread inertia carries the base
  across short gaps. At 10/60 the 300 ms gaps exceed that coasting window: the
  base moved in visible stop-start steps and drifted about 15 degrees left as
  per-side stiction stopped cancelling.
- Conclusion: duty cycling is not a usable proportional speed control. The base
  stays binary full-power/stop, and motion is commanded as timed full-power
  segments using the calibrated speeds above.

### Receiver wake-up discovered; turns calibrated

- Turn results were erratic in a way command duration could not explain: 1.30 s
  produced less rotation than 1.27 s, and 0.622 s sometimes produced none at
  all. Fitting models to this repeatedly failed.
- Root cause: the receiver ignores the opening of a command while it wakes, and
  drops a variable amount each time. Sending one throwaway packet, waiting, then
  issuing the real command removes it. The same 1.27 s left turn went from about
  180 deg unprimed to 300 deg primed.
- `scripts/drive_primed.py` implements this. A single 0.05 s packet is used: a
  3-packet prime moved the base whenever the receiver happened to be awake
  already, adding its own variable rotation.
- Measured the wake window with `scripts/test_wake_timeout.py`. An unprimed
  command after 5 s idle ran at full strength; after 10 s it produced 60 deg
  where 90 deg was expected. So the receiver stays awake roughly 5 s. Priming is
  only needed after a longer gap, and during a route with about 1 s between
  segments one prime at the start is enough.
- Calibrated turns, walked in physically rather than derived:

  | Turn | Duration |
  | --- | --- |
  | left 90 deg | 0.53 s |
  | left 180 deg | 1.15 s |
  | right 90 deg | 0.49 s |
  | right 180 deg | 1.14 s |

- Left and right agree to within 0.01 s, matching the symmetric front-drive
  rebuild. The 450/390 deg left/right split measured on 2026-08-20 was a wake
  artefact rather than a drivetrain imbalance.
- 360 deg was not calibrated; it is not needed and the scatter over a run that
  long exceeded the adjustments being made.

### Reinterpreted

- The "intermittent right tread" observed repeatedly today is at least partly
  this wake behaviour. If the receiver wakes its channels unevenly, one tread
  starts late, which looks exactly like a dropout and pulls the base off
  heading. That reading fits the symptoms clustering at the starts of runs.
  A loose gear was genuinely found and fixed on 2026-08-22, so both effects have
  been present; how much of today belongs to each is not established.
- Measurements taken today before the prime was introduced should be treated as
  contaminated. That includes the straight-line startup dead time, the turn
  timings, and the scale route.

### Invalidated

- `config/local/base_calibration*` describes the previous drivetrain. The fitted
  track width, four direction-specific tread speeds, and footprint must be
  re-measured before any calibrated motion is trusted.
- The command-slot duty trim explored on 2026-08-22 (left duty 0.50 for a
  straight forward run) described the old asymmetric drivetrain and no longer
  applies.

## 2026-08-21 - Post-rebuild motor mapping re-verified

### Verified

- After the latest rebuild, re-tested each of the four matrix-gated raw
  actions in isolation, one action per trial: row5/col4, row7/col6, row9/col8,
  row11/col10. Each was repeated until reproducible (3 consecutive matching
  trials): row5/col4 drives the right tread backward, row7/col6 the right
  tread forward, row9/col8 the left tread backward, row11/col10 the left
  tread forward.
- Verified all four combined two-tread actions against that per-tread map
  using `matrix-drive`: `forward` (row5/col4 + row9/col8) moved both treads
  backward, `reverse` (row7/col6 + row11/col10) moved both treads forward,
  `left` produced a counterclockwise pivot, and `right` produced a clockwise
  pivot — all matched prediction with no contradictions.
- Confirmed end-to-end through the production path: `send_motor_command.py
  forward` and `reverse` against the running `house-bot-motors.service` both
  produced the correct physical motion.

### Observed

- The row5/col4 action was not reproducible at first: trial 1 moved both
  treads backward (consistent with the pre-matrix shared-column artifact
  described above), trial 2 showed only the left tread backward, before
  stabilizing to "right tread backward" for three consecutive trials.
  Recorded as provisionally settled rather than fully explained.

### Changed

- None. The `house-bot-motors.service` flags (`--invert-left --invert-right
  --swap-sides`), already deployed since the 2026-08-20 rebuild calibration,
  reproduce exactly the mapping re-verified here. No code, pin-table, or
  deployment change was required — this rebuild did not change the
  Pi-GPIO-to-remote wiring, only the chassis/tread assembly.

### Next action

- None outstanding for direction mapping. If the base is rebuilt again,
  repeat this one-raw-action-at-a-time verification before trusting the
  deployed service flags.

## 2026-08-20 - Fractional original-remote control rejected

### Observed

- Three timestamped 50% pulse-density routes were attempted. During most short
  segments only the left tread moved.
- The third route was repeated after the remote was explicitly woken and showed
  the same physical failure, so receiver timeout does not explain the tread
  asymmetry.
- The third capture still provided healthy SLAM evidence: DPV-SLAM initialized,
  processed 1,200 poses, retained 22 keyframes, and sustained 26.80 tracking
  FPS. It is rejected as metric-scale evidence because the commanded physical
  motion was not executed by both treads.
- All route packets and stops were acknowledged. This confirms transport and
  watchdog behavior, not physical motor execution.

### Changed

- Made the Pi motor service binary full-power/stop by default. Fractional
  magnitudes are rejected unless the service is started with an explicitly
  experimental pulse-density flag.
- Corrected the command client so a stop acknowledgement cannot make a route
  appear successful when every drive packet was rejected.
- Added a `proportional_control_verified` gate that prevents the ROS base bridge
  from arming under the current actuation model.
- Changed the reusable visual-scale route to full-power segments of at least
  1.5 seconds. The scale tool rejects legacy/fractional routes, requires four
  agreeing straight legs, and requires explicit operator confirmation that both
  treads moved.

### Next evidence

- Run one observer-confirmed full-power timestamped route for DPV-SLAM metric
  scale. Treat the result as visual calibration only, not authorization for
  autonomous driving.
- Identify a proportional motor driver and wheel-feedback path suitable for the
  standard ROS 2 differential-drive controller before connecting the real base
  to Nav2.

## 2026-08-20 - Rebuilt tread polarity corrected

### Observed

- The first three-second command was invalid as a calibration sample because
  the robot was being adjusted when it started.
- The semantic `forward` command moved the rebuilt tread base backward, showing
  that the motor orientation changed during the rebuild.

### Changed

- Added independent left/right inversion flags at the Pi motor-service boundary
  so `/cmd_vel` and calibration tools retain standard forward/left semantics.
- Selected both inversion flags for the current rebuilt base. Direction must be
  reconfirmed with a short bounded command before restarting calibration.
- Deployed the inverted service and confirmed it remained active with the new
  arguments. The first stop-only probe received no acknowledgement; service
  status showed it running normally, and an immediate retry acknowledged the
  stop. No movement command was sent after the invalid trial.
- A subsequent 1.5-second semantic-forward check moved physically forward. The
  first measured three-second forward run delivered an acknowledged stop and
  ended about 21 in forward and 9 in right at roughly 15-20 degrees right. This
  is recorded as an approximate 0.58 m displacement and -17.5 degree heading
  change; the solver now uses straight-run heading to retain startup asymmetry.
- The three-second reverse run ended about 29 in backward and 3 in right with
  an approximately 5-degree counterclockwise heading change, recorded as
  0.740 m and +5 degrees. Reverse travel was faster and straighter than forward.
- The first semantic left-pivot trial instead rotated approximately 330 degrees
  clockwise, ending near the starting 11-o'clock heading. The sample was
  discarded; straight polarity was correct, identifying swapped left/right
  tread channels after the rebuild. The Pi boundary now also swaps sides.
- After deploying the side swap, a 1.5-second check turned counterclockwise as
  intended. The repeated measured three-second left pivot ended around the
  2:00-2:30 clock heading after turning counterclockwise, recorded as 293 degrees.
- The measured three-second right pivot turned clockwise to about the 11-o'clock
  heading, matching the earlier observed endpoint and recorded as 330 degrees.
- The resulting coarse fit passed its repeatability gate. It estimated a 0.244 m
  effective skid-steer width, direction-specific full-scale tread speeds of
  0.194-0.239 m/s, a 0.148 m conservative footprint radius, and at most 9.3%
  coefficient of variation across each fitted tread/direction observation.
- The generated real-base ROS parameters launched against the Pi for five
  seconds in the expected disabled/open-loop state and sent no motion command.
- A repeated three-second 50% forward command moved about 10 in forward and
  2 in right, a 0.259 m displacement. Later multi-segment tests showed that
  this single endpoint was misleading: most fractional commands activated only
  the left tread. D-020 supersedes the earlier linear-model conclusion.

## 2026-08-20 - Real-base calibration and Nav2 handoff implemented

### Verified

- Added scan-window pulse-density control to the Pi service; 0.5 duty selected
  exactly four of eight synthetic remote scan windows, while repeated 20 Hz
  network refreshes preserved phase.
- All 20 host-side motor, GPIO, protocol, and calibration tests passed.
- The ROS Jazzy image built and all 11 navigation-package tests passed.
- A synthetic calibrated base launch stayed disarmed, published the explicit
  open-loop warning, and exited cleanly after a three-second smoke test.
- The full loopback Nav2 Kitchen route succeeded at `(2.504, -0.061)`, 0.114 m
  from the target. The first attempt exposed an acceptance-test startup race;
  the harness now waits for `bt_navigator` lifecycle state `active` before
  sending a goal.
- Deployed the updated Pi service to `192.168.0.241`; its systemd user service
  reported `active`, and a stop-only UDP check received an acknowledged stop.
  No movement command was sent during deployment verification.

### Changed

- Added a repeatable measurement worksheet and solver for effective skid-steer
  width, footprint, four direction-specific tread rates, and the C920 transform.
- Recorded the rebuilt base as approximately 6.5 x 6 in, including its rear
  extension. Recorded the adjustable C920 mount as centered, about 3 in forward
  and 6 in high, with a nominal 5-degree downward pitch and conservative 1 in
  footprint margin.
- Added the initially disarmed ROS `/cmd_vel` to UDP bridge, high-covariance
  command-integrated `/odom`, TF publication, acknowledgement timeout, enable
  service, and latched emergency stop.
- Preserved Nav2's NavFn, regulated pure pursuit, velocity smoothing, behaviors,
  lifecycle, waypoint, and Vizanti layers; custom code remains at the unusual
  remote and pose-adapter boundaries.

### Next physical evidence

- Repeat an observer-confirmed, full-power natural-scene DPV-SLAM route for
  metric scale alignment. Do not use fractional pulse-density commands.

## 2026-08-20 - One-second directional motor command test

### Verified

- Confirmed `house-bot-motors.service` was active on `housebot` at
  `192.168.0.241` before testing.
- Sent each command for 1.0 second through `scripts/send_motor_command.py` in
  this order: `forward`, `reverse` (backward), `left`, and `right`.
- Each test sent 20 drive packets, received 21 acknowledgements including the
  stop packet, and reported `stop acknowledged=True`.
- The client therefore confirmed command delivery and clean stop handling for
  all four directions.

## 2026-08-19 - Original remote selected as the no-new-parts motor bridge

### Verified

- The `GT004TX-V01` remote uses two AAA cells and has a single-sided PCB with
  an unmarked radio/control IC, 16 MHz resonator, and printed 2.4 GHz antenna.
- Bluetooth discovery and advertisement experiments did not produce a hub
  pairing or motor response.
- The Pi 3B and PC Wi-Fi/Bluetooth interfaces cannot directly emit the likely
  XN297-family proprietary packets through their normal APIs.

### Changed

- Added a Pi GPIO probe/controller that senses a button pad as a high-impedance
  input before it permits active-low motor pulses.
- The controller releases buttons as GPIO inputs and only ever drives them
  low; it never drives the remote circuit high.
- Added the exact one-wire-at-a-time verification and provisional four-button
  wiring procedure in `docs/MOTOR_REMOTE_GPIO.md`.

### Next evidence

- Live probing observed brief `1 -> 0 -> 1` scan windows while the physical
  button was held. A two-second continuous column-low pulse moved both motors,
  proving the remote buttons are matrix-scanned rather than independent
  active-low inputs.
- The opposite button pad exposed a 215.8 us median target-row low window every
  40.15 ms. The Pi mirrored 49 windows in two seconds and moved only the
  left wheel forward, proving software-gated matrix control.
- The left-reverse pair measured a 220.9 us median BCM7 row window every
  40.13 ms; gating its BCM6 column moved only the left wheel backward.
- The right-forward pair measured a 219.4 us median BCM9 row window every
  40.13 ms; gating its BCM8 column moved only the right wheel forward.
- The right-reverse pair measured a 221.1 us median BCM11 row window every
  40.11 ms; gating its BCM10 column moved only the right wheel backward.
- All four directions and the paired forward, reverse, pivot-left, and
  pivot-right commands moved the correct wheels.
- Installed an auto-starting Pi UDP motor service on port 8765 with monotonic
  command sessions, stale-packet rejection, an acknowledged stop, and a 350 ms
  command-loss release. The stop-only WSL network test passed; live network
  motion and the `/cmd_vel` adapter are next.
- Saved the complete nine-wire pinout in
  `docs/REMOTE_WIRING_PINOUT.md`.

Detailed record: `docs/experiments/2026-08-19-gt004-remote-control.md`.

## 2026-08-17 - Hardware-free Nav2 and browser UI implemented

### Verified

- The container built from the digest-pinned Jazzy base and revision-pinned
  Vizanti source, then all Nav2 lifecycle nodes reached active state.
- Four navigation-package unit tests passed.
- The end-to-end named Kitchen route succeeded at `(2.506, -0.059)`, 0.111 m
  from its `(2.6, 0.0)` target and inside the 0.30 m acceptance gate.
- The rendered browser UI showed the mock occupancy map, transforms, live
  planned path, and named buttons; its browser console reported no warnings or
  errors. Clicking Home in the browser invoked Nav2 and the return route also
  succeeded.

### Changed

- Added a pinned ROS 2 Jazzy Docker/Compose runtime with Nav2, RTAB-Map ROS,
  rosbridge, and Vizanti revision `ab43643b`.
- Added a deterministic mock house and the official Nav2 loopback simulator as
  the temporary `/cmd_vel`/`/odom` mobile base.
- Configured a small differential-base Nav2 stack using NavFn, regulated pure
  pursuit, velocity smoothing, static costmaps, behaviors, and waypoint
  actions.
- Added a House Bot adapter for point goals, waypoint lists, named-destination
  services, cancellation, and latched JSON state.
- Added a project Vizanti layout for map, TF, odometry trail, active path,
  point goals, waypoints, four named mock rooms, and navigation state.
- Added one-command launch and an end-to-end test that must reach the mock
  Kitchen within 0.30 m.

### Hardware handoff

- The real Pi/base adapter must replace loopback by consuming `/cmd_vel` and
  publishing `/odom` plus `odom -> base_link`.
- Metric localization/mapping must publish `/map` and `map -> odom`.
- Provisional 0.16 m radius and velocity limits must be replaced with measured
  assembled-base values.

Detailed guide: `docs/NAVIGATION.md`.

## 2026-08-17 - Measured C920 calibration and longer Pi route passed

### Verified

- The portable-power-bank C920 stream delivered 451 frames over 14.9934 source
  seconds, or 30.013 FPS, despite current throttle/undervoltage bits in 10 of
  12 simultaneous Pi samples.
- A fixed-focus 9x6 checkerboard capture retained 29 of 32 diverse views and
  solved 1280x720 intrinsics at 0.5706 px RMS reprojection error; the maximum
  retained-view error was 1.0503 px.
- The measured-calibration larger-space run stayed initialized for 1,000 poses
  over 60.2667 source seconds, produced poses at 16.576 Hz, and sustained 16.887
  synchronized tracking FPS.
- The run retained 154 keyframes, formed 132 long-range frame pairs, kept
  newest-frame queue age below 54.82 ms, and left no Pi camera process running.
- Relative to the closest initialized early-route pose, the final pose had a
  1.150% path-normalized translation residual and 6.10 degree orientation
  residual. Scale remains ambiguous and the checkerboard made this loop easier
  than a natural-scene return.

### Changed

- Added a reusable screen checkerboard, automatic diverse-view collector,
  calibration quality gates, and one-command Pi calibration launcher.
- Added `config/c920-dpvo-measured.txt` and selected it for Pi DPV-SLAM runs.
- Locked calibration and navigation capture to manual focus value zero and
  precomputed the undistortion maps used by the live runner.
- Added a post-initialization loop metric so DPV-SLAM's motion-probe origin no
  longer distorts route closure reporting.

Detailed record:
`docs/experiments/2026-08-17-c920-calibration-long-route.md`.

## 2026-08-17 - Raspberry Pi C920 stream reached DPV-SLAM

### Verified

- Discovered `housebot` at `192.168.0.241`: Raspberry Pi OS Lite based on
  Debian 13, aarch64, with 905 MiB RAM, 904 MiB swap, and no throttle flags.
- The attached C920 is `/dev/video0` and exposes native H.264 at
  1280x720/30 FPS.
- After disabling exposure-driven dynamic framerate, a 10.004-second native
  H.264/MPEG-TS test contained 301 frames and used 3.9 MiB.
- A bounded Pi-to-WSL DPV-SLAM smoke decoded 215 frames, processed 150, and
  deliberately dropped 65 stale frames. P95 newest-frame queue age was
  0.169 ms and p95 decoder-to-processing completion was 14.21 ms.
- The stationary camera correctly remained in DPVO's motion-probe stage; a
  physical moving-camera pass was then used for trajectory evidence.
- The launcher left no camera/FFmpeg process running on the Pi after exit.
- The moving run initialized DPV-SLAM, saved 300 poses over a 17.63-second
  source span, retained 54 keyframes, and produced seven long-range frame
  pairs. Effective live output was 16.96 Hz and synchronized compute throughput
  was 18.15 FPS.
- P95 pose processing was 59.27 ms, only 2 of 290 steady-state frames missed
  the 15 Hz compute deadline, and newest-frame age never exceeded 31.88 ms.

### Changed

- Installed FFmpeg on the fresh Pi and deployed a passwordless project SSH
  key outside the repository; the supplied password was not stored.
- Changed the Pi C920 sender to native H.264 passthrough in MPEG-TS and disabled
  dynamic exposure framerate before capture.
- Added PyAV to the DPVO environment, a newest-frame live DPV-SLAM runner, and
  a one-command Pi deploy/start/run/cleanup launcher.
- Marked the Pi newest-frame live pose gate complete; measured C920 calibration
  and a longer robot-mounted route are next.

Detailed record:
`docs/experiments/2026-08-17-pi-c920-transport.md`.

## 2026-08-17 - DPVO / DPV-SLAM production pose selected

### Verified

- Built pinned DPVO commit `859bbbfd` with PyTorch 2.8.0, CUDA 12.8, and native
  RTX 5060 Ti `sm_120` extensions.
- DPVO pose-only processed the 796-sample C920 loop at 17.33 FPS.
- Stock DPV-SLAM processed the one-minute loop at 16.07 FPS, but fell to 11.90
  FPS on a five-minute graph-growth test and no longer kept up with input.
- The upstream 48-patch fast profile reached 34.05 FPS but its normalized loop
  residual worsened to 19.67%, so it was rejected.
- The 96-patch navigation profile processed one minute at 17.59 FPS and five
  minutes at 16.43 FPS. Its final 481-frame window remained at 15.78 FPS, with
  a 4.70% five-loop residual and 2.52% steady-state deadline misses.

### Changed

- Added reproducible DPVO bootstrap, environment check, benchmark runner,
  official model checksum, C920 approximate calibration, and Blackwell patch.
- Added `config/dpvo-navigation.yaml`, reducing synchronous loop-optimization
  frequency from every 15 retained keyframes to every 60 without reducing the
  default 96 tracking patches.
- Selected this DPV-SLAM profile for the next live C920/Pi pose test while
  retaining MASt3R-SLAM for dense diagnostic maps.

Detailed record:
`docs/experiments/2026-08-17-dpvo-dpv-slam-benchmark.md`.

## 2026-08-17 - Navigation-oriented SLAM performance decision

### Verified

- The user's moving live dense-viewer run processed only about 1.9-2.1 FPS;
  the viewer was a separate bottleneck from headless MASt3R inference.
- Quarter-density pointmaps completed the same 796-sample room recording in
  117.68 seconds versus the 138.7-second baseline.
- The tuned run retained 71 versus 73 keyframes, estimated a 12.297 versus
  12.332 m path, and reduced the PLY from 7.38 million to 1.86 million points.
- A 300-sample visualized comparison reached 7.21 cumulative FPS with a 2 Hz
  full-map refresh versus 3.70 FPS with a 10 Hz refresh.
- A 60-sample live C920 headless smoke test completed with exit code 0 and
  reported 9.21 FPS at sample 30.

### Changed

- Repaired upstream `img_downsample` shared-buffer sizing.
- Added `config/c920-navigation.yaml` and applied the same pointmap/viewer
  limits to the live C920 configuration.
- Added a reproducible bounded viewer benchmark.
- Reframed MASt3R-SLAM as the dense mapping baseline, with DPVO/DPV-SLAM next
  for the production pose benchmark.

Detailed record:
`docs/experiments/2026-08-17-mast3r-navigation-performance.md`.

## 2026-08-17 - Live C920 SLAM and viewer completed

### Verified

- Windows FFmpeg connected to a WSL TCP listener and streamed live C920 H.264
  MPEG-TS at 960x540.
- PyAV 18.1.0 decoded continuously while exposing only the newest frame.
- A bounded 60-sample headless run completed at approximately 8.39 FPS and
  saved a PLY and trajectory with exit code 0.
- A second 60-sample run opened the interactive `MASt3R-SLAM (Ubuntu)` WSLg
  viewer, completed mapping, and saved artifacts while the viewer remained
  open.

### Fixed

- Replaced spawned visualization with a thread to avoid invalid CUDA IPC
  resource handles under WSL/Blackwell.
- Added environment-local `libGL.so` and `libEGL.so` loader links.
- Selected TCP after UDP failed across the Windows/WSL boundary.
- Added `--max-frames` for bounded live automation.

Detailed record: `docs/experiments/2026-08-17-live-c920.md`.

## 2026-08-17 - Pi camera transport prepared

### Verified

- No Pi responded as `raspberrypi.local`, `house-bot.local`, or
  `housebot.local`.
- Added WSL outbound `tcp-connect://` support so the Pi can serve the stream
  without exposing WSL through Windows NAT.
- Added raw MJPEG decoding for a Pi sender that does not spend Pi 3B CPU on
  video transcoding.
- A Windows-hosted stand-in MJPEG server connected to WSL, supplied 30 live
  samples to MASt3R-SLAM, and completed with exit code 0.

### Added

- `scripts/pi_stream_c920.sh`: V4L2 C920 MJPEG TCP server for the Pi.
- `scripts/run_pi_camera_slam.sh`: WSL viewer/mapper connection to a Pi host.

### Next evidence needed

- Pi hostname or IP, Raspberry Pi OS version, and SSH availability.
- C920 V4L2 device path and supported MJPEG mode on the Pi.
- Actual Wi-Fi throughput, delay, dropped-frame behavior, and reconnect result.

## 2026-08-17 - First C920 room map completed

### Verified

- Captured 60.033 seconds / 1,593 frames from the C920 at 1280x720.
- Processed 796 samples using every second frame in 138.7 seconds.
- Saved 73 keyframes, a 110,772,451-byte PLY, and a 73-pose trajectory.
- Reconstruction contains 7,384,818 colored points.
- Tracker successfully relocalized four times after skipped frames.
- Estimated path length is 12.332 m; final position is 0.558 m from the initial
  position. No external ground truth is available for an accuracy score.

### Fixed

- Replaced unreliable per-frame OpenCV seeking with sequential video decoding.
- Corrected video timestamps when input subsampling is enabled.
- Verified all 796 sampled positions before the successful rerun.

Detailed record:
`docs/experiments/2026-08-17-c920-room-loop.md`.

## 2026-08-17 - First SLAM result completed

### Verified

- Built PyTorch 2.8.0/CUDA 12.8 plus the MASt3R-SLAM, lietorch, and RoPE CUDA
  extensions for the RTX 5060 Ti's `sm_120` target.
- Environment GPU matrix-multiply and compiled-module import checks pass.
- The Freiburg room sequence completed headlessly in 138.7 seconds at roughly
  5.5 processing FPS after startup.
- It saved 51 keyframes, a 134,410,876-byte PLY reconstruction, and a
  TUM-format pose trajectory.
- `evo_ape` measured 0.063373 m translation RMSE after SE(3) alignment.
- Windows FFmpeg detects the Logitech as `HD Pro Webcam C920`; a 1280x720 MP4
  capture into WSL-visible `data/input/` completed and was readable by
  `ffprobe`.
- MASt3R-SLAM processed the resulting 2.03-second, 31-frame C920 smoke clip with
  exit code 0 in 16.1 seconds, proving the Windows capture-to-WSL decoder path.

### Changed

- Added reproducible bootstrap, asset download, sample download, run,
  environment-check, reference-evaluation, and C920-capture scripts.
- Added `docs/SLAM_BRINGUP.md` with the exact known-good commands and results.
- Patched upstream Blackwell compatibility, current PyTorch extension APIs, and
  optional RealSense loading without modifying the pinned upstream revision.

### Next action

- Record a deliberate 60-90 second handheld C920 room loop, run it through the
  working pipeline, and inspect the saved reconstruction and loop closure.

## 2026-08-17 - Direction changed to SLAM first

### Decision

- Removed the hardware-first and teleoperation-first sequence.
- Made standalone MASt3R-SLAM the first implementation target.
- ROS, Pi transport, mobile-base integration, navigation, and interaction now
  follow a demonstrated live mapper.

### Environment findings

- The RTX 5060 Ti reports compute capability 12.0 (`sm_120`).
- WSL can see the GPU through the Windows driver.
- WSL currently has no CUDA toolkit/compiler, PyTorch, Conda/Mamba, FFmpeg, or
  exposed `/dev/video*` device.
- Upstream MASt3R-SLAM documents PyTorch 2.5.1 with CUDA up to 12.4; RTX 50-series
  compatibility work requires a newer Blackwell-capable toolchain and extension
  API changes.

### Next action

- Add and run the reproducible MASt3R-SLAM bootstrap for PyTorch/CUDA with
  `sm_120` support.

## 2026-08-17 - Repository and MVP plan established

### Verified

- Repository exists at `/home/james/Repos/house-bot` in Ubuntu WSL.
- WSL is Ubuntu 24.04.4 LTS on WSL2, x86_64.
- Host GPU reports NVIDIA GeForce RTX 5060 Ti, 16,311 MiB VRAM, Windows driver
  596.49.
- Available robot hardware currently described as a wheeled battery platform,
  Raspberry Pi 3B, Logitech 1080p webcam, and Wyze Cam Pan v2.

### Changed

- Reworked the initial research-heavy roadmap into measurable MVP phases.
- Selected the Logitech as the fixed baseline camera.
- Added a non-destructive, time-boxed Wyze evaluation path.
- Added durable repository instructions, hardware inventory, decisions,
  references, and ignore rules.

### Unknown / next evidence needed

- Wheel platform, motor, motor driver, encoder, battery, regulator, charger, and
  disconnect details.
- Raspberry Pi OS, storage, and current software state.
- Robot-side microphone/speaker availability.
- Stable local Wyze stream, PTZ/IR control, position feedback, latency, and power
  behavior.
- ROS 2, CUDA Python, Foxglove, rosbag, and WSL-to-LAN discovery are planned but
  not yet installed or validated for this repo.
