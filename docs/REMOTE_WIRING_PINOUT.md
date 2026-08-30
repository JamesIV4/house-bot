# GT004 Remote-to-Raspberry-Pi Wiring Pinout

**Matrix model corrected:** 2026-08-30 — the remote is 5x2, not 2x2
**Row/column map verified live:** 2026-08-30 by `identify-rows`
**Tread directions verified live:** 2026-08-30 by `matrix-pulse`, on blocks
**Pi:** Raspberry Pi 3B, 40-pin header
**Remote PCB:** `GT004TX-V01` — Mould King 4.0 family
**Total Pi-to-remote wires:** 9

This is the authoritative pinout for the current motor-control wiring. The Pi
does not power the remote or the motors. The remote retains its two AAA cells,
and the original receiver/battery unit remains the motor power stage.

## The remote is a 5x2 matrix, not a 2x2

This file described a 2x2 matrix from 2026-08-27 until 2026-08-30. That was
wrong, and the error cost roughly a week. The four wired buttons really are a
clean 2x2 **sub-block**, which is why every measurement of them agreed — but
they are a sub-block of a ten-button keypad.

The board carries **eight directional buttons in two four-way pads, plus two
function keys, E and F**. The manufacturer's manual defines them:

```
A button : A channel powers motor forward and reverse
B button : B channel powers motor forward and reverse
C button : C channel powers motor forward and reverse
D button : D channel powers motor forward and reverse
E button : Merges A and B channel
F button : Merges A and C channel
E+F buttons : Merges A, B, and C channels
```

Its detail page settles the direction of a merge: *"merging channels A and C.
**Press the A button**, and all PFs connected to ports A and C [run]"* —
**channel A is always the master.**

|          | direction 1        | direction 2         | port |
| ---      | ---                | ---                 | ---  |
| **chan A** | left pad up      | left pad down       | **left tread** |
| **chan B** | left pad left    | left pad right      | empty |
| **chan C** | right pad up     | right pad down      | empty |
| **chan D** | right pad left   | right pad right     | **right tread** |
| **func**   | E                | F                   | — |

A **scan line** is private to one channel and driven by the remote's own MCU. A
**direction line** is read by the MCU and is **shared by every button on the
keypad, E and F included.** That distinction is the whole story below.

## Why the remote kept latching, and why it no longer can

The Pi cannot bridge a button's two terminals — a GPIO can only drive a level,
not connect two pins. So the gating instead grounds a *direction* line during
the target channel's scan window. That line is shared. It is low for every
button on it, and only release timing keeps the press looking like one button.

Hold it a few hundred microseconds too long and the scan reaches another row.
When the treads were on channels **A and C**, one of those rows was the
function row: the Pi ghost-pressed **F**, which merges A and C — producing
exactly the reported fault, *"left buttons drive both treads, right buttons
drive only the right tread."* Not corruption. F, working as designed, latched
until the transmitter was power-cycled.

No human can do this: a real button bridges one row to one column and is
electrically inert during every other channel's turn.

**The fix was to move off the channels F can reach.** No merge function touches
channel D, and channel A only ever appears as a *master* — merging it into an
empty port changes nothing. So with the treads on **A and D**:

| Latched state | Effect |
| --- | --- |
| F — merges A + C | A drives the left tread and empty port C. No change. |
| E — merges A + B | A drives the left tread and empty port B. No change. |
| E + F | A drives the left tread and two empty ports. No change. |
| channel D | never merged by anything; always independent. |

The Pi can still ghost-press F. It has nowhere to land. See
`docs/experiments/2026-08-27-remote-remap-and-imu.md` for the investigation.

## Electrical topology: four nets, eight wires

The harness wires every net to the Pi **twice**, once at each of the two
buttons that touch it. Measured 2026-08-30 by `identify-rows`:

| Net | Wires | Pi pins | Role |
| --- | --- | --- | --- |
| direction 1 | blue, red | BCM17, BCM19 | steady; read by the remote |
| direction 2 | purple, yellow | BCM20, BCM22 | steady; read by the remote |
| chan A scan | brown, black | BCM18, BCM23 | ~225 us low every 40.16 ms |
| chan D scan | orange, white | BCM16, BCM26 | ~225 us low every 40.16 ms |

**Only one pin per net may be used.** `MATRIX_PINS` therefore uses BCM18,
BCM16, BCM17 and BCM20, leaving BCM23, BCM26, BCM19 and BCM22 unconfigured as
duplicates. Driving one pin of a net low and releasing the other does not
release the net, so one intended press becomes two buttons.

## Complete connection table

| Robot action / role | Wire | Pi physical pin | BCM GPIO | Net | Used |
| --- | --- | ---: | ---: | :-: | --- |
| Common ground | green | 14 | GND | - | PCB pad `V-` |
| chan A, direction 1 | blue | 11 | 17 | dir 1 | yes |
| chan A scan | brown | 12 | 18 | A scan | yes |
| chan A, direction 2 | yellow | 15 | 22 | dir 2 | duplicate of BCM20 |
| chan A scan | black | 16 | 23 | A scan | duplicate of BCM18 |
| chan D, direction 1 | red | 35 | 19 | dir 1 | duplicate of BCM17 |
| chan D scan | orange | 36 | 16 | D scan | yes |
| chan D scan | white | 37 | 26 | D scan | duplicate of BCM16 |
| chan D, direction 2 | purple | 38 | 20 | dir 2 | yes |

Blue and brown land on the left pad's **up** button; yellow and black on the
left pad's **down** button; red and orange on the right pad's **left** button;
white and purple on the right pad's **right** button.

Row and column do **not** follow odd/even pin position. Role is an electrical
property of the pad a wire landed on and is measured, never inferred from
geometry — when the right side moved from channel C to channel D on
2026-08-30, keeping the same relative leg positions, **the roles swapped
sides**: red and purple went from scan lines to direction lines, and orange and
white from direction lines to scan lines.

## Action-oriented mapping

Every intersection was pressed on its own for 0.4 s through `matrix-pulse`,
base on blocks, with the physical result reported before the next was sent:

| Robot action | Scan line | Direction line | Live tread result |
| --- | --- | --- | --- |
| Left forward | BCM18 (chan A) | BCM17 (dir 1) | left tread forward |
| Left reverse | BCM18 (chan A) | BCM20 (dir 2) | left tread backward |
| Right forward | BCM16 (chan D) | BCM20 (dir 2) | right tread forward |
| Right reverse | BCM16 (chan D) | BCM17 (dir 1) | right tread backward |

A clean bijection: every action moves the tread its name claims, in the
direction its name claims. **The service runs with no inversion flags.**

`identify-rows` resolves the electrical roles but cannot know which way a tread
turns. Channel D inverted the right side relative to channel C, so the two
right-hand labels had to be swapped after the on-blocks test. Always re-measure
directions after a rewire; never inherit them.

## Measured scan timing

Measured passively on the Pi, 2026-08-30, both scan lines as high-impedance
inputs:

| Interval | Median |
| --- | ---: |
| chan A low window | 225.5 us |
| chan D low window | 225.1 us |
| chan A rises -> chan D falls | 419.1 us |
| chan D rises -> chan A falls | 39.30 ms |
| frame period | 40.16 ms |

Channel D sits **two scan slots** from channel A; channel C sat one, giving
216 us. The release deadline for a consequential ghost therefore roughly
doubled when the right tread moved.

> **The 39.30 ms is not slack.** It is the distance to the next scan line *the
> harness can see*. Three unmonitored rows — channels B and C and the function
> row — are scanned somewhere inside it. No action is safe by geometry.

## Measured GPIO gating performance

Measured on the Pi 2026-08-29 with the production call sequence, on spare pins
BCM12/13 under live service load, 40,000 cycles and 583,000 loop passes:

| Operation | median | p99.9 | max |
| --- | ---: | ---: | ---: |
| `setup(OUT)` — assert | 46.9 us | 143.5 us | 243.5 us |
| `setup(IN)` — release | 50.9 us | 146.4 us | 225.9 us |
| loop blind time between passes | 33.6 us | 59.9 us | 628.8 us |

The service *was* `SCHED_OTHER` at nice 0, taking **20,574 involuntary
preemptions in 44 minutes** -- the loop's own blind time, not the release cost,
was the dominant term against the 419 us budget.

**Fixed 2026-08-30: the service now runs `SCHED_FIFO` priority 10**, and takes
about **2 involuntary preemptions per restart** instead. It requests this
itself at startup and logs a warning if refused.

The permission cannot come from the unit file: a *user* unit cannot set
`AmbientCapabilities` (systemd exits `218/CAPABILITIES`) and the user's rtprio
hard limit is 0, so `LimitRTPRIO=` cannot raise it either. It comes from a
root-owned file instead:

```bash
echo 'james - rtprio 20' | sudo tee /etc/security/limits.d/99-housebot-rtprio.conf
sudo systemctl restart user@1000
```

Confirm with `ps -o cls,rtprio -p $(systemctl --user show house-bot-motors.service -p MainPID --value)`;
healthy is `FF 10`, not `TS -`.

> **Method note.** An earlier measurement — "0 ghost presses in 49 windows" —
> was used to close this question. At roughly 1 late release in 10^4 windows,
> 49 windows has an expected count of 0.005: seeing zero was guaranteed
> whether or not the mechanism was real. **95% power needs about 30,000
> windows, or 20 minutes of continuous driving.** Do not accept a two-second
> null result about a rare event.

`RPi.GPIO` on this Pi is the `rpi-lgpio` shim, so `setup()` is a kernel line
free-and-reclaim rather than a register write, and lines are claimed
exclusively — the motor service must be stopped before any manual GPIO work.
> **The open-drain optimisation does not work on this kernel** (6.18.34,
> lgpio 0.2.2), contrary to the note carried in this file since 2026-08-27.
> Tested 2026-08-30: `gpio_claim_output` with an initial level of **1 is
> rejected outright** (`xGpioHandleRequest: Invalid argument`); with level 0 it
> claims, and `gpio_write` is genuinely fast at 6.1 us, but **writing 1 never
> releases the line to high impedance** -- verified electrically by reading the
> duplicate wire on the same net, which stayed low through release/press/release.
> Worse, `gpio_free` does not restore the pin: it was left driving low. Do not
> retry this without a scope. The release cost stays at `setup()`'s 51 us
> median / 226 us max.

## Re-identifying after a rewire

```bash
# Stop the service first: lgpio claims lines exclusively.
systemctl --user stop house-bot-motors.service

# Nothing is driven except brief net-grouping pulses, so keep the
# receiver/motor unit POWERED OFF. The remote must be ON and awake.
python3 ~/remote_gpio_controller.py identify-rows --seconds 2
```

Each pin is profiled for 2 s. A scan line shows a roughly 225 us low window
every 40 ms; a direction line sits steady. The command prints a ready-to-paste
`MATRIX_PINS` and names any pair it could not resolve rather than guessing. A
pair where neither wire scans means dead remote cells, a broken joint, or a
sleeping remote; a pair where both scan means the two wires are on the same
electrical edge of the switch, and that button can never be pressed.

Then verify directions one at a time, base on blocks:

```bash
python3 ~/remote_gpio_controller.py matrix-pulse --row-pin 18 --column-pin 17 --duration 0.4
```

## Tactile-switch connection rule

Each button has four legs but only two electrical sides. The two legs on one
edge are already connected internally; pressing the button connects that edge
to the opposite edge. Each scan/direction pair must therefore land on opposite
electrical edges, not two legs on the same edge.

## Power rules

- Keep the remote's two AAA batteries installed.
- **The remote sleeps after roughly 2-5 minutes idle.** Its MCU stops scanning
  entirely and both scan lines go flat, so gating actuates nothing until it
  wakes. Confirm the lines are alive before trusting any command, and treat a
  first command after a pause as possibly lost. This is the remote, not the
  receiver.
- Do **not** connect Pi pin 1, Pi 3.3 V, or Pi 5 V to the remote.
- Connect only Pi ground to remote `V-`.
- Do not connect any Pi GPIO directly to a motor lead.
- Change soldered connections only while the Pi, remote, and receiver are off.

## Electrical behavior

The remote uses a scanned button matrix. Each direction line is shared by
**five** buttons in five different rows, so holding one continuously low
actuates all five; the software instead:

1. observes the target channel's scan line as a high-impedance input;
2. waits for that line's roughly 225 us low window;
3. pulls only the paired direction line low during that window;
4. returns it to a pull-disabled input immediately afterward.

No control path ever drives a remote wire high. Within a single poll pass,
**every column that must go high is released before any column is asserted** —
applying them in pin order once let both direction lines sit low inside one
scan window, which the remote reads as both directions of a channel pressed
at once.

## Receiver

Four motor ports, A-D, on the rechargeable battery unit.

| Port | Contents |
| --- | --- |
| A | **left tread** |
| B | empty |
| C | empty — vacated 2026-08-30 |
| D | **right tread** |

Recorded 2026-08-29. This file listed the port letters as "still unrecorded"
from the day it was written until then; they turned out to be the fact the
whole diagnosis rested on.

## Software locations

- Verified mapping: `MATRIX_PINS` in `scripts/remote_gpio_controller.py`.
- Persistent Pi service: `scripts/pi_motor_service.py`.
- PC/WSL command client: `scripts/send_motor_command.py`.
- Route driver: `scripts/drive_route.py` — drive multi-leg routes with this,
  one process and one continuous stream, never a loop of one-shot commands.
- Pi user service: `deploy/house-bot-motors.service`.
- Investigation and raw measurements:
  `docs/experiments/2026-08-19-gt004-remote-control.md` and
  `docs/experiments/2026-08-27-remote-remap-and-imu.md`.

The installed Pi service listens on UDP port 8765 and releases all columns if
valid command refreshes stop for 350 ms.

## Still unrecorded

- Which of the two direction lines carries **F** and which carries **E**. A
  continuity check with the remote switched off would settle it. It no longer
  affects correctness now that the treads are on channels A and D, but it
  would name which pivot direction was historically the dangerous one.
- The scan-order position of the function row within the 40.16 ms frame.
