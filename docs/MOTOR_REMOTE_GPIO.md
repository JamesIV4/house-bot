# Original Remote GPIO Motor Control

For the concise authoritative wiring table, use
`docs/REMOTE_WIRING_PINOUT.md`.

This is the zero-new-parts motor path: keep the original `GT004TX-V01` remote
as the proprietary 2.4 GHz transmitter and make the Raspberry Pi electrically
press its four drive buttons.

> **Live result:** the remote uses a scanned button matrix. The original
> one-wire `pulse`, `drive`, and `sequence` procedure below is retained as the
> experiment record but must not be used for control: holding one shared column
> low moved both motors. Continue at **Observed matrix behavior** for the active
> two-wire experiment.

The remote uses two AAA cells, so its button logic is in the Pi GPIO voltage
range. Keep the batteries in the remote. Do **not** connect the Pi 3.3 V rail to
the remote; the only power connection between them is common ground.

## First proof: identify one active-low button pad

Do this before soldering four signal wires.

1. Shut down the Pi and turn off the remote and motor receiver.
2. Connect Pi physical pin 6 (`GND`) to the remote PCB pad marked `V-`.
3. Connect Pi physical pin 7 (`BCM4`) to one side of the tactile switch for the
   first motor button. On a four-leg tactile switch, the two legs on each side
   are already common; use one leg from either side.
4. Keep the remote's two AAA cells installed. Lift the drive wheels clear of
   the table, power the Pi, receiver, and remote, then run:

   ```bash
   python3 ~/remote_gpio_controller.py sense --pin 4 --seconds 12
   ```

5. Physically press and release that same remote button several times.

A usable signal side reports `level=1` while released, `level=0` while the
button is held, and `level=1` again after release. If it stays at zero, move the
BCM4 wire to the switch's opposite side and repeat. If neither side produces a
clean `1 -> 0 -> 1`, stop: the remote likely scans a button matrix and direct
GPIO switching is not yet proven safe.

## First motor pulse

Only after the sense test shows a clean active-low transition, leave BCM4 on
that verified signal pad and run a 0.20-second pulse:

```bash
python3 ~/remote_gpio_controller.py pulse left-forward --duration 0.20
```

The script implements an open-drain-style switch:

- released: GPIO input, pull disabled, electrically high impedance;
- pressed: GPIO output low;
- never: GPIO output high.

This matches an active-low remote button without fighting the remote's own
pull-up. The pulse is released even if the script is interrupted or an error is
raised.

## Four-button wiring

After repeating the sense test for all four drive buttons, use this default
mapping:

| Function | BCM GPIO | Pi physical pin |
| --- | ---: | ---: |
| Left motor forward | 4 | 7 |
| Left motor reverse | 5 | 29 |
| Right motor forward | 6 | 31 |
| Right motor reverse | 7 | 26 |
| Common ground to remote `V-` | GND | 6 |

Pulse each wire separately while the wheels are lifted:

```bash
python3 ~/remote_gpio_controller.py sequence --duration 0.15 --pause 1.0
```

Then test paired differential-drive actions:

```bash
python3 ~/remote_gpio_controller.py drive forward --duration 0.20
python3 ~/remote_gpio_controller.py drive reverse --duration 0.20
python3 ~/remote_gpio_controller.py drive left --duration 0.20
python3 ~/remote_gpio_controller.py drive right --duration 0.20
```

Pin overrides appear before the subcommand, for example:

```bash
python3 ~/remote_gpio_controller.py --left-forward-pin 17 pulse left-forward
```

## Why this path

- Pi Wi-Fi and Bluetooth cannot emit the receiver's proprietary XN297-style
  packet waveform through their normal interfaces.
- The original remote already supplies the paired radio protocol.
- Pi GPIO cannot power the motors directly; the receiver remains the motor
  power stage and absorbs motor current and back-EMF.
- The button controller gives the future Pi base adapter four deterministic
  digital outputs. It can consume the PC's `/cmd_vel` commands over Wi-Fi after
  the physical mapping is verified.

## Observed matrix behavior

The first live BCM4 test changed high to low only in short scan windows while
the physical button was held. Pulling that shared column low for two seconds
activated both motors. Therefore the simple one-wire `pulse` command is not a
valid drive interface for this remote.

The next no-new-parts experiment uses two wires on one button:

| Role | Connection |
| --- | --- |
| Confirmed shared input column | Current button pad to BCM4, physical pin 7 |
| Target scan row | Opposite electrical side of that button to BCM5, physical pin 29 |

With the button released, profile the opposite pad:

```bash
python3 ~/remote_gpio_controller.py profile --pin 5 --seconds 3
```

If BCM5 exposes repeatable low scan windows, the gated command observes BCM5
without driving it and pulls BCM4 low only during those windows:

```bash
python3 ~/remote_gpio_controller.py matrix-pulse \
  --row-pin 5 --column-pin 4 --duration 0.20
```

### Verified live result

- BCM5 low window: 205.7-232.6 us, 215.8 us median.
- Falling-edge period: 40.13-40.18 ms, 40.15 ms median (~24.9 Hz).
- A two-second gated pulse mirrored 49 target-row windows.
- Only the left motor moved forward; the shared-column two-motor activation
  was eliminated.

Verified first action:

| Robot action | Scan row | Input column |
| --- | --- | --- |
| Left wheel forward | BCM5, physical pin 29 | BCM4, physical pin 7 |
| Left wheel reverse | BCM7, physical pin 26 | BCM6, physical pin 31 |
| Right wheel forward | BCM9, physical pin 21 | BCM8, physical pin 24 |
| Right wheel reverse | BCM11, physical pin 23 | BCM10, physical pin 19 |

The left-reverse row measured 220.9 us median low every 40.13 ms. A two-second
BCM7-to-BCM6 gated command mirrored 50 windows and moved only the left wheel
backward.

The right-forward row measured 219.4 us median low every 40.13 ms. A two-second
BCM9-to-BCM8 gated command mirrored 49 windows and moved only the right wheel
forward.

The right-reverse row measured 221.1 us median low every 40.11 ms. A two-second
BCM11-to-BCM10 gated command mirrored 50 windows and moved only the right wheel
backward.

All four verified pairs are built into the differential-drive command:

```bash
python3 ~/remote_gpio_controller.py matrix-drive forward --duration 0.20
python3 ~/remote_gpio_controller.py matrix-drive reverse --duration 0.20
python3 ~/remote_gpio_controller.py matrix-drive left --duration 0.20
python3 ~/remote_gpio_controller.py matrix-drive right --duration 0.20
```

This proves the Pi 3B can react within this remote's scan window. Repeat the
row/column identification for the remaining three differential-drive actions,
then use the same gate primitive in the live base adapter.

This is not yet the navigation interface. If later multi-button gating proves
unreliable, the remaining no-new-parts route is to open the receiver/battery
unit and connect the Pi to its motor-driver logic inputs. Pi GPIO must still
never connect directly to the motor leads.

The working solution will initially provide direction but not proportional
speed or wheel odometry. Navigation integration follows after four independent
drive directions are confirmed.
