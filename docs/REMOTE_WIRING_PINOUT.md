# GT004 Remote-to-Raspberry-Pi Wiring Pinout

**Verified live:** 2026-08-19  
**Pi:** Raspberry Pi 3B, 40-pin header  
**Remote PCB:** `GT004TX-V01`  
**Total Pi-to-remote wires:** 9

This is the authoritative pinout for the current motor-control wiring. The Pi
does not power the remote or the motors. The remote retains its two AAA cells,
and the original receiver/battery unit remains the motor power stage.

## Complete connection table

| Robot action / role | Pi physical pin | BCM GPIO | Remote connection | Electrical role |
| --- | ---: | ---: | --- | --- |
| Common ground | 6 | GND | PCB pad marked `V-` | Voltage reference only |
| Left wheel forward column | 7 | 4 | One electrical edge of left-forward button | Input column, pulled low only while its row is low |
| Left wheel forward row | 29 | 5 | Opposite electrical edge of left-forward button | Input-only scan-row sensor |
| Left wheel reverse column | 31 | 6 | One electrical edge of left-reverse button | Input column, pulled low only while its row is low |
| Left wheel reverse row | 26 | 7 | Opposite electrical edge of left-reverse button | Input-only scan-row sensor |
| Right wheel forward column | 24 | 8 | One electrical edge of right-forward button | Input column, pulled low only while its row is low |
| Right wheel forward row | 21 | 9 | Opposite electrical edge of right-forward button | Input-only scan-row sensor |
| Right wheel reverse column | 19 | 10 | One electrical edge of right-reverse button | Input column, pulled low only while its row is low |
| Right wheel reverse row | 23 | 11 | Opposite electrical edge of right-reverse button | Input-only scan-row sensor |

## Action-oriented mapping

| Robot action | Scan row | Input column | Live result |
| --- | --- | --- | --- |
| Left wheel forward | BCM5 / pin 29 | BCM4 / pin 7 | Correct wheel forward |
| Left wheel reverse | BCM7 / pin 26 | BCM6 / pin 31 | Correct wheel backward |
| Right wheel forward | BCM9 / pin 21 | BCM8 / pin 24 | Correct wheel forward |
| Right wheel reverse | BCM11 / pin 23 | BCM10 / pin 19 | Correct wheel backward |

Paired commands were also verified:

- forward: left forward + right forward;
- reverse: left reverse + right reverse;
- pivot left: left reverse + right forward;
- pivot right: left forward + right reverse.

## Header-only quick reference

```text
Pi pin  6  GND     -> remote V-
Pi pin  7  BCM4    -> left-forward COLUMN
Pi pin 29  BCM5    -> left-forward ROW
Pi pin 31  BCM6    -> left-reverse COLUMN
Pi pin 26  BCM7    -> left-reverse ROW
Pi pin 24  BCM8    -> right-forward COLUMN
Pi pin 21  BCM9    -> right-forward ROW
Pi pin 19  BCM10   -> right-reverse COLUMN
Pi pin 23  BCM11   -> right-reverse ROW
```

## Tactile-switch connection rule

Each button has four legs but only two electrical sides. The two legs on one
edge are already connected internally; pressing the button connects that edge
to the opposite edge. Each row/column pair above must therefore land on
opposite electrical edges, not two legs on the same edge.

The geometric edge chosen for row versus column depends on the soldering
orientation. The table records the electrically verified role of each Pi wire.

## Power rules

- Keep the remote's two AAA batteries installed.
- Do **not** connect Pi pin 1, Pi 3.3 V, or Pi 5 V to the remote.
- Connect only Pi ground to remote `V-`.
- Do not connect any Pi GPIO directly to a motor lead.
- Change soldered connections only while the Pi, remote, and receiver are off.

## Electrical behavior

The remote uses a scanned button matrix. Holding a shared column continuously
low moved both motors, so the software instead:

1. observes the selected row as a high-impedance input;
2. waits for that row's roughly 205-233 us low scan window;
3. pulls only the paired column low during that window;
4. returns the column to a pull-disabled input immediately afterward.

No control path ever drives a remote wire high.

| Action | Median row-low width | Median scan period |
| --- | ---: | ---: |
| Left forward | 215.8 us | 40.15 ms |
| Left reverse | 220.9 us | 40.13 ms |
| Right forward | 219.4 us | 40.13 ms |
| Right reverse | 221.1 us | 40.11 ms |

## Software locations

- Verified mapping: `MATRIX_PINS` in `scripts/remote_gpio_controller.py`.
- Persistent Pi service: `scripts/pi_motor_service.py`.
- PC/WSL command client: `scripts/send_motor_command.py`.
- Pi user service: `deploy/house-bot-motors.service`.
- Investigation and raw measurements:
  `docs/experiments/2026-08-19-gt004-remote-control.md`.

The installed Pi service listens on UDP port 8765 and releases all columns if
valid command refreshes stop for 350 ms.

## Still unrecorded

The physical receiver port letters used by the left and right motors have not
yet been identified in the repository. Record those port letters here once
observed; they do not change the verified Pi-to-remote pinout above.

## Proposed remap (2026-08-27) — NOT YET VERIFIED

The remote's button wires were re-soldered on 2026-08-27 and the Pi header
assignment is being changed at the same time, to free BCM2/BCM3 for the
MPU-6050 (`docs/IMU_WIRING.md`) and to stop consuming SPI0 and both hardware
PWM channels for nothing.

**The table above remains authoritative until `identify-rows` has run.**
`MATRIX_PINS` in `scripts/remote_gpio_controller.py` is unchanged for the same
reason.

Design rules: the IMU owns the top-left corner (pins 1-9 odd); each button's
two wires land on a facing odd/even pin pair; left tread at the top of the
header, right tread at the bottom; I2C, the serial console, SPI0, and PWM0/PWM1
on BCM12/BCM13 all stay free.

| Robot action | Wire | Pi physical pin | BCM |
| --- | --- | ---: | ---: |
| Common ground | green | 14 | GND |
| Left forward | blue | 11 | 17 |
| Left forward | brown | 12 | 18 |
| Left reverse | yellow | 15 | 22 |
| Left reverse | black | 16 | 23 |
| Right forward | red | 35 | 19 |
| Right forward | orange | 36 | 16 |
| Right reverse | white | 37 | 26 |
| Right reverse | purple | 38 | 20 |

Which wire of each pair is the scan **row** and which is the **column** is an
electrical property of the pad the wire landed on, not of its position on the
switch, and the left and right button clusters are mirrored on this PCB. It is
therefore measured, not assumed:

```bash
# Receiver/motor unit POWERED OFF; remote AAA cells in. Nothing is driven.
python3 ~/remote_gpio_controller.py identify-rows --seconds 2
```

Each pin is profiled for 2 s. A row shows a roughly 215 us low window every
40 ms; a column sits steady. The command prints a ready-to-paste `MATRIX_PINS`
and names any pair it could not resolve. A pair where neither wire scans means
dead remote cells or two wires on the same electrical edge; a pair where both
scan means two wires on the same edge.

After it resolves cleanly, paste the printed map into `MATRIX_PINS`, update the
authoritative table above, update `test_remote_gpio_controller.py`, redeploy,
and re-verify one direction at a time before any paired motion.
