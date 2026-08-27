# GT004 Remote-to-Raspberry-Pi Wiring Pinout

**Row/column map verified live:** 2026-08-27 by `identify-rows`  
**Tread directions verified live:** 2026-08-27  
**Pi:** Raspberry Pi 3B, 40-pin header  
**Remote PCB:** `GT004TX-V01`  
**Total Pi-to-remote wires:** 9

This is the authoritative pinout for the current motor-control wiring. The Pi
does not power the remote or the motors. The remote retains its two AAA cells,
and the original receiver/battery unit remains the motor power stage.

The remote's button wires were re-soldered and the Pi header assignment moved
on 2026-08-27, freeing BCM2/BCM3 for the MPU-6050 (`docs/IMU_WIRING.md`) and
releasing SPI0, the serial console, and both hardware PWM channels. The
pre-2026-08-27 mapping on BCM4-BCM11 is in git history.

## Electrical topology: four nets, eight wires

The GT004 is a **2x2 matrix**, and the harness wires every net to the Pi
*twice* -- once at each of the two buttons that touch it. Measured 2026-08-27
by driving each wire low and reading the others:

| Net | Wires | Pi pins | Role |
| --- | --- | --- | --- |
| A | blue, white | BCM17, BCM26 | column |
| B | brown, black | BCM18, BCM23 | row |
| C | yellow, orange | BCM22, BCM16 | column |
| D | red, purple | BCM19, BCM20 | row |

|  | column net A | column net C |
| --- | --- | --- |
| **row net B** | left-forward | left-reverse |
| **row net D** | right-reverse | right-forward |

**Only one pin per net may be used.** `MATRIX_PINS` therefore uses BCM18,
BCM19, BCM17 and BCM22, and leaves BCM23, BCM20, BCM26 and BCM16 unconfigured
as duplicates.

## Complete connection table

| Robot action / role | Wire | Pi physical pin | BCM GPIO | Net | Used |
| --- | --- | ---: | ---: | :-: | --- |
| Common ground | green | 14 | GND | - | PCB pad `V-` |
| Left forward column | blue | 11 | 17 | A | yes |
| Left forward row | brown | 12 | 18 | B | yes |
| Left reverse column | yellow | 15 | 22 | C | yes |
| Left reverse row | black | 16 | 23 | B | duplicate of BCM18 |
| Right forward row | red | 35 | 19 | D | yes |
| Right forward column | orange | 36 | 16 | C | duplicate of BCM22 |
| Right reverse column | white | 37 | 26 | A | duplicate of BCM17 |
| Right reverse row | purple | 38 | 20 | D | duplicate of BCM19 |

Header layout rules: the IMU owns the top-left corner (pins 1-9 odd); each
button's two wires land on a facing odd/even pin pair; left tread at the top of
the header, right tread at the bottom. I2C, the serial console, SPI0, and
PWM0/PWM1 on BCM12/BCM13 all stay free.

Row and column do **not** follow odd/even pin position. Role is an electrical
property of the pad a wire landed on and is measured, never inferred from
geometry.

## The duplicate-net fault

Treating the eight wires as eight independent pins is subtly wrong and cost a
long debugging session, because **every action works correctly in isolation**.
A single action only ever touches one pin per net, so all four verified
perfectly one at a time. Only commands that press two buttons at once break:
driving net C low through BCM22 while releasing it through BCM16 does not
release the net, so an intended pair actuates a third button.

Symptoms seen, all from this one cause:

- `forward` drove one tread and spun the base hard right instead of driving
  straight, intermittently;
- a single left-side press moved *both* treads on a tank-style base;
- pivots ran on one tread much of the time, which still rotates the base and so
  went unnoticed, showing up only as unexplained variance in pivot rate
  (84-150 dps) and coast (30.4-37.9 deg).

After the fix, forward drives straight, and pivot coast tightened to 28.1-29.3
deg.

`identify-rows` now detects this: after resolving rows and columns it drives
each wire low, groups the wires into nets, and rewrites the printed
`MATRIX_PINS` to one pin per net.

## Action-oriented mapping

| Robot action | Scan row | Input column | Measured row scan | Live tread result |
| --- | --- | --- | --- | --- |
| Left forward | BCM18 (net B) | BCM17 (net A) | 218.5 us every 40.16 ms | left tread forward |
| Left reverse | BCM18 (net B) | BCM22 (net C) | shared row with left-forward | left tread backward |
| Right forward | BCM19 (net D) | BCM22 (net C) | 428 us after net B | right tread forward |
| Right reverse | BCM19 (net D) | BCM17 (net A) | shared row with right-forward | right tread backward |

Rows B and D scan 428 us apart. The column is held low for 160-249 us with two
actions active, so a press never reaches the next row's window; that was
measured directly and ruled out as a fault mode.

## Header-only quick reference

```text
Pi pin 14  GND     -> remote V-              (green)
Pi pin 11  BCM17   -> net A column           (blue)     USED
Pi pin 12  BCM18   -> net B row              (brown)    USED
Pi pin 15  BCM22   -> net C column           (yellow)   USED
Pi pin 35  BCM19   -> net D row              (red)      USED
Pi pin 16  BCM23   -> net B row              (black)    duplicate
Pi pin 36  BCM16   -> net C column           (orange)   duplicate
Pi pin 37  BCM26   -> net A column           (white)    duplicate
Pi pin 38  BCM20   -> net D row              (purple)   duplicate
```

## Re-identifying after a rewire

```bash
# Nothing is driven; both wires of each pair are read as high-impedance inputs.
python3 ~/remote_gpio_controller.py identify-rows --seconds 2
```

Each pin is profiled for 2 s. A row shows a roughly 215 us low window every
40 ms; a column sits steady. The command prints a ready-to-paste `MATRIX_PINS`
and names any pair it could not resolve, rather than guessing. A pair where
neither wire scans means dead remote cells or a broken joint; a pair where both
scan means the two wires are on the same electrical edge of the switch.

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

## Tread-direction verification

Each button was pressed on its own for 0.8 s through `matrix-pulse`, below the
motor service so no software inversion sat in the path, and the physical result
was observed before the next was run. All four actions moved the tread their
name claims, in the direction their name claims.

The motor service therefore runs with **no inversion flags**. The
`--invert-left` it carried until 2026-08-27 was calibrated against the
pre-rebuild drivetrain and is now wrong; it was removed from
`deploy/house-bot-motors.service` when these results were recorded.
