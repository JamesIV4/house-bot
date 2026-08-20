# GT004 Remote Motor-Control Investigation

**Date:** 2026-08-19

## Observed hardware

- Remote PCB: `GT004TX-V01`, dated `2021/01/11`.
- Remote clock marking: `16M12`.
- Remote power: two AAA cells, nominally 3 V.
- Radio/control IC: unmarked 16-pin device next to a printed 2.4 GHz antenna.
- Receiver/battery hub: four motor ports marked A-D and USB-C charging.
- Remote PCB underside: blank; all components and traces are on the front.

## Disproved paths

- The hub did not appear in Linux or Windows Bluetooth discovery.
- Capturing Pi Bluetooth traffic while using the working remote produced no
  relevant controller traffic.
- A native Windows BLE advertisement was confirmed at the BLE layer, but the
  hub did not pair or move a motor.
- The Pi 3B's BCM43430 Wi-Fi/Bluetooth radio does not expose arbitrary
  XN297-compatible packets through normal Wi-Fi or Bluetooth APIs. The Nexmon
  software-defined-radio path does not support the Pi 3B's 802.11n PHY.

## Closest protocol evidence

The DIY Multiprotocol project's `MouldKg` implementation targets inexpensive
four-output Technic-style battery boxes and uses an nRF24L01/XN297-family
packet format. This is a strong hardware-family candidate but cannot be sent by
the current Pi or PC radios without an external compatible radio.

References:

- <https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Protocols_Details.md#mouldkg---90>
- <https://github.com/pascallanger/DIY-Multiprotocol-TX-Module/blob/master/Multiprotocol/MouldKg_nrf24l01.ino>
- <https://github.com/seemoo-lab/mobisys2018_nexmon_software_defined_radio>

## Selected no-new-parts experiment

Use the original remote as the already-paired radio and emulate active-low
button presses with Pi GPIO. The remote remains battery powered; Pi ground is
connected to remote `V-`. Each candidate button pad is first observed through
a high-impedance GPIO input. Only a pad that cleanly changes from high to low
when the physical button is pressed will be driven by the open-drain utility.

Implementation and exact wiring: `docs/MOTOR_REMOTE_GPIO.md` and
`scripts/remote_gpio_controller.py`.

## Pending evidence

- The BCM4 button pad produced brief `1 -> 0 -> 1` windows while the physical
  button was held.
- A two-second continuous BCM4-low pulse moved both motors, proving BCM4 is a
  shared matrix column rather than an independent active-low button input.
- The target button's opposite pad on BCM5 produced a 215.8 us median low
  window every 40.15 ms.
- Two seconds of BCM5-to-BCM4 low-window mirroring captured 49 scan windows and
  moved only the left wheel forward.
- The left-reverse row on BCM7 measured 220.9 us median low every 40.13 ms; a
  two-second BCM7-to-BCM6 gated command mirrored 50 windows and moved only the
  left wheel backward.
- The right-forward row on BCM9 measured 219.4 us median low every 40.13 ms; a
  two-second BCM9-to-BCM8 gated command mirrored 49 windows and moved only the
  right wheel forward.
- The right-reverse row on BCM11 measured 221.1 us median low every 40.11 ms; a
  two-second BCM11-to-BCM10 gated command mirrored 50 windows and moved only
  the right wheel backward.
- Paired forward, reverse, pivot-left, and pivot-right commands each mirrored
  49-50 windows per selected direction and moved the correct wheel pair.
- A persistent Pi user service now listens on UDP 8765, starts at boot, and
  releases all matrix columns after 350 ms without valid command refreshes.
- The stop-only WSL-to-Pi network test sent ten command packets and received an
  acknowledged final stop. Live motor motion through this service is pending.
- If the timing gate fails, inspect and inject the receiver's onboard
  motor-driver logic rather than connecting GPIO to the motors.
- Determine whether the receiver or motors provide any encoder feedback.
