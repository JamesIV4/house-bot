#!/usr/bin/env python3
"""Probe and control an active-low RC remote with Raspberry Pi GPIO.

GPIO outputs are used as open-drain switches: released is high impedance and
pressed is output-low. The script never drives a remote button input high.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from collections.abc import Callable, Iterable, Sequence
from typing import Any


BUTTON_NAMES = (
    "left-forward",
    "left-reverse",
    "right-forward",
    "right-reverse",
)

DEFAULT_PINS = {
    "left-forward": 4,
    "left-reverse": 5,
    "right-forward": 6,
    "right-reverse": 7,
}

MOTIONS = {
    "forward": ("left-forward", "right-forward"),
    "reverse": ("left-reverse", "right-reverse"),
    "left": ("left-reverse", "right-forward"),
    "right": ("left-forward", "right-reverse"),
}

# The GT004 is a 2x2 matrix and the harness wires every net to the Pi TWICE,
# once at each button that touches it. Measured 2026-08-27 by driving each wire
# low and reading the others:
#
#   net A = BCM17 (blue)   == BCM26 (white)    column
#   net B = BCM18 (brown)  == BCM23 (black)    row
#   net C = BCM22 (yellow) == BCM16 (orange)   column
#   net D = BCM19 (red)    == BCM20 (purple)   row
#
#              col net A        col net C
#   row net B  left-forward     left-reverse
#   row net D  right-reverse    right-forward
#
# Only one pin per net may be used. Treating the eight wires as eight
# independent pins meant driving and releasing the same net through two
# different pins: releasing BCM16 does nothing while BCM22 still holds net C
# low, so one intended press became two buttons. Single actions were unaffected
# because they only ever touch one pin per net, which is why every action
# verified correctly on its own and only pairs misbehaved.
#
# BCM23, BCM20, BCM16 and BCM26 are deliberately unused: they are duplicates of
# BCM18, BCM19, BCM22 and BCM17 respectively.
MATRIX_PINS = {
    "left-forward": (18, 17),
    "left-reverse": (18, 22),
    "right-forward": (19, 22),
    "right-reverse": (19, 17),
}

# Both wires of each net, for documentation and for identify-rows.
MATRIX_NETS = {
    "A-column": (17, 26),
    "B-row": (18, 23),
    "C-column": (22, 16),
    "D-row": (19, 20),
}

MATRIX_MOTIONS = {
    "forward": ("left-forward", "right-forward"),
    "reverse": ("left-reverse", "right-reverse"),
    "left": ("left-reverse", "right-forward"),
    "right": ("left-forward", "right-reverse"),
}

# Both GPIO soldered to one button, in no particular order. Which of the two is
# the scan row is an electrical property of the pad the wire landed on, not of
# its position on the switch, so it is measured rather than assumed. Feed this
# to `identify-rows` after any rewire and paste the result into MATRIX_PINS.
CANDIDATE_PAIRS = {
    "left-forward": (17, 18),
    "left-reverse": (22, 23),
    "right-forward": (19, 16),
    "right-reverse": (26, 20),
}

# A row is driven low for roughly 215 us every 40 ms; see
# docs/experiments/2026-08-19-gt004-remote-control.md. A column sits at a
# steady level until a button bridges it to a row.
ROW_LOW_WIDTH_US = (100.0, 500.0)
ROW_PERIOD_US = (30_000.0, 55_000.0)
NOMINAL_SCAN_PERIOD_S = 0.040


def bounded_duration(value: str) -> float:
    duration = float(value)
    if not 0.02 <= duration <= 2.0:
        raise argparse.ArgumentTypeError("duration must be between 0.02 and 2.0 seconds")
    return duration


def positive_duration(value: str) -> float:
    duration = float(value)
    if duration <= 0:
        raise argparse.ArgumentTypeError("duration must be greater than zero")
    return duration


def gpio_pin(value: str) -> int:
    pin = int(value)
    if not 0 <= pin <= 27:
        raise argparse.ArgumentTypeError("BCM GPIO must be between 0 and 27")
    return pin


def validate_pin_map(pins: dict[str, int]) -> None:
    values = list(pins.values())
    if len(set(values)) != len(values):
        raise ValueError("each button must use a different BCM GPIO")


class RpiOpenDrainButtons:
    """Turn GPIO pins into active-low, otherwise high-impedance switches."""

    def __init__(self, pins: dict[str, int], gpio_module: Any | None = None) -> None:
        validate_pin_map(pins)
        if gpio_module is None:
            try:
                import RPi.GPIO as gpio_module  # type: ignore[import-not-found]
            except ImportError as exc:
                raise RuntimeError(
                    "RPi.GPIO is unavailable; run this on the Raspberry Pi or use --dry-run"
                ) from exc

        self._gpio = gpio_module
        self.pins = dict(pins)
        self._gpio.setwarnings(False)
        self._gpio.setmode(self._gpio.BCM)
        self.release_all()

    def release(self, names: Iterable[str]) -> None:
        for name in names:
            self._gpio.setup(
                self.pins[name],
                self._gpio.IN,
                pull_up_down=self._gpio.PUD_OFF,
            )

    def release_all(self) -> None:
        self.release(BUTTON_NAMES)

    def press(self, names: Sequence[str]) -> None:
        self.release_all()
        for name in names:
            # initial=LOW is deliberate: no connected button wire is ever
            # configured as a driven-high output.
            self._gpio.setup(
                self.pins[name],
                self._gpio.OUT,
                initial=self._gpio.LOW,
            )

    def close(self) -> None:
        self.release_all()
        self._gpio.cleanup(list(self.pins.values()))


class DryRunButtons:
    def press(self, names: Sequence[str]) -> None:
        print(f"DRY RUN press: {', '.join(names)}")

    def release_all(self) -> None:
        print("DRY RUN release: all buttons")

    def close(self) -> None:
        self.release_all()


def pulse_buttons(
    backend: RpiOpenDrainButtons | DryRunButtons,
    names: Sequence[str],
    duration: float,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    backend.press(names)
    try:
        sleep_fn(duration)
    finally:
        backend.release_all()


def sense_pin(pin: int, seconds: float, interval: float = 0.01) -> int:
    """Print GPIO level changes while the user physically presses a button."""
    try:
        import RPi.GPIO as gpio  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("RPi.GPIO is unavailable; sense must run on the Pi") from exc

    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
    deadline = time.monotonic() + seconds
    previous: int | None = None
    transitions = 0

    print(
        f"Sensing BCM{pin} for {seconds:.1f}s. Physically press and release "
        "the target remote button several times."
    )
    try:
        while time.monotonic() < deadline:
            level = int(gpio.input(pin))
            if level != previous:
                print(f"{time.monotonic():.3f} level={level}", flush=True)
                if previous is not None:
                    transitions += 1
                previous = level
            time.sleep(interval)
    finally:
        gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
        gpio.cleanup(pin)

    return transitions


def profile_scan_pin(pin: int, seconds: float) -> tuple[list[float], list[float]]:
    """Busy-poll a matrix row and return low widths and falling-edge periods in us."""
    try:
        import RPi.GPIO as gpio  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("RPi.GPIO is unavailable; profile must run on the Pi") from exc

    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
    deadline = time.perf_counter_ns() + int(seconds * 1_000_000_000)
    previous = int(gpio.input(pin))
    falling_ns: int | None = None
    last_falling_ns: int | None = None
    low_widths_us: list[float] = []
    periods_us: list[float] = []

    try:
        while time.perf_counter_ns() < deadline:
            level = int(gpio.input(pin))
            if level == previous:
                continue
            now_ns = time.perf_counter_ns()
            if level == 0:
                falling_ns = now_ns
                if last_falling_ns is not None:
                    periods_us.append((now_ns - last_falling_ns) / 1_000.0)
                last_falling_ns = now_ns
            elif falling_ns is not None:
                low_widths_us.append((now_ns - falling_ns) / 1_000.0)
                falling_ns = None
            previous = level
    finally:
        gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
        gpio.cleanup(pin)

    return low_widths_us, periods_us


def format_timing(name: str, values: Sequence[float]) -> str:
    if not values:
        return f"{name}: none observed"
    return (
        f"{name}: count={len(values)} min={min(values):.1f}us "
        f"median={statistics.median(values):.1f}us max={max(values):.1f}us"
    )


def classify_scan_pin(
    low_widths_us: Sequence[float],
    periods_us: Sequence[float],
    seconds: float,
) -> tuple[str, str]:
    """Decide whether a profiled pin is a scan row, a column, or unreadable.

    Kept free of GPIO so the decision rule is unit-tested rather than only
    observed on the bench.
    """
    if not periods_us:
        return "column", "no periodic scan windows; steady level"
    expected = seconds / NOMINAL_SCAN_PERIOD_S
    observed = len(periods_us) + 1
    median_width = statistics.median(low_widths_us) if low_widths_us else 0.0
    median_period = statistics.median(periods_us)
    detail = (
        f"{observed} windows in {seconds:.1f}s, "
        f"median width {median_width:.1f}us, median period {median_period / 1000.0:.2f}ms"
    )
    if observed < expected * 0.5:
        return "column", f"only intermittent activity: {detail}"
    if not ROW_LOW_WIDTH_US[0] <= median_width <= ROW_LOW_WIDTH_US[1]:
        return "unclear", f"low width outside the scan range: {detail}"
    if not ROW_PERIOD_US[0] <= median_period <= ROW_PERIOD_US[1]:
        return "unclear", f"period outside the scan range: {detail}"
    return "row", detail


def identify_matrix_rows(
    pairs: dict[str, tuple[int, int]],
    seconds: float,
    profile_fn: Callable[[int, float], tuple[list[float], list[float]]] = profile_scan_pin,
) -> tuple[dict[str, tuple[int, int]], list[str]]:
    """Profile both GPIO of every pair and order each as (row, column)."""
    resolved: dict[str, tuple[int, int]] = {}
    report: list[str] = []
    for name, (first, second) in pairs.items():
        verdicts: dict[int, tuple[str, str]] = {}
        for pin in (first, second):
            low_widths_us, periods_us = profile_fn(pin, seconds)
            verdict, detail = classify_scan_pin(low_widths_us, periods_us, seconds)
            verdicts[pin] = (verdict, detail)
            report.append(f"{name}: BCM{pin} -> {verdict} ({detail})")
        rows = [pin for pin, (verdict, _) in verdicts.items() if verdict == "row"]
        if len(rows) == 1:
            row_pin = rows[0]
            column_pin = second if row_pin == first else first
            resolved[name] = (row_pin, column_pin)
        elif not rows:
            report.append(
                f"{name}: UNRESOLVED, neither BCM{first} nor BCM{second} is scanning. "
                "Check the remote's AAA cells and that both wires reach opposite "
                "electrical edges of the same switch."
            )
        else:
            report.append(
                f"{name}: UNRESOLVED, BCM{first} and BCM{second} both scan. "
                "The two wires are probably on the same electrical edge."
            )
    return resolved, report


def drive_low_and_read(pin: int, others: Sequence[int], samples: int = 40) -> list[int]:
    """Hold one wire low and report which others follow it."""
    try:
        import RPi.GPIO as gpio  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("RPi.GPIO is unavailable; net detection must run on the Pi") from exc

    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    for other in others:
        gpio.setup(other, gpio.IN, pull_up_down=gpio.PUD_OFF)
    followers: list[int] = []
    try:
        gpio.setup(pin, gpio.OUT, initial=gpio.LOW)
        time.sleep(0.004)
        for other in others:
            # Rows are scanned, so a shared net reads low almost always while a
            # merely scanning wire dips low only briefly once per period.
            lows = sum(1 for _ in range(samples) if int(gpio.input(other)) == 0)
            if lows >= samples - 2:
                followers.append(other)
    finally:
        gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
        time.sleep(0.004)
    return followers


def detect_shared_nets(
    pins: Sequence[int],
    drive_low_fn: Callable[[int, Sequence[int]], list[int]],
) -> dict[int, list[int]]:
    """Group the harness wires by electrical net.

    Each wire is driven low in turn and the others are read; any that follow
    are on the same net. Only ever drives low, never high, so it obeys the same
    rule as every other path here.

    This matters because the GT004 harness wires each net to the Pi twice, once
    at each button touching it. `identify-rows` alone cannot see that: it
    classifies each wire correctly and every action then works in isolation,
    while any command pressing two buttons at once drives and releases the same
    net through two different pins and actuates a button nobody asked for.
    """
    nets: dict[int, list[int]] = {}
    for pin in pins:
        others = [other for other in pins if other != pin]
        followers = drive_low_fn(pin, others)
        nets[pin] = sorted(followers)
    return nets


def net_groups(nets: dict[int, list[int]]) -> list[tuple[int, ...]]:
    """Collapse the follower map into one sorted tuple per distinct net."""
    groups: list[set[int]] = []
    for pin, followers in nets.items():
        members = {pin, *followers}
        for group in groups:
            if group & members:
                group |= members
                break
        else:
            groups.append(members)
    return sorted(tuple(sorted(group)) for group in groups)


def matrix_pins_from_nets(
    resolved: dict[str, tuple[int, int]],
    groups: Sequence[tuple[int, ...]],
) -> dict[str, tuple[int, int]]:
    """Rewrite a resolved map so each net is referenced by exactly one pin."""
    canonical: dict[int, int] = {}
    for group in groups:
        for pin in group:
            canonical[pin] = group[0]
    return {
        name: (canonical.get(row, row), canonical.get(column, column))
        for name, (row, column) in resolved.items()
    }


def format_matrix_pins(resolved: dict[str, tuple[int, int]]) -> str:
    lines = ["MATRIX_PINS = {"]
    for name in BUTTON_NAMES:
        if name in resolved:
            row_pin, column_pin = resolved[name]
            lines.append(f'    "{name}": ({row_pin}, {column_pin}),')
        else:
            lines.append(f'    "{name}": (?, ?),  # UNRESOLVED')
    lines.append("}")
    return "\n".join(lines)


def apply_matrix_row_level(
    gpio: Any,
    column_pin: int,
    row_low: bool,
    sink_active: bool,
) -> bool:
    """Mirror only a row's low window onto a confirmed input column."""
    if row_low == sink_active:
        return sink_active
    if row_low:
        gpio.setup(column_pin, gpio.OUT, initial=gpio.LOW)
    else:
        gpio.setup(column_pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
    return row_low


def gated_matrix_pulse(row_pin: int, column_pin: int, duration: float) -> int:
    """Emulate one matrix switch without grounding its shared column continuously."""
    if row_pin == column_pin:
        raise ValueError("row and column GPIO must be different")
    try:
        import RPi.GPIO as gpio  # type: ignore[import-not-found]
    except ImportError as exc:
        raise RuntimeError("RPi.GPIO is unavailable; matrix-pulse must run on the Pi") from exc

    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    gpio.setup(row_pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
    gpio.setup(column_pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
    deadline = time.perf_counter_ns() + int(duration * 1_000_000_000)
    sink_active = False
    low_windows = 0

    try:
        while time.perf_counter_ns() < deadline:
            row_low = int(gpio.input(row_pin)) == 0
            if row_low and not sink_active:
                low_windows += 1
            sink_active = apply_matrix_row_level(
                gpio,
                column_pin,
                row_low,
                sink_active,
            )
    finally:
        gpio.setup(column_pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
        gpio.setup(row_pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
        gpio.cleanup([row_pin, column_pin])

    return low_windows


def validate_matrix_actions(actions: Sequence[str]) -> None:
    unknown = set(actions) - set(MATRIX_PINS)
    if unknown:
        raise ValueError(f"unknown matrix action: {sorted(unknown)[0]}")
    if "left-forward" in actions and "left-reverse" in actions:
        raise ValueError("left wheel cannot run forward and reverse together")
    if "right-forward" in actions and "right-reverse" in actions:
        raise ValueError("right wheel cannot run forward and reverse together")


def gated_matrix_actions(
    actions: Sequence[str],
    duration: float,
    gpio_module: Any | None = None,
) -> dict[str, int]:
    """Gate one direction per wheel against the remote's live matrix rows."""
    validate_matrix_actions(actions)
    if gpio_module is None:
        try:
            import RPi.GPIO as gpio_module  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("RPi.GPIO is unavailable; matrix-drive must run on the Pi") from exc

    gpio = gpio_module
    gpio.setwarnings(False)
    gpio.setmode(gpio.BCM)
    all_pins = sorted({pin for pair in MATRIX_PINS.values() for pin in pair})
    for pin in all_pins:
        gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)

    active = tuple(actions)
    sink_active = {name: False for name in active}
    low_windows = {name: 0 for name in active}
    deadline = time.perf_counter_ns() + int(duration * 1_000_000_000)

    try:
        while time.perf_counter_ns() < deadline:
            for name in active:
                row_pin, column_pin = MATRIX_PINS[name]
                row_low = int(gpio.input(row_pin)) == 0
                if row_low and not sink_active[name]:
                    low_windows[name] += 1
                sink_active[name] = apply_matrix_row_level(
                    gpio,
                    column_pin,
                    row_low,
                    sink_active[name],
                )
    finally:
        for pin in all_pins:
            gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_OFF)
        gpio.cleanup(all_pins)

    return low_windows


def pin_map_from_args(args: argparse.Namespace) -> dict[str, int]:
    pins = {
        "left-forward": args.left_forward_pin,
        "left-reverse": args.left_reverse_pin,
        "right-forward": args.right_forward_pin,
        "right-reverse": args.right_reverse_pin,
    }
    validate_pin_map(pins)
    return pins


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Probe/control a 3 V RC remote through active-low Pi GPIO"
    )
    parser.add_argument("--dry-run", action="store_true", help="print actions without GPIO")
    parser.add_argument("--left-forward-pin", type=gpio_pin, default=4, metavar="BCM")
    parser.add_argument("--left-reverse-pin", type=gpio_pin, default=5, metavar="BCM")
    parser.add_argument("--right-forward-pin", type=gpio_pin, default=6, metavar="BCM")
    parser.add_argument("--right-reverse-pin", type=gpio_pin, default=7, metavar="BCM")

    commands = parser.add_subparsers(dest="command", required=True)

    sense = commands.add_parser(
        "sense",
        help="observe one high-impedance GPIO while physically pressing a remote button",
    )
    sense.add_argument("--pin", type=gpio_pin, default=4, metavar="BCM")
    sense.add_argument("--seconds", type=positive_duration, default=12.0)

    profile = commands.add_parser(
        "profile",
        help="busy-poll a matrix row to measure its scan timing",
    )
    profile.add_argument("--pin", type=gpio_pin, default=5, metavar="BCM")
    profile.add_argument("--seconds", type=positive_duration, default=3.0)

    identify = commands.add_parser(
        "identify-rows",
        help="profile both GPIO of every button pair and print the resolved MATRIX_PINS",
    )
    identify.add_argument(
        "--seconds",
        type=positive_duration,
        default=2.0,
        help="profile window per pin; 2s covers about 50 scan periods",
    )

    matrix_pulse = commands.add_parser(
        "matrix-pulse",
        help="pull a confirmed input column low only while one scan row is low",
    )
    matrix_pulse.add_argument("--row-pin", type=gpio_pin, default=5, metavar="BCM")
    matrix_pulse.add_argument("--column-pin", type=gpio_pin, default=4, metavar="BCM")
    matrix_pulse.add_argument("--duration", type=bounded_duration, default=0.20)

    matrix_drive = commands.add_parser(
        "matrix-drive",
        help="run a mapped two-wheel differential-drive action",
    )
    matrix_drive.add_argument("motion", choices=tuple(MATRIX_MOTIONS))
    matrix_drive.add_argument("--duration", type=bounded_duration, default=0.20)

    pulse = commands.add_parser("pulse", help="press one mapped remote button briefly")
    pulse.add_argument("button", choices=BUTTON_NAMES)
    pulse.add_argument("--duration", type=bounded_duration, default=0.20)

    drive = commands.add_parser("drive", help="press a differential-drive button pair briefly")
    drive.add_argument("motion", choices=tuple(MOTIONS))
    drive.add_argument("--duration", type=bounded_duration, default=0.20)

    sequence = commands.add_parser("sequence", help="pulse each mapped button in order")
    sequence.add_argument("--duration", type=bounded_duration, default=0.15)
    sequence.add_argument("--pause", type=positive_duration, default=1.0)

    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.command == "sense":
        if args.dry_run:
            print(f"DRY RUN sense BCM{args.pin} for {args.seconds:.1f}s")
            return 0
        try:
            transitions = sense_pin(args.pin, args.seconds)
        except RuntimeError as exc:
            parser.error(str(exc))
        if transitions == 0:
            print("No level transitions observed.", file=sys.stderr)
            return 2
        return 0

    if args.command == "profile":
        if args.dry_run:
            print(f"DRY RUN profile BCM{args.pin} for {args.seconds:.1f}s")
            return 0
        try:
            low_widths_us, periods_us = profile_scan_pin(args.pin, args.seconds)
        except RuntimeError as exc:
            parser.error(str(exc))
        print(format_timing("low width", low_widths_us))
        print(format_timing("falling-edge period", periods_us))
        return 0 if low_widths_us else 2

    if args.command == "identify-rows":
        pins = sorted({pin for pair in CANDIDATE_PAIRS.values() for pin in pair})
        if args.dry_run:
            print(
                f"DRY RUN identify-rows over BCM{pins} at {args.seconds:.1f}s per pin "
                f"({len(pins) * args.seconds:.0f}s total)"
            )
            return 0
        print(
            "Profiling both wires of each button pair. Keep the remote's AAA cells "
            "in and the receiver/motor unit POWERED OFF; nothing is driven here.",
            flush=True,
        )
        try:
            resolved, report = identify_matrix_rows(CANDIDATE_PAIRS, args.seconds)
        except RuntimeError as exc:
            parser.error(str(exc))
        for line in report:
            print(line)

        try:
            nets = detect_shared_nets(pins, drive_low_and_read)
        except RuntimeError as exc:
            parser.error(str(exc))
        groups = net_groups(nets)
        print()
        print(f"Electrical nets found: {len(groups)} across {len(pins)} wires")
        for group in groups:
            members = ", ".join(f"BCM{pin}" for pin in group)
            note = "  <- one net, wired to the Pi more than once" if len(group) > 1 else ""
            print(f"  {members}{note}")
        if len(groups) < len(pins):
            print(
                "\nDuplicate wires collapsed. Only one pin per net may be used: "
                "driving one and releasing the other does not release the net, "
                "so a command pressing two buttons actuates a third."
            )
            resolved = matrix_pins_from_nets(resolved, groups)

        print()
        print(format_matrix_pins(resolved))
        missing = sorted(set(CANDIDATE_PAIRS) - set(resolved))
        if missing:
            print(f"\nUnresolved: {', '.join(missing)}", file=sys.stderr)
            return 2
        return 0

    if args.command == "matrix-pulse":
        if args.dry_run:
            print(
                f"DRY RUN matrix-pulse row=BCM{args.row_pin} "
                f"column=BCM{args.column_pin} duration={args.duration:.2f}s"
            )
            return 0
        try:
            low_windows = gated_matrix_pulse(
                args.row_pin,
                args.column_pin,
                args.duration,
            )
        except (RuntimeError, ValueError) as exc:
            parser.error(str(exc))
        print(f"Mirrored {low_windows} target-row low windows.")
        return 0 if low_windows else 2

    if args.command == "matrix-drive":
        actions = MATRIX_MOTIONS[args.motion]
        if args.dry_run:
            pairs = ", ".join(
                f"{name}=row BCM{MATRIX_PINS[name][0]}/column BCM{MATRIX_PINS[name][1]}"
                for name in actions
            )
            print(
                f"DRY RUN matrix-drive {args.motion} for {args.duration:.2f}s: {pairs}"
            )
            return 0
        try:
            low_windows = gated_matrix_actions(actions, args.duration)
        except (RuntimeError, ValueError) as exc:
            parser.error(str(exc))
        for name in actions:
            print(f"{name}: mirrored {low_windows[name]} row windows")
        return 0 if all(low_windows.values()) else 2

    try:
        pins = pin_map_from_args(args)
    except ValueError as exc:
        parser.error(str(exc))

    try:
        backend: RpiOpenDrainButtons | DryRunButtons
        backend = DryRunButtons() if args.dry_run else RpiOpenDrainButtons(pins)
    except RuntimeError as exc:
        parser.error(str(exc))

    try:
        if args.command == "pulse":
            pulse_buttons(backend, (args.button,), args.duration)
        elif args.command == "drive":
            pulse_buttons(backend, MOTIONS[args.motion], args.duration)
        elif args.command == "sequence":
            for index, name in enumerate(BUTTON_NAMES):
                print(f"Pulsing {name} on BCM{pins[name]}", flush=True)
                pulse_buttons(backend, (name,), args.duration)
                if index + 1 < len(BUTTON_NAMES):
                    time.sleep(args.pause)
    except KeyboardInterrupt:
        print("Interrupted; releasing all remote buttons.", file=sys.stderr)
        return 130
    finally:
        backend.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
