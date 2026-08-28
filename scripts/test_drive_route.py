#!/usr/bin/env python3
"""Tests for the single-process route driver."""

import unittest

import drive_route
from drive_route import Leg, build_timeline, leg_at, parse_segment, run_route


class ParseSegmentTests(unittest.TestCase):
    def test_a_valid_segment_parses(self) -> None:
        self.assertEqual(parse_segment("forward:3"), ("forward", 3.0))

    def test_an_unknown_motion_is_refused(self) -> None:
        with self.assertRaises(Exception):
            parse_segment("sideways:1")

    def test_a_missing_duration_is_refused(self) -> None:
        with self.assertRaises(Exception):
            parse_segment("forward")

    def test_the_shell_splitting_bug_cannot_pass_silently(self) -> None:
        """Regression: a shell loop once passed "forward 1.0" as one argument.

        Under zsh an unquoted expansion does not word-split, so every command
        became an invalid single token, argparse rejected it, and a whole route
        drove nothing while still printing as if it had run.
        """
        with self.assertRaises(Exception):
            parse_segment("forward 1.0")

    def test_an_over_long_segment_is_refused(self) -> None:
        with self.assertRaises(Exception):
            parse_segment("forward:99")


class TimelineTests(unittest.TestCase):
    def test_gaps_become_explicit_stop_legs(self) -> None:
        legs = build_timeline([("forward", 3.0), ("reverse", 3.0)], gap_s=2.5, tail_s=0.0)
        labels = [leg.label for leg in legs]
        self.assertEqual(labels, ["forward", "gap", "reverse"])
        self.assertTrue(legs[1].is_stop)
        self.assertAlmostEqual(legs[1].duration_s, 2.5)

    def test_no_trailing_gap_after_the_last_segment(self) -> None:
        legs = build_timeline([("forward", 1.0), ("reverse", 1.0)], gap_s=2.0, tail_s=0.0)
        self.assertEqual(legs[-1].label, "reverse")

    def test_legs_are_contiguous(self) -> None:
        legs = build_timeline([("forward", 3.0), ("left", 0.7)], gap_s=1.0)
        for earlier, later in zip(legs, legs[1:]):
            self.assertAlmostEqual(earlier.end_s, later.start_s)

    def test_an_empty_route_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            build_timeline([], gap_s=1.0)

    def test_an_over_long_route_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            build_timeline([("forward", 10.0)] * 40, gap_s=5.0)

    def test_leg_at_finds_the_covering_leg_and_ends(self) -> None:
        legs = build_timeline([("forward", 2.0), ("reverse", 2.0)], gap_s=1.0, tail_s=0.0)
        self.assertEqual(leg_at(legs, 0.5).label, "forward")
        self.assertEqual(leg_at(legs, 2.5).label, "gap")
        self.assertEqual(leg_at(legs, 3.5).label, "reverse")
        self.assertIsNone(leg_at(legs, 99.0))


class FakeSocket:
    def __init__(self) -> None:
        self.sent: list[tuple[float, float]] = []

    def sendto(self, payload: bytes, _address) -> None:
        import json
        document = json.loads(payload.decode())
        self.sent.append((document["left"], document["right"]))

    def recvfrom(self, _size):
        raise BlockingIOError

    def close(self) -> None:
        pass


class RunRouteTests(unittest.TestCase):
    def drive(self, legs):
        clock = {"t": 0.0}

        def now() -> float:
            return clock["t"]

        def sleep(seconds: float) -> None:
            clock["t"] += max(seconds, 0.001)

        sock = FakeSocket()
        results = run_route(
            legs, "host", 1, rate_hz=20.0, sock=sock, now_fn=now, sleep_fn=sleep
        )
        return sock, results

    def test_the_whole_route_streams_without_silence(self) -> None:
        """Every tick carries a command, including during gaps."""
        legs = build_timeline([("forward", 1.0), ("reverse", 1.0)], gap_s=1.0, tail_s=0.0)
        sock, results = self.drive(legs)
        self.assertTrue(all(r.packets > 0 for r in results))
        # A gap is commanded zeros, never an absence of packets.
        gap = next(r for r in results if r.leg.label == "gap")
        self.assertGreater(gap.packets, 0)

    def test_gap_packets_command_a_stop(self) -> None:
        legs = build_timeline([("forward", 1.0), ("reverse", 1.0)], gap_s=1.0, tail_s=0.0)
        sock, _ = self.drive(legs)
        self.assertIn((0.0, 0.0), sock.sent)
        self.assertIn((1.0, 1.0), sock.sent)
        self.assertIn((-1.0, -1.0), sock.sent)

    def test_a_stop_is_always_sent_last(self) -> None:
        legs = build_timeline([("forward", 1.0)], gap_s=0.0, tail_s=0.0)
        sock, _ = self.drive(legs)
        self.assertEqual(sock.sent[-1], (0.0, 0.0))


if __name__ == "__main__":
    unittest.main()


class SchedulingTests(unittest.TestCase):
    def test_ticks_are_counted_not_derived_from_float_division(self) -> None:
        """Regression: `elapsed // interval` froze the clock and flooded the wire.

        At 20 Hz the 6th tick lands on elapsed=0.25, where 0.25 // 0.05 is 4
        rather than 5 under binary floating point. The computed target then
        equalled the current time, nothing slept, and the send loop spun.
        """
        legs = build_timeline([("forward", 2.0)], gap_s=0.0, tail_s=0.0)
        clock = {"t": 0.0}
        sock = FakeSocket()
        run_route(
            legs, "host", 1, rate_hz=20.0, sock=sock,
            now_fn=lambda: clock["t"],
            sleep_fn=lambda s: clock.__setitem__("t", clock["t"] + max(s, 0.001)),
        )
        # 2 s at 20 Hz is about 40 packets plus a handful of stops. A spinning
        # loop produced tens of thousands.
        self.assertLess(len(sock.sent), 200)
        self.assertGreater(len(sock.sent), 30)
