#!/usr/bin/env python3
"""Send refreshed wheel commands to the House Bot Pi motor service."""

from __future__ import annotations

import argparse
import json
import secrets
import socket
import time


MOTIONS = {
    "forward": (1.0, 1.0),
    "reverse": (-1.0, -1.0),
    "left": (-1.0, 1.0),
    "right": (1.0, -1.0),
    "stop": (0.0, 0.0),
}


def encode_command(session: str, sequence: int, left: float, right: float) -> bytes:
    return json.dumps(
        {"session": session, "sequence": sequence, "left": left, "right": right},
        separators=(",", ":"),
    ).encode("utf-8")


def send_motion(
    host: str,
    port: int,
    left: float,
    right: float,
    duration: float,
    rate_hz: float,
) -> tuple[int, int, bool]:
    interval = 1.0 / rate_hz
    deadline = time.monotonic() + duration
    sequence = 0
    acknowledged_sequences: set[int] = set()
    session = secrets.token_hex(8)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    def drain_acknowledgements() -> None:
        while True:
            try:
                payload, _address = sock.recvfrom(4096)
            except BlockingIOError:
                return
            try:
                response = json.loads(payload.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if response.get("ok") and response.get("session") == session:
                response_sequence = response.get("sequence")
                if isinstance(response_sequence, int):
                    acknowledged_sequences.add(response_sequence)

    try:
        while True:
            now = time.monotonic()
            if now >= deadline:
                break
            sock.sendto(encode_command(session, sequence, left, right), (host, port))
            sequence += 1
            drain_acknowledgements()
            sleep_seconds = interval - (time.monotonic() - now)
            if sleep_seconds > 0:
                time.sleep(sleep_seconds)
    finally:
        stop_sequence = sequence
        stop_payload = encode_command(session, stop_sequence, 0.0, 0.0)
        stop_deadline = time.monotonic() + 0.6
        next_stop_send = 0.0
        while time.monotonic() < stop_deadline and stop_sequence not in acknowledged_sequences:
            now = time.monotonic()
            if now >= next_stop_send:
                sock.sendto(stop_payload, (host, port))
                next_stop_send = now + 0.05
            drain_acknowledgements()
            time.sleep(0.005)
        for _attempt in range(2):
            sock.sendto(stop_payload, (host, port))
            time.sleep(0.01)
        sock.close()

    return sequence, len(acknowledged_sequences), stop_sequence in acknowledged_sequences


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Drive House Bot through the Pi UDP service")
    parser.add_argument("motion", choices=tuple(MOTIONS))
    parser.add_argument("--host", default="192.168.0.241")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--rate", type=float, default=20.0, help="command refresh rate in Hz")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.duration < 0:
        raise SystemExit("--duration must be non-negative")
    if not 5.0 <= args.rate <= 50.0:
        raise SystemExit("--rate must be between 5 and 50 Hz")
    left, right = MOTIONS[args.motion]
    packets, acknowledgements, stop_acknowledged = send_motion(
        args.host,
        args.port,
        left,
        right,
        args.duration,
        args.rate,
    )
    print(
        f"Sent {packets} command packets, received {acknowledgements} acknowledgements; "
        f"stop acknowledged={stop_acknowledged}."
    )
    return 0 if acknowledgements > 0 and stop_acknowledged else 2


if __name__ == "__main__":
    raise SystemExit(main())
