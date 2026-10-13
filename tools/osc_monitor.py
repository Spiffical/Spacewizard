#!/usr/bin/env python3
"""Watch the SpaceWizard public OSC stream.

    python3 tools/osc_monitor.py            # compact live view, UDP port 9000
    python3 tools/osc_monitor.py --raw      # every message as it arrives
    python3 tools/osc_monitor.py --port 9001

Standalone: needs only python-osc (pip install python-osc). Run it on any computer
on the same network as the candle's Raspberry Pi.
"""

import argparse
import socket
import time

from pythonosc.osc_bundle import OscBundle
from pythonosc.osc_message import OscMessage


def messages(dgram):
    if not OscBundle.dgram_is_bundle(dgram):
        yield OscMessage(dgram)
        return
    stack = [OscBundle(dgram)]
    while stack:
        for item in stack.pop():
            if isinstance(item, OscBundle):
                stack.append(item)
            else:
                yield item


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", type=int, default=9000)
    parser.add_argument("--raw", action="store_true", help="print every message")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    sock.bind(("", args.port))
    print(f"Listening for SpaceWizard OSC on UDP port {args.port} (Ctrl+C to stop)")

    latest, senders, last_print = {}, set(), 0.0
    while True:
        dgram, (host, _) = sock.recvfrom(65535)
        if host not in senders:
            senders.add(host)
            print(f"\nReceiving from {host}")
        for msg in messages(dgram):
            name = msg.address.rsplit("/", 1)[-1]
            if args.raw:
                print(msg.address, *msg.params)
            elif name in ("pickup", "impact"):
                print(f"\n** {name} {' '.join(f'{v:.2f}' if isinstance(v, float) else str(v) for v in msg.params)}")
            latest[name] = msg.params
        now = time.monotonic()
        if not args.raw and now - last_print >= 0.2:
            last_print = now
            v = lambda key, n=1: latest.get(key, [0] * n)  # noqa: E731
            pitch, roll, yaw = v("orientation", 3)
            ax, ay, az = v("accel", 3)
            gx, gy, gz = v("gyro", 3)
            print(
                f"\rvalid {v('valid')[0]} active {v('active')[0]} "
                f"energy {v('energy')[0]:.2f} activity {v('activity')[0]:.2f} | "
                f"pitch {pitch:6.1f} roll {roll:6.1f} yaw {yaw:5.1f} | "
                f"accel {ax:5.2f} {ay:5.2f} {az:5.2f} g | gyro {gx:6.1f} {gy:6.1f} {gz:6.1f} dps   ",
                end="", flush=True,
            )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
