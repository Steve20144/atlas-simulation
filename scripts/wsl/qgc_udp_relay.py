#!/usr/bin/env python3
"""Relay MAVLink between Gazebo Classic's QGC socket and QGroundControl on the Windows host.

PX4's Gazebo Classic mavlink_interface plugin sends telemetry for the ground station as a UDP
*broadcast* to port 14550 and forwards whatever comes back on that socket to the Pixhawk. WSL2's
NAT network delivers the broadcast to sockets inside WSL but never to Windows, so QGroundControl
on the host sees nothing. This relay binds the broadcast port inside WSL, forwards each datagram
to the Windows host (WSL's default gateway) as unicast, and returns QGC's replies to the plugin's
own source port, which it learns from the first datagram. Nothing to configure in QGC.

    python3 qgc_udp_relay.py [--listen 14550] [--host <windows ip>] [--port 14550]
"""

from __future__ import annotations

import argparse
import select
import socket
import subprocess
import sys
import time


def default_gateway() -> str:
    out = subprocess.run(["ip", "route", "show", "default"], capture_output=True, text=True)
    parts = out.stdout.split()
    return parts[parts.index("via") + 1] if "via" in parts else "127.0.0.1"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--listen", type=int, default=14550,
                    help="port the plugin broadcasts to (qgc_udp_port)")
    ap.add_argument("--host", default=None,
                    help="Windows host address (default: WSL default gateway)")
    ap.add_argument("--port", type=int, default=14550,
                    help="QGroundControl's listening port on the host")
    args = ap.parse_args()
    host = args.host or default_gateway()

    sim = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # receives the plugin's broadcasts
    sim.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sim.bind(("0.0.0.0", args.listen))
    gcs = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # talks to QGC on the host
    gcs.bind(("0.0.0.0", 0))
    plugin_addr = None
    n_up = n_down = 0
    last_report = time.monotonic()
    print(f"[qgc_relay] WSL :{args.listen}  <->  QGroundControl {host}:{args.port}", flush=True)
    while True:
        readable, _, _ = select.select([sim, gcs], [], [], 1.0)
        for s in readable:
            data, addr = s.recvfrom(65535)
            if s is sim:
                if addr[0] == host:
                    continue  # our own relayed traffic looping back, ignore
                plugin_addr = addr
                gcs.sendto(data, (host, args.port))
                n_up += 1
            elif plugin_addr is not None:
                sim.sendto(data, plugin_addr)
                n_down += 1
        now = time.monotonic()
        if now - last_report > 10.0:
            state = (f"plugin {plugin_addr[0]}:{plugin_addr[1]}" if plugin_addr
                     else "waiting for Gazebo")
            print(f"[qgc_relay] {state}; to QGC {n_up} datagrams, from QGC {n_down}", flush=True)
            last_report = now
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
