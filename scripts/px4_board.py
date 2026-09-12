"""Talk to the Pixhawk over MAVLink: parameters, preflight, attitude, logs.

    python3 scripts/px4_board.py [--udp HOST:PORT | --dev /dev/ttyACMn] <command> ...

    status                      heartbeat (HIL flag), preflight result, attitude, velocity
    shell "cmd" ["cmd" ...]     run NSH commands on the board and print their output
    push  <file.params>         load a QGC .params file through the board shell, then param save
    verify <file.params>        read every parameter back and report mismatches
    pull-log <out_dir>          download the newest ulog (highest id) into out_dir

Connection: while the Gazebo Classic HITL sim runs it owns the serial port, so use the sim's SDK
link, --udp 127.0.0.1:14540 (the default). With no sim running use --dev /dev/ttyACM0.

Why the shell for writing parameters: PX4 carries INT32 parameters byte-wise in PARAM_VALUE (the
float field holds the integer's bits). A plain float PARAM_SET corrupts every integer parameter
and a float read-back agrees with itself, so writes go through NSH ``param set`` and reads
decode INT32 with struct (see decode_param_value).
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

INT32 = 6  # MAV_PARAM_TYPE_INT32
REAL32 = 9  # MAV_PARAM_TYPE_REAL32
HIL_FLAG = 32  # MAV_MODE_FLAG_HIL_ENABLED
SHELL_DEV = 10  # SERIAL_CONTROL_DEV_SHELL
# SERIAL_CONTROL_FLAG_RESPOND | SERIAL_CONTROL_FLAG_EXCLUSIVE, as Tools/mavlink_shell.py sends
# them. REPLY (1) marks a message as coming from the board, and the board ignores it.
SHELL_FLAGS = 2 | 4


# ---------------------------------------------------------------- pure helpers (unit tested)
def parse_params_file(text: str) -> list[tuple[str, float, int]]:
    """QGC .params lines ``vehicle component name value type`` -> (name, value, type)."""
    out = []
    for line in text.splitlines():
        if line.startswith("#") or not line.strip():
            continue
        f = line.split()
        if len(f) >= 5:
            out.append((f[2], float(f[3]), int(f[4])))
    return out


def decode_param_value(raw: float, ptype: int) -> float:
    """PARAM_VALUE.param_value as the number PX4 means; INT32 travels as raw bits."""
    if ptype == INT32:
        return float(struct.unpack("<i", struct.pack("<f", raw))[0])
    return raw


def shell_commands(params: list[tuple[str, float, int]]) -> list[str]:
    """``param set`` lines for the board shell (ints written as ints), then ``param save``."""
    lines = []
    for name, value, ptype in params:
        shown = str(int(value)) if ptype == INT32 else repr(float(value))
        lines.append(f"param set {name} {shown}")
    lines.append("param save")
    return lines


def value_matches(got: float | None, want: float) -> bool:
    return got is not None and abs(got - want) <= max(1e-4, abs(want) * 1e-4)


# ---------------------------------------------------------------- MAVLink side
def connect(args: argparse.Namespace):
    from pymavlink import mavutil

    target = f"udpin:{args.udp}" if args.udp else args.dev
    m = mavutil.mavlink_connection(target, baud=57600)
    m.wait_heartbeat(timeout=40)
    return m


def shell(m, commands: list[str], settle_s: float = 1.5) -> str:
    """Run NSH commands through SERIAL_CONTROL and return the console text."""
    out = bytearray()

    def drain(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.3)
            if msg and msg.count:
                out.extend(bytes(msg.data[: msg.count]))

    for cmd in commands:
        data = (cmd + "\n").encode()
        for i in range(0, len(data), 70):
            chunk = data[i : i + 70]
            m.mav.serial_control_send(SHELL_DEV, SHELL_FLAGS, 0, 0, len(chunk),
                                      list(chunk) + [0] * (70 - len(chunk)))
        drain(settle_s)
    drain(0.5)
    text = out.decode(errors="replace")
    # strip the ANSI erase sequences the NSH prompt repaints with
    import re

    return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text)


def read_param(m, name: str, timeout: float = 3.0) -> tuple[float, int] | None:
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    end = time.time() + timeout
    while time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=timeout)
        if msg and msg.param_id == name:
            return msg.param_value, msg.param_type
    return None


def cmd_status(m) -> int:
    import math

    # the sim's mavlink_interface also heartbeats (sysid 1, compid 200) with no HIL flag; only the
    # autopilot component (compid 1) tells the truth here
    hb, end = None, time.time() + 10
    while time.time() < end:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        if msg and msg.get_srcComponent() == 1:
            hb = msg
            break
    print("HIL flag:", bool(hb.base_mode & HIL_FLAG) if hb else "no autopilot heartbeat")
    att = m.recv_match(type="ATTITUDE", blocking=True, timeout=5)
    pos = m.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=5)
    if att:
        r, pch, y = (math.degrees(v) for v in (att.roll, att.pitch, att.yaw))
        print(f"attitude deg: roll {r:.2f} pitch {pch:.2f} yaw {y:.2f}")
    if pos:
        print(f"velocity m/s: {pos.vx:.2f} {pos.vy:.2f} {pos.vz:.2f}   z {pos.z:.2f} m")
    # vehicle_status.hil_state is the authoritative answer; the heartbeat flag above is advisory
    text = shell(m, ["listener vehicle_status 1", "commander check"], settle_s=3.0)
    lines = [ln.strip() for ln in text.splitlines()]
    hil = next((ln for ln in lines if "hil_state:" in ln), "hil_state: (unread)")
    pre = next((ln for ln in lines if "Preflight check" in ln), "(no preflight line)")
    print(hil)
    print(pre)
    return 0


def cmd_push(m, path: Path) -> int:
    params = parse_params_file(path.read_text())
    text = shell(m, shell_commands(params), settle_s=0.4)
    changed = [ln.strip() for ln in text.splitlines() if "curr:" in ln]
    print(f"{len(params)} parameters sent, {len(changed)} changed (console echo):")
    for ln in changed:
        print("  " + ln)
    print("run `verify` to confirm; the echo is advisory, the read-back is the truth")
    return 0


def cmd_verify(m, path: Path) -> int:
    params = parse_params_file(path.read_text())
    bad = []
    for name, want, ptype in params:
        got = read_param(m, name)
        val = decode_param_value(got[0], ptype) if got else None
        if not value_matches(val, want):
            bad.append((name, want, val))
    print(f"{len(params) - len(bad)} of {len(params)} match")
    for name, want, val in bad:
        print(f"  MISMATCH {name}: wanted {want}, board has {val}")
    return 1 if bad else 0


def cmd_pull_log(m, out_dir: Path) -> int:
    ts, tc = m.target_system, m.target_component
    m.mav.log_request_list_send(ts, tc, 0, 0xFFFF)
    entries, end = {}, time.time() + 20
    while time.time() < end:
        e = m.recv_match(type="LOG_ENTRY", blocking=True, timeout=3)
        if e is None:
            if entries:
                break
            continue
        entries[e.id] = e
        if e.num_logs and len(entries) >= e.num_logs:
            break
    if not entries:
        print("no logs on the board")
        return 1
    newest = max(entries.values(), key=lambda e: e.id)  # sess logs report time_utc 0
    size = newest.size
    buf, have = bytearray(size), bytearray(size)
    m.mav.log_request_data_send(ts, tc, newest.id, 0, 0xFFFFFFFF)
    idle = time.time()
    while sum(have) < size:
        d = m.recv_match(type="LOG_DATA", blocking=True, timeout=3)
        if d is not None and d.id == newest.id and d.count:
            stop = min(d.ofs + d.count, size)
            buf[d.ofs:stop] = bytes(d.data[: stop - d.ofs])
            have[d.ofs:stop] = b"\x01" * (stop - d.ofs)
            idle = time.time()
        elif time.time() - idle > 4:  # stalled: ask again from the first gap
            gap = have.find(b"\x00")
            nxt = have.find(b"\x01", gap)
            m.mav.log_request_data_send(ts, tc, newest.id, gap, (nxt if nxt > 0 else size) - gap)
            idle = time.time()
    m.mav.log_request_end_send(ts, tc)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"board_log{newest.id:03d}.ulg"
    path.write_bytes(buf)
    print(f"saved {path} ({size} bytes, log id {newest.id} of {len(entries)})")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    link = p.add_mutually_exclusive_group()
    link.add_argument("--udp", default="0.0.0.0:14540", help="listen for the sim's SDK link")
    link.add_argument("--dev", help="serial device when no sim owns the port")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sub.add_parser("shell").add_argument("commands", nargs="+")
    sub.add_parser("push").add_argument("file", type=Path)
    sub.add_parser("verify").add_argument("file", type=Path)
    sub.add_parser("pull-log").add_argument("out_dir", type=Path)
    args = p.parse_args()
    if args.dev:
        args.udp = None
    m = connect(args)
    if args.cmd == "status":
        return cmd_status(m)
    if args.cmd == "shell":
        print(shell(m, args.commands, settle_s=2.0))
        return 0
    if args.cmd == "push":
        return cmd_push(m, args.file)
    if args.cmd == "verify":
        return cmd_verify(m, args.file)
    return cmd_pull_log(m, args.out_dir)


if __name__ == "__main__":
    sys.exit(main())
