"""Pixhawk 6X Pro over USB MAVLink: find the port, report status, push parameters, verify.

The board shows up as a USB CDC serial device (``COMn`` on Windows, ``/dev/ttyACMn`` on Linux).
The v1.17.0 fmu-v6x firmware enumerates with Holybro's vendor id 0x3185 and product id 0x0035
(``boards/px4/fmu-v6x/nuttx-config/nsh/defconfig`` lines 81 and 85 of the pinned tree); older
boards used 3D Robotics' 0x26AC. ``list_ports`` flags matches so the UI can pick one without asking.

Writing parameters: PX4 carries INT32 parameters byte-wise in ``PARAM_VALUE`` (the float field
holds the integer's bits, see ``src/modules/mavlink/mavlink_parameters.cpp`` ``send_param``).
A plain float ``PARAM_SET`` therefore corrupts every integer parameter, so writes go through the
NSH shell (``param set NAME VALUE`` over ``SERIAL_CONTROL``, then ``param save``) and reads decode
INT32 with struct. The pure helpers mirror ``scripts/px4_board.py``, which must stay standalone
because it also runs under WSL's system python without this package installed.

Every push writes the board's current values of the touched parameters to a timestamped
``.params`` file first (PLAN.md M8: backup before any push), then reads every value back.
"""

from __future__ import annotations

import re
import struct
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Protocol

from tiltlab.core.params_px4 import (
    PARAM_TYPE_FLOAT,
    PARAM_TYPE_INT32,
    default_header,
    params_file_from_dict,
    write_params_file,
)
from tiltlab.export.naming import timestamped_name

PIXHAWK_USB_IDS = {(0x3185, 0x0035), (0x26AC, 0x0011), (0x26AC, 0x0010)}
PIXHAWK_WORDS = ("px4", "pixhawk", "fmu", "holybro")
HIL_FLAG = 32  # MAV_MODE_FLAG_HIL_ENABLED
ARMED_FLAG = 128  # MAV_MODE_FLAG_SAFETY_ARMED
AUTOPILOT_COMPONENT = 1  # MAV_COMP_ID_AUTOPILOT1; the sim's bridge heartbeats too, from 200
SHELL_DEV = 10  # SERIAL_CONTROL_DEV_SHELL
SHELL_FLAGS = 2 | 4  # RESPOND | EXCLUSIVE, as Tools/mavlink_shell.py sends them
MSG_AUTOPILOT_VERSION = 148
CMD_REQUEST_MESSAGE = 512
# how long the shell console is drained after each command, and how long a PARAM_VALUE may take
SHELL_SETTLE_S = 0.4
PARAM_TIMEOUT_S = 2.0
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

# one board, one link at a time: status and push must not interleave on the serial port
LOCK = threading.Lock()


class BoardError(RuntimeError):
    """No board, no heartbeat, or the port is busy."""


class Link(Protocol):
    """The slice of pymavlink's mavfile this module uses (a fake stands in for tests)."""

    target_system: int
    target_component: int
    mav: Any

    def recv_match(
        self, type: str | None = None, blocking: bool = False, timeout: float | None = None
    ) -> Any: ...

    def close(self) -> None: ...


# ---------------------------------------------------------------- pure helpers
def decode_param_value(raw: float, ptype: int) -> float:
    """PARAM_VALUE.param_value as the number PX4 means; INT32 travels as raw bits."""
    if ptype == PARAM_TYPE_INT32:
        return float(struct.unpack("<i", struct.pack("<f", raw))[0])
    return float(raw)


def shell_commands(params: dict[str, int | float], types: dict[str, int]) -> list[str]:
    """``param set`` lines for the board shell (ints written as ints), then ``param save``."""
    lines = []
    for name, value in params.items():
        ptype = types.get(name, PARAM_TYPE_FLOAT)
        shown = str(int(value)) if ptype == PARAM_TYPE_INT32 else repr(float(value))
        lines.append(f"param set {name} {shown}")
    lines.append("param save")
    return lines


def value_matches(got: float | None, want: float) -> bool:
    """Float32 round trip tolerance: 1e-4 relative or absolute, whichever is larger."""
    return got is not None and abs(got - want) <= max(1e-4, abs(want) * 1e-4)


def firmware_string(msg: Any) -> str:
    """AUTOPILOT_VERSION.flight_sw_version packed as major.minor.patch.type -> '1.17.0'."""
    v = int(msg.flight_sw_version)
    return f"{(v >> 24) & 0xFF}.{(v >> 16) & 0xFF}.{(v >> 8) & 0xFF}"


def is_pixhawk(vid: int | None, pid: int | None, description: str) -> bool:
    if vid is not None and (vid, pid) in PIXHAWK_USB_IDS:
        return True
    return any(w in description.lower() for w in PIXHAWK_WORDS)


# ---------------------------------------------------------------- ports
def list_ports() -> list[dict[str, Any]]:
    """Serial ports on this host with a ``pixhawk`` flag; Pixhawks first."""
    from serial.tools import list_ports  # pyserial, declared in backend/pyproject.toml

    out = []
    for p in list_ports.comports():
        desc = " ".join(s for s in (p.description, p.manufacturer, p.product) if s)
        out.append(
            {
                "device": p.device,
                "description": desc,
                "vid": p.vid,
                "pid": p.pid,
                "pixhawk": is_pixhawk(p.vid, p.pid, desc),
            }
        )
    return sorted(out, key=lambda d: (not d["pixhawk"], d["device"]))


def pick_port(port: str | None) -> str | None:
    """Explicit port, or the first Pixhawk-looking one; None when nothing is plugged in."""
    if port and port != "auto":
        return port
    return next((p["device"] for p in list_ports() if p["pixhawk"]), None)


def connect(port: str, timeout_s: float = 5.0) -> Link:
    """Open the serial link and wait for the autopilot heartbeat (USB CDC ignores the baud)."""
    from pymavlink import mavutil

    try:
        m = mavutil.mavlink_connection(port, baud=115200, source_system=250, autoreconnect=False)
    except Exception as exc:  # pyserial raises SerialException; keep the API message simple
        raise BoardError(f"cannot open {port}: {exc}") from exc
    end = time.time() + timeout_s
    while time.time() < end:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if hb is not None and hb.get_srcComponent() == AUTOPILOT_COMPONENT:
            m.target_system, m.target_component = hb.get_srcSystem(), AUTOPILOT_COMPONENT
            return m
    m.close()
    raise BoardError(f"no autopilot heartbeat on {port} within {timeout_s:.0f} s")


# ---------------------------------------------------------------- status
@dataclass
class BoardStatus:
    connected: bool
    port: str | None = None
    system_id: int | None = None
    firmware: str | None = None
    board_id: int | None = None
    armed: bool | None = None
    hil: bool | None = None
    mode: str | None = None
    ports: list[dict[str, Any]] = field(default_factory=list)
    message: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def mode_string(hb: Any) -> str:
    """PX4 flight mode name from the heartbeat (pymavlink decodes custom_mode per autopilot)."""
    from pymavlink import mavutil

    try:
        return str(mavutil.mode_string_v10(hb))
    except Exception:  # a message without the pymavlink accessors (tests, foreign autopilots)
        return f"custom_mode {int(hb.custom_mode)}"


def read_status(m: Link, port: str) -> BoardStatus:
    """Heartbeat flags plus AUTOPILOT_VERSION (requested with MAV_CMD_REQUEST_MESSAGE)."""
    hb = None
    end = time.time() + 3.0
    while hb is None and time.time() < end:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=1.5)
        if msg is not None and msg.get_srcComponent() == AUTOPILOT_COMPONENT:
            hb = msg
    if hb is None:
        return BoardStatus(connected=False, port=port, message="heartbeat lost")
    m.mav.command_long_send(
        m.target_system, m.target_component, CMD_REQUEST_MESSAGE, 0,
        MSG_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0,
    )
    ver = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=2.0)
    return BoardStatus(
        connected=True,
        port=port,
        system_id=m.target_system,
        firmware=firmware_string(ver) if ver is not None else None,
        board_id=int(ver.board_version) if ver is not None else None,
        armed=bool(hb.base_mode & ARMED_FLAG),
        hil=bool(hb.base_mode & HIL_FLAG),
        mode=mode_string(hb),
        message="autopilot heartbeat received",
    )


# ---------------------------------------------------------------- parameters
def read_param(m: Link, name: str) -> tuple[float, int] | None:
    m.mav.param_request_read_send(m.target_system, m.target_component, name.encode(), -1)
    end = time.time() + PARAM_TIMEOUT_S
    while time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=PARAM_TIMEOUT_S)
        if msg is not None and str(msg.param_id).rstrip("\x00") == name:
            return float(msg.param_value), int(msg.param_type)
    return None


def read_params(m: Link, names: list[str], types: dict[str, int]) -> dict[str, float | None]:
    """Decoded values by name; None when the board does not answer (unknown parameter)."""
    out: dict[str, float | None] = {}
    for name in names:
        got = read_param(m, name)
        out[name] = decode_param_value(got[0], types.get(name, got[1])) if got else None
    return out


def shell(m: Link, commands: list[str]) -> str:
    """Run NSH commands through SERIAL_CONTROL and return the console text."""
    out = bytearray()

    def drain(seconds: float) -> None:
        end = time.time() + seconds
        while time.time() < end:
            msg = m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
            if msg is not None and msg.count:
                out.extend(bytes(msg.data[: msg.count]))

    for cmd in commands:
        data = (cmd + "\n").encode()
        for i in range(0, len(data), 70):
            chunk = data[i : i + 70]
            m.mav.serial_control_send(
                SHELL_DEV, SHELL_FLAGS, 0, 0, len(chunk), list(chunk) + [0] * (70 - len(chunk))
            )
        drain(SHELL_SETTLE_S)
    drain(SHELL_SETTLE_S)
    return _ANSI.sub("", out.decode(errors="replace"))


@dataclass
class PushResult:
    port: str
    sent: int
    changed: list[str]
    verified: int
    mismatches: list[dict[str, Any]]
    backup: str | None
    console: str

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def backup_current(
    m: Link, params: dict[str, int | float], types: dict[str, int], out_dir: Path, stem: str
) -> tuple[Path, dict[str, float | None]]:
    """Read the board's current values of `params` and write them as <ts>_<stem>_before.params."""
    current = read_params(m, list(params), types)
    known = {k: v for k, v in current.items() if v is not None}
    typed: dict[str, int | float] = {
        k: (int(v) if types.get(k) == PARAM_TYPE_INT32 else v) for k, v in known.items()
    }
    header = default_header(note=f"board values before tiltlab push ({stem})")
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / timestamped_name(f"{stem}_before", "params")
    write_params_file(params_file_from_dict(typed, types, header), path)
    return path, current


def push_params(
    m: Link,
    port: str,
    params: dict[str, int | float],
    types: dict[str, int],
    backup_dir: Path | None,
    stem: str,
) -> PushResult:
    """Backup, ``param set`` each entry through the shell, ``param save``, read everything back."""
    backup_path: Path | None = None
    before: dict[str, float | None] = {}
    if backup_dir is not None:
        backup_path, before = backup_current(m, params, types, backup_dir, stem)
    console = shell(m, shell_commands(params, types))
    after = read_params(m, list(params), types)
    mismatches = [
        {"name": k, "wanted": float(v), "board": after.get(k)}
        for k, v in params.items()
        if not value_matches(after.get(k), float(v))
    ]
    if before:
        changed = [k for k, v in params.items() if not value_matches(before.get(k), float(v))]
    else:
        changed = list(params)
    return PushResult(
        port=port,
        sent=len(params),
        changed=changed,
        verified=len(params) - len(mismatches),
        mismatches=mismatches,
        backup=str(backup_path) if backup_path else None,
        console=console[-2000:],
    )
