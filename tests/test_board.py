"""Pixhawk USB module against a fake MAVLink link: INT32 decoding, shell writes, backup and
read-back, and the /api/board endpoints with the link monkeypatched (no hardware in CI)."""

from __future__ import annotations

import json
import struct
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

import tiltlab.api.app as app_module
from tiltlab.api.app import app
from tiltlab.core.params_px4 import PARAM_TYPE_FLOAT, PARAM_TYPE_INT32, read_params_file
from tiltlab.px4 import board

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"


class FakeLink:
    """A board with a parameter store: answers PARAM_REQUEST_READ, echoes shell commands and
    applies ``param set`` lines the way NSH would."""

    def __init__(self, store: dict[str, tuple[float, int]], heartbeat: bool = True) -> None:
        self.store = dict(store)
        self.queue: list = []
        self.target_system, self.target_component = 1, 1
        self.heartbeat = heartbeat
        self.mav = SimpleNamespace(
            param_request_read_send=self._request_read,
            serial_control_send=self._serial,
            command_long_send=self._command,
        )
        self.closed = False

    def _request_read(self, sysid, compid, name, index) -> None:
        name = name.decode()
        if name in self.store:
            value, ptype = self.store[name]
            raw = value
            if ptype == PARAM_TYPE_INT32:
                raw = struct.unpack("<f", struct.pack("<i", int(value)))[0]
            self.queue.append(
                SimpleNamespace(
                    get_type=lambda: "PARAM_VALUE", param_id=name, param_value=raw, param_type=ptype
                )
            )

    def _serial(self, dev, flags, timeout, baud, count, data) -> None:
        text = bytes(data[:count]).decode()
        for line in text.splitlines():
            parts = line.split()
            if parts[:2] == ["param", "set"]:
                name, value = parts[2], float(parts[3])
                # NSH decides the type from the stored parameter; the fake decides from the text
                is_float = "." in parts[3] or "e" in parts[3]
                default = PARAM_TYPE_FLOAT if is_float else PARAM_TYPE_INT32
                ptype = self.store.get(name, (0.0, default))[1]
                self.store[name] = (value, ptype)
                echo = f"{name}: curr: 0 -> new: {value}\n".encode()
                self.queue.append(
                    SimpleNamespace(
                        get_type=lambda: "SERIAL_CONTROL", count=len(echo), data=list(echo)
                    )
                )

    def _command(self, *args) -> None:
        self.queue.append(
            SimpleNamespace(
                get_type=lambda: "AUTOPILOT_VERSION",
                flight_sw_version=(1 << 24) | (17 << 16),
                board_version=63,
            )
        )

    def recv_match(self, type=None, blocking=False, timeout=None):
        if type == "HEARTBEAT":
            if not self.heartbeat:
                return None
            return SimpleNamespace(
                get_srcComponent=lambda: 1, get_srcSystem=lambda: 1,
                base_mode=32 | 64, custom_mode=0, type=2, autopilot=12,
            )
        for i, msg in enumerate(self.queue):
            if msg.get_type() == type:
                return self.queue.pop(i)
        return None

    def close(self) -> None:
        self.closed = True


@pytest.fixture(autouse=True)
def fast_link(monkeypatch: pytest.MonkeyPatch) -> None:
    """The fake answers at once, so do not sit in the settle and timeout loops."""
    monkeypatch.setattr(board, "SHELL_SETTLE_S", 0.0)
    monkeypatch.setattr(board, "PARAM_TIMEOUT_S", 0.01)


def scenario_json(name: str) -> dict:
    return json.loads((SCENARIOS / f"{name}.json").read_text())


def test_int32_travels_as_raw_bits() -> None:
    bits_of_101 = struct.unpack("<f", struct.pack("<i", 101))[0]
    assert board.decode_param_value(bits_of_101, PARAM_TYPE_INT32) == 101.0
    assert board.decode_param_value(0.384, PARAM_TYPE_FLOAT) == 0.384


def test_shell_commands_write_ints_as_ints_then_save() -> None:
    cmds = board.shell_commands(
        {"CA_ROTOR0_AX": 0.25881904, "CA_ROTOR_COUNT": 10}, {"CA_ROTOR_COUNT": PARAM_TYPE_INT32}
    )
    assert cmds == [
        "param set CA_ROTOR0_AX 0.25881904",
        "param set CA_ROTOR_COUNT 10",
        "param save",
    ]


def test_is_pixhawk_by_usb_id_or_name() -> None:
    assert board.is_pixhawk(0x3185, 0x0035, "USB Serial Device")  # fmu-v6x nsh defconfig
    assert board.is_pixhawk(None, None, "PX4 FMU v6X.x")
    assert not board.is_pixhawk(0x0403, 0x6001, "FTDI USB Serial")


def test_firmware_string_unpacks_px4_version() -> None:
    msg = SimpleNamespace(flight_sw_version=(1 << 24) | (17 << 16))
    assert board.firmware_string(msg) == "1.17.0"


def test_push_backs_up_then_writes_and_verifies(tmp_path: Path) -> None:
    link = FakeLink(
        {
            "CA_ROTOR0_AX": (0.0, PARAM_TYPE_FLOAT),
            "CA_ROTOR_COUNT": (4, PARAM_TYPE_INT32),
            "CA_ROTOR0_CT": (6.5, PARAM_TYPE_FLOAT),
        }
    )
    params = {"CA_ROTOR0_AX": 0.2588, "CA_ROTOR_COUNT": 10, "CA_ROTOR0_CT": 6.5}
    types = {"CA_ROTOR_COUNT": PARAM_TYPE_INT32}
    res = board.push_params(link, "COM7", params, types, tmp_path, "demo")
    assert res.sent == 3 and res.verified == 3 and res.mismatches == []
    assert res.changed == ["CA_ROTOR0_AX", "CA_ROTOR_COUNT"]  # CT was already 6.5
    assert link.store["CA_ROTOR_COUNT"] == (10.0, PARAM_TYPE_INT32)
    backup = read_params_file(res.backup)
    by_name = {e.name: e for e in backup.entries}
    assert by_name["CA_ROTOR_COUNT"].value == 4
    assert by_name["CA_ROTOR_COUNT"].type_code == PARAM_TYPE_INT32
    assert by_name["CA_ROTOR0_AX"].value == 0.0


def test_read_status_reports_flags_and_firmware() -> None:
    st = board.read_status(FakeLink({}), "COM7")
    assert st.connected and st.hil and not st.armed
    assert st.firmware == "1.17.0" and st.board_id == 63


@pytest.fixture()
def client(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> TestClient:
    monkeypatch.setattr(app_module, "BOARD_BACKUP_DIR", tmp_path / "board")
    with TestClient(app) as c:
        yield c


FTDI = {"device": "COM3", "description": "FTDI", "vid": 0x0403, "pid": 0x6001, "pixhawk": False}
PIXHAWK = {
    "device": "COM7", "description": "PX4 FMU v6X.x", "vid": 0x3185, "pid": 0x0035, "pixhawk": True,
}


def test_status_without_board_is_fast_and_lists_ports(client: TestClient, monkeypatch) -> None:
    monkeypatch.setattr(board, "list_ports", lambda: [FTDI])
    r = client.get("/api/board/status").json()
    assert r["connected"] is False and r["ports"][0]["device"] == "COM3"
    assert "no Pixhawk" in r["message"]


def test_status_and_push_through_fake_link(client: TestClient, monkeypatch) -> None:
    link = FakeLink({})
    monkeypatch.setattr(board, "list_ports", lambda: [PIXHAWK])
    monkeypatch.setattr(board, "connect", lambda port, timeout_s=5.0: link)
    st = client.get("/api/board/status").json()
    assert st["connected"] and st["port"] == "COM7" and st["firmware"] == "1.17.0"
    assert link.closed

    body = {"scenario": scenario_json("baseline_dihedral30"), "concept": "fully_actuated"}
    r = client.post("/api/board/push", json=body)
    assert r.status_code == 200, r.text
    res = r.json()
    assert res["sent"] == res["verified"] > 80 and res["mismatches"] == []
    assert link.store["CA_ROTOR_COUNT"][0] == 10 and link.store["CA_METHOD"][0] == 0
    assert Path(res["backup"]).is_file() and res["backup"].endswith("_before.params")


def test_push_without_board_is_404(client: TestClient, monkeypatch) -> None:
    monkeypatch.setattr(board, "list_ports", lambda: [])
    r = client.post("/api/board/push", json={"scenario": scenario_json("baseline_dihedral30")})
    assert r.status_code == 404
