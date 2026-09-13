"""Pixhawk USB module against a fake MAVLink link: INT32 decoding, shell writes, backup and
read-back, and the /api/board endpoints with the link monkeypatched (no hardware in CI)."""

from __future__ import annotations

import json
import struct
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi.testclient import TestClient

import vectra.api.app as app_module
from vectra.api.app import app
from vectra.core.params_px4 import PARAM_TYPE_FLOAT, PARAM_TYPE_INT32, read_params_file
from vectra.px4 import board
from vectra.scenario import Scenario

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

    def _command(self, sysid, compid, cmd, conf, *params) -> None:
        self.commands = getattr(self, "commands", []) + [(cmd, params[0])]
        if cmd == board.CMD_PREFLIGHT_REBOOT_SHUTDOWN:
            self.queue.append(SimpleNamespace(get_type=lambda: "COMMAND_ACK", result=0))
            return
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


def test_read_status_reports_flags_firmware_and_sys_hitl() -> None:
    st = board.read_status(FakeLink({"SYS_HITL": (1, PARAM_TYPE_INT32)}), "COM7")
    assert st.connected and st.hil and not st.armed
    assert st.firmware == "1.17.0" and st.board_id == 63 and st.sys_hitl == 1
    assert board.read_status(FakeLink({}), "COM7").sys_hitl is None


def test_set_param_uses_the_board_type_and_flags_reboot() -> None:
    link = FakeLink({"SYS_HITL": (0, PARAM_TYPE_INT32)})
    res = board.set_param(link, "SYS_HITL", 1)
    assert res.before == 0 and res.after == 1 and res.verified and res.reboot_required
    assert link.store["SYS_HITL"] == (1.0, PARAM_TYPE_INT32)
    with pytest.raises(board.BoardError):
        board.set_param(link, "NOT_A_PARAM", 1)


def test_reboot_sends_preflight_reboot_with_param1_one() -> None:
    link = FakeLink({})
    board.reboot(link)
    assert link.commands[-1] == (board.CMD_PREFLIGHT_REBOOT_SHUTDOWN, 1)


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


def test_flight_params_restores_hitl_set_from_backup() -> None:
    """SYS_HITL 0 alone is undone at boot by airframe 1001 (param set SYS_HITL 1), so the flight
    set must move SYS_AUTOSTART and the rest of the HITL set back to the backup's values."""
    from vectra.export.params import latest_backup_params
    from vectra.px4.flight import flight_params

    base = read_params_file(latest_backup_params())
    fs = flight_params(base, None)
    p, src = fs.params, fs.sources
    assert p["SYS_HITL"] == 0 and src["SYS_HITL"] == "hitl_undo"
    assert p["SYS_AUTOSTART"] == 4001 and src["SYS_AUTOSTART"] == "backup"
    assert p["EKF2_EN"] == 1 and p["SYS_HAS_MAG"] == 1 and p["SYS_HAS_BARO"] == 1
    assert p["CBRK_SUPPLY_CHK"] == 0 and p["GPS_1_CONFIG"] == 201
    assert p["CAL_ACC0_ID"] == 5767194 and p["CAL_GYRO0_ID"] == 5767194
    assert p["CAL_ACC1_ID"] == 2818066 and fs.types["CAL_ACC0_XOFF"] == PARAM_TYPE_FLOAT
    assert p["CAL_ACC0_XOFF"] == pytest.approx(-0.035091143)
    assert all(p[f"HIL_ACT_FUNC{i}"] == 0 for i in range(1, 11))
    assert any("controller gains" in w for w in fs.warnings)
    assert not any("recalibrate" in w for w in fs.warnings)

    # with the scenario the HITL gains come back from the backup too (THR_MDL_FAC flew as 0)
    sc = Scenario.model_validate(json.loads((SCENARIOS / "baseline_dihedral30.json").read_text()))
    fs2 = flight_params(base, sc)
    assert fs2.params["THR_MDL_FAC"] == 0 and fs2.sources["THR_MDL_FAC"] == "backup"

    # no backup at all: PX4 defaults, the sim IMU id cleared and a recalibration warning
    fs3 = flight_params(None, None)
    assert fs3.params["SYS_AUTOSTART"] == 4001 and fs3.sources["SYS_AUTOSTART"] == "default"
    assert fs3.params["CAL_ACC0_ID"] == 0 and "CAL_ACC0_XOFF" not in fs3.params
    assert any("recalibrate" in w for w in fs3.warnings)
    assert any("SYS_AUTOSTART" in w for w in fs3.warnings)


def test_flight_endpoint_writes_the_set_and_backs_up(client: TestClient, monkeypatch) -> None:
    hitl_board = {
        "SYS_HITL": (1, PARAM_TYPE_INT32),
        "SYS_AUTOSTART": (1001, PARAM_TYPE_INT32),
        "EKF2_EN": (0, PARAM_TYPE_INT32),
        "CAL_ACC0_ID": (1310988, PARAM_TYPE_INT32),
        "HIL_ACT_FUNC1": (101, PARAM_TYPE_INT32),
        "THR_MDL_FAC": (1.0, PARAM_TYPE_FLOAT),
    }
    link = FakeLink(hitl_board)
    monkeypatch.setattr(board, "list_ports", lambda: [PIXHAWK])
    monkeypatch.setattr(board, "connect", lambda port, timeout_s=5.0: link)
    body = {"scenario": scenario_json("baseline_dihedral30")}
    r = client.post("/api/board/flight", json=body)
    assert r.status_code == 200, r.text
    res = r.json()
    assert res["reboot_required"] and res["mismatches"] == []
    assert res["params"]["SYS_HITL"] == 0 and res["sources"]["SYS_AUTOSTART"] == "backup"
    assert link.store["SYS_HITL"][0] == 0 and link.store["SYS_AUTOSTART"][0] == 4001
    assert link.store["EKF2_EN"][0] == 1 and link.store["HIL_ACT_FUNC1"][0] == 0
    assert link.store["CAL_ACC0_ID"][0] == 5767194 and link.store["THR_MDL_FAC"][0] == 0
    assert "SYS_HITL" in res["changed"] and "SYS_AUTOSTART" in res["changed"]
    assert Path(res["backup"]).name.endswith("_hitl_off_before.params")
    assert client.get("/api/board/status").json()["sys_hitl"] == 0
    r = client.post("/api/board/flight", json={"base": "nope.params"})
    assert r.status_code == 400


def test_hil_toggle_and_reboot_endpoints(client: TestClient, monkeypatch) -> None:
    link = FakeLink({"SYS_HITL": (0, PARAM_TYPE_INT32)})
    monkeypatch.setattr(board, "list_ports", lambda: [PIXHAWK])
    monkeypatch.setattr(board, "connect", lambda port, timeout_s=5.0: link)
    r = client.post("/api/board/param", json={"name": "SYS_HITL", "value": 1}).json()
    assert r["verified"] and r["reboot_required"] and r["before"] == 0 and r["after"] == 1
    assert client.get("/api/board/status").json()["sys_hitl"] == 1
    assert client.post("/api/board/param", json={"name": "bad name", "value": 1}).status_code == 422
    assert client.post("/api/board/param", json={"name": "NOPE_X", "value": 1}).status_code == 503
    r = client.post("/api/board/reboot", json={}).json()
    assert r == {"port": "COM7", "rebooted": True}
