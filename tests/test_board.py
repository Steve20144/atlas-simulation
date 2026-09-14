"""Pixhawk USB module against a fake MAVLink link: INT32 decoding, shell writes, backup and
read-back, and the /api/board endpoints with the link monkeypatched (no hardware in CI)."""

from __future__ import annotations

import json
import struct
import time
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
            log_request_list_send=self._log_list,
            log_request_data_send=self._log_data,
            log_request_end_send=lambda sysid, compid: None,
        )
        self.closed = False
        # id -> (time_utc, bytes); the first full-range data request drops one chunk so the
        # gap re-request path runs
        self.logs: dict[int, tuple[int, bytes]] = {}
        self.dropped_once = False

    def _log_list(self, sysid, compid, start, end) -> None:
        ids = sorted(self.logs)
        for lid in ids:
            t, data = self.logs[lid]
            self.queue.append(
                SimpleNamespace(
                    get_type=lambda: "LOG_ENTRY", id=lid, num_logs=len(ids),
                    last_log_num=ids[-1], time_utc=t, size=len(data),
                )
            )

    def _log_data(self, sysid, compid, lid, ofs, count) -> None:
        data = self.logs[lid][1]
        stop = len(data) if count == 0xFFFFFFFF else min(ofs + count, len(data))
        for k, o in enumerate(range(ofs, stop, board.LOG_CHUNK)):
            if count == 0xFFFFFFFF and k == 1 and not self.dropped_once:
                self.dropped_once = True
                continue
            chunk = data[o : min(o + board.LOG_CHUNK, stop)]
            self.queue.append(
                SimpleNamespace(
                    get_type=lambda: "LOG_DATA", id=lid, ofs=o, count=len(chunk),
                    data=list(chunk) + [0] * (board.LOG_CHUNK - len(chunk)),
                )
            )

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
        types = list(type) if isinstance(type, (list, tuple)) else [type]
        for i, msg in enumerate(self.queue):
            if msg.get_type() in types:
                return self.queue.pop(i)
        if "HEARTBEAT" in types and len(types) == 1:
            if not self.heartbeat:
                return None
            return SimpleNamespace(
                get_srcComponent=lambda: 1, get_srcSystem=lambda: 1,
                base_mode=32 | 64, custom_mode=0, type=2, autopilot=12,
            )
        return None

    def close(self) -> None:
        self.closed = True


@pytest.fixture(autouse=True)
def fast_link(monkeypatch: pytest.MonkeyPatch) -> None:
    """The fake answers at once, so do not sit in the settle and timeout loops."""
    monkeypatch.setattr(board, "SHELL_SETTLE_S", 0.0)
    monkeypatch.setattr(board, "PARAM_TIMEOUT_S", 0.01)
    monkeypatch.setattr(board, "LOG_RECV_TIMEOUT_S", 0.0)
    monkeypatch.setattr(board, "LOG_STALL_S", 0.0)
    monkeypatch.setattr(board, "LOG_LIST_TIMEOUT_S", 1.0)


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


def test_pull_latest_log_picks_newest_flight_and_fills_gaps(tmp_path: Path) -> None:
    """Newest = highest UTC time then id (a bench session with time 0 does not win over an older
    id with GPS time); a dropped chunk is re-requested from the first gap; bytes match."""
    flight = bytes(range(256)) * 9 + b"ULog tail"  # 2313 bytes, not a chunk multiple
    link = FakeLink({})
    link.logs = {3: (0, b"sess" * 60), 5: (1_757_700_000, flight), 7: (0, b"later bench" * 20)}
    res = board.pull_latest_log(link, "COM7", tmp_path / "logs")
    assert res.log_id == 5 and res.num_logs == 3 and res.size == len(flight)
    assert link.dropped_once and Path(res.path).read_bytes() == flight
    assert Path(res.path).name == "20250912_1800_flight_log005.ulg"
    link.logs = {2: (0, b"x" * 100)}
    res2 = board.pull_latest_log(link, "COM7", tmp_path / "logs")
    assert res2.log_id == 2 and Path(res2.path).name.endswith("_board_log002.ulg")
    with pytest.raises(board.BoardError, match="no logs"):
        link.logs = {}
        board.pull_latest_log(link, "COM7", tmp_path / "logs")


def test_pull_log_endpoint_saves_and_serves_the_file(
    client: TestClient, monkeypatch, tmp_path: Path
) -> None:
    link = FakeLink({})
    link.logs = {1: (1_757_700_000, b"ULog" * 300)}
    monkeypatch.setattr(board, "list_ports", lambda: [PIXHAWK])
    monkeypatch.setattr(board, "connect", lambda port, timeout_s=5.0: link)
    monkeypatch.setattr(app_module, "BOARD_LOGS_DIR", tmp_path / "logs")
    r = client.post("/api/board/pull_log", json={"port": "COM7"})
    assert r.status_code == 200, r.text
    res = r.json()
    assert res["log_id"] == 1 and res["size"] == 1200 and res["url"].endswith(".ulg")
    assert Path(res["path"]).is_file()
    got = client.get(res["url"])
    assert got.status_code == 200 and got.content == b"ULog" * 300
    assert client.get("/api/board/logs/nope.ulg").status_code == 404
    assert client.get("/api/board/logs/..%2Fx.ulg").status_code in (400, 404)


def _feed_messages(armed: bool) -> list:
    hb = SimpleNamespace(
        get_type=lambda: "HEARTBEAT", get_srcComponent=lambda: 1,
        base_mode=(128 if armed else 0) | 64, custom_mode=(7 << 24), type=2, autopilot=12,
    )
    chans = {f"chan{i}_raw": 1500 for i in range(1, 9)}
    rc = SimpleNamespace(get_type=lambda: "RC_CHANNELS", chancount=8, **chans)
    rc.chan3_raw = 1200
    mc = SimpleNamespace(get_type=lambda: "MANUAL_CONTROL", x=0, y=0, z=230, r=0)
    at = SimpleNamespace(get_type=lambda: "ATTITUDE_TARGET", thrust=0.35)
    zeros = {f"servo{i}_raw": 0 for i in range(1, 17)}
    main = SimpleNamespace(get_type=lambda: "SERVO_OUTPUT_RAW", port=0, **zeros)
    for i in range(1, 9):
        setattr(main, f"servo{i}_raw", 1300 + i)
    aux = SimpleNamespace(get_type=lambda: "SERVO_OUTPUT_RAW", port=1, **zeros)
    aux.servo1_raw, aux.servo2_raw = 1400, 1410
    return [hb, rc, mc, at, main, aux]


def test_thrust_feed_records_stick_throttle_thrust_and_outputs(monkeypatch) -> None:
    """MANUAL_CONTROL z 230 -> throttle 0.23 (z = (throttle + 1) * 500), ATTITUDE_TARGET.thrust,
    SERVO_OUTPUT_RAW ports 0 and 1 -> MAIN / AUX pulse widths, armed from HEARTBEAT."""
    from vectra.px4 import telemetry

    monkeypatch.setattr(telemetry, "SAMPLE_PERIOD_S", 0.0)
    rc3 = {"RC3_MIN": (1000.0, PARAM_TYPE_FLOAT), "RC3_MAX": (2000.0, PARAM_TYPE_FLOAT)}
    link = FakeLink(
        {**rc3, "MPC_THR_HOVER": (0.5, PARAM_TYPE_FLOAT), "RC_MAP_THROTTLE": (3, PARAM_TYPE_INT32)}
    )
    feed = telemetry.ThrustFeed(link, "COM7")
    feed.start()
    link.queue.extend(_feed_messages(armed=True))
    deadline = time.time() + 3.0
    while time.time() < deadline and feed.snapshot()["counts"].get("SERVO_OUTPUT_RAW", 0) < 2:
        time.sleep(0.02)
    snap = feed.snapshot()
    feed.stop()
    lt = snap["latest"]
    assert lt["rc_raw"] == 1200 and lt["throttle"] == pytest.approx(0.23)
    assert lt["throttle_source"] == "MANUAL_CONTROL"
    assert lt["thrust_sp"] == pytest.approx(0.35) and lt["armed"] is True
    assert lt["main_us"] == [1301, 1302, 1303, 1304, 1305, 1306, 1307, 1308]
    assert lt["aux_us"] == [1400, 1410]
    assert snap["params"]["MPC_THR_HOVER"] == 0.5 and snap["params"]["MPC_MANTHR_MIN"] is None
    assert len(snap["samples"]) >= 1 and snap["samples"][-1]["thrust_sp"] == pytest.approx(0.35)
    assert link.closed
    # without MANUAL_CONTROL the stick position comes from the raw channel and the RC3 range
    feed2 = telemetry.ThrustFeed(FakeLink(rc3), "COM7")
    feed2.params = {"RC3_MIN": 1000.0, "RC3_MAX": 2000.0, "RC_MAP_THROTTLE": None}
    feed2._ingest(_feed_messages(False)[1])
    assert feed2.latest["throttle"] == pytest.approx(0.2)
    assert feed2.latest["throttle_source"] == "RC_CHANNELS"


def test_thrust_feed_endpoints_hold_the_board_lock(client: TestClient, monkeypatch) -> None:
    from vectra.px4 import telemetry

    monkeypatch.setattr(telemetry, "SAMPLE_PERIOD_S", 0.0)
    link = FakeLink({})
    link.queue.extend(_feed_messages(armed=False))
    monkeypatch.setattr(board, "list_ports", lambda: [PIXHAWK])
    monkeypatch.setattr(board, "connect", lambda port, timeout_s=5.0: link)
    r = client.post("/api/board/thrust_feed/start", json={"port": "COM7"})
    assert r.status_code == 200, r.text
    assert r.json()["running"] is True and r.json()["port"] == "COM7"
    # the feed owns the link: other board operations must not open the port meanwhile
    assert client.get("/api/board/status?port=COM7").status_code == 409
    deadline = time.time() + 3.0
    while time.time() < deadline:
        if client.get("/api/board/thrust_feed").json()["latest"]["thrust_sp"] is not None:
            break
        time.sleep(0.02)
    snap = client.get("/api/board/thrust_feed?seconds=5").json()
    assert snap["latest"]["thrust_sp"] == pytest.approx(0.35) and snap["latest"]["armed"] is False
    assert client.post("/api/board/thrust_feed/stop", json={}).json() == {"stopped": True}
    assert link.closed and not board.LOCK.locked()
    assert client.get("/api/board/thrust_feed").status_code == 404
    assert client.post("/api/board/thrust_feed/stop", json={}).json() == {"stopped": False}


def test_read_param_endpoint(client: TestClient, monkeypatch) -> None:
    link = FakeLink(
        {"SYS_HITL": (0, PARAM_TYPE_INT32), "MPC_THR_HOVER": (0.409, PARAM_TYPE_FLOAT)}
    )
    monkeypatch.setattr(board, "list_ports", lambda: [PIXHAWK])
    monkeypatch.setattr(board, "connect", lambda port, timeout_s=5.0: link)
    r = client.get("/api/board/param?name=MPC_THR_HOVER").json()
    assert r["name"] == "MPC_THR_HOVER" and r["type_code"] == PARAM_TYPE_FLOAT
    assert r["value"] == pytest.approx(0.409, abs=1e-6)
    assert client.get("/api/board/param?name=SYS_HITL").json()["value"] == 0
    assert client.get("/api/board/param?name=NOPE_X").status_code == 503
    assert client.get("/api/board/param?name=bad").status_code == 422
    assert link.closed and not board.LOCK.locked()


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
