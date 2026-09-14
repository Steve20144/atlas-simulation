"""scripts/flight_debug.py against the golden logs: the report runs, the sections exist and the
known signatures fire where the logs show them (log 36: eight wing motors at zero all flight)."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
FIX = REPO / "tests" / "fixtures"


def _load():
    spec = importlib.util.spec_from_file_location(
        "flight_debug", REPO / "scripts" / "flight_debug.py"
    )
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    return mod


def test_log40_report_sections_and_stick_to_thrust(tmp_path: Path) -> None:
    fd = _load()
    rep = fd.analyse(FIX / "log_40_2026-9-9-15-09-54.ulg")
    assert rep["log"]["params"]["SYS_AUTOSTART"] == 4001 and rep["log"]["params"]["SYS_HITL"] == 0
    arm = rep["arming"]
    assert arm["nav_states"] == ["STAB"] and arm["arming_reasons"] == ["rc_switch"]
    assert arm["armed_windows_s"] and arm["rc_calibration_in_progress_ever"] is False
    st = rep["stick_to_thrust"]
    assert st["rc_throttle_us"]["min"] == 1001 and 0.5 < st["thrust_per_stick_below_40pct"] < 1.2
    assert len(rep["motors"]["actuator_motors_armed"]) == 10
    assert rep["motors"]["allocator_armed"]["motor_saturation_lower_pct"] > 50
    text = fd.render(rep)
    sections = ("## log", "## events", "## arming", "## stick -> thrust", "## motors")
    for section in sections:
        assert section in text
    out = tmp_path / "r.json"
    out.write_text(json.dumps(rep), encoding="utf-8")
    assert json.loads(out.read_text(encoding="utf-8"))["duration_s"] == rep["duration_s"]


def test_log36_flags_dead_wing_motors_and_kill_switch() -> None:
    fd = _load()
    rep = fd.analyse(FIX / "log_36_2026-9-9-14-11-36.ulg")
    per = rep["motors"]["actuator_motors_armed"]
    assert all(m["at_zero_pct"] == 100.0 for m in per[:8]) and per[8]["max"] > 0.1
    assert rep["arming"]["kill_ever"] is True
    joined = " ".join(rep["findings"])
    assert "motors 0, 1, 2, 3, 4, 5, 6, 7 never left 0" in joined
    assert "kill switch" in joined
    assert "torque setpoint achieved only 0.0%" in joined
