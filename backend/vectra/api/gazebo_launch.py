"""Launch, watch, fly and stop the Gazebo harness from the app (SITL in gz sim or HITL in Classic).

The backend runs natively on the Windows machine that owns the Pixhawk (PLAN.md section 3), so it
drives WSL through ``wsl.exe``: the launcher ``scripts/wsl/vectra_gazebo.sh`` installs the harness
into the PX4 checkout and starts the sim; everything around it (stale sims, readiness, takeoff,
land, log copy, live probe) lives here so the app is the one code path. What was learnt the hard
way and is baked in:

- a launch first stops whatever simulator is up in that distro (any owner), never refuses;
- PX4's shell gets an open pipe on stdin: with /dev/null it reads EOF and reprints its prompt in a
  busy loop (6 GB of console log in 25 minutes);
- the console log is truncated per launch and read from its tail only;
- readiness is PX4's ``Ready for takeoff!`` line, not a fixed sleep;
- WSLg's copy-mode fault (grey window titled [WARN:COPY MODE]) is detected from weston.log and
  reported; only ``wsl --shutdown`` clears it, which :func:`wsl_shutdown` does on request;
- a wedged commander (armed per ``commander status``, standby per ``vehicle_status``) cannot be
  reset in place; :func:`probe` shows the stale arming timestamp and the fix is stop and launch.

Inside Docker or on a machine without WSL the endpoints answer 501 with the command to run by
hand. ``VECTRA_GAZEBO_DRY_RUN=1`` returns commands without starting anything (tests).
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Literal

from vectra.export.gazebo import export_gazebo
from vectra.export.gazebo_classic_hitl import export_gazebo_classic_hitl
from vectra.scenario import Scenario

Mode = Literal["sitl", "hitl"]

# the repo is reachable in both distros as ~/utopia/vibe-coded (a symlink; the space in
# "Utopia Labs" breaks quoting through wsl.exe), see scripts/vectra_menu.py
WSL_REPO = os.environ.get("VECTRA_WSL_REPO", "~/utopia/vibe-coded")
DISTRO: dict[str, str] = {
    "sitl": os.environ.get("VECTRA_WSL_SITL_DISTRO", "Ubuntu-24.04"),  # gz sim Harmonic
    "hitl": os.environ.get("VECTRA_WSL_HITL_DISTRO", "Ubuntu-22.04"),  # Gazebo Classic 11
}
LAUNCHER = "scripts/wsl/vectra_gazebo.sh"
ROOTFS = "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs"
READY_MARK = "Ready for takeoff!"
COPY_MODE_MARK = "rdp_allocate_shared_memory.*Input/output error"
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
# PX4 navigation_state values (msg/versioned/VehicleStatus.msg, v1.17.0)
NAV_STATE = {
    0: "Manual", 1: "Altitude", 2: "Position", 3: "Mission", 4: "Hold", 5: "Return", 10: "Acro",
    12: "Descend", 13: "Termination", 14: "Offboard", 15: "Stabilized", 17: "Takeoff", 18: "Land",
    19: "Follow", 20: "Precision land", 21: "Orbit", 22: "VTOL takeoff",
}  # fmt: skip

_proc: subprocess.Popen[bytes] | None = None
_state: dict[str, Any] = {
    "mode": None, "log": None, "command": None, "harness": None, "headless": False,
    "started": None, "wslg_copy_mode": False,
}  # fmt: skip


def available() -> bool:
    return sys.platform == "win32" and shutil.which("wsl.exe") is not None


def dry_run() -> bool:
    return os.environ.get("VECTRA_GAZEBO_DRY_RUN") == "1"


def wsl_path(path: Path, repo_root: Path) -> str:
    """Path spelled for WSL: through the ~/utopia/vibe-coded symlink when it is inside the repo,
    otherwise wslpath (or, without WSL, a plain /mnt/<drive>/... spelling for a dry run)."""
    resolved = path.resolve()
    try:
        return f"{WSL_REPO}/{resolved.relative_to(repo_root.resolve()).as_posix()}"
    except ValueError:
        pass
    if available() and not dry_run():
        out = subprocess.run(["wsl.exe", "--", "wslpath", "-a", str(resolved)],
                             capture_output=True, text=True, check=False)
        if out.stdout.strip():
            return out.stdout.strip()
    drive, tail = resolved.drive.rstrip(":").lower(), resolved.as_posix()[len(resolved.drive):]
    return f"/mnt/{drive}{tail}" if drive else resolved.as_posix()


def bash_path(p: str) -> str:
    """Quote for bash while keeping a leading ~ expandable (no spaces in these paths by design)."""
    return '"$HOME"' + p[1:] if p.startswith("~/") else p


def _wsl(mode: Mode, inner: str, timeout: float = 60) -> subprocess.CompletedProcess[str]:
    """Run one bash command line in the distro of ``mode``. Arguments go through wsl.exe as a
    list, so nothing on the Windows side expands ``$``; bash inside WSL does."""
    if dry_run() or not available():
        return subprocess.CompletedProcess(["wsl.exe"], 0, stdout="", stderr="")
    return subprocess.run(["wsl.exe", "-d", DISTRO[mode], "--", "bash", "-lc", inner],
                          capture_output=True, text=True, timeout=timeout, check=False,
                          encoding="utf-8", errors="replace")


def wsl_command(mode: Mode, harness_wsl: str, headless: bool = False) -> list[str]:
    inner = (
        ("HEADLESS=1 " if headless else "")
        + f"bash {bash_path(WSL_REPO + '/' + LAUNCHER)} --mode {mode} --yes "
        f"--harness {bash_path(harness_wsl)}"
    )
    return ["wsl.exe", "-d", DISTRO[mode], "--", "bash", "-lc", inner]


def _stop_distro(mode: Mode) -> None:
    """Take down every simulator process in that distro, whoever started it."""
    inner = f"bash {bash_path(WSL_REPO + '/' + LAUNCHER)} --stop"
    _wsl(mode, inner, timeout=60)


def wslg_copy_mode() -> bool:
    """WSLg's shared-memory channel failed: every window is grey, titled [WARN:COPY MODE]."""
    r = _wsl("sitl", f"grep -c '{COPY_MODE_MARK}' /mnt/wslg/weston.log 2>/dev/null || true",
             timeout=20)
    return r.stdout.strip() not in ("", "0")


def wsl_shutdown() -> dict[str, Any]:
    """``wsl --shutdown``: stops both distros and restarts WSLg, the only cure for copy mode.
    Any HITL USB attach is dropped with it."""
    global _proc
    if dry_run() or not available():
        return status() | {"wsl_shutdown": True, "dry_run": True}
    if _proc is not None and _proc.poll() is None:
        _proc.terminate()
    _proc = None
    subprocess.run(["wsl.exe", "--shutdown"], capture_output=True, timeout=60, check=False)
    _state["wslg_copy_mode"] = False
    return status() | {"wsl_shutdown": True}


def _start(cmd: list[str], log: Path, truncate: bool) -> None:
    global _proc
    # stdin stays an open pipe, never written: with DEVNULL the PX4 shell (pxh) reads EOF, reprints
    # its prompt in a busy loop and wrote 6 GB of '[2Kpxh> ' into this log in 25 minutes.
    with open(log, "wb" if truncate else "ab") as fh:
        _proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.PIPE)
    _state["started"] = time.time()


def launch(mode: Mode, scenario: Scenario, exports_dir: Path, repo_root: Path,
           headless: bool = False) -> dict[str, Any]:
    """Export the harness for ``scenario``, stop whatever sim is up in that distro, start the
    launcher. Returns the status right after the start (``ready`` turns true later)."""
    if mode == "sitl":
        harness = Path(export_gazebo(scenario, exports_dir / "gazebo")["root"])
    else:
        harness = Path(export_gazebo_classic_hitl(scenario, exports_dir / "gazebo_hitl")["root"])
    cmd = wsl_command(mode, wsl_path(harness, repo_root), headless)
    log = exports_dir / "logs" / f"gazebo_{mode}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    _state.update(mode=mode, log=str(log), command=" ".join(cmd), harness=str(harness),
                  headless=headless, started=None)
    if dry_run():
        return status() | {"dry_run": True}
    if not available():
        raise RuntimeError("wsl.exe is not available here; run the command shown by hand")
    stop()  # a stale or wedged sim never blocks a launch
    _state["wslg_copy_mode"] = False if headless else wslg_copy_mode()
    _start(cmd, log, truncate=True)
    return status()


def _log_lines() -> list[str]:
    if not _state["log"] or not Path(_state["log"]).exists():
        return []
    raw = Path(_state["log"]).read_bytes()[-200_000:].decode("utf-8", errors="replace")
    out: list[str] = []
    for ln in _ANSI.sub("", raw).splitlines():
        ln = ln.strip()
        if ln and "gc_relay]" not in ln and not ln.startswith("pxh>") and "pxh>" not in ln[:12]:
            out.append(ln)
    return out


def status(tail: int = 12) -> dict[str, Any]:
    running = _proc is not None and _proc.poll() is None
    lines = _log_lines()
    ready = running and any(READY_MARK in ln for ln in lines)
    started = _state["started"]
    return {
        "available": available(),
        "running": running,
        "ready": ready,
        "mode": _state["mode"],
        "headless": _state["headless"],
        "harness": _state["harness"],
        "command": _state["command"],
        "log": _state["log"],
        "tail": lines[-tail:],
        "uptime_s": round(time.time() - started, 1) if running and started else None,
        "wslg_copy_mode": bool(_state["wslg_copy_mode"]),
        "returncode": None if _proc is None else _proc.poll(),
    }


def stop() -> dict[str, Any]:
    """Take down whichever simulator is up (launcher --stop in both distros) and drop our handle."""
    global _proc
    if dry_run() or not available():
        _proc = None
        return status() | {"stopped": True, "dry_run": dry_run()}
    for mode in ("sitl", "hitl"):
        _stop_distro(mode)
    if _proc is not None and _proc.poll() is None:
        _proc.terminate()
    _proc = None
    _state["started"] = None
    return status() | {"stopped": True}


def relaunch() -> dict[str, Any]:
    """Start the launcher again for the harness already exported (no re-export)."""
    mode, harness = _state["mode"], _state["harness"]
    if not mode or not harness:
        raise RuntimeError("nothing to relaunch")
    cmd = wsl_command(mode, wsl_path(Path(harness), Path(harness).parents[2]),
                      bool(_state["headless"]))
    _start(cmd, Path(_state["log"]), truncate=False)
    return status()


def reset() -> dict[str, Any]:
    """Disarm and put the model back where it spawned; in HITL, if flight termination has latched
    (past 60 deg of roll or pitch) do the hard path: stop, reboot the board, relaunch."""
    mode, harness = _state["mode"], _state["harness"]
    if not mode or not harness:
        raise RuntimeError("no session to reset")
    if dry_run() or not available():
        return status() | {"reset": True, "dry_run": True}
    inner = (f"bash {bash_path(WSL_REPO + '/' + LAUNCHER)} --reset --mode {mode} "
             f"--harness {bash_path(wsl_path(Path(harness), Path(harness).parents[2]))}")
    r = _wsl(mode, inner, timeout=180)
    out = _ANSI.sub("", r.stdout + r.stderr).strip().splitlines()[-4:]
    if r.returncode == 3:  # termination latched: the board must reboot, which drops the serial link
        stop()
        dev_cmd = ('D=$(ls /dev/ttyACM* | head -1); '
                   'python3 "$HOME"/utopia/vibe-coded/scripts/px4_board.py --dev "$D" shell reboot')
        _wsl(mode, dev_cmd, timeout=60)
        time.sleep(25)
        return relaunch() | {"reset": True, "rebooted": True, "note": out}
    return status() | {"reset": r.returncode == 0, "note": out}


# ---------------------------------------------------------------- flying the SITL vehicle

def _require_session() -> Mode:
    mode = _state["mode"]
    if not mode:
        raise RuntimeError("no simulator session; launch one first")
    return mode


def _px4(command: str, timeout: float = 25) -> str:
    """Run one px4-<cmd> client against the running SITL instance; console text back."""
    mode = _require_session()
    inner = f'cd {ROOTFS} && timeout {int(timeout) - 5} ../bin/px4-{command} 2>&1'
    r = _wsl(mode, inner, timeout=timeout)
    return _ANSI.sub("", r.stdout + r.stderr).strip()


def takeoff() -> dict[str, Any]:
    """``commander takeoff``: arms and climbs to MIS_TAKEOFF_ALT, then holds."""
    note = _px4("commander takeoff")
    return status() | {"takeoff": True, "note": note[-300:]}


def land() -> dict[str, Any]:
    note = _px4("commander land")
    return status() | {"land": True, "note": note[-300:]}


_INT = re.compile(r"^\s*(arming_state|nav_state):\s*(\d+)", re.M)
_FLT = re.compile(r"^\s*(z|vz):\s*(-?[\d.]+)", re.M)
_BOOL = re.compile(r"^\s*(torque_setpoint_achieved|thrust_setpoint_achieved):\s*(True|False)", re.M)
_AGE = re.compile(r"^\s*timestamp:\s*\d+\s*\(([\d.]+) seconds ago\)", re.M)


def probe() -> dict[str, Any]:
    """One snapshot of the vehicle: armed, flight mode, height, whether the allocator meets its
    setpoints, and how old the arming topic is (minutes means the commander wedged)."""
    mode = _require_session()
    inner = (
        f"cd {ROOTFS} && for t in vehicle_status actuator_armed vehicle_local_position "
        "control_allocator_status; do echo \"## $t\"; timeout 4 ../bin/px4-listener $t 1 2>&1; done"
    )
    if dry_run() or not available():
        return {"ok": False, "note": "dry run"}
    r = _wsl(mode, inner, timeout=40)
    text = _ANSI.sub("", r.stdout + r.stderr)
    parts = {p.split("\n", 1)[0].strip(): p for p in text.split("## ")[1:]}
    ints = {k: int(v) for k, v in _INT.findall(parts.get("vehicle_status", ""))}
    flts = {k: float(v) for k, v in _FLT.findall(parts.get("vehicle_local_position", ""))}
    bools = {k: v == "True" for k, v in _BOOL.findall(parts.get("control_allocator_status", ""))}
    age = _AGE.search(parts.get("actuator_armed", ""))
    nav = ints.get("nav_state")
    return {
        "ok": "arming_state" in ints,
        "armed": None if "arming_state" not in ints else ints["arming_state"] == 2,
        "nav_state": nav,
        "nav_mode": NAV_STATE.get(nav, str(nav)) if nav is not None else None,
        "height_m": round(-flts["z"], 2) if "z" in flts else None,
        "climb_mps": round(-flts["vz"], 2) if "vz" in flts else None,
        "torque_achieved": bools.get("torque_setpoint_achieved"),
        "thrust_achieved": bools.get("thrust_setpoint_achieved"),
        "arming_topic_age_s": round(float(age.group(1)), 1) if age else None,
    }


def pull_log(exports_dir: Path) -> dict[str, Any]:
    """Copy the newest SITL ulog out of the PX4 rootfs into exports/logs as sitl_<name>.ulg."""
    _require_session()
    if dry_run() or not available():
        return {"path": None, "dry_run": True}
    inner = (
        f'L=$(ls -t {ROOTFS}/log/*/*.ulg 2>/dev/null | head -1); [ -n "$L" ] || exit 4; '
        f'mkdir -p {bash_path(WSL_REPO)}/exports/logs && '
        f'cp "$L" {bash_path(WSL_REPO)}/exports/logs/sitl_$(basename "$L") && '
        'echo "$(basename "$L") $(stat -c %s "$L")"'
    )
    r = _wsl("sitl", inner, timeout=120)
    if r.returncode != 0 or not r.stdout.strip():
        raise FileNotFoundError("no ulog in the SITL rootfs yet")
    name, size = r.stdout.split()[:2]
    path = exports_dir / "logs" / f"sitl_{name}"
    return {"path": str(path), "name": path.name, "size": int(size)}
