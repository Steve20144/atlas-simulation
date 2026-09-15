"""Launch, watch, fly and stop the Gazebo harness from the app (SITL in gz sim or HITL in Classic).

The backend runs natively on the Windows machine that owns the Pixhawk (PLAN.md section 3), so it
drives WSL through ``wsl.exe``: the launcher ``scripts/wsl/vectra_gazebo.sh`` installs the harness
into the PX4 checkout and starts the sim; everything around it (stale sims, readiness, takeoff,
land, log copy, live probe) lives here so the app is the one code path. What was learnt the hard
way and is baked in:

- the sim is never a child of the backend: it is started detached inside WSL (setsid, nohup),
  its identity is kept in a state file and its liveness comes from pgrep, so uvicorn --reload, a
  backend restart or a second client neither kill it nor lose it;
- a launch first stops whatever simulator is up in that distro (any owner), never refuses;
- PX4's shell gets a FIFO on stdin that is held open and never written: with /dev/null it reads
  EOF and reprints its prompt in a busy loop (6 GB of console log in 25 minutes);
- scripts for WSL go in as bytes on stdin and run from a file: ``wsl.exe -- <args>`` passes its
  arguments through the distro's default shell first (``$var`` and ``$(...)`` expand before bash
  sees them), text mode would add ``\\r`` to every line, and a script read straight from stdin is
  eaten by the first px4 client that reads stdin;
- readiness has two levels: ``ready`` (PX4 booted, the gz bridge found the world) gates the
  buttons, ``preflight_pass`` from the live probe is what ``commander takeoff`` needs; after a
  cold start the height estimate can take a minute or two;
- WSLg's copy-mode fault (grey window titled [WARN:COPY MODE]) is detected from weston.log and
  reported; only ``wsl --shutdown`` clears it, which :func:`wsl_shutdown` does on request;
- a wedged commander (armed per ``commander status``, standby per ``vehicle_status``) cannot be
  reset in place; :func:`probe` shows the stale arming timestamp and the fix is stop and launch.

Inside Docker or on a machine without WSL the endpoints answer 501 with the command to run by
hand. ``VECTRA_GAZEBO_DRY_RUN=1`` returns commands without starting anything (tests).
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import time
import uuid
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

_state: dict[str, Any] = {
    "mode": None, "log": None, "command": None, "harness": None, "headless": False,
    "started": None, "wslg_copy_mode": False,
}  # fmt: skip
# The session outlives this process. The state file remembers what was launched; a pgrep in the
# distro says whether it still runs (cached a few seconds, it costs a wsl.exe round trip).
STATE_FILE = Path(__file__).resolve().parents[3] / "exports" / "logs" / "gazebo_session.json"
_alive_cache: dict[str, Any] = {"t": 0.0, "running": False}
ALIVE_CACHE_S = 5.0


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


def _wsl(mode: Mode, script: str, timeout: float = 60) -> subprocess.CompletedProcess[str]:
    """Run a bash script in the distro of ``mode`` and return its text output.

    The script travels as bytes on stdin, is copied to a file and run from there with stdin on
    /dev/null; see the module docstring for the three ways any other route breaks."""
    if dry_run() or not available():
        return subprocess.CompletedProcess(["wsl.exe"], 0, stdout="", stderr="")
    tmp = f"/tmp/vectra_{uuid.uuid4().hex}.sh"
    outer = f"cat > {tmp}; exec bash -l {tmp} < /dev/null"  # exec: the script's exit code is ours
    body = f"rm -f {tmp}\n{script}\n"  # unlinking an open script is fine on Linux; no litter
    r = subprocess.run(["wsl.exe", "-d", DISTRO[mode], "--", "sh", "-c", outer],
                       input=body.encode("utf-8"), capture_output=True, timeout=timeout,
                       check=False)
    return subprocess.CompletedProcess(
        r.args, r.returncode,
        stdout=r.stdout.decode("utf-8", errors="replace"),
        stderr=r.stderr.decode("utf-8", errors="replace"),
    )


def _save_state() -> None:
    try:
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps(_state), encoding="utf-8")
    except OSError:
        pass


def _load_state() -> None:
    try:
        saved = json.loads(STATE_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return
    if isinstance(saved, dict) and saved.get("mode") in ("sitl", "hitl"):
        _state.update({k: saved.get(k, _state[k]) for k in _state})


def _launcher_alive(mode: Mode) -> bool:
    """Is a launcher for ``mode`` running in its distro (started by anyone)? Cached."""
    now = time.time()
    if now - _alive_cache["t"] < ALIVE_CACHE_S:
        return bool(_alive_cache["running"])
    r = _wsl(mode, f"pgrep -f 'vectra_gazebo.sh --mode {mode}' >/dev/null && echo yes || echo no",
             timeout=20)
    _alive_cache.update(t=now, running=r.stdout.strip() == "yes")
    return bool(_alive_cache["running"])


def _running() -> bool:
    if not _state["mode"]:
        return False
    if dry_run() or not available():
        return False
    return _launcher_alive(_state["mode"])


def _forget_alive() -> None:
    _alive_cache.update(t=0.0, running=False)


def launcher_command(mode: Mode, harness_wsl: str, headless: bool = False) -> str:
    """The launcher invocation as bash sees it (shown in the UI and used by the detached start)."""
    return (
        ("HEADLESS=1 " if headless else "")
        + f"bash {bash_path(WSL_REPO + '/' + LAUNCHER)} --mode {mode} --yes "
        f"--harness {bash_path(harness_wsl)}"
    )


def wsl_command(mode: Mode, harness_wsl: str, headless: bool = False) -> list[str]:
    """Kept for callers that print the by-hand command."""
    return ["wsl.exe", "-d", DISTRO[mode], "--", "bash", "-lc",
            launcher_command(mode, harness_wsl, headless)]


def _detached_start(mode: Mode, command: str, log_wsl: str) -> None:
    """Start the launcher so it outlives us: setsid + nohup, stdout to the console log, stdin a
    FIFO held open by a sleeping writer (PX4's shell must not see EOF)."""
    fifo = f"/tmp/vectra_{mode}_stdin"
    script = (
        f"rm -f {fifo}; mkfifo {fifo}\n"
        f"setsid nohup sleep infinity > {fifo} 2>/dev/null < /dev/null & disown\n"
        f"setsid nohup {command} < {fifo} > {log_wsl} 2>&1 & disown\n"
        "sleep 1; pgrep -f 'vectra_gazebo.sh --mode' | head -1\n"
    )
    r = _wsl(mode, script, timeout=60)
    if not r.stdout.strip():
        raise RuntimeError("the launcher did not start in WSL: " + (r.stderr or r.stdout)[-300:])


def _stop_distro(mode: Mode) -> None:
    """Take down every simulator process in that distro, whoever started it."""
    _wsl(mode, f"bash {bash_path(WSL_REPO + '/' + LAUNCHER)} --stop; "
               f"pkill -f 'sleep infinity' 2>/dev/null; rm -f /tmp/vectra_{mode}_stdin; true",
         timeout=60)


def wslg_copy_mode() -> bool:
    """WSLg's shared-memory channel failed: every window is grey, titled [WARN:COPY MODE]."""
    r = _wsl("sitl", f"grep -c '{COPY_MODE_MARK}' /mnt/wslg/weston.log 2>/dev/null || true",
             timeout=20)
    return r.stdout.strip() not in ("", "0")


def wsl_shutdown() -> dict[str, Any]:
    """``wsl --shutdown``: stops both distros and restarts WSLg, the only cure for copy mode.
    Any HITL USB attach is dropped with it."""
    if not dry_run() and available():
        subprocess.run(["wsl.exe", "--shutdown"], capture_output=True, timeout=60, check=False)
    _state.update(wslg_copy_mode=False, started=None)
    _forget_alive()
    _save_state()
    return status() | {"wsl_shutdown": True, "dry_run": dry_run()}


def launch(mode: Mode, scenario: Scenario, exports_dir: Path, repo_root: Path,
           headless: bool = False) -> dict[str, Any]:
    """Export the harness for ``scenario``, stop whatever sim is up in that distro, start the
    launcher detached. Returns the status right after the start (``ready`` turns true later)."""
    if mode == "sitl":
        harness = Path(export_gazebo(scenario, exports_dir / "gazebo")["root"])
    else:
        harness = Path(export_gazebo_classic_hitl(scenario, exports_dir / "gazebo_hitl")["root"])
    command = launcher_command(mode, wsl_path(harness, repo_root), headless)
    log = exports_dir / "logs" / f"gazebo_{mode}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    _state.update(mode=mode, log=str(log), harness=str(harness), headless=headless,
                  command=" ".join(wsl_command(mode, wsl_path(harness, repo_root), headless)),
                  started=None, wslg_copy_mode=False)
    if dry_run():
        return status() | {"dry_run": True}
    if not available():
        raise RuntimeError("wsl.exe is not available here; run the command shown by hand")
    stop()  # a stale or wedged sim never blocks a launch
    _state["wslg_copy_mode"] = False if headless else wslg_copy_mode()
    log.write_bytes(b"")  # one launch, one log
    _detached_start(mode, command, wsl_path(log, repo_root))
    _state["started"] = time.time()
    _forget_alive()
    _save_state()
    return status()


def _log_lines() -> list[str]:
    if not _state["log"] or not Path(_state["log"]).exists():
        return []
    raw = Path(_state["log"]).read_bytes()[-200_000:].decode("utf-8", errors="replace")
    out: list[str] = []
    for ln in _ANSI.sub("", raw).splitlines():
        ln = ln.strip()
        if ln and "gc_relay]" not in ln and "pxh>" not in ln[:12]:
            out.append(ln)
    return out


def status(tail: int = 12) -> dict[str, Any]:
    running = _running()
    lines = _log_lines()
    started = _state["started"]
    uptime = time.time() - started if running and started else 0.0
    preflight_ok = running and any(READY_MARK in ln for ln in lines)
    # "Ready for takeoff!" is printed only when the preflight checks pass before the first arm;
    # after a cold start the height estimate can stay "not stable" for a minute or two. ready
    # therefore means booted (PX4 startup script done, gz bridge on the world) so the buttons
    # work; the probe's preflight_pass says when a takeoff will be accepted.
    booted = any("Startup script returned successfully" in ln for ln in lines) and any(
        "[gz_bridge] world:" in ln or "[simulator_mavlink]" in ln for ln in lines
    )
    return {
        "available": available(),
        "running": running,
        "ready": preflight_ok or (running and booted),
        "preflight_ok": preflight_ok,
        "mode": _state["mode"],
        "headless": bool(_state["headless"]),
        "harness": _state["harness"],
        "command": _state["command"],
        "log": _state["log"],
        "tail": lines[-tail:],
        "uptime_s": round(uptime, 1) if running and started else None,
        "wslg_copy_mode": bool(_state["wslg_copy_mode"]),
        "returncode": None,
    }


def stop() -> dict[str, Any]:
    """Take down whichever simulator is up (launcher --stop in both distros)."""
    if dry_run() or not available():
        _state["started"] = None
        return status() | {"stopped": True, "dry_run": dry_run()}
    for mode in ("sitl", "hitl"):
        _stop_distro(mode)
    _state["started"] = None
    _alive_cache.update(t=time.time(), running=False)
    _save_state()
    return status() | {"stopped": True}


def relaunch() -> dict[str, Any]:
    """Start the launcher again for the harness already exported (no re-export)."""
    mode, harness = _state["mode"], _state["harness"]
    if not mode or not harness:
        raise RuntimeError("nothing to relaunch")
    repo_root = Path(harness).parents[2]
    command = launcher_command(mode, wsl_path(Path(harness), repo_root), bool(_state["headless"]))
    _detached_start(mode, command, wsl_path(Path(_state["log"]), repo_root))
    _state["started"] = time.time()
    _forget_alive()
    _save_state()
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
    if not dry_run() and available() and not _running():
        raise RuntimeError(f"the {mode} session is not running any more; launch again")
    return mode


def _px4(command: str, timeout: float = 25) -> str:
    """Run one px4-<cmd> client against the running SITL instance; console text back."""
    mode = _require_session()
    r = _wsl(mode, f"cd {ROOTFS} && timeout {int(timeout) - 5} ../bin/px4-{command} 2>&1",
             timeout=timeout)
    return _ANSI.sub("", r.stdout + r.stderr).strip()


TAKEOFF_WAIT_S = 120.0  # cold start: the height estimate can take this long to settle


def takeoff() -> dict[str, Any]:
    """``commander takeoff``: arms and climbs to MIS_TAKEOFF_ALT, then holds.

    PX4 answers "Arming denied: Resolve system health failures first" while the preflight checks
    fail, and after a cold start "height estimate not stable" flickers for a minute or two, so a
    single probe sample is not enough. Wait until the preflight passes on two samples 5 s apart,
    send the command, confirm the vehicle armed, retry once. The note says what happened."""
    _require_session()
    if dry_run() or not available():
        return status() | {"takeoff": True, "note": "dry run"}
    deadline = time.time() + TAKEOFF_WAIT_S
    streak, waited = 0, 0.0
    while streak < 2 and time.time() < deadline:
        if probe().get("preflight_pass"):
            streak += 1
        else:
            streak = 0
        if streak < 2:
            time.sleep(5.0)
            waited += 5.0
    if streak < 2:
        return status() | {"takeoff": False,
                           "note": f"preflight checks still failing after {waited:.0f} s"}
    notes: list[str] = []
    for attempt in (1, 2):
        out = _px4("commander takeoff")
        time.sleep(6.0)
        p = probe()
        if p.get("armed"):
            notes.append(f"armed, {p.get('nav_mode')}, waited {waited:.0f} s for preflight")
            return status() | {"takeoff": True, "note": "; ".join(notes)[-300:]}
        notes.append(f"attempt {attempt}: not armed ({out[-120:] or 'no console text'})")
        time.sleep(4.0)
    return status() | {"takeoff": False, "note": "; ".join(notes)[-300:]}


def land() -> dict[str, Any]:
    note = _px4("commander land")
    return status() | {"land": True, "note": note[-300:]}


_INT = re.compile(r"^\s*(arming_state|nav_state):\s*(\d+)", re.M)
_FLT = re.compile(r"^\s*(z|vz):\s*(-?[\d.]+)", re.M)
_BOOL = re.compile(
    r"^\s*(torque_setpoint_achieved|thrust_setpoint_achieved|pre_flight_checks_pass):\s*(True|False)",
    re.M,
)
_AGE = re.compile(r"^\s*timestamp:\s*\d+\s*\(([\d.]+) seconds ago\)", re.M)


def probe() -> dict[str, Any]:
    """One snapshot of the vehicle: armed, flight mode, height, whether the preflight checks
    pass (takeoff will be accepted), whether the allocator meets its setpoints, and how old the
    arming topic is (minutes means the commander wedged)."""
    mode = _require_session()
    if dry_run() or not available():
        return {"ok": False, "note": "dry run"}
    script = (
        f"cd {ROOTFS} || exit 2\n"
        "for t in vehicle_status actuator_armed vehicle_local_position "
        "control_allocator_status; do\n"
        '  echo "## $t"; timeout 4 ../bin/px4-listener "$t" 1 2>&1\n'
        "done\n"
    )
    r = _wsl(mode, script, timeout=40)
    text = _ANSI.sub("", r.stdout + r.stderr)
    parts = {p.split("\n", 1)[0].strip(): p for p in text.split("## ")[1:]}
    ints = {k: int(v) for k, v in _INT.findall(parts.get("vehicle_status", ""))}
    flts = {k: float(v) for k, v in _FLT.findall(parts.get("vehicle_local_position", ""))}
    bools = {k: v == "True" for k, v in _BOOL.findall(
        parts.get("vehicle_status", "") + parts.get("control_allocator_status", ""))}
    age = _AGE.search(parts.get("actuator_armed", ""))
    nav = ints.get("nav_state")
    return {
        "ok": "arming_state" in ints,
        "armed": None if "arming_state" not in ints else ints["arming_state"] == 2,
        "nav_state": nav,
        "nav_mode": NAV_STATE.get(nav, str(nav)) if nav is not None else None,
        "height_m": round(-flts["z"], 2) if "z" in flts else None,
        "climb_mps": round(-flts["vz"], 2) if "vz" in flts else None,
        "preflight_pass": bools.get("pre_flight_checks_pass"),
        "torque_achieved": bools.get("torque_setpoint_achieved"),
        "thrust_achieved": bools.get("thrust_setpoint_achieved"),
        "arming_topic_age_s": round(float(age.group(1)), 1) if age else None,
    }


def pull_log(exports_dir: Path) -> dict[str, Any]:
    """Copy the newest SITL ulog out of the PX4 rootfs into exports/logs as sitl_<name>.ulg."""
    _require_session()
    if dry_run() or not available():
        return {"path": None, "dry_run": True}
    script = (
        f'L=$(ls -t {ROOTFS}/log/*/*.ulg 2>/dev/null | head -1)\n'
        '[ -n "$L" ] || exit 4\n'
        f"mkdir -p {bash_path(WSL_REPO)}/exports/logs\n"
        f'cp "$L" {bash_path(WSL_REPO)}/exports/logs/sitl_$(basename "$L") || exit 5\n'
        'echo "$(basename "$L") $(stat -c %s "$L")"\n'
    )
    r = _wsl("sitl", script, timeout=120)
    if r.returncode != 0 or not r.stdout.strip():
        raise FileNotFoundError("no ulog in the SITL rootfs yet")
    name, size = r.stdout.split()[:2]
    path = exports_dir / "logs" / f"sitl_{name}"
    return {"path": str(path), "name": path.name, "size": int(size)}


if not dry_run():  # adopt a session launched before this process started (reload, other client)
    _load_state()
