"""Launch, watch and stop the Gazebo harness from the app (SITL in gz sim or HITL in Classic).

The backend runs natively on the Windows machine that owns the Pixhawk (PLAN.md section 3), so it
starts the WSL launcher ``scripts/wsl/tiltlab_gazebo.sh`` through ``wsl.exe`` and hands the user
the same console the CLI menu would. One launcher, one code path: the app only picks arguments.

Inside Docker or on a machine without WSL the endpoints answer 501 with the command to run by
hand. ``TILTLAB_GAZEBO_DRY_RUN=1`` returns the command without starting anything (tests).
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Literal

from tiltlab.export.gazebo import export_gazebo
from tiltlab.export.gazebo_classic_hitl import export_gazebo_classic_hitl
from tiltlab.scenario import Scenario

Mode = Literal["sitl", "hitl"]

# the repo is reachable in both distros as ~/utopia/vibe-coded (a symlink; the space in
# "Utopia Labs" breaks quoting through wsl.exe), see scripts/tiltlab_menu.py
WSL_REPO = os.environ.get("TILTLAB_WSL_REPO", "~/utopia/vibe-coded")
DISTRO: dict[str, str] = {
    "sitl": os.environ.get("TILTLAB_WSL_SITL_DISTRO", "Ubuntu-24.04"),  # gz sim Harmonic
    "hitl": os.environ.get("TILTLAB_WSL_HITL_DISTRO", "Ubuntu-22.04"),  # Gazebo Classic 11
}
LAUNCHER = "scripts/wsl/tiltlab_gazebo.sh"
_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")

_proc: subprocess.Popen[bytes] | None = None
_state: dict[str, Any] = {"mode": None, "log": None, "command": None, "harness": None}


def available() -> bool:
    return sys.platform == "win32" and shutil.which("wsl.exe") is not None


def dry_run() -> bool:
    return os.environ.get("TILTLAB_GAZEBO_DRY_RUN") == "1"


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


def wsl_command(mode: Mode, harness_wsl: str) -> list[str]:
    inner = (
        f"bash {bash_path(WSL_REPO + '/' + LAUNCHER)} --mode {mode} --yes "
        f"--harness {bash_path(harness_wsl)}"
    )
    return ["wsl.exe", "-d", DISTRO[mode], "--", "bash", "-lc", inner]


def launch(mode: Mode, scenario: Scenario, exports_dir: Path, repo_root: Path) -> dict[str, Any]:
    """Export the harness for ``scenario`` and start the launcher; returns what was started."""
    global _proc
    if mode == "sitl":
        harness = Path(export_gazebo(scenario, exports_dir / "gazebo")["root"])
    else:
        harness = Path(export_gazebo_classic_hitl(scenario, exports_dir / "gazebo_hitl")["root"])
    cmd = wsl_command(mode, wsl_path(harness, repo_root))
    log = exports_dir / "logs" / f"gazebo_{mode}.log"
    log.parent.mkdir(parents=True, exist_ok=True)
    _state.update(mode=mode, log=str(log), command=" ".join(cmd), harness=str(harness))
    if dry_run():
        return status() | {"dry_run": True}
    if not available():
        raise RuntimeError("wsl.exe is not available here; run the command shown by hand")
    if _proc is not None and _proc.poll() is None:
        raise RuntimeError(f"a {_state['mode']} session is already running; stop it first")
    with open(log, "wb") as fh:
        _proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
    return status()


def status(tail: int = 12) -> dict[str, Any]:
    running = _proc is not None and _proc.poll() is None
    lines: list[str] = []
    if _state["log"] and Path(_state["log"]).exists():
        raw = Path(_state["log"]).read_bytes()[-20000:].decode("utf-8", errors="replace")
        for ln in _ANSI.sub("", raw).splitlines():
            ln = ln.strip()
            if ln and "gc_relay]" not in ln and not ln.startswith("pxh>"):
                lines.append(ln)
        lines = lines[-tail:]
    return {
        "available": available(),
        "running": running,
        "mode": _state["mode"],
        "harness": _state["harness"],
        "command": _state["command"],
        "log": _state["log"],
        "tail": lines,
        "returncode": None if _proc is None else _proc.poll(),
    }


def stop() -> dict[str, Any]:
    """Take down whichever simulator is up (launcher --stop) and drop our handle."""
    global _proc
    if dry_run() or not available():
        _proc = None
        return status() | {"stopped": True, "dry_run": dry_run()}
    for mode in ("sitl", "hitl"):
        inner = f"bash {bash_path(WSL_REPO + '/' + LAUNCHER)} --stop"
        subprocess.run(["wsl.exe", "-d", DISTRO[mode], "--", "bash", "-lc", inner],
                       capture_output=True, timeout=60, check=False)
    if _proc is not None and _proc.poll() is None:
        _proc.terminate()
    _proc = None
    return status() | {"stopped": True}


def relaunch() -> dict[str, Any]:
    """Start the launcher again for the harness already exported (no re-export)."""
    global _proc
    mode, harness = _state["mode"], _state["harness"]
    if not mode or not harness:
        raise RuntimeError("nothing to relaunch")
    cmd = wsl_command(mode, wsl_path(Path(harness), Path(harness).parents[2]))
    log = Path(_state["log"])
    with open(log, "ab") as fh:
        _proc = subprocess.Popen(cmd, stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
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
    r = subprocess.run(["wsl.exe", "-d", DISTRO[mode], "--", "bash", "-lc", inner],
                       capture_output=True, text=True, timeout=180, check=False)
    out = _ANSI.sub("", r.stdout + r.stderr).strip().splitlines()[-4:]
    if r.returncode == 3:  # termination latched: the board must reboot, which drops the serial link
        stop()
        dev_cmd = ('D=$(ls /dev/ttyACM* | head -1); '
                   'python3 "$HOME"/utopia/vibe-coded/scripts/px4_board.py --dev "$D" shell reboot')
        subprocess.run(["wsl.exe", "-d", DISTRO[mode], "--", "bash", "-lc", dev_cmd],
                       capture_output=True, timeout=60, check=False)
        import time

        time.sleep(25)
        return relaunch() | {"reset": True, "rebooted": True, "note": out}
    return status() | {"reset": r.returncode == 0, "note": out}
