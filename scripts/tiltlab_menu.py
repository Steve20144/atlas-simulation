"""Interactive menu for the tiltlab stack: app, SITL, HITL, exports, firmware, logs.

    uv run --project backend python scripts/tiltlab_menu.py [--dry-run]

Every entry drives a tool that already exists (the Makefile, the WSL launcher, the exporters,
hover_report.py); this only picks the arguments and shows the command before running it, so
nothing here is a second implementation of the thing it launches. --dry-run prints the commands
and runs nothing.

Windows paths are translated for WSL through the ``~/utopia/vibe-coded`` symlink that exists in
both distros (the space in "Utopia Labs" breaks quoting through wsl.exe).
"""

from __future__ import annotations

import argparse
import shlex
import shutil
import subprocess
import sys
from collections.abc import Callable
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
WSL_REPO = "~/utopia/vibe-coded"
WSL_HELPERS = "~/utopia/vibe-coded/scripts/wsl"
SITL_DISTRO = "Ubuntu-24.04"  # gz sim (Harmonic)
HITL_DISTRO = "Ubuntu-22.04"  # Gazebo Classic 11, the only one PX4 speaks HIL through
DRY_RUN = False


def bash_path(path: str) -> str:
    """Quote a WSL path for bash, keeping a leading ``~`` expandable."""
    if path.startswith("~/"):
        return '"$HOME"' + shlex.quote("/" + path[2:])
    return shlex.quote(path)


def to_wsl(path: Path) -> str:
    """Map a path under the repo to its WSL spelling; fall back to wslpath elsewhere."""
    resolved = path.resolve()
    try:
        rel = resolved.relative_to(REPO)
    except ValueError:
        out = subprocess.run(
            ["wsl.exe", "-d", HITL_DISTRO, "--", "wslpath", "-a", str(resolved)],
            capture_output=True, text=True, check=False,
        )
        return out.stdout.strip() or str(resolved)
    return WSL_REPO if not rel.parts else f"{WSL_REPO}/{rel.as_posix()}"


def run(cmd: list[str], *, cwd: Path | None = None) -> int:
    """Show a command and run it with inherited stdio (so consoles and prompts work)."""
    shown = [a if " " not in a else '"' + a + '"' for a in cmd]
    print("\n$ " + " ".join(shown) + "\n")
    if DRY_RUN:
        return 0
    try:
        return subprocess.run(cmd, cwd=cwd, check=False).returncode
    except FileNotFoundError as exc:
        print(f"  not found: {exc.filename}")
        return 127


def run_wsl(distro: str, inner: str) -> int:
    return run(["wsl.exe", "-d", distro, "--", "bash", "-lc", inner])


def ask(prompt: str, default: str = "") -> str:
    suffix = f" [{default}]" if default else ""
    try:
        answer = input(f"{prompt}{suffix}: ").strip()
    except EOFError:
        return default
    return answer or default


def pick_dir(root: Path, what: str) -> Path | None:
    """List the export directories under root, newest first, and let the user pick or type one."""
    if root.is_dir():
        dirs = sorted((d for d in root.iterdir() if d.is_dir()), key=lambda d: -d.stat().st_mtime)
    else:
        dirs = []
    if not dirs:
        print(f"  no {what} under {root.relative_to(REPO)}; export one first (option 4)")
    for i, d in enumerate(dirs, 1):
        stamp = datetime.fromtimestamp(d.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
        print(f"  {i}) {d.name:42} {stamp}")
    answer = ask(f"pick a {what} by number, or type a path", "1" if dirs else "")
    if not answer:
        return None
    if answer.isdigit() and dirs:
        index = int(answer)
        if not 1 <= index <= len(dirs):
            print(f"  no entry {index}")
            return None
        return dirs[index - 1]
    return Path(answer).expanduser()


def pick_file(root: Path, pattern: str, what: str) -> Path | None:
    files = sorted(root.glob(pattern), key=lambda f: -f.stat().st_mtime) if root.is_dir() else []
    for i, f in enumerate(files[:15], 1):
        stamp = datetime.fromtimestamp(f.stat().st_mtime).strftime("%Y-%m-%d %H:%M")
        print(f"  {i}) {f.name:42} {stamp}")
    if not files:
        print(f"  no {what} under {root}")
    answer = ask(f"pick a {what} by number, or type a path", "1" if files else "")
    if not answer:
        return None
    if answer.isdigit() and files:
        index = int(answer)
        return files[index - 1] if 1 <= index <= len(files) else None
    return Path(answer).expanduser()


def open_app() -> None:
    """make dev: backend on :8000 (serves the built UI), Vite on :5173. Ctrl-C stops both."""
    make = shutil.which("make") or shutil.which("make", path=str(Path.home() / ".local/bin"))
    if not make:
        print("  make is not on PATH: run `source scripts/env.sh` in this shell first")
        return
    print("  UI: http://127.0.0.1:8000 (built UI) or http://127.0.0.1:5173 (Vite, hot reload)")
    run([make, "dev"], cwd=REPO)


def launch(mode: str) -> None:
    """Launch the WSL harness runner for 'sitl' or 'hitl' after asking which export to fly."""
    hitl = mode == "hitl"
    root = REPO / "exports" / ("gazebo_hitl" if hitl else "gazebo")
    harness = pick_dir(root, f"{mode.upper()} harness")
    if harness is None:
        return
    arg = str(harness) if str(harness).startswith(("/", "~")) else to_wsl(harness)
    if hitl:
        print("  fans and ESCs unpowered; params loaded and SYS_HITL 1 confirmed (see docs)")
    inner = (
        f"bash {bash_path(WSL_HELPERS + '/tiltlab_gazebo.sh')} "
        f"--mode {mode} --harness {bash_path(arg)}"
    )
    run_wsl(HITL_DISTRO if hitl else SITL_DISTRO, inner)


def export_harness() -> None:
    """Write the Gazebo SITL and/or Gazebo Classic HITL harness for one scenario."""
    scenarios = sorted((REPO / "scenarios").glob("*.json"))
    for i, s in enumerate(scenarios, 1):
        print(f"  {i}) {s.stem}")
    answer = ask("scenario by number, or a path", "1" if scenarios else "")
    if not answer:
        return
    path = scenarios[int(answer) - 1] if answer.isdigit() else Path(answer).expanduser()
    kind = ask("export sitl / hitl / both", "both").lower()
    if DRY_RUN:
        print(f"\n  would export {kind} from {path}\n")
        return
    import json

    from tiltlab.export.gazebo import export_gazebo
    from tiltlab.export.gazebo_classic_hitl import export_gazebo_classic_hitl
    from tiltlab.scenario import Scenario

    scenario = Scenario.model_validate(json.loads(path.read_text()))
    if kind in ("sitl", "both"):
        print("  SITL ->", export_gazebo(scenario, REPO / "exports/gazebo")["root"])
    if kind in ("hitl", "both"):
        out = export_gazebo_classic_hitl(scenario, REPO / "exports/gazebo_hitl")
        print("  HITL ->", out["root"])


def attach_usb() -> None:
    """usbipd-win: share the Pixhawk with WSL so Gazebo Classic can open /dev/ttyACM0."""
    if run(["usbipd.exe", "list"]) != 0:
        return
    busid = ask("BUSID of the Pixhawk (blank to cancel)")
    if not busid:
        return
    print("  if STATE says 'Not shared', run this once in an administrator PowerShell:")
    print(f"    usbipd bind --busid {busid}")
    run(["usbipd.exe", "attach", "--wsl", HITL_DISTRO, "--busid", busid])
    run_wsl(HITL_DISTRO, "ls -l /dev/ttyACM* 2>/dev/null || echo 'no /dev/ttyACM*'")


def build_firmware() -> None:
    """px4_fmu-v6x_hitl: the board label that carries pwm_out_sim and fits in flash."""
    harness = pick_dir(REPO / "exports/gazebo_hitl", "HITL harness")
    if harness is None:
        return
    arg = str(harness) if str(harness).startswith(("/", "~")) else to_wsl(harness)
    inner = (
        f"bash {bash_path(WSL_HELPERS + '/tiltlab_gazebo.sh')} "
        f"--build-firmware --harness {bash_path(arg)}"
    )
    run_wsl(HITL_DISTRO, inner)


def hover_report() -> None:
    log = pick_file(REPO / "exports/logs", "*.ulg", "ulog")
    if log is None:
        return
    run([sys.executable, str(REPO / "scripts/hover_report.py"), str(log)], cwd=REPO)


def stop_sim() -> None:
    """Gazebo Classic leaves gzserver on port 11345; a stale one blocks the next launch."""
    inner = "pkill -x gzclient; pkill -x gzserver; pkill -f qgc_udp_relay.py; echo stopped"
    for distro in (SITL_DISTRO, HITL_DISTRO):
        run_wsl(distro, inner + " " + distro)


MENU: tuple[tuple[str, Callable[[], None]], ...] = (
    ("tiltlab simulator (make dev, UI on :8000)", open_app),
    (f"launch SITL  (gz sim, {SITL_DISTRO})", lambda: launch("sitl")),
    (f"launch HITL  (Gazebo Classic + Pixhawk, {HITL_DISTRO})", lambda: launch("hitl")),
    ("export a harness from a scenario", export_harness),
    ("attach the Pixhawk to WSL (usbipd)", attach_usb),
    ("build the HITL firmware (px4_fmu-v6x_hitl)", build_firmware),
    ("hover report from a ulog", hover_report),
    ("stop a running Gazebo (clear a stale gzserver)", stop_sim),
)


def main() -> int:
    global DRY_RUN
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dry-run", action="store_true", help="print commands, run nothing")
    DRY_RUN = parser.parse_args().dry_run
    while True:
        print("\ntiltlab" + ("  [dry run]" if DRY_RUN else ""))
        for i, (label, _) in enumerate(MENU, 1):
            print(f"  {i}) {label}")
        print("  q) quit")
        choice = ask("choice", "q")
        if choice in ("q", "quit", "exit"):
            return 0
        if not choice.isdigit() or not 1 <= int(choice) <= len(MENU):
            print("  pick a number from the list")
            continue
        try:
            MENU[int(choice) - 1][1]()
        except KeyboardInterrupt:
            print("\n  interrupted")


if __name__ == "__main__":
    raise SystemExit(main())
