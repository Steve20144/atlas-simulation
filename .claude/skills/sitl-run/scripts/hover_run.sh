#!/usr/bin/env bash
# hover_run.sh: one scripted hover test against the PX4 SITL that vectra_gazebo.sh started.
# Runs INSIDE WSL (Ubuntu-24.04). From Windows Git Bash:
#
#   wsl -d Ubuntu-24.04 -- bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/hover_run.sh \
#       [--hover 40] [--tag run2] [NAME VALUE ...]
#
# Sets the given PX4 parameters live, takes off, hovers, lands, waits for the log to close and
# copies the newest .ulg to <repo>/exports/logs/<tag>_<original>.ulg. Prints one line per step and
# the copied file name last. Exit 2 when no SITL is running, 3 when the vehicle did not climb.
set -euo pipefail
HOVER=40; TAG=""
while [ $# -gt 0 ]; do
  case "$1" in
    --hover) HOVER="$2"; shift 2 ;;
    --tag) TAG="$2"; shift 2 ;;
    *) break ;;
  esac
done
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
CTL="$REPO/scripts/wsl/px4ctl.sh"
OUT="$REPO/exports/logs"
R="$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs"

pgrep -f "bin/px4" >/dev/null || { echo "no PX4 SITL running (start it with vectra_gazebo.sh --mode sitl)"; exit 2; }
alt() { bash "$CTL" status 2>/dev/null | awk '/vehicle .* at z/ {print $(NF-1); exit}' || true; }

while [ $# -ge 2 ]; do bash "$CTL" param "$1" "$2"; shift 2; done
before="$(ls -t "$R"/log/*/*.ulg 2>/dev/null | head -1 || true)"
echo "takeoff (hover ${HOVER}s)"; bash "$CTL" takeoff >/dev/null
sleep 25
z="$(alt)"; echo "altitude after 25 s: ${z:-?} m"
awk -v z="${z:-0}" 'BEGIN{exit !(z+0 > 0.5)}' || { echo "vehicle did not climb; check px4 console / hover_report on the last log"; bash "$CTL" land >/dev/null || true; exit 3; }
sleep "$((HOVER > 25 ? HOVER - 25 : 1))"
echo "altitude before landing: $(alt) m"
bash "$CTL" land >/dev/null; echo "landing"
for i in $(seq 1 30); do sleep 1; z="$(alt)"; awk -v z="${z:-9}" 'BEGIN{exit !(z+0 < 0.4)}' && break; done
sleep 6  # commander disarm delay + logger flush
L="$(ls -t "$R"/log/*/*.ulg | head -1)"
[ "$L" != "$before" ] || echo "warning: newest log is the same file as before the run"
mkdir -p "$OUT"; name="${TAG:+${TAG}_}$(basename "$L")"; cp "$L" "$OUT/$name"
echo "log: exports/logs/$name"
