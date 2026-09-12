#!/usr/bin/env bash
# px4ctl.sh: talk to the PX4 SITL instance that tiltlab_gazebo.sh started (same WSL distro).
#
#   bash px4ctl.sh status                    # is PX4 / Gazebo running, vehicle altitude
#   bash px4ctl.sh takeoff | land            # commander takeoff / land
#   bash px4ctl.sh param MC_PITCH_P 0.8 [MORE NAME VALUE ...]   # set parameters live
#   bash px4ctl.sh show MC_PITCH_P           # read one parameter
#   bash px4ctl.sh log [<windows dir>]       # copy the newest .ulg out (default: the tiltlab exports dir)
#   bash px4ctl.sh reset-params              # delete saved SITL params (next launch starts from the airframe)
#   bash px4ctl.sh reset                     # restart PX4 and respawn the vehicle, clock to 0 (= Gazebo's reset button)
#   bash px4ctl.sh stop                      # stop PX4 and Gazebo
#
# Live parameter changes are lost when PX4 restarts unless you also put them in the exporter
# (tiltlab/export/gazebo.py px4_tuning) or the airframe file; PX4 persists them in
# build/px4_sitl_default/rootfs/parameters.bson, which tiltlab_gazebo.sh offers to clear.
set -euo pipefail
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
R="$PX4_DIR/build/px4_sitl_default/rootfs"
OUT_DEFAULT="$(cd "$(dirname "$0")/../.." && pwd)/exports/logs"  # <repo>/exports/logs
cmd="${1:-status}"; shift || true
px4() { (cd "$R" && timeout 10 ../bin/px4-"$@"); }
case "$cmd" in
  status)
    pgrep -a -f "bin/px4|gz sim" | cut -c1-80 || echo "nothing running"
    M="$(gz model --list 2>/dev/null | grep -E "_0$" | head -1 | tr -d ' -')"
    [ -n "$M" ] && timeout 3 gz topic -e -t "/model/$M/pose" -n 1 2>/dev/null | awk '/position/{f=1} f&&/z:/{printf "vehicle %s at z = %.2f m\n", "'"$M"'", $2; exit}'
    ;;
  takeoff|land) px4 commander "$cmd" ;;
  param)
    while [ $# -ge 2 ]; do px4 param set "$1" "$2" >/dev/null && echo "set $1 = $2"; shift 2; done ;;
  show) px4 param show "$1" ;;
  log)
    OUT="${1:-$OUT_DEFAULT}"; mkdir -p "$OUT"
    L="$(ls -t "$R"/log/*/*.ulg | head -1)"; cp "$L" "$OUT/"; echo "copied $(basename "$L") to $OUT"
    echo "report: uv run --project backend python scripts/hover_report.py exports/logs/$(basename "$L")" ;;
  reset-params) rm -f "$R"/parameters*.bson && echo "saved SITL parameters removed" ;;
  reset) bash "$(dirname "$0")/tiltlab_gazebo.sh" --reset --mode sitl ;;
  stop) pkill -f "bin/px4" 2>/dev/null || true; sleep 1; pkill -f "gz sim" 2>/dev/null || true; echo "stopped" ;;
  *) sed -n '2,14p' "$0"; exit 2 ;;
esac
