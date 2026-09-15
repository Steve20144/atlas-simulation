#!/usr/bin/env bash
# launch_detached.sh: start the SITL launcher so it outlives the shell that started it.
# Runs INSIDE WSL (Ubuntu-24.04). From Windows Git Bash:
#
#   wsl -d Ubuntu-24.04 -- bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/launch_detached.sh <harness name> [--headless]
#
# <harness name> is a directory under ~/utopia/vibe-coded/exports/gazebo. The launcher
# (scripts/wsl/vectra_gazebo.sh --mode sitl --yes) is started with setsid + nohup so it reparents
# to init and survives the end of the tool call. Its stdin is a FIFO held open by a sleeping
# writer: with /dev/null the PX4 shell reads EOF and reprints its prompt in a busy loop (6 GB of
# log in 25 minutes). Console output goes to ~/sitl_<name>.log. Refuses to start when a launcher
# or PX4 is already running (use vectra_gazebo.sh --stop first).
set -euo pipefail
NAME="${1:?harness name}"; shift || true
HEADLESS=0
[ "${1:-}" = "--headless" ] && HEADLESS=1
REPO="$HOME/utopia/vibe-coded"
HARNESS="$REPO/exports/gazebo/$NAME"
[ -d "$HARNESS/px4/airframes" ] || { echo "no harness at $HARNESS (export it first)"; exit 2; }
if pgrep -f "vectra_gazebo.sh --mode sitl|bin/px4" >/dev/null; then
  echo "a launcher or PX4 is already running; stop it first: bash $REPO/scripts/wsl/vectra_gazebo.sh --stop"; exit 3
fi
FIFO="/tmp/sitl_${NAME}_stdin"; LOG="$HOME/sitl_${NAME}.log"
rm -f "$FIFO"; mkfifo "$FIFO"
setsid nohup sleep infinity > "$FIFO" 2>/dev/null < /dev/null & disown   # keeps the FIFO open, writes nothing
setsid nohup env HEADLESS="$HEADLESS" bash "$REPO/scripts/wsl/vectra_gazebo.sh" --mode sitl --yes --harness "$HARNESS" \
  < "$FIFO" > "$LOG" 2>&1 & disown
sleep 2
pgrep -af "vectra_gazebo.sh --mode sitl" | head -1 | cut -c1-100
echo "log: $LOG"
echo "wait for 'Ready for takeoff!' then about 10 s: grep -c 'Ready for takeoff' $LOG"
