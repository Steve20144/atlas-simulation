#!/usr/bin/env bash
# wait_ready.sh: block until the detached SITL launcher reports 'Ready for takeoff!' (or time out),
# then print the log highlights, the processes and the vehicle altitude. Runs INSIDE WSL:
#
#   wsl -d Ubuntu-24.04 -- bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/wait_ready.sh <harness name> [timeout s]
set -uo pipefail
NAME="${1:?harness name}"; LIMIT="${2:-150}"
LOG="$HOME/sitl_${NAME}.log"
for _ in $(seq 1 $((LIMIT / 3))); do
  grep -q 'Ready for takeoff' "$LOG" 2>/dev/null && break
  pgrep -f "vectra_gazebo.sh --mode sitl" >/dev/null || { echo "launcher exited"; break; }
  sleep 3
done
echo "ready lines: $(grep -c 'Ready for takeoff' "$LOG" 2>/dev/null || echo 0); log size: $(stat -c %s "$LOG" 2>/dev/null || echo 0) bytes"
echo "== log highlights"
tr -d '\r' < "$LOG" | grep -vE 'pxh>|^[[:space:]]*$' | grep -iE 'error|warn|fail|ready|airframe|spawn|bridge|lockstep|denied' | tail -12 | cut -c1-140
echo "== processes"
pgrep -af 'bin/px4|gz sim' | grep -v pgrep | cut -c1-70
echo "== vehicle"
bash "$HOME/utopia/vibe-coded/scripts/wsl/px4ctl.sh" status 2>/dev/null | tail -1
