#!/usr/bin/env bash
# wsl_clean.sh: tidy one WSL distro used by Vectra. Stops every simulator process, drops stale
# FIFOs and console logs, optionally deletes the PX4 SITL ulogs, and purges the harness copies
# (gz model, world, airframe and its CMake line; Classic model and world) of scenarios that no
# longer exist. Runs INSIDE WSL:
#
#   wsl -d Ubuntu-24.04 -- bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/wsl_clean.sh [--logs] [scenario ...]
#   wsl -d Ubuntu-22.04 -- bash ~/utopia/vibe-coded/.claude/skills/sitl-run/scripts/wsl_clean.sh [scenario ...]
#
# Follow it with 'wsl --shutdown' from PowerShell when WSLg is in copy mode (grey Gazebo window
# titled [WARN:COPY MODE]); that fault lives in the WSLg session and only a shutdown clears it.
set -uo pipefail
LOGS=0
[ "${1:-}" = "--logs" ] && { LOGS=1; shift; }
PX4="$HOME/PX4-Autopilot"
GZ="$PX4/Tools/simulation/gz"
AF="$PX4/ROMFS/px4fmu_common/init.d-posix/airframes"
AF_BUILD="$PX4/build/px4_sitl_default/etc/init.d-posix/airframes"
CL="$PX4/Tools/simulation/gazebo-classic/sitl_gazebo-classic"

echo "== processes"
for pat in "vectra_gazebo.sh" "bin/px4" "gz sim" "qgc_udp_relay.py" "sleep infinity"; do
  n=$(pgrep -fc -- "$pat" || true); [ "${n:-0}" -gt 0 ] && { pkill -f -- "$pat" 2>/dev/null; echo "  killed $n x $pat"; }
done
for x in gzserver gzclient; do pgrep -x "$x" >/dev/null && { pkill -x "$x"; echo "  killed $x"; }; done
sleep 1
pgrep -f "gz sim|bin/px4|gzserver" >/dev/null && { pkill -9 -f "gz sim|bin/px4|gzserver" 2>/dev/null; echo "  forced"; }
echo "  left: $(pgrep -af 'gz sim|bin/px4|gzserver|vectra_gazebo' | grep -vc pgrep || true)"

echo "== stale files"
rm -fv /tmp/sitl_*_stdin "$HOME"/sitl_*.log "$PX4/build/px4_sitl_default/vectra.reset" 2>/dev/null | sed 's/^/  /'
if [ "$LOGS" = 1 ] && [ -d "$PX4/build/px4_sitl_default/rootfs/log" ]; then
  du -sh "$PX4/build/px4_sitl_default/rootfs/log" | sed 's/^/  ulogs removed: /'
  rm -rf "$PX4/build/px4_sitl_default/rootfs/log"/*
fi

for name in "$@"; do
  echo "== purge harness copies of $name"
  rm -rfv "$GZ/models/$name" "$GZ/worlds/$name.sdf" 2>/dev/null | tail -1 | sed 's/^/  /'
  for f in "$AF"/*_gz_"$name" "$AF_BUILD"/*_gz_"$name"; do
    [ -e "$f" ] || continue
    rm -f "$f"; echo "  airframe $(basename "$f")"
    [ -f "$AF/CMakeLists.txt" ] && sed -i "/^\t$(basename "$f")\$/d" "$AF/CMakeLists.txt"
  done
  rm -rfv "$CL/models/${name}_hitl" "$CL/models/$name" "$CL/worlds/hitl_${name}_hitl.world" 2>/dev/null | tail -1 | sed 's/^/  /'
done
echo "== left in the PX4 tree"
ls "$GZ/models" 2>/dev/null | grep -i atlas | tr '\n' ' '; echo
ls "$AF" 2>/dev/null | grep -E '_gz_' | tr '\n' ' '; echo
