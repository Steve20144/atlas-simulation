#!/usr/bin/env bash
# tiltlab_gazebo.sh: take a Gazebo harness exported by the tiltlab app (vibe-coded/exports/...)
# into Gazebo on this WSL/Ubuntu machine.
#
#   bash tiltlab_gazebo.sh                    # HITL: Gazebo Classic + the real Pixhawk over USB (default)
#   bash tiltlab_gazebo.sh --mode sitl        # SITL: gz sim + PX4 built on this machine, no hardware
#   bash tiltlab_gazebo.sh --check            # only report what is installed, change nothing
#   bash tiltlab_gazebo.sh --harness <dir>    # a specific export directory (default: newest one)
#   bash tiltlab_gazebo.sh --px4 <dir>        # PX4 checkout (default ~/PX4-Autopilot, tag v1.17.0)
#   bash tiltlab_gazebo.sh --serial /dev/ttyACM1
#   bash tiltlab_gazebo.sh --build-firmware   # HITL: also build px4_fmu-v6x with pwm_out_sim (and offer upload)
#   bash tiltlab_gazebo.sh --yes              # answer yes to every question
#   bash tiltlab_gazebo.sh --stop             # stop PX4 SITL, gz sim, Gazebo Classic and the QGC relay
#   bash tiltlab_gazebo.sh --reset --mode M --harness D   # disarm, reset model poses; exit 3 if termination latched
#   bash tiltlab_gazebo.sh --reset --hard ... # also stop, reboot the board, relaunch (HITL after a flip)
#
# Steps: 1 check packages (with versions), 2 install what is missing (asks first), 3 ask whether to
# continue when everything is present, 4 copy the harness into PX4's Gazebo tree and launch.

set -euo pipefail

# ---------------------------------------------------------------- options
MODE="hitl"
CHECK_ONLY=0
YES=0
BUILD_FW=0
STOP=0
RESET=0
HARD=0
HARNESS=""
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_TAG="v1.17.0"
SERIAL_DEV="/dev/ttyACM0"
TILTLAB_WIN="${TILTLAB_WIN:-$(cd "$(dirname "$0")/../.." && pwd)}"  # the repo this script lives in

while [ $# -gt 0 ]; do
  case "$1" in
    --mode) MODE="$2"; shift 2 ;;
    --check) CHECK_ONLY=1; shift ;;
    --yes|-y) YES=1; shift ;;
    --build-firmware) BUILD_FW=1; shift ;;
    --stop) STOP=1; shift ;;
    --reset) RESET=1; shift ;;
    --hard) HARD=1; shift ;;
    --harness) HARNESS="$2"; shift 2 ;;
    --px4) PX4_DIR="$2"; shift 2 ;;
    --serial) SERIAL_DEV="$2"; shift 2 ;;
    --tiltlab) TILTLAB_WIN="$2"; shift 2 ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done
case "$MODE" in hitl|sitl) ;; *) echo "--mode must be hitl or sitl"; exit 2 ;; esac
if [ "$STOP" = 1 ]; then
  # one command for the app and the menu: whichever simulator is up, take it down
  pkill -f "bin/px4" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  pkill -x gzclient 2>/dev/null || true
  pkill -x gzserver 2>/dev/null || true
  pkill -f qgc_udp_relay.py 2>/dev/null || true
  sleep 1
  pgrep -af "bin/px4|gz sim|gzserver|gzclient|qgc_udp_relay" && exit 1
  echo "stopped"; exit 0
fi

if [ "$RESET" = 1 ]; then
  # Gazebo's "Reset Time" only rewinds the clock. A usable reset is: disarm the flight controller,
  # put the model back where it spawned (poses only: a time reset sends timestamps backwards into
  # PX4), and, in HITL, notice a latched flight termination (past 60 deg of roll or pitch), which
  # only a board reboot clears. --hard does that reboot and relaunches the same harness.
  [ -n "$HARNESS" ] || { echo "--reset needs --harness <dir> (and --mode)"; exit 2; }
  W="$(grep -ho '<world name="[^"]*"' "$HARNESS"/worlds/* 2>/dev/null | head -1 | sed 's/.*="//; s/"//')"
  [ -n "$W" ] || { echo "no <world name=...> in $HARNESS/worlds"; exit 2; }
  BOARD="python3 $TILTLAB_WIN/scripts/px4_board.py"
  if [ "$MODE" = hitl ]; then
    (cd "$TILTLAB_WIN" && timeout 40 $BOARD shell 'commander disarm -f' >/dev/null 2>&1) || true
    gz world -w "$W" --reset-models >/dev/null 2>&1 && echo "Gazebo Classic: model poses reset in $W"
    FD="$(cd "$TILTLAB_WIN" && timeout 40 $BOARD shell 'listener vehicle_status 1' 2>/dev/null | awk '/failure_detector_status:/{print $2}' | tr -d '\r')"
    if [ "${FD:-0}" != 0 ] || [ "$HARD" = 1 ]; then
      echo "board: failure_detector_status ${FD:-0}; flight termination latches until reboot"
      if [ "$HARD" = 1 ]; then
        "$0" --stop >/dev/null 2>&1 || true
        DEV="$(ls /dev/ttyACM* 2>/dev/null | head -1)"
        (cd "$TILTLAB_WIN" && timeout 30 $BOARD --dev "$DEV" shell reboot >/dev/null 2>&1) || true
        sleep 25
        exec "$0" --mode "$MODE" --yes --harness "$HARNESS"
      fi
      exit 3
    fi
  else
    R="$PX4_DIR/build/px4_sitl_default/rootfs"
    (cd "$R" && timeout 10 ../bin/px4-commander disarm -f >/dev/null 2>&1) || true
    gz service -s "/world/$W/control" --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
      --timeout 3000 --req 'reset: {model_only: true}' >/dev/null 2>&1 && echo "gz sim: model poses reset in $W"
  fi
  echo "reset done"; exit 0
fi

# ---------------------------------------------------------------- helpers
bold()  { printf '\033[1m%s\033[0m\n' "$*"; }
ok()    { printf '  \033[32m[ok]\033[0m   %s\n' "$*"; }
miss()  { printf '  \033[31m[miss]\033[0m %s\n' "$*"; }
warn()  { printf '  \033[33m[warn]\033[0m %s\n' "$*"; }
info()  { printf '  %s\n' "$*"; }
ask() {  # ask "question" -> 0 for yes
  if [ "$YES" = 1 ]; then echo "  $1 [auto-yes]"; return 0; fi
  read -r -p "  $1 [y/N] " reply
  [[ "$reply" =~ ^[Yy]$ ]]
}
MISSING_APT=()
need_apt() {  # need_apt <package> [<binary for version>] [<version args>]
  local pkg="$1" bin="${2:-}" args="${3:---version}"
  if dpkg -s "$pkg" >/dev/null 2>&1; then
    local ver; ver="$(dpkg-query -W -f='${Version}' "$pkg" 2>/dev/null)"
    ok "$pkg $ver"
  elif [ -n "$bin" ] && command -v "$bin" >/dev/null 2>&1; then
    ok "$bin ($("$bin" $args 2>&1 | head -1))"
  else
    miss "$pkg"; MISSING_APT+=("$pkg")
  fi
}

# ---------------------------------------------------------------- 1. checks
bold "tiltlab Gazebo harness ($MODE)"
bold "1. Environment"
if ! command -v apt-get >/dev/null 2>&1 || [ ! -r /etc/os-release ]; then
  miss "this is not an apt-based Linux (Ubuntu). Run it inside your WSL Ubuntu shell:"
  info "  wsl -d Ubuntu   then   bash \"/mnt/c/Users/stefa/Documents/Utopia Labs/wsl/tiltlab_gazebo.sh\""
  exit 1
fi
if grep -qi microsoft /proc/version 2>/dev/null; then ok "WSL kernel: $(uname -r)"; else info "not WSL: $(uname -r)"; fi
if command -v lsb_release >/dev/null 2>&1; then
  DISTRO="$(lsb_release -rs)"; ok "Ubuntu $DISTRO ($(lsb_release -cs))"
else
  DISTRO="$(. /etc/os-release && echo "${VERSION_ID:-0}")"; warn "no lsb_release; VERSION_ID=$DISTRO"
fi
if [ -n "${WAYLAND_DISPLAY:-}${DISPLAY:-}" ]; then ok "GUI display available (${WAYLAND_DISPLAY:-$DISPLAY})"; else warn "no DISPLAY/WAYLAND_DISPLAY: Gazebo GUI needs WSLg (Windows 11) or an X server"; fi

bold "2. Build tools (PX4 v1.17 needs these)"
need_apt git git
need_apt cmake cmake
need_apt build-essential gcc
need_apt ninja-build ninja
need_apt python3 python3
need_apt python3-pip pip3
need_apt python3-jinja2
need_apt python3-numpy
need_apt python3-packaging
info "python: empy, kconfiglib, pyros-genmsg, toml are installed by PX4's Tools/setup/ubuntu.sh via pip"
need_apt protobuf-compiler protoc
need_apt libprotobuf-dev
need_apt libeigen3-dev
need_apt libopencv-dev
need_apt libgstreamer1.0-dev
need_apt libgstreamer-plugins-base1.0-dev
need_apt gstreamer1.0-plugins-bad
need_apt libxml2-utils xmllint

bold "3. Gazebo"
HAVE_CLASSIC=0; HAVE_GZ=0
if command -v gazebo >/dev/null 2>&1; then
  CLASSIC_VER="$(gazebo --version 2>/dev/null | grep -oE 'version [0-9.]+' | head -1 || true)"
  ok "Gazebo Classic: ${CLASSIC_VER:-unknown version} ($(command -v gazebo))"; HAVE_CLASSIC=1
  need_apt libgazebo-dev
elif [ "$MODE" = hitl ]; then
  miss "Gazebo Classic (gazebo, libgazebo-dev)"
else
  info "Gazebo Classic not installed (not needed for SITL with gz sim)"
fi
if command -v gz >/dev/null 2>&1; then
  GZ_VER="$(gz sim --version 2>/dev/null | head -1 || true)"
  ok "gz sim: ${GZ_VER:-installed} ($(command -v gz))"; HAVE_GZ=1
else
  info "gz sim (Gazebo Harmonic/Fortress) not found"
fi
if [ "$MODE" = hitl ] && [ "$HAVE_CLASSIC" = 0 ]; then
  warn "HITL needs Gazebo Classic 11: PX4 v1.17 speaks HIL only through the Classic mavlink_interface plugin"
  if [ "${DISTRO%%.*}" -ge 24 ] 2>/dev/null; then
    warn "Ubuntu $DISTRO has no Gazebo Classic package. Two options:"
    info "  a) a second WSL distro for HITL:  wsl --install -d Ubuntu-22.04   then run this script there"
    info "  b) no hardware for now: rerun with --mode sitl to fly the same geometry in gz sim with PX4 SITL"
  else
    MISSING_APT+=(gazebo libgazebo-dev)
  fi
fi
if [ "$MODE" = sitl ] && [ "$HAVE_GZ" = 0 ]; then
  warn "SITL harness targets gz sim (Harmonic). Install: https://gazebosim.org/docs/harmonic/install_ubuntu"
fi

bold "4. PX4 source and plugins"
NEED_PX4_CLONE=0; NEED_PLUGIN_BUILD=0
if [ -d "$PX4_DIR/.git" ]; then
  PX4_DESC="$(git -C "$PX4_DIR" describe --tags --always 2>/dev/null || echo '?')"
  ok "PX4 at $PX4_DIR ($PX4_DESC)"
  [ "$PX4_DESC" = "$PX4_TAG" ] || warn "expected tag $PX4_TAG; the harness was generated against that source"
else
  miss "PX4 checkout at $PX4_DIR"; NEED_PX4_CLONE=1
fi
if [ "$MODE" = hitl ]; then
  PLUGIN_SO="$(ls "$PX4_DIR"/build/px4_sitl_default/build_gazebo-classic/libgazebo_mavlink_interface.so 2>/dev/null | head -1 || true)"
  if [ -n "$PLUGIN_SO" ]; then ok "sitl_gazebo-classic plugins built ($PLUGIN_SO)"; else miss "sitl_gazebo-classic plugins (make px4_sitl_default gazebo-classic)"; NEED_PLUGIN_BUILD=1; fi
else
  if [ -x "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then ok "px4_sitl_default built"; else miss "px4_sitl_default not built"; NEED_PLUGIN_BUILD=1; fi
fi

bold "5. Serial access (HITL)"
if [ "$MODE" = hitl ]; then
  if id -nG "$USER" | grep -qw dialout; then ok "$USER is in dialout"; else miss "$USER not in dialout (sudo usermod -aG dialout $USER; log out and in)"; fi
  if ls /dev/ttyACM* >/dev/null 2>&1; then ok "serial devices: $(ls /dev/ttyACM* | tr '\n' ' ')"; else
    warn "no /dev/ttyACM*: on WSL attach the Pixhawk with usbipd from PowerShell: usbipd list; usbipd bind --busid <id>; usbipd attach --wsl --busid <id>"
  fi
  if command -v QGroundControl >/dev/null 2>&1 || ls ~/QGroundControl*.AppImage >/dev/null 2>&1; then ok "QGroundControl found in WSL"; else info "QGroundControl: use the Windows install; Gazebo forwards MAVLink to UDP 14550 (works across WSL2 with mirrored networking or a Windows host IP; see README of the harness)"; fi
fi

bold "6. Harness from tiltlab"
if [ -z "$HARNESS" ]; then
  SUB="gazebo_hitl"; [ "$MODE" = sitl ] && SUB="gazebo"
  HARNESS="$(ls -td "$TILTLAB_WIN"/exports/"$SUB"/*/ 2>/dev/null | head -1 || true)"
  HARNESS="${HARNESS%/}"
fi
if [ -n "$HARNESS" ] && [ -f "$HARNESS/README.md" ]; then
  ok "harness: $HARNESS"
  NAME="$(basename "$HARNESS")"
  ls "$HARNESS"/models 2>/dev/null | sed 's/^/     model: /'
else
  miss "no harness found. In the tiltlab app load your scenario and click Export > ${MODE^^} (HITL) or Gazebo (SITL), or pass --harness <dir>"
fi

if [ "$CHECK_ONLY" = 1 ]; then exit 0; fi
[ -n "$HARNESS" ] && [ -f "$HARNESS/README.md" ] || { echo; echo "Export a harness first, then rerun."; exit 1; }
if [ "$MODE" = hitl ] && [ "$HAVE_CLASSIC" = 0 ] && [ "${DISTRO%%.*}" -ge 24 ] 2>/dev/null; then
  echo
  bold "Cannot continue with HITL on Ubuntu $DISTRO"
  info "Gazebo Classic is not packaged for this release and the PX4 gazebo-classic plugins need it."
  info "Either run this script in an Ubuntu 22.04 WSL distro (wsl --install -d Ubuntu-22.04),"
  info "or rerun with --mode sitl to use the gz sim already installed here."
  exit 1
fi

# ---------------------------------------------------------------- 2. install what is missing
NEED_INSTALL=0
if [ ${#MISSING_APT[@]} -gt 0 ] || [ "$NEED_PX4_CLONE" = 1 ] || [ "$NEED_PLUGIN_BUILD" = 1 ]; then NEED_INSTALL=1; fi
if [ "$NEED_INSTALL" = 1 ]; then
  echo
  bold "Missing pieces"
  [ ${#MISSING_APT[@]} -gt 0 ] && info "apt packages: ${MISSING_APT[*]}"
  [ "$NEED_PX4_CLONE" = 1 ] && info "PX4 $PX4_TAG clone into $PX4_DIR (about 1 GB with submodules)"
  [ "$NEED_PLUGIN_BUILD" = 1 ] && info "PX4 SITL build ($( [ "$MODE" = hitl ] && echo 'with gazebo-classic plugins' || echo 'px4_sitl_default'), 10 to 30 minutes)"
  if ask "Install/build the missing pieces now?"; then
    if [ ${#MISSING_APT[@]} -gt 0 ]; then
      sudo apt update
      if [[ " ${MISSING_APT[*]} " == *" gazebo "* ]] && [ "${DISTRO%%.*}" -ge 24 ]; then
        warn "Ubuntu $DISTRO has no Gazebo Classic package; HITL needs Ubuntu 22.04 (or 20.04). Stopping."
        exit 1
      fi
      AVAILABLE=(); UNAVAILABLE=()
      for pkg in "${MISSING_APT[@]}"; do
        # a package is installable only when apt has a candidate version for it
        cand="$(apt-cache policy "$pkg" 2>/dev/null | awk '/Candidate:/{print $2}')"
        if [ -n "$cand" ] && [ "$cand" != "(none)" ]; then AVAILABLE+=("$pkg"); else UNAVAILABLE+=("$pkg"); fi
      done
      [ ${#UNAVAILABLE[@]} -gt 0 ] && warn "not in this release's archive, skipped: ${UNAVAILABLE[*]}"
      [ ${#AVAILABLE[@]} -gt 0 ] && sudo apt install -y "${AVAILABLE[@]}"
    fi
    if [ "$NEED_PX4_CLONE" = 1 ]; then
      git clone --recursive -b "$PX4_TAG" https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
      bash "$PX4_DIR/Tools/setup/ubuntu.sh" --no-nuttx --no-sim-tools
    fi
    if [ "$NEED_PLUGIN_BUILD" = 1 ]; then
      if [ "$MODE" = hitl ]; then
        (cd "$PX4_DIR" && DONT_RUN=1 make px4_sitl_default gazebo-classic)
      else
        (cd "$PX4_DIR" && make px4_sitl_default)
      fi
    fi
  else
    echo "Nothing installed. Rerun when ready."; exit 1
  fi
else
  echo
  bold "Everything required is present."
  ask "Continue and bring the tiltlab harness into Gazebo ($MODE)?" || { echo "Stopped."; exit 0; }
fi

# ---------------------------------------------------------------- 2b. optional HITL firmware
if [ "$MODE" = hitl ] && [ "$BUILD_FW" = 1 ]; then
  bold "Firmware with pwm_out_sim for the Pixhawk 6X / 6X Pro"
  # default + pwm_out_sim overflows the v6x flash by 22884 B, so build the harness label
  # instead: a delta on default.px4board that also drops fixed-wing and VTOL (94.6% of flash).
  BOARD_CFG="$PX4_DIR/boards/px4/fmu-v6x/hitl.px4board"
  if [ -f "$HARNESS/px4/fmu-v6x_hitl.px4board" ]; then
    tr -d '\r' < "$HARNESS/px4/fmu-v6x_hitl.px4board" > "$BOARD_CFG"
    ok "board label -> $BOARD_CFG"
  else
    miss "no px4/fmu-v6x_hitl.px4board in $HARNESS (re-export the harness from tiltlab)"; exit 1
  fi
  if [ -d "$PX4_DIR/platforms/nuttx" ] && command -v arm-none-eabi-gcc >/dev/null 2>&1; then
    (cd "$PX4_DIR" && make px4_fmu-v6x_hitl)
    if ask "Upload to the Pixhawk now (USB attached to WSL, QGC closed)?"; then (cd "$PX4_DIR" && make px4_fmu-v6x_hitl upload); fi
  else
    warn "ARM toolchain missing: run bash $PX4_DIR/Tools/setup/ubuntu.sh (without --no-nuttx) then rerun with --build-firmware"
  fi
fi

# ---------------------------------------------------------------- 4. harness into Gazebo
echo
bold "Installing the harness"
if [ "$MODE" = hitl ]; then
  SG="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
  [ -d "$SG/models" ] || { miss "sitl_gazebo-classic submodule missing at $SG (git -C $PX4_DIR submodule update --init --recursive)"; exit 1; }
  MODEL_DIR="$(ls -d "$HARNESS"/models/*/ | head -1)"; MODEL="$(basename "$MODEL_DIR")"
  rm -rf "$SG/models/$MODEL"; cp -r "$MODEL_DIR" "$SG/models/$MODEL"
  cp "$HARNESS"/worlds/*.world "$SG/worlds/"
  WORLD="$(basename "$(ls "$HARNESS"/worlds/*.world | head -1)")"
  if [ "$SERIAL_DEV" != "/dev/ttyACM0" ]; then
    sed -i "s|<serialDevice>[^<]*</serialDevice>|<serialDevice>$SERIAL_DEV</serialDevice>|" "$SG/models/$MODEL/model.sdf"
  fi
  # the Classic mavlink_interface binds its QGC socket to qgc_addr (so it must stay INADDR_ANY)
  # and sends telemetry as a UDP broadcast to 14550, which WSL2 never delivers to Windows. A small
  # relay inside WSL picks the broadcast up and unicasts it to QGroundControl on the Windows host.
  sed -i "s|<qgc_addr>[^<]*</qgc_addr>|<qgc_addr>INADDR_ANY</qgc_addr>|" "$SG/models/$MODEL/model.sdf"
  RELAY=""
  if grep -qi microsoft /proc/version 2>/dev/null; then
    RELAY="$(dirname "$0")/qgc_udp_relay.py"
    [ -f "$RELAY" ] || RELAY="/mnt/c/Users/stefa/Documents/Utopia Labs/wsl/qgc_udp_relay.py"
    [ -f "$RELAY" ] || { warn "qgc_udp_relay.py not found next to this script; QGC on Windows will not see the vehicle"; RELAY=""; }
  fi
  sed -i 's/\r$//' "$SG/models/$MODEL/model.sdf" "$SG/models/$MODEL/model.config" "$SG/worlds/$WORLD"
  xmllint --noout "$SG/models/$MODEL/model.sdf" && ok "model.sdf well formed"
  ok "model -> $SG/models/$MODEL"; ok "world -> $SG/worlds/$WORLD"
  PARAMS="$(ls "$HARNESS"/px4/*.params | head -1)"
  echo
  bold "Before launching"
  info "1. Pixhawk firmware must include pwm_out_sim (stock v6x builds do not): see $HARNESS/README.md step 2 or rerun with --build-firmware"
  info "2. Load $PARAMS with QGroundControl (Parameters > Tools > Load from file), reboot, check 'pwm_out_sim status' in the MAVLink console"
  info "3. Close QGroundControl (Gazebo owns the serial port; QGC reconnects over UDP 14550)"
  info "4. Fans and ESCs unpowered"
  # A board reboot re-enumerates through usbipd and often comes back as /dev/ttyACM1, so a
  # default of ttyACM0 points Gazebo at a node that no longer exists ("Error opening serial
  # device: set_option: Input/output error", HITL 2026-09-11). If the default is absent and
  # exactly one ttyACM exists, use that one.
  if [ ! -e "$SERIAL_DEV" ]; then
    FOUND=( /dev/ttyACM* )
    if [ ${#FOUND[@]} -eq 1 ] && [ -e "${FOUND[0]}" ]; then
      warn "$SERIAL_DEV not present; using ${FOUND[0]}"
      SERIAL_DEV="${FOUND[0]}"
      sed -i "s|<serialDevice>[^<]*</serialDevice>|<serialDevice>$SERIAL_DEV</serialDevice>|" "$SG/models/$MODEL/model.sdf"
    fi
  fi
  if [ ! -e "$SERIAL_DEV" ]; then
    warn "$SERIAL_DEV not present now; attach the Pixhawk (usbipd) before launching"
  elif [ ! -w "$SERIAL_DEV" ]; then
    miss "$SERIAL_DEV exists but $USER cannot open it (owner root, group dialout). Fix once, then reopen the shell:"
    info "   sudo usermod -aG dialout $USER      # then in PowerShell: wsl --terminate $(. /etc/os-release; echo ${NAME// /-}-${VERSION_ID})  and start a new WSL shell"
    info "   (or for this session only: sudo chmod a+rw $SERIAL_DEV)"
    exit 1
  else
    ok "$SERIAL_DEV present and writable"
  fi
  # Gazebo Classic leaves gzserver holding the master port (11345) whenever the client window
  # goes away without a clean shutdown; the next launch then attaches to that stale scene
  # instead of starting its own, which looks like Gazebo refusing to open twice.
  if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
    warn "a previous Gazebo is still running (gzserver holds port 11345)"
    if ask "Stop it and start fresh?"; then
      pkill -x gzclient 2>/dev/null || true
      pkill -x gzserver 2>/dev/null || true
      for _ in $(seq 10); do pgrep -x gzserver >/dev/null 2>&1 || break; sleep 0.5; done
      pgrep -x gzserver >/dev/null 2>&1 && { pkill -9 -x gzserver 2>/dev/null || true; sleep 1; }
      ok "previous Gazebo stopped"
    else
      info "leaving it alone; close that window or run: pkill -x gzclient; pkill -x gzserver"
      exit 1
    fi
  fi
  # WSLg draws every window grey and titled [WARN:COPY MODE] once its shared-memory channel
  # failed at session start ("rdp_allocate_shared_memory ... Input/output error", use_gfxredir
  # = 0). Nothing inside WSL fixes it: the whole VM has to go down.
  if grep -q 'rdp_allocate_shared_memory.*Input/output error' /mnt/wslg/weston.log 2>/dev/null; then
    warn "WSLg is in copy mode: the Gazebo window will be grey and titled [WARN:COPY MODE]"
    info "   fix from Windows: wsl --shutdown   (closes every distro), then launch again"
  fi
  if ask "Launch Gazebo Classic with $WORLD now?"; then
    # shellcheck disable=SC1090
    # setup_gazebo.bash appends to these; under "set -u" an unset one aborts the script
    export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}" GAZEBO_MODEL_PATH="${GAZEBO_MODEL_PATH:-}" LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
    source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$PX4_DIR/build/px4_sitl_default"
    if [ -n "$RELAY" ]; then
      pkill -f qgc_udp_relay.py 2>/dev/null || true
      python3 "$RELAY" &
      RELAY_PID=$!
      trap 'kill $RELAY_PID 2>/dev/null || true; pkill -x gzclient 2>/dev/null || true; pkill -x gzserver 2>/dev/null || true' EXIT INT TERM
      ok "QGC relay running (pid $RELAY_PID): Gazebo broadcast -> Windows host UDP 14550"
    fi
    [ -n "$RELAY" ] || trap 'pkill -x gzclient 2>/dev/null || true; pkill -x gzserver 2>/dev/null || true' EXIT INT TERM
    cd "$SG" && gazebo --verbose "worlds/$WORLD"
  fi
else
  GZ_DIR="$PX4_DIR/Tools/simulation/gz"
  [ -d "$GZ_DIR/models" ] || { miss "gz models dir missing at $GZ_DIR (git -C $PX4_DIR submodule update --init --recursive)"; exit 1; }
  MODEL_DIR="$(ls -d "$HARNESS"/models/*/ | head -1)"; MODEL="$(basename "$MODEL_DIR")"
  rm -rf "$GZ_DIR/models/$MODEL"; cp -r "$MODEL_DIR" "$GZ_DIR/models/$MODEL"
  cp "$HARNESS"/worlds/*.sdf "$GZ_DIR/worlds/"
  # PX4's gz bridge defines only 8 ESC channels (SIM_GZ_EC_FUNC1..8, gz_bridge/module.yaml
  # "__max_num_servos: 8"); with more motors Gazebo prints "You tried to access index N of the
  # Actuator velocity array". Raise the limit to 12 and rebuild so the parameters exist.
  N_MOTORS="$(grep -c '<motorNumber>' "$GZ_DIR/models/$MODEL/model.sdf" || echo 0)"
  GZ_YAML="$PX4_DIR/src/modules/simulation/gz_bridge/module.yaml"
  if [ "$N_MOTORS" -gt 8 ] && grep -q '^__max_num_servos: &max_num_servos 8' "$GZ_YAML"; then
    sed -i 's/^__max_num_servos: &max_num_servos 8/__max_num_servos: \&max_num_servos 12/' "$GZ_YAML"
    ok "patched $GZ_YAML: ESC channels 8 -> 12 (model has $N_MOTORS motors); PX4 will rebuild"
  elif [ "$N_MOTORS" -gt 8 ]; then
    grep -q '^__max_num_servos: &max_num_servos 1[0-9]' "$GZ_YAML" && ok "gz bridge already allows more than 8 ESC channels" || warn "check $GZ_YAML: the model has $N_MOTORS motors and the ESC channel count must be at least that"
  fi
  # Second gz_bridge limit: GZMixingInterfaceESC::motorSpeedCallback copies every motor speed into
  # esc_status.esc[], an array of esc_status_s::CONNECTED_ESC_MAX (8) entries. With more motors it
  # overruns the stack and glibc aborts PX4 (__stack_chk_fail) right after "gz_bridge world: ...".
  GZ_ESC="$PX4_DIR/src/modules/simulation/gz_bridge/GZMixingInterfaceESC.cpp"
  if [ "$N_MOTORS" -gt 8 ] && grep -q 'esc_status.esc_count = actuators.velocity_size();' "$GZ_ESC"; then
    sed -i 's/esc_status.esc_count = actuators.velocity_size();/esc_status.esc_count = actuators.velocity_size() < esc_status_s::CONNECTED_ESC_MAX ? actuators.velocity_size() : esc_status_s::CONNECTED_ESC_MAX; \/\/ tiltlab: esc_status holds 8 entries/' "$GZ_ESC"
    sed -i 's/for (int i = 0; i < actuators.velocity_size(); i++) {/for (int i = 0; i < esc_status.esc_count; i++) {/' "$GZ_ESC"
    grep -q 'i < esc_status.esc_count' "$GZ_ESC" && ok "patched $GZ_ESC: esc_status feedback clamped to 8 entries (10-motor PX4 would abort otherwise); PX4 will rebuild" || warn "could not patch $GZ_ESC; PX4 will abort with 10 motors, see the harness README"
  fi
  AIRFRAME="$(ls "$HARNESS"/px4/airframes/* | head -1)"; AF="$(basename "$AIRFRAME")"
  AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"; AF_ID="${AF%%_*}"
  CMAKE="$AF_DIR/CMakeLists.txt"
  # remove earlier tiltlab copies of this model under another id (the old 4010 collided with x500_mono_cam)
  for OLD in "$AF_DIR"/*_gz_"$MODEL"; do
    if [ -e "$OLD" ] && [ "$(basename "$OLD")" != "$AF" ]; then
      rm -f "$OLD"; sed -i "/^\t$(basename "$OLD")\$/d" "$CMAKE"; warn "removed stale airframe $(basename "$OLD")"
    fi
  done
  # PX4's rcS sources every airframes/<SYS_AUTOSTART>_* file and keeps the last one, so the id must be unique
  for CLASH in $(ls "$AF_DIR" | grep "^${AF_ID}_" | grep -v "^$AF\$" || true); do
    if grep -q "Generated by tiltlab" "$AF_DIR/$CLASH"; then
      rm -f "$AF_DIR/$CLASH"; sed -i "/^\t$CLASH\$/d" "$CMAKE"; warn "removed older tiltlab airframe $CLASH (same autostart id $AF_ID)"
    else
      miss "autostart id $AF_ID is already used by stock PX4 airframe $CLASH; PX4 would load that one instead. Rename the scenario (the id is derived from its name) and re-export"; exit 1
    fi
  done
  cp "$AIRFRAME" "$AF_DIR/"
  # files exported on Windows may carry CRLF; PX4's shell then reads "param set-default X 1\r" and every
  # airframe line fails, leaving SIM_GZ_EC_FUNC5.. unset (Gazebo: "Actuator velocity array ... size 4")
  sed -i 's/\r$//' "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/$AF" "$GZ_DIR/models/$MODEL"/model.sdf "$GZ_DIR/models/$MODEL"/model.config "$GZ_DIR/worlds/"*.sdf
  ok "line endings normalised to LF (airframe, model, world)"
  # PX4 keeps parameters changed at runtime (QGC, PX4_PARAM_* overrides, param set) in
  # rootfs/parameters.bson and only resets them when the autostart id changes, so an old value
  # silently overrides the airframe's set-default lines on the next run of the same id.
  if ls "$PX4_DIR"/build/px4_sitl_default/rootfs/parameters*.bson >/dev/null 2>&1; then
    if ask "Saved SITL parameters from an earlier run exist. Delete them so this airframe starts from its own defaults?"; then
      rm -f "$PX4_DIR"/build/px4_sitl_default/rootfs/parameters*.bson; ok "saved SITL parameters removed"
    else
      warn "keeping saved parameters; values set at runtime earlier override the airframe file"
    fi
  fi
  grep -q "$AF" "$CMAKE" || sed -i "0,/^\t[0-9]\{4,5\}_gz_/s//\t$AF\n&/" "$CMAKE"
  ok "model -> $GZ_DIR/models/$MODEL"; ok "airframe -> $AF (added to CMakeLists)"
  # A Gazebo left over from an earlier (crashed) PX4 run keeps the old model and stale sensors;
  # PX4 would attach to it instead of spawning a fresh vehicle.
  if pgrep -f "gz sim" >/dev/null 2>&1; then
    if ask "Gazebo is already running (probably from an earlier run). Stop it so PX4 starts a fresh world?"; then
      pkill -f "gz sim"; sleep 2; ok "stopped the old Gazebo"
    else
      warn "keeping the running Gazebo; PX4 will attach to the existing $MODEL""_0 if present"
    fi
  fi
  pgrep -f "bin/px4" >/dev/null 2>&1 && { pkill -f "bin/px4"; sleep 1; warn "stopped a PX4 instance that was still running"; }
  if ask "Build and launch PX4 SITL with gz sim ($MODEL) now?"; then
    # PX4 SITL sends MAVLink to localhost only; WSL2 has its own network, so QGroundControl on Windows
    # never sees it. Broadcasting (rcS honours PX4_PARAM_<name> overrides) reaches the Windows host.
    export PX4_PARAM_MAV_0_BROADCAST=1
    # WSLg: Mesa often cannot open the GPU for OpenGL (libEGL "failed to get driver name", ZINK
    # "failed to choose pdev") and the gz GUI then shows an empty grey window. Software OpenGL is
    # slow but renders; set TILTLAB_GPU=1 to try the GPU instead.
    if grep -qi microsoft /proc/version 2>/dev/null && [ "${TILTLAB_GPU:-0}" != 1 ]; then
      export LIBGL_ALWAYS_SOFTWARE=1 QT_QPA_PLATFORM=xcb
      info "Gazebo GUI on software OpenGL (WSLg); export TILTLAB_GPU=1 before running to use the GPU"
    fi
    WSL_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
    HOST_IP="$(ip route show default 2>/dev/null | awk '{print $3}' | head -n 1)"
    info "QGroundControl on Windows should auto-connect: PX4 sends MAVLink straight to the Windows host ${HOST_IP:-<gateway>}:14550 (WSL2 drops broadcasts)."
    info "If it stays Disconnected: QGC > Application Settings > Comm Links > Add: UDP, listening port 14551 (14550 is taken by auto-connect), server ${WSL_IP:-<wsl ip>}:18570, then Connect."
    info "Fly from the PX4 console: commander takeoff / commander land (needs QGC connected and Ready to fly)."
    cd "$PX4_DIR" && exec make px4_sitl "gz_$MODEL"
  fi
fi
echo "Done."
