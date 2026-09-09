#!/usr/bin/env bash
# Recreate the pinned, sparse PX4 v1.17.0 checkout under third_party/PX4-Autopilot.
# Only the paths needed for the ports are checked out (blob filter keeps it around 7 MB).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DEST="$ROOT/third_party/PX4-Autopilot"
TAG="v1.17.0"
EXPECTED_COMMIT="d6f12ad1c4f70ad3230afd7d86e971421e02fef4"

if [ -d "$DEST/.git" ]; then
  echo "already present: $DEST"
else
  mkdir -p "$ROOT/third_party"
  git clone --filter=blob:none --no-checkout --depth 1 --branch "$TAG" \
    https://github.com/PX4/PX4-Autopilot.git "$DEST"
fi

cd "$DEST"
git sparse-checkout init --cone
git sparse-checkout set \
  src/lib/control_allocation \
  src/lib/matrix \
  src/lib/mathlib \
  src/lib/rate_control \
  src/lib/mixer_module \
  src/modules/control_allocator \
  src/modules/mc_rate_control \
  src/modules/mc_att_control \
  src/modules/mc_pos_control \
  src/modules/land_detector \
  src/modules/simulation/pwm_out_sim \
  src/modules/simulation/simulator_mavlink \
  msg \
  ROMFS/px4fmu_common/init.d/airframes \
  ROMFS/px4fmu_common/init.d-posix/airframes \
  Tools/simulation \
  platforms/posix/cmake \
  boards/px4/fmu-v6x
git checkout -q "$TAG"

HEAD="$(git rev-parse HEAD)"
if [ "$HEAD" != "$EXPECTED_COMMIT" ]; then
  echo "unexpected commit $HEAD (expected $EXPECTED_COMMIT)" >&2
  exit 1
fi
echo "PX4 $TAG at $HEAD"
