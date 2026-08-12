#!/bin/bash
# ============================================================
# fix_falcon_visgrid_cadence.sh
#
# Stops the occupancy-grid visualisation publishes from scaling with the
# explored volume. Applies falcon_visgrid_cadence.patch, touching ONE file
# no other fix_falcon_*.sh does:
#
#   falcon_planner/voxel_mapping/src/map_server.cpp
#
# Root cause it fixes:
#   MapServer::publishOccupancyGrid() sweeps the ENTIRE voxel box every
#   0.5 s and serialises occupied, FREE and UNKNOWN point clouds whenever
#   anyone subscribes. The free/unknown clouds grow with the explored
#   volume (hundreds of thousands to millions of points), so publish AND
#   subscriber parse cost grow linearly with mapping progress -- the
#   "the further the mission gets, the slower everything runs" failure.
#   The occupied cloud (small, safety-critical for the follower's brake
#   gate) keeps its 2 Hz cadence; free/unknown drop to every 10th cycle.
#
# Applied BEFORE the catkin_make step so it is compiled in.
# Self-verifies at the end; fails the build loudly if it did not take.
# ============================================================
set -euo pipefail

FALCON_SRC="/catkin_ws/src/FALCON"
PATCH="/tmp/falcon_visgrid_cadence.patch"

if [ ! -f "${PATCH}" ]; then
  echo "[fix_visgrid] ERROR: ${PATCH} not found (COPY it in the Dockerfile)." >&2
  exit 1
fi

cd "${FALCON_SRC}"
echo "[fix_visgrid] Applying falcon_visgrid_cadence.patch"

if git apply --check "${PATCH}" 2>/dev/null; then
  git apply "${PATCH}"
elif patch -p1 --forward --fuzz=3 < "${PATCH}"; then
  echo "[fix_visgrid] Applied via patch(1) with fuzz."
else
  echo "[fix_visgrid] ERROR: patch did not apply. Has upstream FALCON moved?" >&2
  exit 1
fi

SRC="falcon_planner/voxel_mapping/src/map_server.cpp"
ok=1
grep -q 'publish_bulk' "${SRC}" || { echo "[fix_visgrid] ERROR: publish_bulk sentinel missing" >&2; ok=0; }
[ "$(grep -c 'if (publish_bulk)' "${SRC}")" = "2" ] || { echo "[fix_visgrid] ERROR: expected 2 gated blocks" >&2; ok=0; }
[ "${ok}" = "1" ] || exit 1

echo "[fix_visgrid] OK: occupied stays 2 Hz; free/unknown publish every 10th cycle."
