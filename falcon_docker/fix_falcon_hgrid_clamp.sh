#!/bin/bash
# ============================================================
# fix_falcon_hgrid_clamp.sh
#
# Bounds-checks UniformGrid::positionToGridCellCenterId. Applies
# falcon_hgrid_clamp.patch, touching ONE file no other fix script does:
#
#   falcon_planner/exploration_preprocessing/src/hierarchical_grid.cpp
#
# Root cause: a position outside the hgrid box (aircraft below the box floor
# during takeoff, or past a face) yields an out-of-range cell id that indexes
# a vector unchecked -- a segfault that killed the planner 14 times in 8
# minutes once the flight box floor was raised above takeoff height. Upstream
# knew: the validity CHECKs at the bottom of the function are commented out.
# The patch clamps to the nearest in-box position, which is the correct
# answer to the caller's question ("which tour cell is the aircraft in").
# ============================================================
set -euo pipefail
FALCON_SRC="/catkin_ws/src/FALCON"
PATCH="/tmp/falcon_hgrid_clamp.patch"
[ -f "${PATCH}" ] || { echo "[fix_hgrid] ERROR: ${PATCH} missing" >&2; exit 1; }
cd "${FALCON_SRC}"
if git apply --check "${PATCH}" 2>/dev/null; then git apply "${PATCH}";
elif patch -p1 --forward --fuzz=3 < "${PATCH}"; then echo "[fix_hgrid] applied with fuzz";
else echo "[fix_hgrid] ERROR: patch did not apply" >&2; exit 1; fi
grep -q 'nearest cell IS the answer' falcon_planner/exploration_preprocessing/src/hierarchical_grid.cpp \
  || { echo "[fix_hgrid] ERROR: sentinel missing" >&2; exit 1; }
echo "[fix_hgrid] OK: out-of-box positions clamp instead of segfaulting."
