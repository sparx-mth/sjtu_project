#!/bin/bash
# ============================================================
# fix_falcon_deadend_guard.sh
#
# Teaches FALCON's coverage tour to give up on a viewpoint the
# aircraft cannot physically reach, instead of grinding on it
# forever. Applies falcon_deadend_guard.patch, which touches three
# upstream files and NONE that the other fix_falcon_*.sh scripts do:
#
#   falcon_planner/exploration_manager/src/exploration_manager.cpp
#   falcon_planner/exploration_preprocessing/include/.../frontier_finder.h
#   falcon_planner/exploration_preprocessing/src/frontier_finder.cpp
#
# Root cause it fixes:
#   The depth camera is blind inside ~0.95 m (its near clip), so an
#   obstacle the aircraft is about to hit never enters the map.
#   FALCON's A* then routes a "clear" path straight through it and
#   re-selects the same blocked viewpoint every cycle -- FALCON only
#   marks a frontier dormant when it has NO viewpoint at all, never
#   when the viewpoint is simply unreachable. The result is a
#   permanent stall (coverage frozen, "next_pos ... same as current",
#   or the aircraft wandering a 1-2 m pocket) that only a crash used
#   to break by wiping the map.
#
# What the patch adds:
#   * FrontierFinder::addBlockedRegion(pos) + a blocked_regions_ list;
#     computeFrontiersToVisit() retires to dormant any cluster whose
#     average falls within blocked_region_radius (default 2.5 m, ROS
#     param /frontier_finder/blocked_region_radius) of a blocked point.
#   * A dead-end guard in ExplorationManager::planExploreMotionHGrid():
#     if the aircraft's whole excursion stays under 2 m for 25 s while
#     it is still not at the chosen viewpoint, that viewpoint is handed
#     to addBlockedRegion() and the tour moves on.
#
# Applied BEFORE the catkin_make step so it is compiled in.
# Self-verifies at the end; fails the build loudly if it did not take.
# ============================================================
set -euo pipefail

FALCON_SRC="/catkin_ws/src/FALCON"
PATCH="/tmp/falcon_deadend_guard.patch"

if [ ! -f "${PATCH}" ]; then
  echo "[fix_deadend] ERROR: ${PATCH} not found (COPY it in the Dockerfile)." >&2
  exit 1
fi
if [ ! -d "${FALCON_SRC}" ]; then
  echo "[fix_deadend] ERROR: ${FALCON_SRC} not found." >&2
  exit 1
fi

cd "${FALCON_SRC}"
echo "[fix_deadend] Applying falcon_deadend_guard.patch"

# Prefer git apply (the tree is a git clone); fall back to patch(1) with
# fuzz so a small upstream shift on the ros1-noetic branch does not wedge
# the build. Either path leaves the same result, verified below.
if git apply --check "${PATCH}" 2>/dev/null; then
  git apply "${PATCH}"
elif patch -p1 --forward --fuzz=3 < "${PATCH}"; then
  echo "[fix_deadend] Applied via patch(1) with fuzz (git apply did not match cleanly)."
else
  echo "[fix_deadend] ERROR: patch did not apply. Has upstream FALCON moved?" >&2
  exit 1
fi

# Self-verify: the three sentinels the patch introduces must now be present.
MGR="falcon_planner/exploration_manager/src/exploration_manager.cpp"
HDR="falcon_planner/exploration_preprocessing/include/exploration_preprocessing/frontier_finder.h"
FF="falcon_planner/exploration_preprocessing/src/frontier_finder.cpp"

ok=1
grep -q 'addBlockedRegion' "${HDR}" || { echo "[fix_deadend] ERROR: addBlockedRegion missing in header" >&2; ok=0; }
grep -q 'blocked_regions_'  "${FF}"  || { echo "[fix_deadend] ERROR: blocked_regions_ missing in frontier_finder" >&2; ok=0; }
grep -q 'shadowed from the coverage tour' "${FF}" || { echo "[fix_deadend] ERROR: shadow log missing in frontier_finder" >&2; ok=0; }
# NB: single-line sentinel -- the guard's full log line is split across two C
# string literals, so grepping the whole phrase would spuriously miss.
grep -q 'confined to <2 m' "${MGR}" || { echo "[fix_deadend] ERROR: dead-end guard missing in exploration_manager" >&2; ok=0; }
grep -q 'never enter LKH' "${MGR}" || { echo "[fix_deadend] ERROR: LKH degenerate-tour guard missing" >&2; ok=0; }
grep -q 'blocked_regions_runtime' "${FF}" || { echo "[fix_deadend] ERROR: blocked-region persistence missing" >&2; ok=0; }
[ "${ok}" = "1" ] || exit 1

echo "[fix_deadend] OK: FALCON now shadows unreachable viewpoints and the coverage tour survives dead ends."

