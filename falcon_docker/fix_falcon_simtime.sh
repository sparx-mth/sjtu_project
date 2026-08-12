#!/bin/bash
# ============================================================
# fix_falcon_simtime.sh
#
# Lets FALCON run under /use_sim_time. Applies falcon_simtime.patch,
# which touches ONE upstream file no other fix_falcon_*.sh does:
#
#   falcon_planner/exploration_manager/src/exploration_node.cpp
#
# Root cause it fixes:
#   exploration_node aborts at startup (glog CHECK) when the global
#   /use_sim_time param is true. In the SJTU Gazebo deployment every
#   data stamp in the graph is ALREADY Gazebo sim time (the bridged
#   depth, odometry, and the camera pose derived from it), so wall
#   clocks make ros::Time::now() run ~12% ahead of the physics
#   (hospital real-time factor ~0.88): B-splines are stamped on a
#   faster clock than the aircraft flies, and the follower burns its
#   catch-up margin closing pure clock skew. Sim time is the only
#   consistent configuration here; the upstream check merely predates
#   anyone running FALCON against a sub-realtime simulator.
#
# What the patch does: replaces the CHECK with a LOG(INFO) noting the
# clock choice. Nothing else in FALCON special-cases wall time.
#
# Applied BEFORE the catkin_make step so it is compiled in.
# Self-verifies at the end; fails the build loudly if it did not take.
# ============================================================
set -euo pipefail

FALCON_SRC="/catkin_ws/src/FALCON"
PATCH="/tmp/falcon_simtime.patch"

if [ ! -f "${PATCH}" ]; then
  echo "[fix_simtime] ERROR: ${PATCH} not found (COPY it in the Dockerfile)." >&2
  exit 1
fi
if [ ! -d "${FALCON_SRC}" ]; then
  echo "[fix_simtime] ERROR: ${FALCON_SRC} not found." >&2
  exit 1
fi

cd "${FALCON_SRC}"
echo "[fix_simtime] Applying falcon_simtime.patch"

if git apply --check "${PATCH}" 2>/dev/null; then
  git apply "${PATCH}"
elif patch -p1 --forward --fuzz=3 < "${PATCH}"; then
  echo "[fix_simtime] Applied via patch(1) with fuzz (git apply did not match cleanly)."
else
  echo "[fix_simtime] ERROR: patch did not apply. Has upstream FALCON moved?" >&2
  exit 1
fi

NODE="falcon_planner/exploration_manager/src/exploration_node.cpp"
ok=1
grep -q 'sim-time permitted' "${NODE}" || { echo "[fix_simtime] ERROR: sentinel missing in exploration_node" >&2; ok=0; }
if grep -q 'CHECK(!use_sim_time)' "${NODE}"; then
  echo "[fix_simtime] ERROR: the use_sim_time CHECK is still present" >&2; ok=0
fi
[ "${ok}" = "1" ] || exit 1

echo "[fix_simtime] OK: FALCON accepts /use_sim_time; the deployment can run on one clock."
