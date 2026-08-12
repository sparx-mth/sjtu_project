#!/usr/bin/env bash
# ============================================================
# run_warehouse.sh — FALCON exploration in the Gazebo warehouse, HEADLESS,
# with a deadlock watchdog that STOPS the run the moment the drone is stuck.
#
#   ./run_warehouse.sh [world] [map] [budget_s]
#
# Brings up, all headless (no gzclient, no RViz):
#   1. Gazebo sim   (sjtu_drone, CycloneDDS, ROS_DOMAIN_ID=20)
#   2. roscore      (ros1_bridge image)
#   3. ros1_bridge  (dynamic, CycloneDDS domain 20 -- matches the sim)
#   4. FALCON       (gazebo_exploration.launch, map_name:=<map>)
#
# Then it WATCHES the flight and ends it as soon as it is going nowhere, so we
# stop wasting runtime staring at a drone that has crashed into a shelf:
#
#   * no_progress -- for NOPROG_WINDOW s the drone discovered < NOPROG_COV m3 of
#     new space AND moved < NOPROG_MOVE m. Wedged / crashed / stuck.
#   * crashed     -- the same, and the drone is > FAR_FROM_PATH m from the point
#     FALCON is commanding: it is far off the plan and not moving.
#   * explored    -- FALCON declared the space covered.
#   * time_budget -- the hard wall-clock cap.
#
# Obstacle tuning is passed straight to gazebo_exploration.launch so it can be
# iterated without editing anything:
#   INFLATE=0.35 SAFE=0.15 ASTAR_INFLATE=0.40 ./run_warehouse.sh
# ============================================================
set -uo pipefail

SJTU="${SJTU_PROJECT_DIR:-/home/nadavc/GIT/sjtu_project}"
FD="${SJTU}/falcon_docker"
BRINGUP="/home/nadavc/GIT/TheAgency/sparx_agency/robots/SJTU/setup/bringup_world.sh"

WORLD="${1:-small_warehouse}"
MAP="${2:-small_warehouse}"
BUDGET_S="${3:-300}"
FALCON_IMAGE="${FALCON_IMAGE:-falcon-ros-custom:v3}"
export SJTU_PROJECT_DIR="${SJTU}"
export DISPLAY="${DISPLAY:-:1}"
LOG_DIR="${LOG_DIR:-/tmp/falcon_warehouse}"
mkdir -p "${LOG_DIR}"

# ---- obstacle / planner tuning (overridable by env) ----
INFLATE="${INFLATE:-0.15}"          # HARD map inflation (ESDF margin for the B-spline)
SAFE="${SAFE:-0.15}"                # SOFT b-spline clearance; must fit the aisle
ASTAR_INFLATE="${ASTAR_INFLATE:-0.30}"   # A* airframe disc (v2+): 0.30 + INFLATE keeps
                                         # the drone out of gaps too small for it while
                                         # leaving the ~2 m aisles passable
ASTAR_START_CLEAR="${ASTAR_START_CLEAR:-0.50}"
MAX_VEL="${MAX_VEL:-0.25}"
# RVIZ=1 launches FALCON's own RViz (occupancy voxels, frontiers, trajectory) so
# the mapping can be watched live, and leaves the whole stack up afterwards for
# inspection. Gazebo itself stays HEADLESS regardless (no gzclient).
WANT_RVIZ="${RVIZ:-0}"

# ---- deadlock thresholds ----
POLL_S=5
NOPROG_COV="${NOPROG_COV:-3.0}"     # m3 new coverage = progress
NOPROG_MOVE="${NOPROG_MOVE:-0.8}"   # m drone movement = progress
NOPROG_WINDOW="${NOPROG_WINDOW:-25}"   # s of neither = deadlock
FAR_FROM_PATH="${FAR_FROM_PATH:-3.0}"  # m drone-to-command gap when stuck = crashed

RED=$'\033[31m'; GRN=$'\033[32m'; CYA=$'\033[36m'; NC=$'\033[0m'
say(){ echo -e "${CYA}[warehouse]${NC} $*"; }

cleanup(){ docker rm -f falcon ros1_bridge roscore "sjtu_drone_${WORLD}" >/dev/null 2>&1 || true; }
# In RViz mode, leave everything running at the end so the map stays on screen.
on_exit(){ if [[ "${WANT_RVIZ}" == 1 ]]; then say "stack left running for inspection -- stop it with: docker rm -f falcon ros1_bridge roscore sjtu_drone_${WORLD}"; else cleanup; fi; }
trap on_exit EXIT

fexec(){ docker exec falcon bash -lc "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://localhost:11311 && $*" 2>/dev/null; }
coverage(){ fexec "grep -oE 'Coverage: [0-9.]+' /tmp/adapter.log 2>/dev/null | tail -1 | grep -oE '[0-9.]+' | tail -1"; }
# position of a (possibly nested) topic field -> "x y z"
topic_xyz(){ fexec "timeout 4 rostopic echo -n1 $1 2>/dev/null" | awk '/x:/{x=$2} /y:/{y=$2} /z:/{z=$2} END{if(x=="")print "nan nan nan"; else print x, y, z}'; }
# FALCON's commanded position -- block-parse the first x/y/z under "position:"
# (a field selector on the custom PositionCommand type comes back empty).
cmd_xyz(){ fexec "timeout 4 rostopic echo -n1 /planning/pos_cmd 2>/dev/null" | awk 'BEGIN{f=0} /^position:/{f=1;next} f&&/x:/&&!gx{x=$2;gx=1} f&&/y:/&&!gy{y=$2;gy=1} f&&/z:/&&!gz{z=$2;gz=1} END{if(gx&&gy&&gz)print x, y, z; else print "nan nan nan"}'; }
dist(){ awk -v a="$1" -v b="$2" 'BEGIN{split(a,A," ");split(b,B," "); if(A[1]=="nan"||B[1]=="nan"){print 999; exit} dx=A[1]-B[1];dy=A[2]-B[2];dz=A[3]-B[3];print sqrt(dx*dx+dy*dy+dz*dz)}'; }

say "world=${WORLD} map=${MAP} image=${FALCON_IMAGE} budget=${BUDGET_S}s"
say "tuning: inflate=${INFLATE} safe=${SAFE} astar_inflate=${ASTAR_INFLATE} max_vel=${MAX_VEL}"
cleanup

# ---- 1. Gazebo sim, HEADLESS ----
say "1/4 Gazebo (${WORLD}) headless ..."
nohup bash "${BRINGUP}" --headless --skip-build "${WORLD}" > "${LOG_DIR}/gazebo.log" 2>&1 &
for i in $(seq 1 60); do
  n=$(docker exec "sjtu_drone_${WORLD}" bash -lc 'source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=20 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///etc/cyclonedds/no_shm.xml; ros2 topic list 2>/dev/null | grep -c simple_drone' 2>/dev/null || echo 0)
  [[ "${n:-0}" -ge 15 ]] && { say "  sim publishing (${n} topics)"; break; }
  sleep 2
  [[ $i -eq 60 ]] && { echo "${RED}sim did not come up${NC}"; exit 1; }
done

# ---- 2. roscore ----
say "2/4 roscore ..."
docker run -d --rm --net=host --name=roscore --entrypoint bash ros1_bridge:noetic-foxy \
  -c "source /opt/ros/noetic/setup.bash && roscore" >/dev/null
sleep 4

# ---- 3. bridge (CycloneDDS domain 20) ----
say "3/4 ros1_bridge (cyclonedds domain 20) ..."
docker run -d --rm --net=host --name=ros1_bridge \
  -e ROS_MASTER_URI=http://localhost:11311 -e ROS_HOSTNAME=localhost \
  -e ROS_DOMAIN_ID=20 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  --entrypoint bash ros1_bridge:noetic-foxy \
  -c "source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && source /bridge_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=20 && exec ros2 run ros1_bridge dynamic_bridge --bridge-all-topics" >/dev/null
sleep 6

# ---- 4. FALCON ----
say "4/4 FALCON (gazebo_exploration.launch$( [[ "${WANT_RVIZ}" == 1 ]] && echo ' + RViz' )) ..."
docker rm -f falcon >/dev/null 2>&1 || true
docker run -d --rm --name falcon --gpus all --net=host \
  -e DISPLAY="${DISPLAY}" -e QT_X11_NO_MITSHM=1 -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_MASTER_URI=http://localhost:11311 \
  --shm-size=2g --ulimit nofile=65536:65536 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${FD}/adapter/launch/gazebo_exploration.launch:/catkin_ws/src/falcon_adapter/launch/gazebo_exploration.launch:ro" \
  -v "${FD}/adapter/scripts/cmd_to_vel.py:/catkin_ws/src/falcon_adapter/scripts/cmd_to_vel.py:ro" \
  -v "${FD}/${MAP}.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/${MAP}.yaml:ro" \
  -v "${FD}/rviz_warehouse.rviz:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/rviz.rviz:ro" \
  --entrypoint bash "${FALCON_IMAGE}" -c "sleep infinity" >/dev/null
sleep 3
# RViz FIRST, so the occupancy map, frontiers and trajectory can be watched
# building from the very first frame. NVIDIA GL offload or it falls back to
# software rendering. Gazebo stays headless -- this is the only viewer.
if [[ "${WANT_RVIZ}" == 1 ]]; then
  xhost +local:docker >/dev/null 2>&1 || true
  say "  RViz up -- watch the map, frontiers and trajectory build live"
  # rosrun, not roslaunch: the rviz.launch wrapper exits the moment rviz's window
  # settles ("process has finished cleanly"), taking RViz down; rosrun holds it.
  docker exec -d falcon bash -lc "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://localhost:11311 && export DISPLAY=${DISPLAY} && export __NV_PRIME_RENDER_OFFLOAD=1 && export __GLX_VENDOR_LIBRARY_NAME=nvidia && rosrun rviz rviz -d /catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/rviz.rviz > /tmp/rviz.log 2>&1"
  sleep 5
fi
docker exec -d falcon bash -lc "source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && export ROS_MASTER_URI=http://localhost:11311 && export DISPLAY=${DISPLAY} && roslaunch falcon_adapter gazebo_exploration.launch map_name:=${MAP} obstacles_inflation:=${INFLATE} safe_distance:=${SAFE} astar_inflate:=${ASTAR_INFLATE} astar_start_clearance:=${ASTAR_START_CLEAR} max_vel:=${MAX_VEL} > /tmp/adapter.log 2>&1"

# ---- watchdog ----
say "watching (no_progress: <${NOPROG_COV} m3 new AND <${NOPROG_MOVE} m moved for ${NOPROG_WINDOW}s)"
start=$(date +%s)
base_t=$start
base_cov=0
base_pos="nan nan nan"
outcome=""; detail=""
while true; do
  now=$(date +%s); el=$((now-start))
  cov=$(coverage); cov=${cov:-0}
  dpos=$(topic_xyz "/simple_drone/odom/pose/pose/position")
  cpos=$(cmd_xyz)
  gap=$(dist "$dpos" "$cpos")
  moved=$(dist "$dpos" "$base_pos")
  dcov=$(awk -v a="$cov" -v b="$base_cov" 'BEGIN{print a-b}')

  printf '[warehouse] t=%4ss cov=%-8s drone=(%s) cmd=(%s) gap=%.2fm moved=%.2fm\n' \
    "$el" "$cov" "$(echo $dpos|tr ' ' ',')" "$(echo $cpos|tr ' ' ',')" "$gap" "$moved"

  # explored?
  if fexec "grep -qiE 'Finish exploration|exploration finished' /tmp/adapter.log"; then
    outcome="explored"; detail="FALCON declared the space covered"; break; fi
  # container died?
  docker ps --format '{{.Names}}' | grep -q '^falcon$' || { outcome="falcon_died"; detail="the FALCON container exited"; break; }

  # progress resets the window
  if awk -v d="$dcov" -v c="$NOPROG_COV" -v m="$moved" -v mm="$NOPROG_MOVE" 'BEGIN{exit !(d>=c || m>=mm)}'; then
    base_t=$now; base_cov=$cov; base_pos="$dpos"
  else
    if (( now-base_t >= NOPROG_WINDOW )); then
      if awk -v g="$gap" -v f="$FAR_FROM_PATH" 'BEGIN{exit !(g>=f)}'; then
        outcome="crashed"; detail="stuck at ($dpos), ${gap}m off the commanded path, no new voxels"
      else
        outcome="no_progress"; detail="stuck at ($dpos), <${NOPROG_MOVE}m moved and <${NOPROG_COV}m3 new for ${NOPROG_WINDOW}s"
      fi
      break
    fi
  fi
  if (( el >= BUDGET_S )); then outcome="time_budget"; detail="reached the ${BUDGET_S}s wall-clock budget"; break; fi
  sleep "$POLL_S"
done

echo
case "$outcome" in
  explored|time_budget) echo -e "${GRN}[warehouse] OUTCOME: ${outcome}${NC} — ${detail}";;
  *)                    echo -e "${RED}[warehouse] OUTCOME: ${outcome}${NC} — ${detail}";;
esac
say "final coverage: $(coverage) m3   drone: $(topic_xyz /simple_drone/odom/pose/pose/position)"
say "FALCON, last words:"
fexec "grep -iE 'Transit state|No path|No frontier|Total time too long|Finish|discontinuity' /tmp/adapter.log | tail -12"
docker cp falcon:/tmp/adapter.log "${LOG_DIR}/${WORLD}_falcon.log" >/dev/null 2>&1 || true
say "full FALCON log: ${LOG_DIR}/${WORLD}_falcon.log"
[[ "$outcome" == "explored" || "$outcome" == "time_budget" ]]

