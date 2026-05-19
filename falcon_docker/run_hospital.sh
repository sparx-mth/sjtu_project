#!/bin/bash
# ============================================================
# falcon_docker/run_hospital.sh — FALCON + external Gazebo drone
#                                  OR real-drone on Jetson
#
# v18:
#   - Auto-detects arch and uses the right NVIDIA flag:
#       x86_64  → --gpus all  (nvidia-container-toolkit)
#       aarch64 → --runtime nvidia + NVIDIA_VISIBLE_DEVICES=all
#                 (works on JetPack 4.x and 5.x+ alike)
#   - Falls back to no-GPU if neither is available, with a warning.
#   - Picks image tag based on arch (falcon-ros:noetic vs
#     falcon-ros:jetson) so you don't accidentally launch the
#     x86 image on Jetson.
#   - All other behaviour preserved from v17.
# ============================================================

CONTAINER="falcon"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ARCH=$(uname -m)
if [ "${ARCH}" = "aarch64" ]; then
  IMAGE="${IMAGE:-falcon-ros:jetson}"
else
  IMAGE="${IMAGE:-falcon-ros:noetic}"
fi
echo "[INFO] Arch: ${ARCH}   Image: ${IMAGE}"

# ── GPU flag selection ────────────────────────────────────────
# On Jetson, --gpus all only works on JetPack 5.x+ with
# nvidia-container-toolkit installed. The legacy --runtime nvidia
# is universally supported on Jetson (and is what the L4T docs
# recommend). On x86 we keep --gpus all.
GPU_ARGS=""
if [ "${ARCH}" = "aarch64" ]; then
  # Verify the nvidia runtime is actually registered with docker.
  if docker info 2>/dev/null | grep -q "Runtimes:.*nvidia"; then
    GPU_ARGS="--runtime nvidia \
              --env NVIDIA_VISIBLE_DEVICES=all \
              --env NVIDIA_DRIVER_CAPABILITIES=all"
    echo "[INFO] GPU: --runtime nvidia (Jetson)"
  else
    echo "[WARN] nvidia runtime not registered with docker.        "
    echo "[WARN] Edit /etc/docker/daemon.json so it contains:      "
    echo "[WARN]   { \"runtimes\": { \"nvidia\": {                 "
    echo "[WARN]       \"path\": \"nvidia-container-runtime\",     "
    echo "[WARN]       \"runtimeArgs\": [] } } }                   "
    echo "[WARN] then 'sudo systemctl restart docker'. Running    "
    echo "[WARN] CPU-only for now (RViz/Gazebo will be very slow)."
  fi
else
  # x86_64: prefer modern --gpus all; warn if missing.
  if docker info 2>/dev/null | grep -q "Runtimes:.*nvidia"; then
    GPU_ARGS="--gpus all \
              --env NVIDIA_DRIVER_CAPABILITIES=all \
              --env NVIDIA_VISIBLE_DEVICES=all"
    echo "[INFO] GPU: --gpus all"
  else
    echo "[WARN] No nvidia runtime detected; running CPU-only."
  fi
fi

# ── Map config ────────────────────────────────────────────────
ENV_NAME="${1:-hospital}"
if [[ $# -ge 1 ]]; then shift; fi

if [[ ! -f "${SCRIPT_DIR}/${ENV_NAME}.yaml" ]]; then
  echo "[ERROR] Map config not found: ${SCRIPT_DIR}/${ENV_NAME}.yaml"
  echo "        Available configs:"
  ls -1 "${SCRIPT_DIR}"/*.yaml 2>/dev/null | xargs -n1 basename || true
  exit 1
fi
echo "[INFO] FALCON env: ${ENV_NAME}  (config: ${SCRIPT_DIR}/${ENV_NAME}.yaml)"

# Auto-chmod +x on host so we don't lose 10 min wondering why nodes
# aren't found. Harmless if they were already executable.
chmod +x "${SCRIPT_DIR}"/adapter/scripts/*.py 2>/dev/null || true

xhost +local:docker 2>/dev/null || true

# ── Volume mounts ─────────────────────────────────────────────
# Mount each adapter script that exists on the host. The original
# v17 list hardcodes 16 mounts — if a file doesn't exist on the
# host (e.g. you trimmed batch_runner.py because you don't run
# batches on Jetson) docker would create an empty directory at
# the target path and the rosrun would fail mysteriously. Loop
# instead so missing files are silently skipped with a single log
# line at startup.
SCRIPTS_HOST="${SCRIPT_DIR}/adapter/scripts"
SCRIPTS_TARGET="/catkin_ws/src/falcon_adapter/scripts"
SCRIPT_MOUNTS=()
for f in falcon_adapter.py cmd_to_vel.py bev_publisher.py \
         exploration_monitor.py run_recorder.py completion_watcher.py \
         batch_runner.py respawn_drone.py sensor_gate.py astar_planner.py \
         waypoint_follower.py voxel_reset_watcher.py bev_click_goal.py \
         pose_adapter.py visual_servoing_controller.py nav_geom.py \
         navdp_client.py pixel_goal_tracker.py trajectory_tracker.py ; do
  if [ -f "${SCRIPTS_HOST}/${f}" ]; then
    SCRIPT_MOUNTS+=( --volume "${SCRIPTS_HOST}/${f}:${SCRIPTS_TARGET}/${f}" )
  else
    echo "[INFO] Skipping missing script: ${f}"
  fi
done

LAUNCH_HOST="${SCRIPT_DIR}/adapter/launch"
LAUNCH_TARGET="/catkin_ws/src/falcon_adapter/launch"
LAUNCH_MOUNTS=()
for f in gazebo_exploration.launch gazebo_waypoint_nav.launch real_drone.launch visual_servoing.launch ; do
  if [ -f "${LAUNCH_HOST}/${f}" ]; then
    LAUNCH_MOUNTS+=( --volume "${LAUNCH_HOST}/${f}:${LAUNCH_TARGET}/${f}" )
  fi
done

# docker.sock is only needed when respawn_drone.py is in play
# (sim-only). On Jetson it's harmless to mount but pointless.
DOCKER_SOCK_MOUNT=()
if [ "${ARCH}" != "aarch64" ] && [ -S /var/run/docker.sock ]; then
  DOCKER_SOCK_MOUNT=( --volume /var/run/docker.sock:/var/run/docker.sock )
fi

# ── Run ───────────────────────────────────────────────────────
docker run -it --rm \
    --name "${CONTAINER}" \
    ${GPU_ARGS} \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --shm-size=2g \
    --ulimit nofile=65536:65536 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${SCRIPT_MOUNTS[@]}" \
    "${LAUNCH_MOUNTS[@]}" \
    "${DOCKER_SOCK_MOUNT[@]}" \
    --volume "${SCRIPT_DIR}/${ENV_NAME}.yaml:/catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/${ENV_NAME}.yaml" \
    --volume "${SCRIPT_DIR}/runs:/home/falcon/runs" \
    --network host \
    "${IMAGE}" \
    "${@:-bash}"