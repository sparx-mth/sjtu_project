#!/bin/bash
# ============================================================
# room_docker/run_room_search.sh
#
# Runs the room_search orchestrator + YOLO detector + object_mapper +
# target_watcher in one container on ROS2 Humble. Shares ROS_DOMAIN_ID
# with the sim / perception containers.
#
# Bind-mounts BOTH semantic_mapper (from ../perception_docker) and
# room_search (from this folder) into the container's /ros2_ws/src,
# so a colcon build inside the entrypoint produces both packages.
#
# Env vars (all optional; defaults shown):
#   ROS_DOMAIN_ID    [20]
#   LLM_BACKEND      [ollama]
#   LLM_BASE_URL     [http://localhost:11434]
#   LLM_MODEL        [qwen2.5:3b-instruct]
#
# Usage:
#   ./run_room_search.sh                        # interactive shell
#   ./run_room_search.sh ros2 launch room_search room_search.launch.py \
#         target_object:=keyboard room_center_x:=4.0 room_center_y:=5.0
# ============================================================
set -eo pipefail

IMAGE="room_search:humble"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SEMANTIC_MAPPER_DIR="${REPO_ROOT}/perception_docker/semantic_mapper"
ROOM_SEARCH_DIR="${SCRIPT_DIR}/room_search"

if [ ! -d "${SEMANTIC_MAPPER_DIR}" ]; then
    echo "[ERROR] semantic_mapper not found at ${SEMANTIC_MAPPER_DIR}"
    echo "        The room_search launch reuses yolo_detector / object_mapper_node /"
    echo "        target_watcher_node from semantic_mapper. Check your checkout."
    exit 1
fi
if [ ! -d "${ROOM_SEARCH_DIR}" ]; then
    echo "[ERROR] room_search package not found at ${ROOM_SEARCH_DIR}"
    exit 1
fi

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found — building (first time ~5 min)..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

xhost +local:docker >/dev/null 2>&1 || true

echo "[INFO] mount semantic_mapper := ${SEMANTIC_MAPPER_DIR}"
echo "[INFO] mount room_search     := ${ROOM_SEARCH_DIR}"

docker run -it --rm \
    --name room_search \
    --gpus all \
    --net=host \
    --env DISPLAY="${DISPLAY}" \
    --env QT_X11_NO_MITSHM=1 \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --env ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}" \
    --env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --env LLM_BACKEND="${LLM_BACKEND:-ollama}" \
    --env LLM_BASE_URL="${LLM_BASE_URL:-http://localhost:11434}" \
    --env LLM_MODEL="${LLM_MODEL:-qwen2.5:3b-instruct}" \
    --env LLM_API_KEY="${LLM_API_KEY:-}" \
    --env LLM_TIMEOUT_S="${LLM_TIMEOUT_S:-30}" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${SEMANTIC_MAPPER_DIR}:/ros2_ws/src/semantic_mapper:rw" \
    --volume "${ROOM_SEARCH_DIR}:/ros2_ws/src/room_search:rw" \
    --volume "${HOME}/.cache/torch:/root/.cache/torch:rw" \
    --volume "${HOME}/.cache/ultralytics:/root/.cache/ultralytics:rw" \
    "${IMAGE}" \
    bash -c '
        source /opt/ros/humble/setup.bash
        cd /ros2_ws
        echo "[INFO] Building semantic_mapper + room_search..."
        colcon build --packages-select semantic_mapper room_search \
            --symlink-install \
            --cmake-args -DBUILD_TESTING=OFF
        source install/setup.bash
        echo "[INFO] Ready."
        echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}"
        exec "$@"
    ' bash "${@:-bash}"

xhost -local:docker >/dev/null 2>&1 || true
