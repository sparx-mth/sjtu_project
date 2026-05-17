#!/bin/bash
# ============================================================
# room_docker/run_room_search.sh
#
# Runs the room_search orchestrator + YOLO detector + object_mapper +
# target_watcher in one container on ROS2 Humble. Shares ROS_DOMAIN_ID
# with the sim / perception containers.
#
# Auto-detects host architecture and picks the right Dockerfile + GPU
# runtime flag:
#
#   x86_64    Dockerfile          image  room_search:humble
#             --gpus all          (NVIDIA Container Toolkit on Linux)
#
#   aarch64   Dockerfile.jetson   image  room_search:humble-jetson
#             --runtime nvidia    (NVIDIA Container Runtime on L4T)
#             --ipc=host          (shared CUDA contexts; the dustynv
#                                  base image expects this)
#
# Override the detection:
#   ROOM_DOCKER_TARGET=jetson ./run_room_search.sh ...
#   ROOM_DOCKER_TARGET=x86    ./run_room_search.sh ...
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ── Target detection ────────────────────────────────────────
ARCH="$(uname -m)"
TARGET="${ROOM_DOCKER_TARGET:-}"
if [[ -z "${TARGET}" ]]; then
    case "${ARCH}" in
        aarch64) TARGET="jetson" ;;
        x86_64)  TARGET="x86"    ;;
        *)
            echo "[ERROR] Unknown arch '${ARCH}'. Set ROOM_DOCKER_TARGET=jetson|x86 explicitly."
            exit 1
            ;;
    esac
fi

case "${TARGET}" in
    jetson)
        DOCKERFILE="Dockerfile.jetson"
        IMAGE="room_search:humble-jetson"
        # Jetson uses --runtime nvidia (NVIDIA Container Runtime / L4T)
        # rather than --gpus all (NVIDIA Container Toolkit / x86).
        # --ipc=host is required by the dustynv base image so PyTorch's
        # shared-memory CUDA contexts work end-to-end.
        GPU_FLAGS=(--runtime nvidia --ipc=host)
        ;;
    x86)
        DOCKERFILE="Dockerfile"
        IMAGE="room_search:humble"
        GPU_FLAGS=(--gpus all)
        ;;
    *)
        echo "[ERROR] ROOM_DOCKER_TARGET must be 'jetson' or 'x86' (got '${TARGET}')."
        exit 1
        ;;
esac

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
    echo "[INFO] Image '${IMAGE}' not found — building from ${DOCKERFILE} (first time ~5–15 min)..."
    docker build -f "${SCRIPT_DIR}/${DOCKERFILE}" -t "${IMAGE}" "${SCRIPT_DIR}"
fi

xhost +local:docker >/dev/null 2>&1 || true

echo "[INFO] target=${TARGET}  arch=${ARCH}  image=${IMAGE}"
echo "[INFO] mount semantic_mapper := ${SEMANTIC_MAPPER_DIR}"
echo "[INFO] mount room_search     := ${ROOM_SEARCH_DIR}"

# Volumes that exist on the host. ~/.cache/torch and ~/.cache/ultralytics
# are created if missing so a first-run download doesn't fail; on Jetson
# the ultralytics cache is where YOLO-World checkpoints land, which is
# painful to re-download over a tethered link.
mkdir -p "${HOME}/.cache/torch" "${HOME}/.cache/ultralytics"

docker run -it --rm \
    --name room_search \
    "${GPU_FLAGS[@]}" \
    --net=host \
    --env DISPLAY="${DISPLAY:-}" \
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
