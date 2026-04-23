#!/bin/bash
# ============================================================
# perception_docker/run_perception.sh
#
# Runs the semantic mapper on ROS2 Humble, sharing ROS_DOMAIN_ID
# with the sim container.
#
# v2: forwards LLM_* env vars into the container so the new
#     room_classifier_node and llm_oracle_node can reach a local
#     (Ollama) or remote (OpenAI-compat) LLM backend.
#
# Env vars (all optional; defaults shown in brackets):
#   LLM_BACKEND   [ollama]       ollama | openai
#   LLM_BASE_URL  [http://localhost:11434]
#   LLM_MODEL     [qwen2.5:3b-instruct]
#   LLM_API_KEY   []             only for openai-compat servers
#   LLM_TEMPERATURE [0.2]
#
# Usage:
#   ./run_perception.sh                       # interactive shell
#   ./run_perception.sh ros2 launch semantic_mapper semantic_pipeline.launch.py
# ============================================================
set -eo pipefail

IMAGE="perception:humble"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "[INFO] Image '${IMAGE}' not found — building (first time ~5 min)..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

xhost +local:docker >/dev/null 2>&1 || true

# Log the effective LLM config so the user can see what the container will try.
echo "[INFO] LLM_BACKEND=${LLM_BACKEND:-ollama}  LLM_MODEL=${LLM_MODEL:-qwen2.5:3b-instruct}  LLM_BASE_URL=${LLM_BASE_URL:-http://localhost:11434}"

docker run -it --rm \
    --name perception \
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
    --env LLM_TEMPERATURE="${LLM_TEMPERATURE:-0.2}" \
    --env LLM_TIMEOUT_S="${LLM_TIMEOUT_S:-30}" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${SCRIPT_DIR}/semantic_mapper:/ros2_ws/src/semantic_mapper:rw" \
    --volume "${HOME}/.cache/torch:/root/.cache/torch:rw" \
    --volume "${HOME}/.cache/ultralytics:/root/.cache/ultralytics:rw" \
    "${IMAGE}" \
    bash -c '
        source /opt/ros/humble/setup.bash
        cd /ros2_ws
        echo "[INFO] Building semantic_mapper..."
        colcon build --packages-select semantic_mapper --symlink-install \
            --cmake-args -DBUILD_TESTING=OFF
        source install/setup.bash
        echo "[INFO] Ready."
        echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}"
        echo "[INFO] LLM: backend=${LLM_BACKEND} model=${LLM_MODEL} url=${LLM_BASE_URL}"
        exec "$@"
    ' bash "${@:-bash}"

xhost -local:docker >/dev/null 2>&1 || true