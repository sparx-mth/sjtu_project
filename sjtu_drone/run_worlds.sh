#!/bin/bash
set -eo pipefail

# -----------------------------
# Config
# -----------------------------
ROS_DISTRO=humble
IMAGE_NAME="sjtu_drone_clean:humble_ros2"
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

# -----------------------------
# Detect workspace paths automatically
# -----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"
CONTAINER_WS="/root/$(basename "$WORKSPACE_DIR")"
HOST_SRC_DIR="${WORKSPACE_DIR}/src"

WORLD_REPOS=(
  "aws-robomaker-hospital-world"
  "aws-robomaker-bookstore-world"
  "aws-robomaker-small-house-world"
)

echo "[INFO] Host workspace:      ${WORKSPACE_DIR}"
echo "[INFO] Container workspace: ${CONTAINER_WS}"

# -----------------------------
# Parse arguments
# -----------------------------
SKIP_MAP=false
if [[ "${1:-}" == "--no-map" ]]; then
  SKIP_MAP=true
  shift
fi

if [[ -z "${1:-}" ]]; then
  echo "Usage: $0 [--no-map] <world_file>"
  echo ""
  echo "Examples:"
  echo "  $0 hospital.world"
  echo "  $0 bookstore.world"
  echo "  $0 small_house.world"
  echo ""
  echo "Available worlds:"
  for repo_name in "${WORLD_REPOS[@]}"; do
    repo_dir="${WORKSPACE_DIR}/${repo_name}"
    if [[ -d "${repo_dir}/worlds" ]]; then
      find "${repo_dir}/worlds" -maxdepth 1 -name "*.world" -printf "  %f   (%s)\n" "${repo_name}" 2>/dev/null || true
    fi
  done
  exit 1
fi

WORLD_FILE="$1"
WORLD_BASE="$(basename "${WORLD_FILE}" .world)"

# -----------------------------
# Find which repo contains the world
# -----------------------------
HOST_WORLD_REPO=""
HOST_WORLD_PATH=""
WORLD_REPO_NAME=""

SPAWN_X=1.0
SPAWN_Y=1.0
SPAWN_Z=2.0

case "${WORLD_FILE}" in
  bookstore.world)
    SPAWN_Y=3.0
    ;;
esac

for repo_name in "${WORLD_REPOS[@]}"; do
  repo_dir="${WORKSPACE_DIR}/${repo_name}"
  candidate="${repo_dir}/worlds/${WORLD_FILE}"
  if [[ -f "${candidate}" ]]; then
    HOST_WORLD_REPO="${repo_dir}"
    HOST_WORLD_PATH="${candidate}"
    WORLD_REPO_NAME="${repo_name}"
    break
  fi
done

if [[ -z "${HOST_WORLD_PATH}" ]]; then
  echo "[ERROR] World file not found: ${WORLD_FILE}"
  echo "[INFO] Searched repos:"
  for repo_name in "${WORLD_REPOS[@]}"; do
    echo "  ${WORKSPACE_DIR}/${repo_name}/worlds/${WORLD_FILE}"
  done
  echo ""
  echo "[INFO] Available worlds:"
  for repo_name in "${WORLD_REPOS[@]}"; do
    repo_dir="${WORKSPACE_DIR}/${repo_name}"
    if [[ -d "${repo_dir}/worlds" ]]; then
      find "${repo_dir}/worlds" -maxdepth 1 -name "*.world" -printf "  %f   (%s)\n" "${repo_name}" 2>/dev/null || true
    fi
  done
  exit 1
fi

WORLD_PATH="${CONTAINER_WS}/${WORLD_REPO_NAME}/worlds/${WORLD_FILE}"

echo "[INFO] Selected repo:       ${WORLD_REPO_NAME}"
echo "[INFO] Selected host world: ${HOST_WORLD_PATH}"
echo "[INFO] Container world:     ${WORLD_PATH}"

# -----------------------------
# Pre-run checks (host)
# -----------------------------
if [[ ! -d "${HOST_WORLD_REPO}" ]]; then
  echo "[ERROR] Missing directory: ${HOST_WORLD_REPO}"
  exit 1
fi

if [[ ! -f "${HOST_WORLD_PATH}" ]]; then
  echo "[ERROR] World file not found: ${HOST_WORLD_PATH}"
  exit 1
fi

# Clone gazebo_ros_2d_map only if needed (and not in --no-map)
if [[ "${SKIP_MAP}" == "false" ]]; then
  if [[ ! -d "${HOST_SRC_DIR}/gazebo_ros_2d_map" ]]; then
    echo "[INFO] Cloning gazebo_ros_2d_map into ${HOST_SRC_DIR} ..."
    mkdir -p "${HOST_SRC_DIR}"
    git clone https://github.com/Minipada/gazebo_ros_2d_map.git "${HOST_SRC_DIR}/gazebo_ros_2d_map"
  else
    echo "[INFO] gazebo_ros_2d_map already exists at ${HOST_SRC_DIR}/gazebo_ros_2d_map"
  fi
else
  echo "[INFO] --no-map: skipping gazebo_ros_2d_map clone check."
fi

# -----------------------------
# Ensure Docker image exists (build if missing)
# -----------------------------
if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "[INFO] Docker image '${IMAGE_NAME}' not found. Attempting to build from ${SCRIPT_DIR}/Dockerfile ..."
  if [[ ! -f "${SCRIPT_DIR}/Dockerfile" ]]; then
    echo "[ERROR] Dockerfile not found at ${SCRIPT_DIR}/Dockerfile"
    echo "        Please place your Dockerfile in sjtu_drone/ or pre-build the image:"
    echo "        docker build -t ${IMAGE_NAME} ${SCRIPT_DIR}"
    exit 1
  fi
  (cd "${SCRIPT_DIR}" && docker build -t "${IMAGE_NAME}" .)
fi

# -----------------------------
# X11 access for GUI apps (Gazebo/Rviz)
# -----------------------------
xhost +local:docker >/dev/null 2>&1 || true

# -----------------------------
# Run container
# -----------------------------
echo "[INFO] Using world: ${WORLD_PATH}"
docker run \
  -it --rm \
  --gpus all \
  --privileged \
  --net=host \
  -v "${XSOCK}:${XSOCK}" \
  -v "${XAUTH}:${XAUTH}" \
  -v "${WORKSPACE_DIR}:${CONTAINER_WS}:rw" \
  -v "${WORKSPACE_DIR}/sjtu_drone/models/april_tag_36h11_0:/root/.gazebo/models/april_tag_36h11_0:ro" \
  -e DISPLAY="${DISPLAY}" \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-15}" \
  -e XAUTHORITY="${XAUTH}" \
  -e QT_X11_NO_MITSHM=1 \
  -e SKIP_MAP="${SKIP_MAP}" \
  -e SPAWN_X="${SPAWN_X}" \
  -e SPAWN_Y="${SPAWN_Y}" \
  -e SPAWN_Z="${SPAWN_Z}" \
  --name="sjtu_drone_${WORLD_BASE}" \
  "${IMAGE_NAME}" \
  bash -c "
    set -eo pipefail

    # --- Source ROS + workspace ---
    source /opt/ros/${ROS_DISTRO}/setup.bash
    if [[ -f '${CONTAINER_WS}/install/setup.bash' ]]; then
      source '${CONTAINER_WS}/install/setup.bash'
    fi
    echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc
    echo '[[ -f ${CONTAINER_WS}/install/setup.bash ]] && source ${CONTAINER_WS}/install/setup.bash' >> /root/.bashrc

    # --- Core env ---
    apt-get update -qq && apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1 || true
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
    export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
    export GAZEBO_MODEL_DATABASE_URI=
    export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:\$GAZEBO_PLUGIN_PATH
    export GAZEBO_MASTER_URI=http://localhost:11345
    export ALSA_CARD=0
    export GAZEBO_AUDIO_DEVICE=null
    export GAZEBO_VERBOSE=1

    WORLD_REPO_PATH='${CONTAINER_WS}/${WORLD_REPO_NAME}'

    # World-specific assets
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$WORLD_REPO_PATH/models
    if [[ -d \"\$WORLD_REPO_PATH/fuel_models\" ]]; then
      export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$WORLD_REPO_PATH/fuel_models
    fi

    export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:\$WORLD_REPO_PATH/worlds
    export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:\$WORLD_REPO_PATH

    # SJTU assets
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/sjtu_drone/sjtu_drone_description
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/sjtu_drone/models

    if [[ -d '${CONTAINER_WS}/install/gazebo_ros_2d_map/lib' ]]; then
      export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:${CONTAINER_WS}/install/gazebo_ros_2d_map/lib
    fi

    echo '================================'
    echo 'Environment ready'
    echo 'World: ${WORLD_FILE}'
    echo 'Repo: ${WORLD_REPO_NAME}'
    echo 'GAZEBO_MODEL_PATH entries:'
    echo \$GAZEBO_MODEL_PATH | tr ':' '\n'
    echo 'GAZEBO_RESOURCE_PATH entries:'
    echo \$GAZEBO_RESOURCE_PATH | tr ':' '\n'
    echo '================================'

    # --- Sanity: world exists ---
    if [[ ! -f '${WORLD_PATH}' ]]; then
      echo '[ERROR] World file missing inside container: ${WORLD_PATH}'
      exit 1
    fi

    # --- Build ---
    echo '[INFO] Building workspace...'
    cd '${CONTAINER_WS}'

    rm -rf build/sjtu_drone_* install/sjtu_drone_* 2>/dev/null || true
    if [[ \"${SKIP_MAP}\" != 'true' ]]; then
      rm -rf build/gazebo_ros_2d_map install/gazebo_ros_2d_map 2>/dev/null || true
    fi

    colcon build --packages-select sjtu_drone_bringup sjtu_drone_description sjtu_drone_control autonomous_system \
      --cmake-args -DBUILD_TESTING=OFF -DRMW_IMPLEMENTATION=rmw_fastrtps_cpp

    if [[ \"${SKIP_MAP}\" != 'true' && -d '${CONTAINER_WS}/src/gazebo_ros_2d_map' ]]; then
      colcon build --packages-select gazebo_ros_2d_map --cmake-args -DBUILD_TESTING=OFF
    fi

    source install/setup.bash 2>/dev/null || true

    # --- Launch world ---
    echo '[INFO] Launching SJTU Drone...'
    ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py world:='${WORLD_PATH}' &
    LAUNCH_PID=\$!

    # Wait for gzserver
    for i in {1..60}; do
      if pgrep -x 'gzserver' >/dev/null; then
        echo '[INFO] Gazebo is running.'
        break
      fi
      sleep 1
    done

    # --- Optional: start 2D map plugin ---
    if [[ \"${SKIP_MAP}\" != 'true' ]] && ros2 pkg list | grep -q gazebo_ros_2d_map; then
      echo '[INFO] Starting 2D map plugin node...'
      mkdir -p '${CONTAINER_WS}/maps'
      ros2 run gazebo_ros_2d_map gazebo_ros_2d_map \
        --ros-args \
        -p map_name:='${WORLD_BASE}_map' \
        -p save_map:=true \
        -p map_path:='${CONTAINER_WS}/maps' \
        -p occupied_thresh:=0.65 \
        -p free_thresh:=0.196 &
    else
      echo '[INFO] 2D map plugin is disabled or not available.'
    fi

    wait \$LAUNCH_PID
  "

# Revoke X11 access
xhost -local:docker >/dev/null 2>&1 || true
echo "[INFO] Container exited."