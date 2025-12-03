#!/bin/bash
set -eo pipefail

# -----------------------------
# Config
# -----------------------------
ROS_DISTRO=humble
#IMAGE_NAME="sjtu_drone_clean:humble_ros2"
IMAGE_NAME="sjtu_drone_apritag:2"
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

# -----------------------------
# Detect workspace paths automatically
# -----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"                          # <workspace_root>
CONTAINER_WS="/root/$(basename "$WORKSPACE_DIR")"                 # e.g., /root/my_project
HOST_WORLDS_DIR="${WORKSPACE_DIR}/aws-robomaker-hospital-world/worlds"
HOST_SRC_DIR="${WORKSPACE_DIR}/src"

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
  echo "Worlds found in:"
  echo "  ${HOST_WORLDS_DIR}"
  ls -1 "${HOST_WORLDS_DIR}"/*.world 2>/dev/null | xargs -n1 basename || true
  exit 1
fi

WORLD_FILE="$1"
WORLD_BASE="$(basename "${WORLD_FILE}" .world)"
WORLD_PATH="${CONTAINER_WS}/aws-robomaker-hospital-world/worlds/${WORLD_FILE}"

# -----------------------------
# Pre-run checks (host)
# -----------------------------
if [[ ! -d "${WORKSPACE_DIR}/aws-robomaker-hospital-world" ]]; then
  echo "[ERROR] Missing directory: ${WORKSPACE_DIR}/aws-robomaker-hospital-world"
  echo "        Expected layout:"
  echo "        <workspace_root>/{sjtu_drone, aws-robomaker-hospital-world}"
  exit 1
fi

if [[ ! -f "${HOST_WORLDS_DIR}/${WORLD_FILE}" ]]; then
  echo "[ERROR] World file not found: ${HOST_WORLDS_DIR}/${WORLD_FILE}"
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
  -e XAUTHORITY="${XAUTH}" \
  -e QT_X11_NO_MITSHM=1 \
  -e SKIP_MAP="${SKIP_MAP}" \
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
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/aws-robomaker-hospital-world/models
    if [[ -d '${CONTAINER_WS}/aws-robomaker-hospital-world/fuel_models' ]]; then
      export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/aws-robomaker-hospital-world/fuel_models
    fi
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/sjtu_drone/sjtu_drone_description
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/sjtu_drone/models

    export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
    export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:${CONTAINER_WS}/aws-robomaker-hospital-world/worlds
    export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:${CONTAINER_WS}/aws-robomaker-hospital-world
    export GAZEBO_MODEL_DATABASE_URI=
    export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:\$GAZEBO_PLUGIN_PATH
    if [[ -d '${CONTAINER_WS}/install/gazebo_ros_2d_map/lib' ]]; then
      export GAZEBO_PLUGIN_PATH=\$GAZEBO_PLUGIN_PATH:${CONTAINER_WS}/install/gazebo_ros_2d_map/lib
    fi
    export GAZEBO_MASTER_URI=http://localhost:11345
    export ALSA_CARD=0
    export GAZEBO_AUDIO_DEVICE=null
    export GAZEBO_VERBOSE=1

    echo '================================'
    echo 'Environment ready'
    echo 'World: ${WORLD_FILE}'
    echo 'GAZEBO_MODEL_PATH entries:'
    echo \$GAZEBO_MODEL_PATH | tr ':' '\n'
    echo '================================'

    # --- Sanity: world exists ---
    if [[ ! -f '${CONTAINER_WS}/aws-robomaker-hospital-world/worlds/${WORLD_FILE}' ]]; then
      echo '[ERROR] World file missing inside container: ${CONTAINER_WS}/aws-robomaker-hospital-world/worlds/${WORLD_FILE}'
      exit 1
    fi

    # --- Optional deps (only if missing) ---
    if ! ros2 pkg list | grep -q rmw_fastrtps_cpp; then
      apt-get update -qq && apt-get install -y -qq ros-${ROS_DISTRO}-rmw-fastrtps-cpp
      source /opt/ros/${ROS_DISTRO}/setup.bash
    fi

    if [[ \"${SKIP_MAP}\" != 'true' ]]; then
      if ! ros2 pkg list | grep -q nav2_costmap_2d; then
        echo '[INFO] Installing Nav2 bits required by 2D map plugin...'
        apt-get update -qq && apt-get install -y -qq \
          ros-${ROS_DISTRO}-nav2-costmap-2d \
          ros-${ROS_DISTRO}-nav2-map-server \
          ros-${ROS_DISTRO}-nav-msgs
        source /opt/ros/${ROS_DISTRO}/setup.bash
      fi
    else
      echo '[INFO] --no-map: skipping Nav2 installation.'
    fi
    # --- Python visualization deps ---
    # --- Python visualization & analysis dependencies ---
    echo '[INFO] Installing Python dependencies for visualization, image I/O, and math...'
    apt-get update -qq && apt-get install -y -qq \
        python3-pip \
        python3-opencv \
        python3-matplotlib \
        python3-tk \
        python3-yaml \
        python3-numpy \
        python3-scipy \
        libgl1 \
        libglib2.0-0 \
        && apt-get clean

    # Fix NumPy / OpenCV compatibility (force reinstall older NumPy)
    pip3 install --no-cache-dir --force-reinstall 'numpy<2' Pillow==10.3.0
    pip3 install --no-cache-dir --upgrade matplotlib==3.9.2 pyyaml==6.0.2


    # --- Build ---
    echo '[INFO] Building workspace...'
    cd '${CONTAINER_WS}'
    # Clean only our packages to avoid nuking other builds in the same WS
    rm -rf build/sjtu_drone_* install/sjtu_drone_* 2>/dev/null || true
    if [[ \"${SKIP_MAP}\" != 'true' ]]; then
      rm -rf build/gazebo_ros_2d_map install/gazebo_ros_2d_map 2>/dev/null || true
    fi

    colcon build --packages-select sjtu_drone_bringup sjtu_drone_description sjtu_drone_control \
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
        -p map_name:='hospital_map' \
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
