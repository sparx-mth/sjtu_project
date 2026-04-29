#!/bin/bash
set -eo pipefail

# -----------------------------
# Config
# -----------------------------
ROS_DISTRO=humble
IMAGE_NAME="sjtu_drone_nadav:humble_ros2"
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

# -----------------------------
# Detect workspace paths automatically
# -----------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"                          # <workspace_root>
CONTAINER_WS="/root/$(basename "$WORKSPACE_DIR")"                 # e.g., /root/my_project
HOST_SRC_DIR="${WORKSPACE_DIR}/src"

# Repos that contribute worlds + models. Add more here as you bring them in.
# For each entry, the script looks for:
#     <repo>/worlds/<env>.world      (the world file)
#     <repo>/models                  (added to GAZEBO_MODEL_PATH if present)
#     <repo>/fuel_models             (added to GAZEBO_MODEL_PATH if present)
WORLD_REPOS=(
  "aws-robomaker-hospital-world"
  "aws-robomaker-small-house-world"
  "aws-robomaker-bookstore-world"
  "aws-robomaker-small-warehouse-world"
  "sjtu_drone/sjtu_drone_description"
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

# Environment name -> world file (defaults to hospital).
# Usage: ./run.sh [--no-map] [env_name]
#   e.g. hospital, small_house, bookstore, small_warehouse, playground
ENV_NAME="${1:-hospital}"
WORLD_FILE="${ENV_NAME}.world"
WORLD_BASE="${ENV_NAME}"

# -----------------------------
# Locate the world file across all configured repos
# -----------------------------
HOST_WORLD_FILE=""
for repo in "${WORLD_REPOS[@]}"; do
  candidate="${WORKSPACE_DIR}/${repo}/worlds/${WORLD_FILE}"
  if [[ -f "${candidate}" ]]; then
    HOST_WORLD_FILE="${candidate}"
    break
  fi
done

if [[ -z "${HOST_WORLD_FILE}" ]]; then
  echo "[ERROR] World file not found: ${WORLD_FILE}"
  echo ""
  echo "Worlds available:"
  any_found=false
  for repo in "${WORLD_REPOS[@]}"; do
    d="${WORKSPACE_DIR}/${repo}/worlds"
    if [[ -d "$d" ]]; then
      mapfile -t worlds < <(ls -1 "$d"/*.world 2>/dev/null | xargs -n1 basename)
      if [[ ${#worlds[@]} -gt 0 ]]; then
        echo "  ${d}:"
        printf '    %s\n' "${worlds[@]}"
        any_found=true
      fi
    fi
  done
  if [[ "${any_found}" == "false" ]]; then
    echo "  (no world repos found under ${WORKSPACE_DIR})"
  fi
  echo ""
  echo "Hint:"
  echo "  small_house     -> https://github.com/aws-robotics/aws-robomaker-small-house-world"
  echo "  bookstore       -> https://github.com/aws-robotics/aws-robomaker-bookstore-world"
  echo "  small_warehouse -> https://github.com/aws-robotics/aws-robomaker-small-warehouse-world  (clone with: -b ros1)"
  exit 1
fi

# Translate host path -> container path
WORLD_PATH="${HOST_WORLD_FILE/${WORKSPACE_DIR}/${CONTAINER_WS}}"
echo "[INFO] Resolved world: ${HOST_WORLD_FILE}"

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

# Build a space-separated list of repo names to expose inside the container.
WORLD_REPOS_STR="${WORLD_REPOS[*]}"

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
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}" \
  -e XAUTHORITY="${XAUTH}" \
  -e QT_X11_NO_MITSHM=1 \
  -e SKIP_MAP="${SKIP_MAP}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e WORLD_REPOS="${WORLD_REPOS_STR}" \
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

    # --- DDS: Install CycloneDDS and disable shared memory ───────
    # CycloneDDS is REQUIRED for bridge compatibility with Foxy.
    # FastRTPS versions between Humble and Foxy are incompatible.
    echo '[INFO] Installing CycloneDDS...'
    apt-get update -qq && apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1
    if ! dpkg -s ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1; then
      echo '[ERROR] Failed to install ros-humble-rmw-cyclonedds-cpp!'
      echo '        The bridge REQUIRES CycloneDDS. Cannot continue.'
      exit 1
    fi
    echo '[INFO] CycloneDDS installed successfully.'

    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

    # Write CycloneDDS config that disables shared memory (iceoryx).
    # Humble's CycloneDDS enables SHM by default; the Foxy bridge
    # does not support SHM, so data sent via SHM is invisible to it.
    cat > /tmp/cyclonedds.xml <<'DDSEOF'
<?xml version=\"1.0\" encoding=\"UTF-8\" ?>
<CycloneDDS xmlns=\"https://cdds.io/config\">
  <Domain>
    <General>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
    <SharedMemory>
      <Enable>false</Enable>
    </SharedMemory>
  </Domain>
</CycloneDDS>
DDSEOF
    export CYCLONEDDS_URI=file:///tmp/cyclonedds.xml

    echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /root/.bashrc
    echo 'export CYCLONEDDS_URI=file:///tmp/cyclonedds.xml' >> /root/.bashrc

    # --- Core env ---
    export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models
    export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11

    # Add models/worlds for every configured repo that exists.
    for repo in \$WORLD_REPOS; do
      if [[ -d \"${CONTAINER_WS}/\$repo/models\" ]]; then
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/\$repo/models
      fi
      if [[ -d \"${CONTAINER_WS}/\$repo/fuel_models\" ]]; then
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/\$repo/fuel_models
      fi
      if [[ -d \"${CONTAINER_WS}/\$repo/worlds\" ]]; then
        export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:${CONTAINER_WS}/\$repo/worlds
      fi
      if [[ -d \"${CONTAINER_WS}/\$repo\" ]]; then
        export GAZEBO_RESOURCE_PATH=\$GAZEBO_RESOURCE_PATH:${CONTAINER_WS}/\$repo
      fi
    done

    # sjtu_drone's own models (kept explicit for clarity).
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/sjtu_drone/sjtu_drone_description
    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${CONTAINER_WS}/sjtu_drone/models

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
    echo 'Path:  ${WORLD_PATH}'
    echo 'RMW:   '\$RMW_IMPLEMENTATION
    echo 'DDS:   '\$CYCLONEDDS_URI
    echo 'Domain:'\$ROS_DOMAIN_ID
    echo 'GAZEBO_MODEL_PATH entries:'
    echo \$GAZEBO_MODEL_PATH | tr ':' '\n' | sed 's/^/  /'
    echo '================================'

    # --- Sanity: world exists ---
    if [[ ! -f '${WORLD_PATH}' ]]; then
      echo '[ERROR] World file missing inside container: ${WORLD_PATH}'
      exit 1
    fi

    # --- Build ---
    echo '[INFO] Building workspace...'
    cd '${CONTAINER_WS}'
    # Clean only our packages to avoid nuking other builds in the same WS
    rm -rf build/sjtu_drone_* install/sjtu_drone_* 2>/dev/null || true
    if [[ \"${SKIP_MAP}\" != 'true' ]]; then
      rm -rf build/gazebo_ros_2d_map install/gazebo_ros_2d_map 2>/dev/null || true
    fi

    colcon build --packages-select sjtu_drone_bringup sjtu_drone_description sjtu_drone_control autonomous_system \
      --cmake-args -DBUILD_TESTING=OFF

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