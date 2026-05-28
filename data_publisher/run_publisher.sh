#!/bin/bash
set -e

# ============================================================
#  Build (if needed) and run the office-replay publisher
#  + the demo-mode state manager.
#  Matches the Jetson: ROS 2 Humble, default DDS, ROS_DOMAIN_ID=5.
#  Edit the parameters below.
# ============================================================

IMAGE="data_publisher:humble"
CONTAINER="data_publisher"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Recordings folder (mounted read-only at /data) ---------
#   <take>/depth_npy/*.npy   and   estimated_trajectory_<take>.json
DATA_DIR="${DATA_DIR:-${HOME}/Desktop}"

# --- Publisher parameters (EDIT THESE) ----------------------
PUBLISH_RATE_HZ=5.0     # office RGB/depth rate
POSE_DELAY_MEAN=0.020    # sec, mean gap depth -> pose
POSE_DELAY_STD=0.010     # sec, std dev of that gap
DEPTH_ENCODING=32FC1     # office uses 16UC1 (mm); use 32FC1 for meters
DEPTH_DIR=""             # empty = auto-discover under /data
JSON_PATH=""             # empty = auto-discover under /data
ROS_DOMAIN_ID=5

# --- CameraInfo ---------------------------------------------
# Host path to the office calibration YAML (e.g. camera_xtend_ros_calib_504_294_resize.yaml).
# Empty disables CameraInfo. If empty but a *.yaml sits in DATA_DIR, it is auto-used.
CAMERA_INFO_YAML=""

# --- State manager ------------------------------------------
INITIAL_MODE=idle        # idle | fly_straight | turning | visual_servoing | finish

# --- ROS settings (match the office) ------------------------
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-5}"
# ------------------------------------------------------------

if [ "${SKIP_BUILD:-0}" != "1" ]; then
    echo "[INFO] Building ${IMAGE} (set SKIP_BUILD=1 to skip)..."
    docker build -t "${IMAGE}" "${SCRIPT_DIR}"
fi

docker rm -f "${CONTAINER}" 2>/dev/null || true

PARAMS="-p publish_rate_hz:=${PUBLISH_RATE_HZ}"
PARAMS="${PARAMS} -p pose_delay_mean:=${POSE_DELAY_MEAN}"
PARAMS="${PARAMS} -p pose_delay_std:=${POSE_DELAY_STD}"
PARAMS="${PARAMS} -p depth_encoding:=${DEPTH_ENCODING}"
[ -n "${DEPTH_DIR}" ] && PARAMS="${PARAMS} -p depth_dir:=${DEPTH_DIR}"
[ -n "${JSON_PATH}" ] && PARAMS="${PARAMS} -p json_path:=${JSON_PATH}"

EXTRA_MOUNTS=()
if [ -n "${CAMERA_INFO_YAML}" ]; then
    EXTRA_MOUNTS+=(-v "${CAMERA_INFO_YAML}:/config/camera_info.yaml:ro")
    PARAMS="${PARAMS} -p camera_info_yaml:=/config/camera_info.yaml"
fi

echo "================================================"
echo "  ${CONTAINER}  (ROS 2 Humble, default DDS)"
echo "  Data   : ${DATA_DIR} -> /data"
echo "  Domain : ${ROS_DOMAIN_ID}"
echo "  Mode   : ${INITIAL_MODE}"
echo "  Args   : --ros-args ${PARAMS}"
echo "================================================"

docker run -it --rm \
    --net=host \
    --ipc=host \
    --name="${CONTAINER}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
    -e INITIAL_MODE="${INITIAL_MODE}" \
    -v "${DATA_DIR}:/data:ro" \
    "${EXTRA_MOUNTS[@]}" \
    "${IMAGE}" \
    --ros-args ${PARAMS}