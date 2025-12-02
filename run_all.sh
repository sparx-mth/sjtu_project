#!/usr/bin/env bash

########################################
# CONFIG
########################################

CONTAINER_NAME="sjtu_drone_hospital"

PROJECT_ROOT="$HOME/sjtu_project/sjtu_drone"
WORLD_FILE="hospital.world"

LOCAL_ROS2_APRILTAG_DIR="$HOME/sjtu_project/ros2_apriltag"
CONTAINER_APRILTAG_DIR="/ros2_ws/src/apriltag_ros"
TRIANGULATION_FILE_NAME="tag_triangulation_node_new.py"
LOGGER_FILE_NAME="tag_pose_logger.py"
IMU_FILE_NAME="tag_and_imu_logger.py"


########################################
# FUNCTIONS
########################################

start_world() {
  echo ">>> Starting world (Gazebo + drone)..."
  cd "$PROJECT_ROOT" || { echo "PROJECT_ROOT not found: $PROJECT_ROOT"; return 1; }
  chmod +x run.sh
  ./run.sh --no-map "$WORLD_FILE"
}

run_apriltag_node() {

  LOG_LEVEL_ARG=""
  if [ "$1" == "debug" ]; then
    LOG_LEVEL_ARG="--log-level debug"
    echo ">>> Starting in DEBUG mode."
  else
    echo ">>> Starting in INFO mode (default)."
  fi

  echo ">>> Running AprilTag node inside container: $CONTAINER_NAME"
  docker exec -it "$CONTAINER_NAME" bash -lc "
    set -e

    echo 'Sourcing ROS distro...'
    source /opt/ros/humble/setup.bash

    cd /ros2_ws/src

    echo 'Checking git...'
    if ! command -v git >/dev/null 2>&1; then
      apt-get update
      apt-get install -y git
    fi

    if [ ! -d apriltag_ros ]; then
      echo 'Cloning apriltag_ros...'
      git clone https://github.com/christianrauch/apriltag_ros.git
    else
      echo 'apriltag_ros already exists, skipping clone.'
    fi

    cd /ros2_ws
    echo 'Running rosdep...'
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y

    echo 'Building workspace...'
    colcon build --symlink-install

    echo 'Sourcing workspace install...'
    source install/setup.bash

    echo 'Starting apriltag_node...'
    ros2 run apriltag_ros apriltag_node --ros-args \
      -r image_rect:=/simple_drone/front/image_raw \
      -r camera_info:=/simple_drone/front/camera_info \
      -p camera_frame:=simple_drone/front_cam_optical \
      -p family:=36h11 \
      -p size:=0.348 \
      -p publish_tf:=true \
      -p use_sim_time:=true \
      $LOG_LEVEL_ARG
  "
}


run_triangulation_node() {
  echo ">>> Running tag_triangulation_node_new.py inside container: $CONTAINER_NAME"

  if [ ! -d "$LOCAL_ROS2_APRILTAG_DIR" ]; then
    echo "ERROR: Local directory not found: $LOCAL_ROS2_APRILTAG_DIR"
    return 1
  fi

  echo ">>> Syncing local ros2_apriltag directory into container..."
  docker cp "$LOCAL_ROS2_APRILTAG_DIR/." "$CONTAINER_NAME:$CONTAINER_APRILTAG_DIR/"

  docker exec -it "$CONTAINER_NAME" bash -lc "
    set -e
    echo 'Sourcing ROS...'
    source /opt/ros/\$ROS_DISTRO/setup.bash

    cd /ros2_ws
    if [ -f install/setup.bash ]; then
      source install/setup.bash
    fi

    cd $CONTAINER_APRILTAG_DIR

    if [ ! -f $TRIANGULATION_FILE_NAME ]; then
      echo 'ERROR: file $TRIANGULATION_FILE_NAME not found in $CONTAINER_APRILTAG_DIR'
      ls -la
      exit 1
    fi

    echo 'Running $TRIANGULATION_FILE_NAME...'
    python3 $TRIANGULATION_FILE_NAME
  "
}


run_pose_logger() {
  echo ">>> Running tag_pose_logger.py inside container: $CONTAINER_NAME"

  if [ ! -d "$LOCAL_ROS2_APRILTAG_DIR" ]; then
    echo "ERROR: Local directory not found: $LOCAL_ROS2_APRILTAG_DIR"
    return 1
  fi

  echo ">>> Syncing local ros2_apriltag directory into container..."
  docker cp "$LOCAL_ROS2_APRILTAG_DIR/." "$CONTAINER_NAME:$CONTAINER_APRILTAG_DIR/"

  docker exec -it "$CONTAINER_NAME" bash -lc "
    set -e
    echo 'Sourcing ROS...'
    source /opt/ros/\$ROS_DISTRO/setup.bash

    cd /ros2_ws
    if [ -f install/setup.bash ]; then
      source install/setup.bash
    fi

    cd $CONTAINER_APRILTAG_DIR

    if [ ! -f $LOGGER_FILE_NAME ]; then
      echo 'ERROR: file $LOGGER_FILE_NAME not found in $CONTAINER_APRILTAG_DIR'
      ls -la
      exit 1
    fi

    echo 'Running $LOGGER_FILE_NAME...'
    python3 $LOGGER_FILE_NAME
  "
}

run_imu_logger(){
  echo ">>> Running tag_pose_logger.py inside container: $CONTAINER_NAME"

  if [ ! -d "$LOCAL_ROS2_APRILTAG_DIR" ]; then
    echo "ERROR: Local directory not found: $LOCAL_ROS2_APRILTAG_DIR"
    return 1
  fi

  echo ">>> Syncing local ros2_apriltag directory into container..."
  docker cp "$LOCAL_ROS2_APRILTAG_DIR/." "$CONTAINER_NAME:$CONTAINER_APRILTAG_DIR/"

  docker exec -it "$CONTAINER_NAME" bash -lc "
    set -e
    echo 'Sourcing ROS...'
    source /opt/ros/\$ROS_DISTRO/setup.bash

    cd /ros2_ws
    if [ -f install/setup.bash ]; then
      source install/setup.bash
    fi

    cd $CONTAINER_APRILTAG_DIR

    if [ ! -f $IMU_FILE_NAME ]; then
      echo 'ERROR: file $IMU_FILE_NAME not found in $CONTAINER_APRILTAG_DIR'
      ls -la
      exit 1
    fi

    echo 'Running $IMU_FILE_NAME...'
    python3 $IMU_FILE_NAME
  "
}

########################################
# MENU
########################################

show_menu() {
  echo ""
  echo "=============================="
  echo "  SJTU Drone + AprilTag Menu"
  echo "=============================="
  echo "1) Start world (run.sh --no-map hospital.world)"
  echo "2) Setup & run AprilTag node (step 2)"
  echo "2d) Setup & run AprilTag node (step 2) [DEBUG mode]" 
  echo "3) Run triangulation node (step 3)"
  echo "4) Run tag_pose_logger.py (step 4)"
  echo "5) Run tag_imu_logger.py (step 5)"
  echo "q) Quit"
  echo "=============================="
  echo ""
}

main() {
  case "$1" in
    world)
      start_world
      exit $?
      ;;
    tags)
      run_apriltag_node
      exit $?
      ;;
    tri)
      run_triangulation_node
      exit $?
      ;;
    shell)
      run_pose_logger
      exit $?
      ;;
  esac

  while true; do
    show_menu
    read -rp "Choose an option: " choice
    case "$choice" in
      1)
        start_world
        ;;
      2)
        run_apriltag_node
        ;;
      2d)
        run_apriltag_node debug # "debug"
        ;;
      3)
        run_triangulation_node
        ;;
      4)
        run_pose_logger
        ;;
      5) run_imu_logger
        ;;
      q|Q)
        echo "Bye :)"
        exit 0
        ;;
      *)
        echo "Invalid choice, try again."
        ;;
    esac
  done
}

main "$@"
