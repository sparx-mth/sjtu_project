#!/bin/bash

ROS_DISTRO=humble
XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority
IMAGE_NAME="sjtu_drone:humble_ros2"

# Default spawn position (1 meter above ground)
SPAWN_X=${2:-0.0}
SPAWN_Y=${3:-0.0}
SPAWN_Z=${4:-3.0}

if [ -z "$1" ]; then
    echo "Usage: $0 <world_file> [x] [y] [z]"
    echo ""
    echo "Examples:"
    echo "  $0 small_house.world                    # Spawn at default position (0,0,1)"
    echo "  $0 small_house.world 2.0 3.0 1.5       # Spawn at custom position"
    echo "  $0 hospital.world 0 0 2.0              # Spawn 2 meters high"
    echo ""
    echo "Available worlds:"
    if [ -d "$HOME/drone_workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds" ]; then
        find $HOME/drone_workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds -name "*.world" -type f 2>/dev/null | while read world; do
            basename "$world"
        done | sed 's/^/  - /'
    fi
    exit 1
fi

WORLD_NAME=$1

# Find the world file
find_world_file() {
    local world_name=$1

    # Check in leonhartyao collection
    if [ -f "$HOME/drone_workspace/gazebo_models_worlds_collection/worlds/${world_name}" ]; then
        echo "/root/drone_workspace/gazebo_models_worlds_collection/worlds/${world_name}"
        return 0
    fi

    # Search recursively in mlherd dataset
    local found_world=$(find "$HOME/drone_workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds" -name "${world_name}" -type f 2>/dev/null | head -n1)
    if [ -n "$found_world" ]; then
        echo "$found_world" | sed "s|$HOME|/root|g"
        return 0
    fi

    return 1
}

WORLD_PATH=$(find_world_file "$WORLD_NAME")
if [ $? -ne 0 ]; then
    echo "Error: World file '${WORLD_NAME}' not found."
    exit 1
fi

echo "Using world: ${WORLD_PATH}"
echo "Spawning drone at position: x=${SPAWN_X}, y=${SPAWN_Y}, z=${SPAWN_Z}"
WORLD_BASE=$(basename "${WORLD_NAME}" .world)

# Get the directory of the world file for model path
WORLD_DIR=$(dirname "${WORLD_PATH}")

xhost +local:docker

# Run with comprehensive model path setup
docker run \
    -it --rm \
    --gpus all \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    -v $HOME/drone_workspace:/root/drone_workspace:rw \
    --name="sjtu_drone_${WORLD_BASE}" \
    ${IMAGE_NAME} \
    bash -c "
        # Clear and setup GAZEBO paths
        export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models

        # Add the standard collections
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/gazebo_models_worlds_collection/models

        # Add world-specific models directory if it exists
        if [ -d '${WORLD_DIR}/models' ]; then
            # For AWS models, we need to add the parent directory
            export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${WORLD_DIR}/models

            # Also add each individual AWS model as a separate path
            for model_dir in ${WORLD_DIR}/models/aws_robomaker_*; do
                if [ -d \"\$model_dir\" ]; then
                    export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:\$model_dir
                fi
            done
        fi

        # Add other world model directories
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/hospital/models
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/bookstore/models
        export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/root/drone_workspace/Dataset-of-Gazebo-Worlds-Models-and-Maps/worlds/office/models

        # Setup resource paths
        export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${WORLD_DIR}

        # Set mesh path as well
        export GAZEBO_MODEL_DATABASE_URI=
        export GAZEBO_MASTER_URI=http://localhost:11345
        export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:\$GAZEBO_PLUGIN_PATH

        echo '================================'
        echo 'Drone spawn position: x=${SPAWN_X}, y=${SPAWN_Y}, z=${SPAWN_Z}'
        echo '================================'

        # Build and launch with position parameters
        cd /root/drone_workspace && \
        colcon build && \
        source install/setup.bash && \
        ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py \
            world:=${WORLD_PATH} \
            x:=${SPAWN_X} \
            y:=${SPAWN_Y} \
            z:=${SPAWN_Z} &

        # Wait for Gazebo to fully load
        sleep 10

        # Remove any TurtleBot models that might be in the world
        echo 'Removing any TurtleBot models...'
        ros2 service call /delete_entity 'gazebo_msgs/srv/DeleteEntity' '{name: \"turtlebot3_waffle_pi\"}' 2>/dev/null || true
        ros2 service call /delete_entity 'gazebo_msgs/srv/DeleteEntity' '{name: \"turtlebot3_waffle\"}' 2>/dev/null || true
        ros2 service call /delete_entity 'gazebo_msgs/srv/DeleteEntity' '{name: \"turtlebot3_burger\"}' 2>/dev/null || true
        ros2 service call /delete_entity 'gazebo_msgs/srv/DeleteEntity' '{name: \"turtlebot3\"}' 2>/dev/null || true

        echo 'TurtleBot models removed (if they existed)'
        echo 'Your drone is now the only robot in the simulation'
        echo ''
        echo 'Control your drone with:'
        echo '  Takeoff: ros2 topic pub /simple_drone/takeoff std_msgs/msg/Empty \"{}\" --once'
        echo '  Land: ros2 topic pub /simple_drone/land std_msgs/msg/Empty \"{}\" --once'
        echo '  Move: ros2 topic pub /simple_drone/cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 1.0}}\" --rate 10'

        # Keep the container running
        wait"

xhost -local:docker