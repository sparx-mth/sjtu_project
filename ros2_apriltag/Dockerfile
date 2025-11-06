# Use a standard ROS 2 Humble base image
FROM ros:humble-ros-base

# Set environment variable to ensure non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# --- 1. System Dependencies and Tools Installation ---
# Install essential build tools, ROS tools, and git
RUN apt-get update && apt-get install -y \
    git \
    build-essential \
    python3-rosdep \
    python3-colcon-common-extensions \
    ros-humble-tf2-tools \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# --- 2. Create and Populate ROS 2 Workspace ---
WORKDIR /ros2_ws

# Create the source directory
RUN mkdir -p src

# Clone the apriltag_ros repository
RUN cd src && git clone https://github.com/christianrauch/apriltag_ros.git

# --- 3. Resolve Dependencies and Build ---
# Update rosdep
RUN rosdep update

# Install dependencies for the cloned packages (ignore source files, auto-approve, use humble)
RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# Build the workspace
RUN colcon build --symlink-install

# --- 4. Define Container Startup ---
# Source the setup file automatically for subsequent commands
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
CMD ["/bin/bash"]

# Note: The apriltag_node will be run manually by the user once the Gazebo/Drone environment is launched.





cd src
git clone https://github.com/christianrauch/apriltag_ros.git
cd /ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 run apriltag_ros apriltag_node --ros-args \
  -r image_rect:=/simple_drone/front/image_raw \
  -r camera_info:=/simple_drone/front/camera_info \
  -p image_transport:=raw \
  -p family:=36h11 \
  -p size:=0.348 \
  -p max_hamming:=1 \
  -p pose_estimation_method:=pnp \
  -p detector.threads:=2 \
  -p detector.decimate:=1.0 \
  -p detector.blur:=0.0 \
  -p detector.refine:=True \
  -p detector.sharpening:=0.25 \
  --log-level debug
 
apt-get update
apt-get install -y ros-humble-tf2-tools ros-humble-rviz2
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo simple_drone/front_cam_link tag36h11:14

