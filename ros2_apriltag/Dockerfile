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

