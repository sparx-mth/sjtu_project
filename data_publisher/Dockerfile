# Use ROS 2 Jazzy base image
FROM ros:jazzy-ros-base

# Install required system dependencies and ROS packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-jazzy-cv-bridge \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install numpy (using break-system-packages to allow pip install in Ubuntu 24.04/Docker)
RUN pip3 install numpy --break-system-packages

# Set working directory inside the container
WORKDIR /app

# Copy the Python script into the container
COPY data_publisher.py /app/data_publisher.py

# Run command: source ROS environment and execute the script
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && python3 /app/data_publisher.py"]