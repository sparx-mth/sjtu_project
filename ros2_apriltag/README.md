AprilTag Detector for Simple Drone Simulation (ROS 2 Humble)

This project provides a portable Docker environment configured with the apriltag_ros package, ready to detect AprilTags (specifically tag36h11) from a Gazebo-simulated drone camera feed.

Prerequisites

Docker: Docker must be installed and running on your system.

ROS 2 Humble Environment: A separate ROS 2 environment (or another container) must be running the Gazebo simulation and publishing the drone's camera data on the required topics.

Network Setup: This AprilTag container must be run with host networking (--network host) to communicate with the Gazebo and drone nodes.

1. Setup and Build

First, build the Docker image using the provided Dockerfile. This step only needs to be performed once.

# Place the Dockerfile in a new folder (e.g., april_tag_detector)
docker build -t simple-drone-apriltag:latest .


2. Usage: Running the Detector Node

After ensuring your Gazebo drone simulation is running and publishing camera topics (e.g., /simple_drone/front/image_raw), run a container from the image.

Important: Use --network host to enable ROS 2 topic communication.

# Run the container interactively
docker run -it --rm --network host simple-drone-apriltag:latest


Once inside the container's shell, source the workspace and run the AprilTag node.

Running the AprilTag Node

The following command launches the detector, subscribing to the drone's front camera topics and setting the parameters for tag36h11 (size 0.348m).

# Inside the container's shell:
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


3. Verification

While the apriltag_node is running in one terminal, open a second terminal (also connected to the same ROS 2 network, either the host machine or another container with --network host) to verify that the spatial transform (TF) has been successfully published.

A. Check the TF Tree (Optional)

This command installs the necessary tool and generates a PDF of the current TF tree, verifying the tag's frame exists.

# This tool was installed in the Dockerfile, but can be run on the host as well
ros2 run tf2_tools view_frames


B. Echo the Transformation

This is the most direct way to check the pose. If the tag (ID 14) is visible to the camera, this command will output the constantly updated translation (x, y, z) and rotation (quaternion) of the tag relative to the camera link.

ros2 run tf2_ros tf2_echo simple_drone/front_cam_link tag36h11:14


Expected Output: If the tag is detected, you will see a stream of translation and rotation data, confirming that the AprilTag node is correctly calculating the 3D pose of the tag.
