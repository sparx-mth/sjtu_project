# AprilTag Detection – SJTU Drone + Hospital World

This guide explains how to launch the hospital world simulation and run AprilTag detection using `apriltag_ros` inside the ROS2 Docker container.

---

## 1. Launch the Hospital World

From your host machine:

```bash
cd /sjtu_project/sjtu_drone
chmod +x run.sh
./run.sh --no-map hospital.world
```

This will:
- Start Gazebo with the `hospital.world`
- Launch the simple_drone and its front camera

### Change the Docker Image (optional)

If you want to use a different Docker image, edit the `run.sh` script:

```bash
nano run.sh
```

Look for the line that runs `docker run ... IMAGE_NAME ...` and replace the image name with the one you want.

---

## 2. Enter the ROS2 Docker Container

In a new terminal on the host:

```bash
docker exec -it <container_name> bash
```

> Replace `<container_name>` with the actual container name (for example: `sjtu_drone_ros2` or whatever appears in `docker ps`).

---

## 3. Clone `apriltag_ros` Inside the Workspace

Inside the container:

```bash
cd /ros2_ws/src
git clone https://github.com/christianrauch/apriltag_ros.git
```

---

## 4. Resolve Dependencies and Build

Still inside the container:

```bash
cd /ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Run the AprilTag Node

Run the AprilTag detector node with the camera topics from the drone:

```bash
ros2 run apriltag_ros apriltag_node --ros-args   -r image_rect:=/simple_drone/front/image_raw   -r camera_info:=/simple_drone/front/camera_info   -p camera_frame:=simple_drone/front_cam_optical   -p family:=36h11   -p size:=1.0   -p publish_tf:=true   --log-level debug
```

Parameters:
- `family`: Tag family (here `36h11`)
- `size`: Physical size of the tag in **meters** (here `1.0`)
- `camera_frame`: Camera frame name used for TF (`simple_drone/front_cam_optical`)
- `publish_tf`: Whether to publish TF transforms for detected tags

---

## 6. View AprilTag Detections

Open another terminal **inside the same container** (or use `tmux`/`screen`) and run:

```bash
ros2 topic echo /detections
```

You should now see messages like:

```yaml
detections:
- family: tag36h11
  id: 14
  ...
```

indicating that the node is detecting AprilTags in the camera stream.

---

## Notes

- Make sure the drone camera is pointing at a visible AprilTag in the `hospital.world`.
- If you change topic names or camera frames, update the arguments in the `ros2 run apriltag_ros apriltag_node` command accordingly.
- If you rebuild the workspace after changes, don’t forget to:

  ```bash
  cd /ros2_ws
  colcon build --symlink-install
  source install/setup.bash
  ```