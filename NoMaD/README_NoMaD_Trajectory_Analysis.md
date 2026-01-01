# NoMaD Navigation – Trajectory Recording and Analysis Guide

This document describes the full workflow for **recording**, **exporting**, and **analyzing** a navigation run using **NoMaD + ROS2**, including trajectory visualization and debugging.

The goal is to understand how the robot/drone actually moved in the environment, and to analyze navigation behavior, controller stability, and model performance.

---

## Overview

The pipeline consists of the following stages:

1. Run NoMaD navigation  
2. Record a ROS2 bag (ROSBAG)  
3. Copy the bag from Docker to the host machine  
4. Export the trajectory using `evo`  
5. Visualize and analyze the trajectory using Python  

---

## 1. Recording a ROS2 Bag

```bash
ros2 bag record -o bags/bags_altitude/nomad_run11 \
  /simple_drone/odom \
  /tf /tf_static \
  /simple_drone/cmd_vel \
  /simple_drone/front/image_raw
```

### Purpose
This command records all relevant data required for post-run analysis.

### Recorded Topics
- `/simple_drone/odom` – Robot/drone odometry (actual trajectory in the world)
- `/tf`, `/tf_static` – Coordinate frame transformations
- `/simple_drone/cmd_vel` – Velocity commands sent by the controller
- `/simple_drone/front/image_raw` – Front camera images

---

## 2. Copying the ROSBAG from Docker to Host

```bash
docker cp <container_id>:/visualnav-transformer/bags/bags_altitude .
```

Copies the recorded ROS2 bag from the Docker container to the local host machine.

---

## 3. Navigate to the Bag Directory

```bash
cd bags_altitude/
```

---

## 4. Exporting the Trajectory using evo

```bash
evo_traj bag2 nomad_run11/nomad_run11_0.db3 /simple_drone/odom --save_as_tum
```

### Output
```
simple_drone_odom.tum
```

Each row in the file contains:
```
timestamp x y z qx qy qz qw
```

---

## 5. Return to the Project Root

```bash
cd ..
```

---

## 6. Visualizing and Analyzing the Trajectory

```bash
python3 show_traj.py
```

The script loads the exported trajectory and visualizes:
- Trajectory
- Start position
- End position
- Goal position (if provided)
- Heading directions

---

## Full Workflow Summary

```
Run NoMaD Navigation
        ↓
ros2 bag record
        ↓
docker cp
        ↓
evo_traj bag2
        ↓
Python visualization & analysis
```

---

## Notes

- Camera FPS is typically much higher than waypoint FPS.
- Low waypoint update rates can cause unstable motion.
- Visualizing heading and velocity helps distinguish model issues from controller issues.

---

## Author

Navigation Analysis Pipeline – NoMaD + ROS2
