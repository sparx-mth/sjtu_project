# AprilTag-Based Camera Azimuth Estimation (Without ROS)

## High-level idea 

**This system estimates the camera’s absolute azimuth (heading) in the world by observing an AprilTag whose orientation in the world is known.**

---

## What does this code compute?

This code computes:

> **The direction the camera is facing (azimuth), in world coordinates**,  
> based on the relative position of a detected AprilTag whose world-facing direction is known in advance.
It computes **only orientation (yaw / azimuth)**.


## Step-by-step explanation

### 1️⃣ What is known in advance (Ground Truth)

#### a) Tag orientation in the world

Defined in a YAML configuration file:

```yaml
10: 0
11: 90
12: 180
13: 270
```

#### b) Physical size of the tag

```bash
--tag_size_m 0.08
```

Meaning:
- The real-world size of the square AprilTag is known

Knowing the size allows converting **pixel geometry → metric distances**, which is required for pose estimation.


#### c) Camera calibration

The camera intrinsic matrix:

```
K = [[fx,  0, cx],
     [ 0, fy, cy],
     [ 0,  0,  1]]
```

This describes:
- Pixel-to-angle projection
- Optical center of the camera
- Camera field of view geometry

### 2️⃣ What is measured from the image

From the AprilTag detector:
https://github.com/pupil-labs/apriltags

```python
corners_2d = [
  (u1, v1),
  (u2, v2),
  (u3, v3),
  (u4, v4)
]
```

These are the four detected tag corners in **pixel coordinates**.


### 3️⃣ What solvePnP computes (core geometry)

The question:

> Which 3D pose of a square produces exactly these pixel projections?

`solvePnP` computes:

#### Translation vector

```python
tvec = [tx, ty, tz]
```

In the camera frame:
- `tx`: left / right
- `tz`: forward distance

#### Rotation vector

Orientation of the tag relative to the camera (not directly used for azimuth).


### 4️⃣ Relative yaw computation

```python
relative_yaw = atan2(-tx, tz)
```

This gives the horizontal angle at which the camera sees the tag.

- Centered tag → 0°
- Left → positive
- Right → negative


### 5️⃣ Absolute azimuth estimation

```python
camera_yaw = wall_azimuth + relative_yaw
```

Because the wall orientation is known, the camera orientation can be recovered.

---

## Final summary

**By observing a tag with known world orientation and measuring its relative angle in the image, the system infers the camera’s absolute azimuth.**

---

## Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install opencv-python pupil-apriltags pyyaml
```

---

## Running the system

```bash
python3 -m apriltag_localization.tasks.tag_azimuth_node \
  --tag_config_path apriltag_localization/config/tags_azimuth.yaml \
  --camera_calib_path apriltag_localization/config/front_camera_calib.yaml \
  --tag_size_m 0.08 \
  --image_dir apriltag_localization/PS \
  --out_json apriltag_localization/results/azimuth_log.jsonl
```

---

## Output

- Visualized detections
- JSONL log containing azimuth per image

