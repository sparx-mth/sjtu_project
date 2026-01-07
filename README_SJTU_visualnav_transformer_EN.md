# visualnav-transformer-ros2 + SJTU Drone  
**Quick & Practical README**

This guide explains how to run **visualnav-transformer-ros2** inside Docker and adapt it to the **SJTU Drone** setup.

The models (GNM / ViNT / NoMaD) predict **visual waypoints** from camera images.  
Actual robot motion is handled by a separate script that converts waypoints into `cmd_vel`.


## Prerequisites (Host)

- Docker installed  
- NVIDIA GPU + NVIDIA Container Runtime (if using GPU)  
- ROS2 running and publishing the drone topics  
- Matching `ROS_DOMAIN_ID` between host and container - export ROS_DOMAIN_ID=0
- X11 enabled (for visualization windows)


## 1) First-Time Run (with GUI)

### 1.1 Allow Docker access to X11 (host)
```bash
xhost +local:root
```

### 1.2 Run the container
```bash
docker run -it --rm \
  --gpus all \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=23 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  visualnav_transformer:latest
```

### 1.3 Verify ROS topics
Inside the container:
```bash
ros2 topic list
```

---

## 2) Adapt Topics for SJTU Drone

### 2.1 Update camera image topic
```bash
sed -i 's|IMAGE_TOPIC = ".*"|IMAGE_TOPIC = "/simple_drone/front/image_raw"|' \
src/visualnav_transformer/deployment/src/topic_names.py

grep IMAGE_TOPIC src/visualnav_transformer/deployment/src/topic_names.py
```

### 2.2 Update velocity command topic
```bash
sed -i 's|self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)|self.publisher = self.create_publisher(Twist, "/simple_drone/cmd_vel", 10)|' \
scripts/publish_cmd.py

grep create_publisher scripts/publish_cmd.py
```

---

## 3) Install Python Dependencies (Poetry)

```bash
poetry --version
poetry install
```

Optional (editor inside container):
```bash
apt update
apt install -y nano
```

---

## 4) Explore Mode (Waypoint Prediction)

### 4.1 Run the model
```bash
poetry run python3 src/visualnav_transformer/deployment/src/explore.py
```

### 4.2 Verify waypoints are published
In another terminal:
```bash
ros2 topic echo /waypoint
```

---

## 5) Enable Robot Motion (cmd_vel)

In a separate terminal inside the container:
```bash
poetry run python3 scripts/publish_cmd.py
```

---

## 6) Visualization (Model Output)

In another terminal:
```bash
poetry run python3 scripts/visualize.py
```

A window should appear showing:
- Current camera image
- Colored waypoint trajectories predicted by the model

---

## 7) Create a Topomap (Visual Map)

### 7.1 Record topomap while driving manually
```bash
poetry run python3 src/visualnav_transformer/deployment/src/create_topomap.py -t 1
```

- Saves one image per second (`-t` controls the interval)
- Drive the drone manually using teleop or your navigation stack

### 7.2 Verify images were saved
```bash
ls topomaps/images/topomap/
```

---

## 8) Navigation Using the Topomap

```bash
poetry run python3 src/visualnav_transformer/deployment/src/navigate.py
```

- Default goal: last image in the topomap
- To select a different goal image:
```bash
poetry run python3 src/visualnav_transformer/deployment/src/navigate.py --goal-node <N>
```

---

## 9) Typical Workflow (4 Terminals)

1) **Navigation**
```bash
poetry run python3 src/visualnav_transformer/deployment/src/navigate.py
```
```bash
poetry run python3 src/visualnav_transformer/deployment/src/navigate_new.py   --model nomad   --dir topomap   --goal-node -1   --goal-x -5.499710   --goal-y 7.285320   --snap-x -7.433326   --snap-y 0.593377   --progress-margin 0.3   --record-bag   --bag-name run_001_bag
```

2) **Velocity Publisher**
```bash
poetry run python3 scripts/publish_cmd.py
```

3) **Visualization**
```bash
poetry run python3 scripts/visualize.py
```

4) **Topomap Creation (only when needed)**
```bash
poetry run python3 src/visualnav_transformer/deployment/src/create_topomap.py -t 1
```

5) **save altitude script:**
```bash
 poetry run python3 altitude_mux.py 
```

***from the sjtu docker - 
5) **run the map with the drone traj:**

```bash
cd /ros2_ws/maps
python3 show_drone_trajectory.py
```


---

## 10) Save Your Changes (Docker Commit)

After the first setup and topic modifications, save the container state:

On the host:
```bash
docker ps
docker commit <CONTAINER_ID> visualnav_transformer:latest
```

From now on, running the container will include all SJTU Drone adaptations.

---

## Common Issues

**No visualization window**
- Ensure on host:
```bash
xhost +local:root
```
- Inside container:
```bash
echo $DISPLAY
```

**Waypoints published but robot does not move**
- Verify `publish_cmd.py` publishes to `/simple_drone/cmd_vel`
- Confirm the drone subscribes to this topic

**No camera images**
- Double-check `IMAGE_TOPIC` matches the SJTU Drone camera topic exactly

---

Happy navigating 🤖🚀
