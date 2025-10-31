# sjtu_drone + AWS RoboMaker Hospital World (ROS2 Humble)

This guide explains how to run the **SJTU Drone simulation** with the **AWS RoboMaker Hospital world** on ROS2 Humble using Docker.

---

## Setup Steps

### 1️⃣ Create workspace folder
```bash
mkdir -p /sjtu_project && cd /sjtu_project
```

### 2️⃣ Clone SJTU drone repo
```bash
git clone https://github.com/NadavCherry/sjtu_drone
```

### 3️⃣ Clone AWS Hospital World
```bash
git clone https://github.com/aws-robotics/aws-robomaker-hospital-world.git
```

### 4️⃣ Edit Dockerfile — switch to ROS2 Humble
In `sjtu_drone/Dockerfile`, replace:
```
ARG ROS_DISTRO=iron
```
with:
```
ARG ROS_DISTRO=humble
```

### 5️⃣ Build Docker image
```bash
cd /april_gazebo/sjtu_drone
docker build -t sjtu_drone_clean:humble_ros2 .
```

### 6️⃣ Run simulation with hospital world
Copy the world file next to the run script:
```bash
cp /april_gazebo/aws-robotics/aws-robomaker-hospital-world/worlds/hospital.world    /april_gazebo/sjtu_drone/hospital.world
```

Run:
```bash
cd /april_gazebo/sjtu_drone
chmod +x run.sh
./run.sh --no-map hospital.world
```

---

## 📂 Folder Structure
```
/sjtu_project
 ├─ sjtu_drone/
 │   ├─ Dockerfile
 │   ├─ run.sh
 │   └─ hospital.world
 └─ aws-robomaker-hospital-world/
     └─ worlds/
         └─ hospital.world
```

---

## 🆘 Notes
- GPU recommended: install NVIDIA container toolkit if available
- First launch may take time to download Gazebo models
- If paths fail, ensure run.sh mounts current folder into the container

---

🎉 **Simulation ready! Fly the drone inside the hospital world.**