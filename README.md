# sjtu_drone + AWS RoboMaker Hospital World (ROS2 Humble)

This guide explains how to run the **SJTU Drone simulation** with the **AWS RoboMaker Hospital world** on ROS2 Humble using Docker.

---

## Setup Steps


### Clone SJTU drone repo
```bash
git clone https://github.com/sparx-mth/sjtu_project
```

### Build Docker image
```bash
cd /sjtu_project/sjtu_drone
docker build -t sjtu_drone_clean:humble_ros2 .
```

### 6️⃣ Run simulation with hospital world
Copy the world file next to the run script:
```bash
cp /sjtu_project/aws-robomaker-hospital-world/worlds/hospital.world    /sjtu_project/sjtu_drone/hospital.world
```

Run:
```bash
cd /sjtu_project/sjtu_drone
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
     └─ models/
         └─ Apriltag36_11_00014
         └─ Apriltag36_11_00015


---

## 🆘 Notes
- GPU recommended: install NVIDIA container toolkit if available
- First launch may take time to download Gazebo models
- If paths fail, ensure run.sh mounts current folder into the container

---

🎉 **Simulation ready! Fly the drone inside the hospital world.**