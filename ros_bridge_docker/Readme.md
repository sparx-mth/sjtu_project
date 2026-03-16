# ROS1 ↔ ROS2 Bridge Docker

Bridges any ROS1 (Noetic) node with any ROS2 (Humble/Jazzy) simulation running on the same machine.

## How It Works

```
[FALCON - ROS1 Noetic]  ←→  [Bridge Container]  ←→  [Your ROS2 Sim]
     runs on host              Noetic + Foxy           any ROS2 distro
                               --net=host              --net=host
```

---

## One-Time Setup: Build the Bridge Image

```bash
cd ~/ros_bridge_docker
docker build -t ros1_bridge:noetic-foxy .
```
⏱ Takes ~10 minutes. Only needed once.

---

## Every Time: Launch Order

> ⚠️ Order is strict. Each step must be fully ready before starting the next.

### Step 1 — Kill any leftover containers (always do this first)
```bash
docker stop roscore ros1_bridge 2>/dev/null; docker rm roscore ros1_bridge 2>/dev/null; true
```

### Step 2 — Start roscore (ROS1 master)
```bash
docker run -it --rm --net=host --name=roscore \
  --entrypoint bash ros1_bridge:noetic-foxy -c \
  "source /opt/ros/noetic/setup.bash && roscore"
```
✅ Wait for this line before continuing:
```
started core service [/rosout]
```

### Step 3 — Start the bridge
Open a new terminal:
```bash
docker run -it --rm \
  --net=host \
  --name="ros1_bridge" \
  -e ROS_MASTER_URI="http://localhost:11311" \
  -e ROS_IP="127.0.0.1" \
  -e ROS_DOMAIN_ID=20 \
  -e RMW_IMPLEMENTATION="rmw_fastrtps_cpp" \
  -e BRIDGE_MODE="dynamic" \
  ros1_bridge:noetic-foxy
```
✅ Wait for this before continuing:
```
[INFO] Using dynamic_bridge — bridging all matched topics
```
❌ If you see `Connection refused` — roscore isn't ready yet. Stop and redo Step 2.

### Step 4 — Start your ROS2 simulation
Open a new terminal:
```bash
cd ~/PycharmProjects/sjtu_project/sjtu_drone
./run.sh hospital.world
```
✅ Wait for Gazebo to fully load (drone PID parameters printed in logs).

### Step 5 — Verify everything is connected
Open a new terminal:
```bash
cd ~/ros_bridge_docker
bash verify_bridge.sh
```
✅ Expected result: `5 passed, 0 failed`

---

## Files

| File | Purpose |
|---|---|
| `Dockerfile` | Builds the bridge image (Ubuntu 20.04, Noetic + Foxy) |
| `entrypoint.sh` | Starts `dynamic_bridge` inside the container |
| `run_bridge.sh` | Convenience wrapper for Step 3 |
| `verify_bridge.sh` | Checks topics are visible on both sides |

---

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `20` | Must match your sim (set in sjtu_drone/run.sh) |
| `ROS_MASTER_URI` | `http://localhost:11311` | ROS1 master address |
| `BRIDGE_MODE` | `dynamic` | `dynamic` = auto-bridge, `static` = use bridge.yaml |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `Connection refused` in bridge | roscore not running | Do Step 2 first, wait for `[/rosout]` |
| No ROS2 topics in verify | Wrong `ROS_DOMAIN_ID` | Ensure `-e ROS_DOMAIN_ID=20` in Step 3 |
| No ROS1 topics in verify | Normal until FALCON runs | ROS1 topics appear only when something subscribes |
| `awesome_allen` or stale container | Previous run didn't clean up | Always run Step 1 cleanup first |
| `ROS_DISTRO` warnings | Both envs sourced together | Safe to ignore, not an error |