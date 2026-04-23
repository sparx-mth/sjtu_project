# ROS1 ↔ ROS2 Bridge (static, QoS-aware, interface-pinned)

Bridges the sjtu_drone (ROS2 Humble, Gazebo) topics to FALCON
(ROS1 Noetic), with explicit per-topic QoS **and** a pinned DDS
network interface so depth images flow reliably.

## Why this replaces dynamic_bridge

Two things broke before:

1. **QoS mismatch** — `dynamic_bridge` subscribes reliable by
   default, but Gazebo's camera publishes best-effort. DDS
   silently refuses to deliver across that mismatch, so pose
   worked but depth silently stalled mid-flight. Fixed by
   `parameter_bridge` with explicit `reliability: best_effort`
   on depth (see `bridge.yaml`).

2. **DDS interface instability** — CycloneDDS picks a network
   interface "arbitrarily" from what it sees. On a docker host,
   the list includes `docker0`, per-project `br-*` bridges,
   plus the real NIC (e.g. `enp129s0`). Two containers that
   pick different interfaces never discover each other. Fixed
   by pinning `NetworkInterfaceAddress` to the default-route
   interface in `entrypoint.sh` (auto-detected per host).

---

## Files

| File | Purpose |
|---|---|
| `Dockerfile` | Builds `ros1_bridge:noetic-foxy` (unchanged) |
| `bridge.yaml` | Topic list + per-topic QoS (depth = best_effort) |
| `entrypoint.sh` | DDS pinning → wait for roscore → rosparam load → parameter_bridge |
| `run_bridge.sh` | Convenience wrapper (static mode, persists log to host) |
| `verify_bridge.sh` | Checks each FALCON topic is actually flowing |
| `bridge.log` | Host-side copy of the bridge's stdout/stderr (written at runtime) |

---

## Launch order (strict)

> Each step must finish before the next starts.

### Step 1 — Clean up
```bash
docker stop roscore ros1_bridge 2>/dev/null
docker rm   roscore ros1_bridge 2>/dev/null
true
```

### Step 2 — Start roscore
```bash
docker run -it --rm --net=host --name=roscore \
    --entrypoint bash ros1_bridge:noetic-foxy \
    -c 'source /opt/ros/noetic/setup.bash && roscore'
```
Wait for: `started core service [/rosout]`

### Step 3 — Start the bridge
New terminal:
```bash
cd ~/ros_bridge_docker
./run_bridge.sh
```
Wait for these lines in order:
```
DDS iface   : enp129s0        (← whatever your main NIC is)
[bridge] roscore is reachable
[bridge] Topics registered: ...
[bridge] Starting parameter_bridge...
Passing message from ROS 2 geometry_msgs/msg/Pose to ROS 1 ...
Passing message from ROS 2 sensor_msgs/msg/Image to ROS 1 ...
```
The last three lines only appear **after** the sim starts.

### Step 4 — Start the ROS2 sim
New terminal:
```bash
cd ~/GIT/sjtu_project/sjtu_drone
./run.sh hospital.world
```
Wait for Gazebo to fully load (drone PID parameters printed).

### Step 5 — Verify
New terminal:
```bash
cd ~/ros_bridge_docker
bash verify_bridge.sh
```
Expected output: `Bridge is healthy — FALCON should map cleanly`,
with depth at ~14 Hz.

### Step 6 — Start FALCON
```bash
cd ~/falcon_docker
./run_hospital.sh
```

---

## Debugging when something breaks

### The bridge container crashed

The bridge mirrors all its output to `./bridge.log` on the host.
If the container is gone (`docker ps` doesn't show `ros1_bridge`),
the log is still there:

```bash
tail -80 ~/ros_bridge_docker/bridge.log
```

### Depth shows 0 Hz in verify

Check the bridge log for the "Passing message from ROS 2
sensor_msgs/msg/Image" line. If it's missing:
- Confirm the sim is running and publishing depth: `docker exec sjtu_drone_hospital bash -c 'ros2 topic hz /simple_drone/front_depth/depth/image_raw'`
- Confirm bridge and sim agree on the DDS interface (both should log the same NIC)
- Confirm domain ID matches on both sides: `docker exec ros1_bridge bash -c 'echo $ROS_DOMAIN_ID'` vs `docker exec sjtu_drone_hospital bash -c 'echo $ROS_DOMAIN_ID'`

### DDS discovery fails intermittently

Rare now that interface is pinned, but if it happens: check
that your sim's `run.sh` also pins the same interface. The
CycloneDDS config in that file should have a `NetworkInterfaceAddress`
matching what the bridge logs at startup.

---

## Adding a new topic

Edit `bridge.yaml` and add:

```yaml
  - topic: /your/topic
    type: pkg_msgs/msg/YourType
    queue_size: 10
    qos:
      history: keep_last
      depth: 10
      reliability: best_effort   # sensor data
      durability: volatile
```

Then restart the bridge (Step 3). No rebuild needed — YAML is
bind-mounted.

---

## Environment variables

| Variable | Default | Description |
|---|---|---|
| `BRIDGE_MODE` | `static` | `static` (recommended) or `dynamic` |
| `ROS_DOMAIN_ID` | `20` | Must match the sim (host env wins via `:-` fallback) |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | Must match the sim |
| `ROS_MASTER_URI` | `http://localhost:11311` | ROS1 master |
| `BRIDGE_YAML` | `/bridge_ws/bridge.yaml` | Path inside container |
| `LOGFILE` | `/tmp/bridge.log` | Path inside container (mounted to host) |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Depth 0 Hz despite bridge running | Interface mismatch | Check `DDS iface` log line; both sides must agree |
| Bridge timed out waiting for roscore | roscore not reachable | `curl http://localhost:11311` should return HTTP 501 |
| Container exits immediately | Entrypoint error | Read `bridge.log` on the host |
| Pose works, depth doesn't | QoS mismatch | Confirm `bridge.yaml` uses `best_effort` for depth |
| Map freezes mid-flight but depth still 14 Hz | Not a bridge issue | Check `box_min/max` in FALCON's `hospital.yaml` |