# FALCON — Docker Setup

Runs [FALCON](https://github.com/HKUST-Aerial-Robotics/FALCON) (ROS 1 Noetic)
inside Docker on Ubuntu 24.04 (or any host with Docker + NVIDIA GPU).

Two run modes are provided:

| Mode | Script | GPU | Description |
|------|--------|-----|-------------|
| **Hospital** (external Gazebo) | `run_hospital.sh` | No (CPU-only) | FALCON receives depth from a Gazebo drone via `ros1_bridge`. GPU stays free for Gazebo. |
| **Octa-maze** (built-in sim) | `run_octa_maze.sh` | Yes | FALCON uses its own `map_render` node to render depth from STL meshes on the GPU. |

---

## Prerequisites (host machine)

```bash
# Docker
sudo apt install docker.io
sudo usermod -aG docker $USER   # log out & back in after this

# NVIDIA Container Toolkit (needed for octa-maze mode)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

---

## Step 1 — Set your GPU compute capability

Edit `Dockerfile` (or `docker-compose.yml`) and set `CUDA_ARCH`:

| GPU family           | CUDA_ARCH |
|----------------------|-----------|
| RTX 50xx (Blackwell) | `120`     |
| RTX 40xx (Ada)       | `89`      |
| RTX 30xx (Ampere)    | `86`      |
| RTX 20xx (Turing)    | `75`      |

Find yours: <https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/>

---

## Step 2 — Build the image

> ⚠️ **First build takes 30–60 minutes** (Open3D 0.18.0 is built from source).
> Subsequent builds use Docker layer cache.

```bash
# Option A: plain docker build
docker build --build-arg CUDA_ARCH=120 -t falcon-ros:noetic .

# Option B: docker-compose (builds once, both services share the same image)
docker compose build falcon-hospital
```

---

## Step 3 — Run

Make the scripts executable:

```bash
chmod +x run_hospital.sh run_octa_maze.sh
```

### Mode A — Hospital world (external Gazebo, CPU-only)

Start Gazebo + `ros1_bridge` on the host first, then:

```bash
# Terminal 1 — open a shell in the container
./run_hospital.sh

# Inside the container:
roslaunch falcon_adapter gazebo_exploration.launch map_name:=hospital
```

```bash
# Terminal 2 — RViz (open a second shell into the running container)
docker exec -it falcon bash
roslaunch exploration_manager rviz.launch
```

Or via docker-compose:

```bash
docker compose run --rm falcon-hospital bash
```

### Mode B — Octa-maze (FALCON built-in simulator, GPU)

No external Gazebo needed — FALCON renders everything internally.

```bash
# Terminal 1 — RViz
./run_octa_maze.sh roslaunch exploration_manager rviz.launch

# Terminal 2 — Planner
docker exec -it falcon-gpu bash
roslaunch exploration_manager exploration.launch map_name:=duplex_office
```

Or via docker-compose:

```bash
docker compose run --rm falcon-octa-maze bash
```

Available built-in maps: `classical_office`, `complex_office`, `darpa_tunnel`,
`duplex_office`, `octa_maze`, `power_plant`

---

## Opening a second shell into a running container

```bash
# Hospital mode (container name: falcon)
docker exec -it falcon bash

# Octa-maze mode (container name: falcon-gpu)
docker exec -it falcon-gpu bash
```

---

## What the Dockerfile does

| Step | What |
|------|------|
| Base image | `nvidia/cuda:12.8.0-devel-ubuntu20.04` |
| ROS | Noetic Desktop-Full (includes RViz) |
| apt deps | glog, libdw, armadillo, libc++, Eigen, GLFW… |
| CMake | 3.26.0-rc6 built from source |
| NLopt | 2.7.1 built from source |
| Open3D | 0.18.0 built from source (no Python) |
| Adapter | `falcon_adapter` package + `hospital.yaml` map config |
| Patch CUDA arch | `sed` fills in `compute_XX` → `compute_${CUDA_ARCH}` |
| Patch Open3D path | `sed` sets `open3d_resource_path` in mesh_render.yaml |
| Build | `catkin_make -DCMAKE_BUILD_TYPE=Release` |

> The `sed` patches only fill in placeholder values — no algorithmic logic is changed.

---

## Troubleshooting

**RViz can't open display**
```bash
xhost +local:docker   # run on host before launching
```

**`catkin_make` fails on Open3D not found**
Check that `/opt/open3d/lib/cmake/Open3D/Open3DConfig.cmake` exists inside
the container.

**Out of memory during build**
Replace `make -j$(nproc)` with `make -j4` in the Open3D build step.

**Octa-maze crashes / black screen**
Make sure you used `run_octa_maze.sh` (not `run_hospital.sh`) — the built-in
simulator needs GPU access.