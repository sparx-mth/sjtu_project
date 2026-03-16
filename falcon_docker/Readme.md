# FALCON — Docker Setup

Runs [FALCON](https://github.com/HKUST-Aerial-Robotics/FALCON) (ROS 1 Noetic)
inside Docker on Ubuntu 24.04 (or any host with Docker + NVIDIA GPU).

---

## Prerequisites (host machine)

```bash
# Docker
sudo apt install docker.io
sudo usermod -aG docker $USER   # log out & back in after this

# NVIDIA Container Toolkit (for GPU access inside Docker)
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

Edit `Dockerfile` (or `docker-compose.yml`) and set `CUDA_ARCH` to match your GPU:

| GPU family           | CUDA_ARCH |
|----------------------|-----------|
| RTX 50xx (Blackwell) | `120` ← **yours** |
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

# Option B: docker-compose (edit CUDA_ARCH in docker-compose.yml first)
docker compose build
```

---

## Step 3 — Run

```bash
chmod +x run.sh
./run.sh          # opens an interactive bash shell
```

Or with docker-compose:

```bash
docker compose run --rm falcon bash
```

---

## Step 4 — Launch FALCON (inside the container)

```bash
# Terminal 1 — RViz
roslaunch exploration_manager rviz.launch

# Terminal 2 — Planner (open a second shell with: ./run.sh or docker exec -it falcon bash)
roslaunch exploration_manager exploration.launch map_name:=octa_maze
```

Available maps: `classical_office`, `complex_office`, `darpa_tunnel`,
`duplex_office`, `octa_maze`, `power_plant`

### Opening a second shell into a running container

```bash
docker exec -it falcon bash
```

---

## What the Dockerfile does (without touching FALCON source logic)

| Step | What |
|------|------|
| Base image | `nvidia/cuda:12.8.0-devel-ubuntu20.04` (first CUDA to support Blackwell sm_120) |
| ROS | Noetic Desktop-Full (includes RViz) |
| apt deps | glog, libdw, armadillo, libc++, Eigen, GLFW… |
| CMake | 3.26.0-rc6 built from source |
| NLopt | 2.7.1 built from source |
| Open3D | 0.18.0 built from source (no Python) |
| Patch CUDA arch | `sed` replaces `compute_XX` → `compute_${CUDA_ARCH}` in CMakeLists |
| Patch Open3D path | `sed` sets `open3d_resource_path` in mesh_render.yaml |
| Build | `catkin_make -DCMAKE_BUILD_TYPE=Release` |

> The two `sed` patches only fill in the placeholder values the README
> explicitly asks you to set — no algorithmic logic is changed.

---

## Troubleshooting

**RViz can't open display**
```bash
xhost +local:docker   # run this on the host before launching
```

**`catkin_make` fails on Open3D not found**
Check that `/opt/open3d/lib/cmake/Open3D/Open3DConfig.cmake` exists inside
the container after the build step.

**Out of memory during `make -j$(nproc)`**
Edit the Dockerfile and replace `make -j$(nproc)` with `make -j4` in the
Open3D build step.