# ============================================================
# FALCON Docker Image  (ROS1 Noetic + Adapter, CPU-only build)
# Base: osrf/ros:noetic-desktop-full (Ubuntu 20.04 + ROS full)
#
# NOTE on GPU usage:
#   This image does NOT contain the CUDA toolkit. Gazebo and RViz
#   STILL get GPU acceleration at runtime via the NVIDIA Container
#   Toolkit (--gpus all on docker run). That uses OpenGL/EGL from
#   the host driver and is independent of CUDA. See run_hospital.sh
#   for the required env vars.
#
# NOTE on the FALCON simulator packages:
#   FALCON ships its own quadrotor simulator under uav_simulator/
#   with two depth renderers: mesh_render (Open3D) and
#   pointcloud_render (CUDA). Our setup uses Gazebo for simulation
#   and pipes Gazebo's depth camera to FALCON via falcon_adapter.py,
#   so we don't need any of FALCON's depth simulators.
#
#   pointcloud_render hard-requires CUDA, and map_render depends on
#   pointcloud_render. The ignore_cuda_pkgs.sh helper seeds
#   CATKIN_IGNORE on those two and then cascades CATKIN_IGNORE to
#   any further transitive dependents in one pass.
# ============================================================
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC  LANG=C.UTF-8  LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

# ── 1. System tools ───────────────────────────────────────────
# docker.io provides the docker CLI client. Combined with mounting
# /var/run/docker.sock at runtime (see run_hospital.sh), this lets
# scripts inside the FALCON container talk to the host's docker
# daemon — which is how respawn_drone.py reaches the sim container
# to teleport the drone between batch runs.
# python3-tk: matplotlib backend for bev_click_goal.py viewer.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl wget git gnupg2 lsb-release ca-certificates \
    pkg-config ninja-build \
    python3-pip python3-dev python3-tk \
    sudo locales \
    docker.io \
    && locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*

RUN rosdep update

# ── 2. FALCON apt deps ───────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgoogle-glog-dev libdw-dev libdwarf-dev libarmadillo-dev \
    libc++-dev libc++abi-dev libeigen3-dev libglfw3-dev \
    libgl1-mesa-dev libglu1-mesa-dev libpng-dev libjpeg-dev \
    libssl-dev xorg-dev libboost-all-dev libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# ── 3. CMake 3.26 ────────────────────────────────────────────
RUN cd /tmp \
    && wget -q https://cmake.org/files/v3.26/cmake-3.26.0-rc6.tar.gz \
    && tar -xzf cmake-3.26.0-rc6.tar.gz && cd cmake-3.26.0-rc6 \
    && ./bootstrap --parallel=$(nproc) && make -j$(nproc) && make install \
    && hash -r && rm -rf /tmp/cmake*

# ── 4. NLopt 2.7.1 ───────────────────────────────────────────
RUN cd /tmp && git clone --depth 1 --branch v2.7.1 https://github.com/stevengj/nlopt.git \
    && cd nlopt && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j$(nproc) && make install \
    && ldconfig && rm -rf /tmp/nlopt

# ── 5. Open3D 0.18.0 (CPU-only, 30-60 min) ───────────────────
ENV OPEN3D_INSTALL_PREFIX=/opt/open3d
ENV OPEN3D_SOURCE=/opt/Open3D
RUN git clone --depth 1 --branch v0.18.0 \
        https://github.com/isl-org/Open3D.git ${OPEN3D_SOURCE}
RUN cd ${OPEN3D_SOURCE} && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX=${OPEN3D_INSTALL_PREFIX} \
             -DBUILD_CUDA_MODULE=OFF \
             -DBUILD_PYTHON_MODULE=OFF -DBUILD_UNIT_TESTS=OFF \
             -DBUILD_BENCHMARKS=OFF .. \
    && make -j$(nproc) && make install && ldconfig

# ── 6. Clone FALCON + patch build placeholders ───────────────
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone --branch ros1-noetic \
        https://github.com/HKUST-Aerial-Robotics/FALCON.git
RUN find /catkin_ws/src/FALCON -name "mesh_render.yaml" \
    | xargs sed -i \
        "s|open3d_resource_path:.*|open3d_resource_path: ${OPEN3D_SOURCE}/build/bin/resources|g"

# ── 6b. Skip CUDA-only packages and any cascading dependents ─
COPY ignore_cuda_pkgs.sh /tmp/ignore_cuda_pkgs.sh
RUN chmod +x /tmp/ignore_cuda_pkgs.sh && /tmp/ignore_cuda_pkgs.sh /catkin_ws/src

# ── 7. Add adapter package + hospital map config ─────────────
COPY adapter/ /catkin_ws/src/falcon_adapter/
COPY hospital.yaml /catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/hospital.yaml

# ── 8. Build everything (no CUDA flags) ──────────────────────
WORKDIR /catkin_ws
RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    catkin_make \
        -DCMAKE_BUILD_TYPE=Release \
        -DOpen3D_DIR=${OPEN3D_INSTALL_PREFIX}/lib/cmake/Open3D \
        -j4 \
    "

# ── 8b. Patch FALCON SOP timeout (1s → 10s) and rebuild ──────
COPY fix_falcon_sop.sh /fix_falcon_sop.sh
RUN bash /fix_falcon_sop.sh

# ── 9. Shell + entrypoint ────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash"          >> /root/.bashrc \
    && echo "source /catkin_ws/devel/setup.bash"       >> /root/.bashrc \
    && echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]