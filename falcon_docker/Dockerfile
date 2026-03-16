# ============================================================
# FALCON Docker Image
# Base: Ubuntu 20.04 + CUDA 11.8 + ROS Noetic
# Repo: https://github.com/HKUST-Aerial-Robotics/FALCON
# ============================================================

FROM nvidia/cuda:12.8.0-devel-ubuntu20.04

# ── Build arguments ─────────────────────────────────────────
# CUDA compute capability for your GPU.
#   120 → RTX 50xx (Blackwell)  ← your RTX 5070
#   89  → RTX 40xx (Ada Lovelace)
#   86  → RTX 30xx (Ampere)
#   75  → RTX 20xx (Turing)
# Find yours: https://arnon.dk/matching-sm-architectures-arch-and-gencode-for-various-nvidia-cards/
ARG CUDA_ARCH=120

# ── Environment ─────────────────────────────────────────────
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic

# ─────────────────────────────────────────────────────────────
# 1. Basic system tools
# ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl wget git gnupg2 lsb-release ca-certificates \
    build-essential pkg-config ninja-build \
    python3 python3-pip python3-dev \
    software-properties-common \
    sudo tzdata locales \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────
# 2. ROS Noetic (Desktop-Full: includes RViz, rviz, etc.)
# ─────────────────────────────────────────────────────────────
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" \
        > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
        | apt-key add -

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    && rosdep init \
    && rosdep update \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────
# 3. FALCON apt dependencies
# ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Explicitly listed in the FALCON README
    libgoogle-glog-dev \
    libdw-dev \
    libdwarf-dev \
    libarmadillo-dev \
    libc++-dev \
    libc++abi-dev \
    # Open3D build dependencies
    libeigen3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libpng-dev \
    libjpeg-dev \
    libssl-dev \
    xorg-dev \
    # General C++ / CMake helpers
    libboost-all-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# ─────────────────────────────────────────────────────────────
# 4. CMake 3.26.0-rc6  (FALCON requires 3.20+)
# ─────────────────────────────────────────────────────────────
RUN cd /tmp \
    && wget -q https://cmake.org/files/v3.26/cmake-3.26.0-rc6.tar.gz \
    && tar -xzf cmake-3.26.0-rc6.tar.gz \
    && cd cmake-3.26.0-rc6 \
    && ./bootstrap --parallel=$(nproc) \
    && make -j$(nproc) \
    && make install \
    && hash -r \
    && rm -rf /tmp/cmake*

# ─────────────────────────────────────────────────────────────
# 5. NLopt 2.7.1
# ─────────────────────────────────────────────────────────────
RUN cd /tmp \
    && git clone --depth 1 --branch v2.7.1 https://github.com/stevengj/nlopt.git \
    && cd nlopt \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig \
    && rm -rf /tmp/nlopt

# ─────────────────────────────────────────────────────────────
# 6. Open3D 0.18.0  (built from source, no Python bindings)
#    ⚠️  This step takes 20–40 min depending on your machine.
#    The image will be ~5-8 GB.
# ─────────────────────────────────────────────────────────────
ENV OPEN3D_INSTALL_PREFIX=/opt/open3d
ENV OPEN3D_SOURCE=/opt/Open3D

RUN git clone --depth 1 --branch v0.18.0 \
        https://github.com/isl-org/Open3D.git ${OPEN3D_SOURCE}

RUN cd ${OPEN3D_SOURCE} \
    && mkdir build && cd build \
    && cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=${OPEN3D_INSTALL_PREFIX} \
        -DBUILD_PYTHON_MODULE=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_BENCHMARKS=OFF \
        .. \
    && make -j$(nproc) \
    && make install \
    && ldconfig

# ─────────────────────────────────────────────────────────────
# 7. Clone FALCON and patch the two placeholders
#    (CUDA arch + Open3D resource path)
#    No source-logic is changed — only build-config values.
# ─────────────────────────────────────────────────────────────
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

RUN git clone --branch ros1-noetic \
        https://github.com/HKUST-Aerial-Robotics/FALCON.git

# No need to patch CMakeLists.txt — CUDA arch is injected via catkin_make below.

# Patch Open3D resource path in mesh_render config
RUN find /catkin_ws/src/FALCON -name "mesh_render.yaml" \
    | xargs sed -i \
        "s|open3d_resource_path:.*|open3d_resource_path: ${OPEN3D_SOURCE}/build/bin/resources|g"

# ─────────────────────────────────────────────────────────────
# 8. Build the catkin workspace
# ─────────────────────────────────────────────────────────────
WORKDIR /catkin_ws

RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    catkin_make \
        -DCMAKE_BUILD_TYPE=Release \
        -DOpen3D_DIR=${OPEN3D_INSTALL_PREFIX}/lib/cmake/Open3D \
        -DCUDA_NVCC_FLAGS=\"-gencode arch=compute_${CUDA_ARCH},code=sm_${CUDA_ARCH}\" \
    "

# ─────────────────────────────────────────────────────────────
# 9. Shell environment & entrypoint
# ─────────────────────────────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash"        >> /root/.bashrc \
    && echo "source /catkin_ws/devel/setup.bash"    >> /root/.bashrc \
    && echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /catkin_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]