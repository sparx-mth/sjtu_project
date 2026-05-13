# ============================================================
# FALCON Docker Image  (ROS1 Noetic + Adapter)
#
# Base image is selected via BASE_IMAGE build-arg:
#
#   x86_64 (WITH_SIM=1, default):
#     BASE_IMAGE=osrf/ros:noetic-desktop-full   amd64-only.
#     Brings RViz + Gazebo + all perception ROS packages.
#
#   Jetson AGX (WITH_SIM=0):
#     BASE_IMAGE=ros:noetic-perception          multiarch (arm64).
#     Brings cv_bridge, pcl_ros, pcl_conversions, laser_geometry,
#     image_transport — everything our active package set needs.
#     No RViz, no Gazebo. We CATKIN_IGNORE rviz_plugins to match.
#
#   (Why not osrf/ros:noetic-desktop-full on Jetson?
#    OSRF's desktop-full has no arm64 manifest. Pulling it on a
#    Jetson silently downloads the amd64 image and you get
#    "exec /bin/sh: exec format error" on the first RUN step.)
#
# ── WITH_SIM build-arg ──────────────────────────────────────
#   WITH_SIM=1 (default): build Open3D + mesh_render + sim packages.
#   WITH_SIM=0 (Jetson): skip Open3D entirely; CATKIN_IGNORE the
#                        simulator-side packages and rviz_plugins.
#
# ── GPU at runtime ──────────────────────────────────────────
#   No CUDA toolkit in image. On x86 -> --gpus all; on Jetson ->
#   --runtime nvidia. run_hospital.sh auto-detects.
# ============================================================
ARG BASE_IMAGE=osrf/ros:noetic-desktop-full
FROM ${BASE_IMAGE}

# Re-declare ARGs after FROM — ARGs declared before FROM are only
# visible to FROM itself.
ARG WITH_SIM=1
ARG MAX_JOBS=4
# MAX_JOBS caps parallel compile. AGX Orin (12c/32GB): 4-6 is safe.
# AGX Xavier (8c/32GB): 4. x86 with plenty of RAM: 8.

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC  LANG=C.UTF-8  LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
# Modern CMake (3.27+) errors on FALCON's cmake_minimum_required(2.8.3).
ENV CMAKE_POLICY_VERSION_MINIMUM=3.5

# ── 1. Common system tools ────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        wget git gnupg2 lsb-release ca-certificates \
        pkg-config \
        python3-tk python3-matplotlib \
        locales \
    && if [ "$WITH_SIM" = "1" ]; then \
         apt-get install -y --no-install-recommends \
             python3-pip python3-dev sudo docker.io ; \
       fi \
    && locale-gen en_US.UTF-8 && rm -rf /var/lib/apt/lists/*

RUN rosdep update

# ── 2. FALCON-required apt deps ───────────────────────────────
# C++ libs:
#   libgoogle-glog-dev — exploration_manager, fast_planner, voxel_mapping
#   libarmadillo-dev   — multi_map_server, odom_visualization, pose_utils, quadrotor_msgs
#   libeigen3-dev      — 10 active packages
#   libboost-dev       — header-only boost (bind, functional/hash)
#
# ROS packages (always installed; if the base image already has them,
# apt is idempotent and exits 0 in ~1 s):
#   cv_bridge, pcl_conversions, pcl_ros, laser_geometry,
#   image_transport, tf2_eigen, tf2_geometry_msgs
# These are present in osrf/ros:noetic-desktop-full but NOT in
# ros:noetic-perception's parent ros-base — installing them here
# means the same Dockerfile works regardless of which BASE_IMAGE
# was passed.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgoogle-glog-dev \
        libarmadillo-dev \
        libeigen3-dev \
        libboost-dev \
        ros-noetic-cv-bridge \
        ros-noetic-pcl-conversions \
        ros-noetic-pcl-ros \
        ros-noetic-laser-geometry \
        ros-noetic-image-transport \
        ros-noetic-tf2-eigen \
        ros-noetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# ── 2b. Open3D-only apt deps (WITH_SIM=1 only) ────────────────
RUN if [ "$WITH_SIM" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
            libpng-dev libjpeg-dev libssl-dev xorg-dev \
        && rm -rf /var/lib/apt/lists/* ; \
    fi

# ── 3. CMake ≥ 3.26 from Kitware APT (multiarch) ──────────────
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
        | gpg --dearmor - > /usr/share/keyrings/kitware-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main" \
        > /etc/apt/sources.list.d/kitware.list \
    && apt-get update && apt-get install -y --no-install-recommends cmake \
    && rm -rf /var/lib/apt/lists/* && cmake --version

# ── 4. NLopt 2.7.1 (used by falcon_planner/trajectory) ────────
RUN cd /tmp && git clone --depth 1 --branch v2.7.1 https://github.com/stevengj/nlopt.git \
    && cd nlopt && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j${MAX_JOBS} && make install \
    && ldconfig && rm -rf /tmp/nlopt

# ── 5. Open3D 0.18.0 (CPU-only) — only when WITH_SIM=1 ────────
ENV OPEN3D_INSTALL_PREFIX=/opt/open3d
ENV OPEN3D_SOURCE=/opt/Open3D
RUN if [ "$WITH_SIM" = "1" ]; then \
        echo "==> Building Open3D 0.18.0 (CPU, no ISPC, no GUI)" && \
        git clone --depth 1 --branch v0.18.0 \
            https://github.com/isl-org/Open3D.git ${OPEN3D_SOURCE} && \
        cd ${OPEN3D_SOURCE} && mkdir build && cd build && \
        cmake -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_INSTALL_PREFIX=${OPEN3D_INSTALL_PREFIX} \
              -DBUILD_CUDA_MODULE=OFF \
              -DBUILD_ISPC_MODULE=OFF \
              -DBUILD_GUI=OFF \
              -DBUILD_TENSORFLOW_OPS=OFF \
              -DBUILD_PYTORCH_OPS=OFF \
              -DBUILD_PYTHON_MODULE=OFF \
              -DBUILD_UNIT_TESTS=OFF \
              -DBUILD_BENCHMARKS=OFF .. && \
        make -j${MAX_JOBS} && make install && ldconfig ; \
    else \
        echo "==> WITH_SIM=0  Skipping Open3D" ; \
    fi

# ── 6. Clone FALCON + (conditionally) patch mesh_render yaml ──
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src
RUN git clone --branch ros1-noetic \
        https://github.com/HKUST-Aerial-Robotics/FALCON.git

RUN if [ "$WITH_SIM" = "1" ]; then \
        find /catkin_ws/src/FALCON -name "mesh_render.yaml" \
          | xargs --no-run-if-empty sed -i \
            "s|open3d_resource_path:.*|open3d_resource_path: ${OPEN3D_SOURCE}/build/bin/resources|g" \
          || true ; \
    fi

# ── 6b. Skip CUDA-only + (Jetson) simulator-side packages ─────
COPY ignore_cuda_pkgs.sh /tmp/ignore_cuda_pkgs.sh
RUN chmod +x /tmp/ignore_cuda_pkgs.sh \
    && WITH_SIM=${WITH_SIM} /tmp/ignore_cuda_pkgs.sh /catkin_ws/src

# ── 7. Add adapter package + map config ───────────────────────
COPY adapter/ /catkin_ws/src/falcon_adapter/
COPY hospital.yaml /catkin_ws/src/FALCON/falcon_planner/exploration_manager/config/map/hospital.yaml

# ── 8. Build everything ───────────────────────────────────────
WORKDIR /catkin_ws
RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    catkin_make \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DOpen3D_DIR=${OPEN3D_INSTALL_PREFIX}/lib/cmake/Open3D \
        -j${MAX_JOBS} \
    "

# ── 8b. Patch FALCON SOP timeout (1s → 10s) and rebuild ───────
COPY fix_falcon_sop.sh /fix_falcon_sop.sh
RUN bash /fix_falcon_sop.sh

# ── 9. Shell + entrypoint ─────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash"               >> /root/.bashrc \
    && echo "source /catkin_ws/devel/setup.bash"           >> /root/.bashrc \
    && echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]