# ============================================================
# FALCON Docker Image  (ROS1 Noetic + Adapter)
# Base: osrf/ros:noetic-desktop-full  — multiarch (amd64/arm64).
#
# ── WITH_SIM build-arg ──────────────────────────────────────
#   WITH_SIM=1 (default — x86 desktop, built-in FALCON sim):
#     Builds Open3D 0.18 and FALCON's mesh_render. Installs the
#     extra apt deps that Open3D 0.18 wants (libpng/libjpeg/libssl
#     for image+download, libglfw3+libgl1-mesa+xorg-dev are pulled
#     in as a precaution even with BUILD_GUI=OFF — Open3D's CMake
#     probes for them).
#
#   WITH_SIM=0 (Jetson, real-drone only):
#     Skips Open3D entirely. CATKIN_IGNORE'd packages: pointcloud_render,
#     map_render, mesh_render, so3_quadrotor_simulator, so3_control,
#     so3_disturbance_generator, poscmd_2_odom, waypoint_generator
#     (cascades to anything that depends on them).
#     The Open3D-only apt deps are NOT installed.
#
# ── What was DROPPED relative to v1 of this Dockerfile ──────
#   Audited every apt package against every active CMakeLists.txt
#   and every #include in non-ignored .cpp/.h files. Removed:
#     libdw-dev / libdwarf-dev    only referenced in CODE COMMENTS
#     libc++-dev / libc++abi-dev  zero uses anywhere in FALCON
#     libyaml-cpp-dev             zero uses; ROS provides it transitively
#     ninja-build                 build uses `make`, never ninja
#     curl                        only wget is used in this Dockerfile
#     libboost-all-dev -> libboost-dev
#                                 only header-only Boost (bind.hpp,
#                                 functional/hash.hpp) is used; ROS
#                                 already provides any compiled libs
#                                 via PCL's dependency chain.
#   Added: python3-matplotlib  (bev_click_goal.py imports matplotlib;
#                               the v1 Dockerfile installed python3-tk
#                               (the backend) but not the library
#                               itself — your file's docstring even
#                               notes "If missing inside the container,
#                               apt-get install -y python3-matplotlib").
#
# ── GPU at runtime ──────────────────────────────────────────
#   No CUDA toolkit in image. NVIDIA Container Toolkit gives GL/EGL.
#   On x86 -> --gpus all. On Jetson -> --runtime nvidia.
#   run_hospital.sh auto-detects.
# ============================================================
FROM osrf/ros:noetic-desktop-full

ARG WITH_SIM=1
ARG MAX_JOBS=4
# MAX_JOBS caps parallel compile. AGX Orin (12c/32GB): 4-6 is safe.
# AGX Xavier (8c/32GB): 4. x86 with plenty of RAM: 8.
# Drop to 2 if dmesg shows oom-kill during build.

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC  LANG=C.UTF-8  LC_ALL=C.UTF-8
ENV ROS_DISTRO=noetic
# Modern CMake (3.27+) errors on FALCON's cmake_minimum_required(2.8.3).
# Setting this once globally is simpler than patching every CMakeLists.
ENV CMAKE_POLICY_VERSION_MINIMUM=3.5

# ── 1. Common system tools ────────────────────────────────────
# Trimmed: dropped curl (wget suffices), ninja-build (unused),
# sudo (not needed as root), and made python3-pip/python3-dev
# conditional (only WITH_SIM=1 keeps them for sim-side debugging).
#
# python3-tk         — TkAgg backend
# python3-matplotlib — bev_click_goal.py viewer
# docker.io          — respawn_drone.py (sim-side respawn)
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

# ── 2. FALCON-required apt deps (audited) ─────────────────────
# libgoogle-glog-dev — exploration_manager, fast_planner, voxel_mapping
# libarmadillo-dev   — multi_map_server, odom_visualization, pose_utils, quadrotor_msgs
# libeigen3-dev      — 10 active packages
# libboost-dev       — header-only boost/bind, boost/functional/hash; ROS
#                       brings any compiled Boost libs via PCL transitively
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgoogle-glog-dev \
        libarmadillo-dev \
        libeigen3-dev \
        libboost-dev \
    && rm -rf /var/lib/apt/lists/*

# ── 2b. Open3D-only apt deps (WITH_SIM=1 only) ────────────────
# These are pulled in by Open3D 0.18's CMake even when BUILD_GUI=OFF
# (some are probes only, but missing them slows the configure step
# substantially and a couple are genuine link-time deps of Open3D's
# bundled 3rd party libs). On Jetson (WITH_SIM=0) Open3D isn't built,
# so we skip the entire group — ~250 MB savings.
RUN if [ "$WITH_SIM" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
            libpng-dev libjpeg-dev libssl-dev xorg-dev \
        && rm -rf /var/lib/apt/lists/* ; \
    fi

# ── 3. CMake ≥ 3.26 from Kitware APT (multiarch) ──────────────
# Replaces the 15-min from-source RC build. Kitware ships both
# amd64 and arm64 packages.
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null \
        | gpg --dearmor - > /usr/share/keyrings/kitware-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main" \
        > /etc/apt/sources.list.d/kitware.list \
    && apt-get update && apt-get install -y --no-install-recommends cmake \
    && rm -rf /var/lib/apt/lists/* && cmake --version

# ── 4. NLopt 2.7.1 ────────────────────────────────────────────
# Used by falcon_planner/trajectory (#include <nlopt.hpp>). Ubuntu
# focal ships NLopt 2.6.1 in apt, but FALCON is tested with 2.7.1
# and the API differs in a few helper signatures, so we build the
# pinned version from source. ~3 min on any platform.
RUN cd /tmp && git clone --depth 1 --branch v2.7.1 https://github.com/stevengj/nlopt.git \
    && cd nlopt && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j${MAX_JOBS} && make install \
    && ldconfig && rm -rf /tmp/nlopt

# ── 5. Open3D 0.18.0 (CPU-only) — only when WITH_SIM=1 ────────
# Only used by mesh_render. Skipped entirely on Jetson.
# On x86, ISPC=OFF is harmless (slight runtime slowdown of mesh
# rendering) and lets the same cmake line work on aarch64 if you
# ever want to flip WITH_SIM=1 on Jetson.
ENV OPEN3D_INSTALL_PREFIX=/opt/open3d
ENV OPEN3D_SOURCE=/opt/Open3D
RUN if [ "$WITH_SIM" = "1" ]; then \
        echo "==> Building Open3D 0.18.0 (CPU, no ISPC, no GUI) — WITH_SIM=1" && \
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
        echo "==> WITH_SIM=0  Skipping Open3D (mesh_render is CATKIN_IGNORE'd)" ; \
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

# ── 8. Build everything (no CUDA flags) ───────────────────────
# Open3D_DIR is only meaningful when WITH_SIM=1; harmless otherwise.
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