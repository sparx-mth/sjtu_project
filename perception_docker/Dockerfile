# ============================================================
# perception_docker/Dockerfile
# ROS2 Humble container for the MORE-style semantic mapper + YOLO-World.
# Shares the ROS_DOMAIN_ID with sjtu_drone and ros1_bridge.
# ============================================================
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ── System deps ─────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-image-transport \
        ros-humble-rviz2 \
        git \
    && rm -rf /var/lib/apt/lists/*

# ── PyTorch, pinned to a CUDA build the host driver supports ──
# The host's NVIDIA driver (570.211.01) supports up to CUDA 12.8.
# We install cu128 torch FIRST so that when ultralytics is
# installed below, pip sees a satisfying torch already present
# and does not upgrade it to a newer-CUDA wheel.
#
# If you ever see the "NVIDIA driver on your system is too old
# (found version 12080)" error again after a rebuild, it means
# a newer torch wheel got pulled in — bump/adjust this pin.
RUN pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cu128 \
        torch torchvision

# ── Python deps ────────────────────────────────────────────
# scikit-image: medial_axis (Voronoi/skeleton extraction) and find_contours
#               (room polygon outlines for RViz). Required by the new
#               MORE-style semantic_mapper_node.
RUN pip install --no-cache-dir \
        "ultralytics==8.3.0" \
        "scipy>=1.10" \
        "numpy>=1.23,<2.0" \
        "scikit-image>=0.21" \
        "openai>=1.30" \
        "transforms3d>=0.4" \
        "ftfy" "regex" "tqdm" \
        "git+https://github.com/openai/CLIP.git"

# ── Sanity check at build time: fail the build early if torch
#    can't import cleanly. (We can't test .cuda.is_available()
#    here because the build container has no GPU, but we can
#    confirm the wheel's CUDA build tag matches what we asked for.)
RUN python3 -c "import torch; print('torch', torch.__version__, 'built for CUDA', torch.version.cuda); assert torch.version.cuda and torch.version.cuda.startswith('12.'), 'unexpected torch CUDA build'"

# ── Workspace ───────────────────────────────────────────────
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

COPY cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]