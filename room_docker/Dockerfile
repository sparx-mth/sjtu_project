# ============================================================
# room_docker/Dockerfile
# ROS2 Humble container for the room_search task — same base as
# perception_docker (YOLO-World + cv_bridge + cyclonedds), with one
# extra ament_python package: room_search (the orchestrator).
#
# Build context expects:
#   ./cyclonedds.xml          ← copied to /etc/cyclonedds.xml
#   ./entrypoint.sh           ← copied to /entrypoint.sh
# At runtime the semantic_mapper and room_search source trees are
# bind-mounted into /ros2_ws/src so edits round-trip without rebuilds.
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
        git \
    && rm -rf /var/lib/apt/lists/*

# ── PyTorch (matches perception_docker so wheel caches are reusable) ──
# See perception_docker/Dockerfile for the rationale on pinning cu128.
RUN pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cu128 \
        torch torchvision

# ── Python deps ────────────────────────────────────────────
# Same set as perception_docker so semantic_mapper imports cleanly when
# it's bind-mounted into /ros2_ws/src alongside room_search.
RUN pip install --no-cache-dir \
        "ultralytics==8.3.0" \
        "scipy>=1.10" \
        "numpy>=1.23,<2.0" \
        "scikit-image>=0.21" \
        "openai>=1.30" \
        "transforms3d>=0.4" \
        "ompl" \
        "ftfy" "regex" "tqdm" \
        "git+https://github.com/openai/CLIP.git"

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
