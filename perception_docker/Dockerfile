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

# ── Workspace ───────────────────────────────────────────────
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

COPY cyclonedds.xml /etc/cyclonedds.xml
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]