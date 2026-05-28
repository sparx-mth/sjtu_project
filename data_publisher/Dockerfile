# Match the Jetson: ROS 2 Humble, default middleware (Fast DDS). No Zenoh, no
# DDS configuration of any kind. Zenoh is not even available on Humble.
FROM ros:humble-ros-base

# The replay nodes only need numpy + std/sensor/geometry msgs (already in ros-base).
# No cv_bridge, no opencv, no sparx_agency / venv -- they are self-contained.
RUN apt-get update && apt-get install -y python3-numpy python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY data_publisher.py demo_mode_manager.py xtend_twist_to_cmd_nav.py entrypoint.sh /app/
RUN chmod +x /app/entrypoint.sh

ENTRYPOINT ["/app/entrypoint.sh"]
# Default arguments (used when none are passed). Override by appending --ros-args.
CMD ["--ros-args", \
     "-p", "publish_rate_hz:=3.0", \
     "-p", "pose_delay_mean:=0.020", \
     "-p", "pose_delay_std:=0.010", \
     "-p", "depth_encoding:=32FC1"]