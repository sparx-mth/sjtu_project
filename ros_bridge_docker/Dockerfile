FROM ubuntu:20.04

# ─────────────────────────────────────────────────────────────────
# Ubuntu 20.04 (Focal) is the ONLY OS where ROS1 + ROS2 coexist:
#   ROS1 Noetic  →  officially supports Focal
#   ROS2 Foxy    →  officially supports Focal
#   ROS2 Humble  →  Jammy only (22.04) — WON'T work here
#
# Your sim (Humble/Jazzy) runs outside this container.
# The bridge's ROS2 (Foxy) side talks to your sim via DDS/--net=host.
# Standard message types are wire-compatible across ROS2 distros.
# ─────────────────────────────────────────────────────────────────

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV ROS1_DISTRO=noetic
ENV ROS2_DISTRO=foxy

# ── Base system ───────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    locales curl gnupg2 lsb-release \
    build-essential cmake git \
    python3-pip \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ── Add ROS1 Noetic repo ──────────────────────────────────────────
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros1-latest.list' && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | apt-key add -

# ── Add ROS2 Foxy repo ────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
      signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros2.list

# ── Install ROS1 Noetic + ROS2 Foxy ──────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-tf2-ros \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-std-msgs \
    ros-foxy-ros-base \
    ros-foxy-rmw-fastrtps-cpp \
    ros-foxy-tf2-ros \
    ros-foxy-geometry-msgs \
    ros-foxy-sensor-msgs \
    ros-foxy-nav-msgs \
    ros-foxy-std-msgs \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# ── colcon + vcstool via pip (not in focal apt) ───────────────────
RUN pip3 install colcon-common-extensions vcstool

# ── Build ros1_bridge from source ────────────────────────────────
# Use the foxy branch — matches our ROS2 side.
# MUST be built with both ROS envs sourced simultaneously so it
# auto-discovers and maps all matching message types.
RUN mkdir -p /bridge_ws/src && \
    git clone --depth 1 -b foxy \
    https://github.com/ros2/ros1_bridge.git \
    /bridge_ws/src/ros1_bridge

RUN bash -c "\
    source /opt/ros/noetic/setup.bash && \
    source /opt/ros/foxy/setup.bash && \
    cd /bridge_ws && \
    colcon build \
      --symlink-install \
      --packages-select ros1_bridge \
      --cmake-force-configure \
      --event-handlers console_direct+"

# ── Entrypoint ────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD []