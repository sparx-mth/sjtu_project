ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core-jammy

RUN apt-get update \
    && apt-get install -y \
    wget curl unzip \
    lsb-release \
    mesa-utils \
    build-essential \
    python3-pip \
    python3-pyqt5 \
    python3-tk \
    && apt-get clean

# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y \
    gazebo \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    python3-colcon-common-extensions python3-rosdep --no-install-recommends \
    && apt-get clean
RUN rosdep init && rosdep update

# Nav2 + rmw + python deps required for planners & mapping
RUN apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-nav-msgs \
        ros-${ROS_DISTRO}-nav2-costmap-2d \
        ros-${ROS_DISTRO}-nav2-map-server \
        ros-${ROS_DISTRO}-ompl \
        libompl-dev \
        libyaml-cpp-dev \
        libopencv-dev \
        python3-opencv python3-yaml python3-numpy python3-scipy \
        libgl1 libglib2.0-0 \
    && apt-get clean

# ---------------------------------------------------------
# Python packages for mapping, plotting & visualization
# ---------------------------------------------------------
RUN pip3 install --upgrade pip \
    && pip3 install --no-cache-dir numpy==1.26.4 matplotlib imageio pillow

RUN mkdir -p /ros2_ws/src
COPY ./sjtu_drone_description /ros2_ws/src/sjtu_drone_description
COPY ./sjtu_drone_bringup /ros2_ws/src/sjtu_drone_bringup
COPY ./sjtu_drone_control /ros2_ws/src/sjtu_drone_control

RUN curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp \
    && mkdir -p ~/.gazebo/models/ \
    && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip

# OmniVLA deps: venv + git + ROS cv_bridge
RUN apt-get update && apt-get install -y \
        python3-venv python3-dev \
        python-is-python3 \
        git \
        ros-${ROS_DISTRO}-cv-bridge \
        python3-cv-bridge \
    && apt-get clean

# Create a stable venv that can see apt/ROS site-packages (cv2, cv_bridge)
RUN python3 -m venv --system-site-packages /opt/omnivla_venv \
    && /opt/omnivla_venv/bin/pip install --upgrade pip

# Install OmniVLA Python deps inside the venv
# Important: do NOT pip-install opencv-python (it can force numpy>=2 and break cv_bridge)
RUN /opt/omnivla_venv/bin/pip install --no-cache-dir \
        numpy==1.26.4 \
        torch==2.2.0 torchvision==0.17.0 torchaudio==2.2.0 \
        transformers==4.40.1 accelerate==1.12.0 \
        timm==0.9.10 \
        draccus==0.8.0 \
        rich==14.2.0 \
        huggingface-hub==0.29.1 \
        safetensors==0.7.0 tokenizers==0.19.1 sentencepiece==0.1.99 \
        utm==0.8.1 \
        matplotlib pillow \
        einops==0.8.1 peft==0.11.1 diffusers==0.11.1 datasets==4.5.0 wandb==0.24.0

# dlimp is installed from GitHub
RUN /opt/omnivla_venv/bin/pip install --no-cache-dir \
        "git+https://github.com/kvablack/dlimp.git"

# Build-time sanity checks (fail fast if something is wrong)
RUN /opt/omnivla_venv/bin/python -c "import numpy as np; print('numpy', np.__version__)" \
 && /opt/omnivla_venv/bin/python -c "import cv2; print('cv2', cv2.__version__)" \
 && /opt/omnivla_venv/bin/python -c "from cv_bridge import CvBridge; print('cv_bridge ok')" \
 && /opt/omnivla_venv/bin/python -c "import torch; print('torch', torch.__version__, 'cuda', torch.cuda.is_available())" \
 && /opt/omnivla_venv/bin/python -c "import transformers, timm, draccus, rich, utm, dlimp; print('omnivla deps ok')"

# Convenience: default to venv python/pip without manual activation
ENV PATH="/opt/omnivla_venv/bin:${PATH}"

WORKDIR /ros2_ws
RUN apt-get update && \
    /bin/bash -c 'cd /ros2_ws/ \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build' && \
    apt-get clean

CMD ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash && bash"]
