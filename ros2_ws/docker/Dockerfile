FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-serial \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY ros2_ws /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
