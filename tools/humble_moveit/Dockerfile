FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Use domestic mirrors for Ubuntu and ROS 2 apt packages.
RUN set -eux; \
    grep -rlI 'packages.ros.org' /etc/apt 2>/dev/null \
        | xargs -r sed -i 's|packages.ros.org/ros2/ubuntu|mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g'; \
    grep -rlI 'archive.ubuntu.com' /etc/apt 2>/dev/null \
        | xargs -r sed -i 's|archive.ubuntu.com/ubuntu|mirrors.tuna.tsinghua.edu.cn/ubuntu|g'; \
    grep -rlI 'security.ubuntu.com' /etc/apt 2>/dev/null \
        | xargs -r sed -i 's|security.ubuntu.com/ubuntu|mirrors.tuna.tsinghua.edu.cn/ubuntu|g'; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        python3-colcon-common-extensions \
        python3-setuptools \
        ros-humble-ament-cmake \
        ros-humble-rviz2 \
        ros-humble-moveit \
        ros-humble-moveit-configs-utils \
        ros-humble-pilz-industrial-motion-planner \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-robot-state-publisher; \
    rm -rf /var/lib/apt/lists/*

# CycloneDDS avoids the Fast DDS graph-discovery incompatibility between the
# laptop image and the vendor Humble image on the robot.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Match the host UID/GID so run_humble.sh has a named user and a writable
# home, avoiding the "I have no name!" prompt.
RUN set -eux; \
    if ! getent group 1000 >/dev/null; then groupadd --gid 1000 user; fi; \
    if ! getent passwd 1000 >/dev/null; then useradd --uid 1000 --gid 1000 -m -s /bin/bash user; fi

# The image entrypoint sources /opt/ros/humble/setup.bash.
# The workspace is mounted at /workspace by run_humble.sh.
WORKDIR /workspace
