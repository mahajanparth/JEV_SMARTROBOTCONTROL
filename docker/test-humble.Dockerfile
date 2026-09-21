FROM ros:humble-ros-base-jammy

# Headless, isolated runtime for the recovery demo's ROS integration checks.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pytest \
    ros-humble-tf2-ros \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=87
WORKDIR /ws
CMD ["sleep", "infinity"]
