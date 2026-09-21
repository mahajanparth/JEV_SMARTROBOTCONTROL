FROM ros:humble-ros-base-jammy

# Keep Gazebo/TurtleBot/RViz dependencies off the host's partial Foxy install.
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-pytest \
    ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 \
    ros-humble-nav2-amcl ros-humble-nav2-map-server ros-humble-nav2-lifecycle-manager \
    ros-humble-tf2-ros ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=88 TURTLEBOT3_MODEL=burger QT_X11_NO_MITSHM=1
WORKDIR /ws
CMD ["sleep", "infinity"]
