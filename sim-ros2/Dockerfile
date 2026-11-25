# syntax=docker/dockerfile:1
#
# Gazebo + TurtleBot3 world. This is the "robot side" of the stack: it publishes
# /odom, /scan, /cmd_vel and /clock, which the collector subscribes to.
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    TURTLEBOT3_MODEL=burger \
    GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models \
    LIBGL_ALWAYS_SOFTWARE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-turtlebot3 \
      ros-humble-turtlebot3-gazebo \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

COPY sim-ros2/entrypoint.sh /usr/local/bin/sim-entrypoint.sh
RUN chmod +x /usr/local/bin/sim-entrypoint.sh

ENTRYPOINT ["/usr/local/bin/sim-entrypoint.sh"]
