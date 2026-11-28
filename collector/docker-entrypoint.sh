#!/bin/sh
# Entrypoint for the collector image.
#
# `ros:humble-ros-base` needs /opt/ros/<distro>/setup.bash sourced so that rclpy
# and the message packages are importable; `/bin/sh` does not read ~/.bashrc and
# the Docker ENTRYPOINT form cannot source a file itself. Simulator-only images
# (python:*-slim via --build-arg BASE_IMAGE) have no ROS installation, so the
# source step is skipped and `--mode sim` keeps working unchanged.
set -e

if [ -z "${ROS_DISTRO:-}" ]; then
    ROS_DISTRO=humble
    export ROS_DISTRO
fi

ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "$ros_setup" ]; then
    # shellcheck disable=SC1090
    . "$ros_setup"
fi

exec python3 -m collector.collector "$@"
