#!/usr/bin/env bash
# Launch the TurtleBot3 Gazebo world (headless by default).
set -euo pipefail

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"
export GAZEBO_MODEL_PATH="/opt/ros/humble/share/turtlebot3_gazebo/models:${GAZEBO_MODEL_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE=1
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_BUFFERED_STREAM=1

LAUNCH_FILE="turtlebot3_gazebo turtlebot3_world.launch.py"
LAUNCH_ARGS=("use_sim_time:=true")

# The Humble launch files differ slightly between patch releases: only pass the
# `gui` argument when the installed file declares it, otherwise the launch
# aborts with "unused argument". gzserver runs headless either way.
if ros2 launch ${LAUNCH_FILE} --show-args 2>/dev/null | grep -q "gui:"; then
  LAUNCH_ARGS+=("gui:=${GAZEBO_GUI:-false}")
fi

echo "[sim-ros2] starting ${LAUNCH_FILE} with ${LAUNCH_ARGS[*]}"
exec ros2 launch ${LAUNCH_FILE} "${LAUNCH_ARGS[@]}"
