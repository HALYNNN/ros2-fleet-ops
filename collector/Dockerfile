# syntax=docker/dockerfile:1
#
# Collector image. Default base is ROS 2 Humble (rclpy is needed for --mode ros2).
# For a simulator-only deployment on a plain machine use:
#   docker build -f collector/Dockerfile --build-arg BASE_IMAGE=python:3.12-slim .
ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    ROS_DOMAIN_ID=30 \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp

WORKDIR /app

COPY collector /app/collector
COPY scripts /app/scripts

RUN if command -v ros2 >/dev/null 2>&1; then \
      apt-get update && apt-get install -y --no-install-recommends python3-pip && \
      rm -rf /var/lib/apt/lists/*; \
    fi \
 && python3 -m pip install --no-cache-dir -r /app/collector/requirements.txt

RUN mkdir -p /data
VOLUME ["/data"]

HEALTHCHECK --interval=15s --timeout=5s --start-period=20s --retries=3 \
  CMD python3 -m collector.healthcheck --db /data/fleet.db --max-age 25 || exit 1

ENTRYPOINT ["python3", "-m", "collector.collector"]
CMD ["--mode", "ros2", "--db", "/data/fleet.db", "--jsonl-dir", "/data/jsonl", "--use-sim-time"]
