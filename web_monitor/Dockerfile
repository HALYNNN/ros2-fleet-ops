# syntax=docker/dockerfile:1
FROM python:3.12-slim

ENV PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    FLEET_DB=/data/fleet.db \
    FLEET_JSONL_DIR=/data/jsonl

WORKDIR /app

COPY collector /app/collector
COPY web_monitor /app/web_monitor

RUN python -m pip install --no-cache-dir --upgrade pip \
 && python -m pip install --no-cache-dir fastapi "uvicorn[standard]" PyYAML

RUN mkdir -p /data
VOLUME ["/data"]

EXPOSE 8000

HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
  CMD python -c "import urllib.request,sys; sys.exit(0 if urllib.request.urlopen('http://127.0.0.1:8000/api/health', timeout=3).status == 200 else 1)"

CMD ["uvicorn", "web_monitor.main:app", "--host", "0.0.0.0", "--port", "8000"]
