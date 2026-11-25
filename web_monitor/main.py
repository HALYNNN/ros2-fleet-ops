"""FastAPI monitoring API and static dashboard for ros2-fleet-ops.

Endpoints (all JSON except `/`):

    GET  /api/health
    GET  /api/summary                 fleet snapshot (robots + alarm counters)
    GET  /api/robots                  latest state, measured rates, health
    GET  /api/robots/{id}/trajectory  recorded /odom path for a run
    GET  /api/runs                    drive records (runs)
    GET  /api/runs/{run_id}           run detail + analysis findings
    GET  /api/alarms                  recent alarms (filter by run/severity)
    POST /api/replay                  start replay of a run
    GET  /api/replay                  replay progress + frames
    POST /api/replay/stop             stop replay
    GET  /                            static dashboard
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:  # allows `uvicorn web_monitor.main:app` from the repo root
    sys.path.insert(0, str(ROOT))

from collector.analyzer import analyze_run  # noqa: E402
from collector.replay import ReplayEngine, load_frames  # noqa: E402
from collector.storage import Store  # noqa: E402


class ReplayRequest(BaseModel):
    run_id: str
    speed: float = 1.0


def _rate_from_samples(samples: list[dict[str, Any]]) -> float | None:
    if len(samples) < 2:
        return None
    span = samples[-1]["source_ts"] - samples[0]["source_ts"]
    if span <= 0:
        return None
    return round((len(samples) - 1) / span, 2)


def robot_status(store: Store, robot_id: str, *, now: float | None = None) -> dict[str, Any]:
    """Live status for one robot built from its latest run's recent samples."""
    now = time.time() if now is None else now
    runs = store.list_runs(limit=1, robot_id=robot_id)
    run = runs[0] if runs else None
    if run is None:
        raise KeyError(robot_id)
    recent = store.recent_samples(run["run_id"], limit=300)
    by_topic: dict[str, list[dict[str, Any]]] = {}
    for sample in recent:
        by_topic.setdefault(sample["topic"], []).append(sample)

    topics = []
    for topic, samples in sorted(by_topic.items()):
        last = samples[-1]
        topics.append(
            {
                "topic": topic,
                "count": len(samples),
                "rate_hz": _rate_from_samples(samples[-60:]),
                "last_recv_ts": last["recv_ts"],
                "latency_ms": round((last["recv_ts"] - last["source_ts"]) * 1000.0, 1),
            }
        )

    odom = by_topic.get("/odom", [])
    pose = None
    if odom:
        data = odom[-1]["data"]
        pose = {
            "x": data.get("x"),
            "y": data.get("y"),
            "yaw": data.get("yaw"),
            "linear_x": data.get("linear_x"),
        }

    alarms = store.fetch_alarms(run_id=run["run_id"], limit=20)
    last_alarm = alarms[0] if alarms else None
    recent_critical = [a for a in alarms if a["severity"] == "critical" and now - a["ts"] < 300]
    recent_warning = [a for a in alarms if a["severity"] == "warning" and now - a["ts"] < 300]
    last_seen = max((sample["recv_ts"] for sample in recent), default=None)
    if last_seen is None:
        health = "stale" if run["open"] else "recorded"
    elif now - last_seen > 10:
        # A finished run is a recording, not a robot that dropped offline.
        health = "stale" if run["open"] else "recorded"
    elif recent_critical:
        health = "critical"
    elif recent_warning:
        health = "warning"
    else:
        health = "ok"

    return {
        "robot_id": robot_id,
        "run_id": run["run_id"],
        "source": run["source"],
        "run_open": run["open"],
        "started_at": run["started_at"],
        "last_seen": last_seen,
        "health": health,
        "pose": pose,
        "topics": topics,
        "last_alarm": last_alarm,
        "alarm_counts": store.alarm_counts(run["run_id"]),
    }


def create_app(db_path: str | Path | None = None, jsonl_dir: str | Path | None = None) -> FastAPI:
    db = str(db_path or os.environ.get("FLEET_DB", ROOT / "data" / "fleet.db"))
    jsonl = str(jsonl_dir or os.environ.get("FLEET_JSONL_DIR", ROOT / "data" / "jsonl"))
    store = Store(db, jsonl)
    state: dict[str, ReplayEngine | None] = {"replay": None}

    app = FastAPI(
        title="ros2-fleet-ops monitor",
        version="0.1.0",
        description="Robot telemetry collection, live monitoring and drive-record replay",
    )
    app.state.store = store
    app.state.replay_state = state

    @app.get("/api/health")
    def health() -> dict[str, Any]:
        heartbeat = store.read_heartbeat(max_age_s=None)
        return {
            "status": "ok",
            "time": time.time(),
            "db": db,
            "runs": len(store.list_runs(limit=1000)),
            "collector": heartbeat,
            "collector_alive": bool(heartbeat and time.time() - heartbeat.get("ts", 0) <= 30),
        }

    @app.get("/api/summary")
    def summary() -> dict[str, Any]:
        runs = store.list_runs(limit=20)
        robot_ids = sorted({run["robot_id"] for run in runs})
        robots = []
        for robot_id in robot_ids:
            try:
                robots.append(robot_status(store, robot_id))
            except KeyError:
                continue
        alarms = store.fetch_alarms(limit=10)
        return {
            "now": time.time(),
            "runs": len(runs),
            "robots": robots,
            "latest_alarms": alarms,
        }

    @app.get("/api/robots")
    def robots() -> list[dict[str, Any]]:
        runs = store.list_runs(limit=50)
        out = []
        for robot_id in sorted({run["robot_id"] for run in runs}):
            try:
                out.append(robot_status(store, robot_id))
            except KeyError:
                continue
        return out

    @app.get("/api/robots/{robot_id}/trajectory")
    def trajectory(
        robot_id: str,
        run_id: str | None = None,
        max_points: int = Query(default=2000, ge=2, le=20000),
    ) -> dict[str, Any]:
        run = store.get_run(run_id) if run_id else (store.list_runs(limit=1, robot_id=robot_id) or [None])[0]
        if run is None or run["robot_id"] != robot_id:
            raise HTTPException(status_code=404, detail=f"run not found for robot {robot_id}")
        points = store.trajectory(run["run_id"], max_points=max_points)
        return {"robot_id": robot_id, "run_id": run["run_id"], "points": points, "count": len(points)}

    @app.get("/api/runs")
    def runs(limit: int = Query(default=30, ge=1, le=200), robot_id: str | None = None) -> list[dict[str, Any]]:
        return store.list_runs(limit=limit, robot_id=robot_id)

    @app.get("/api/runs/{run_id}")
    def run_detail(run_id: str) -> dict[str, Any]:
        run = store.get_run(run_id)
        if run is None:
            raise HTTPException(status_code=404, detail=f"run not found: {run_id}")
        try:
            analysis = analyze_run(store, run_id)
        except KeyError:
            raise HTTPException(status_code=404, detail=f"run not found: {run_id}") from None
        return {
            "run": run,
            "topics": analysis["topics"],
            "findings": analysis["findings"],
            "verdict": analysis["verdict"],
            "alarms": analysis["alarms"],
        }

    @app.get("/api/alarms")
    def alarms(
        limit: int = Query(default=50, ge=1, le=500),
        severity: str | None = None,
        run_id: str | None = None,
        robot_id: str | None = None,
    ) -> list[dict[str, Any]]:
        return store.fetch_alarms(run_id=run_id, severity=severity, limit=limit, robot_id=robot_id)

    @app.post("/api/replay")
    def start_replay(request: ReplayRequest) -> dict[str, Any]:
        run = store.get_run(request.run_id)
        if run is None:
            raise HTTPException(status_code=404, detail=f"run not found: {request.run_id}")
        frames = load_frames(store, request.run_id)
        if not frames:
            raise HTTPException(status_code=409, detail="run has no samples to replay")
        previous = state.get("replay")
        if previous is not None:
            previous.stop()
        speed = min(20.0, max(0.1, float(request.speed)))
        engine = ReplayEngine(frames, run_id=request.run_id, speed=speed)
        engine.start()
        state["replay"] = engine
        return engine.status()

    @app.get("/api/replay")
    def replay_status(
        since: int = Query(default=0, ge=0), max_frames: int = Query(default=2000, ge=1, le=10000)
    ) -> dict[str, Any]:
        engine = state.get("replay")
        if engine is None:
            return {"state": "idle", "index": 0, "total": 0, "progress": 0.0, "frames": []}
        return engine.status(since_index=since, max_frames=max_frames)

    @app.post("/api/replay/stop")
    def stop_replay() -> dict[str, Any]:
        engine = state.get("replay")
        if engine is None:
            return {"state": "idle"}
        engine.stop()
        return engine.status()

    static_dir = Path(__file__).resolve().parent / "static"

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(static_dir / "index.html")

    app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")
    return app


app = create_app()
