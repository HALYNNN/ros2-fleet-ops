from __future__ import annotations

import time

import pytest
from fastapi.testclient import TestClient

from collector.storage import Store
from web_monitor.main import create_app


@pytest.fixture()
def seeded(tmp_path):
    db = tmp_path / "api.db"
    jsonl = tmp_path / "jsonl"
    store = Store(db, jsonl)
    now = time.time()
    run_id = "api-run"
    store.start_run("tb3-01", source="sim", run_id=run_id, started_at=now - 6.0)
    for index in range(40):
        ts = now - 5.0 + index * 0.05
        store.insert_sample(
            run_id=run_id,
            robot_id="tb3-01",
            topic="/odom",
            seq=index + 1,
            source_ts=ts,
            recv_ts=ts + 0.008,
            data={"x": 0.01 * index, "y": 0.0, "yaw": 0.0, "linear_x": 0.2},
        )
    for index in range(20):
        ts = now - 5.0 + index * 0.1
        store.insert_sample(
            run_id=run_id,
            robot_id="tb3-01",
            topic="/scan",
            seq=index + 1,
            source_ts=ts,
            recv_ts=ts + 0.01,
            data={"point_count": 36, "min_range": 0.6},
        )
    store.insert_alarm(
        {
            "run_id": run_id,
            "robot_id": "tb3-01",
            "ts": now - 2.0,
            "kind": "topic_drop",
            "severity": "warning",
            "topic": "/odom",
            "message": "메시지 2건 결번",
            "value": 2.0,
            "threshold": 0.0,
        }
    )
    store.write_heartbeat({"run_id": run_id, "samples": 60})
    store.finish_run(run_id, finished_at=now)
    store.close()

    app = create_app(db, jsonl)
    client = TestClient(app)
    yield client, run_id, now
    app.state.store.close()


def test_health(seeded):
    client, run_id, _ = seeded
    response = client.get("/api/health")
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "ok"
    assert body["runs"] == 1
    assert body["collector_alive"] is True
    assert body["collector"]["run_id"] == run_id


def test_robots_status(seeded):
    client, run_id, _ = seeded
    robots = client.get("/api/robots").json()
    assert len(robots) == 1
    robot = robots[0]
    assert robot["robot_id"] == "tb3-01"
    assert robot["health"] in ("ok", "warning", "stale")
    assert robot["pose"]["x"] is not None
    topics = {topic["topic"]: topic for topic in robot["topics"]}
    assert topics["/odom"]["rate_hz"] == pytest.approx(20.0, abs=1.0)
    assert robot["alarm_counts"] == {"topic_drop": 1}


def test_summary_and_alarm_feed(seeded):
    client, run_id, _ = seeded
    summary = client.get("/api/summary").json()
    assert summary["runs"] == 1
    assert len(summary["robots"]) == 1
    assert summary["latest_alarms"][0]["kind"] == "topic_drop"

    alarms = client.get("/api/alarms", params={"severity": "warning"}).json()
    assert len(alarms) == 1
    assert client.get("/api/alarms", params={"severity": "critical"}).json() == []


def test_trajectory(seeded):
    client, run_id, _ = seeded
    response = client.get("/api/robots/tb3-01/trajectory", params={"run_id": run_id})
    assert response.status_code == 200
    body = response.json()
    assert body["count"] == 40
    assert body["points"][0]["x"] == 0.0
    assert body["points"][-1]["x"] == pytest.approx(0.39)

    assert client.get("/api/robots/tb3-02/trajectory").status_code == 404


def test_run_detail(seeded):
    client, run_id, _ = seeded
    detail = client.get(f"/api/runs/{run_id}").json()
    assert detail["run"]["run_id"] == run_id
    assert detail["verdict"] in ("degraded", "critical")
    assert detail["alarms"]["by_kind"]["topic_drop"] == 1
    assert any(finding["kind"] == "topic_drop" for finding in detail["findings"])
    assert client.get("/api/runs/missing").status_code == 404


def test_runs_listing(seeded):
    client, run_id, _ = seeded
    runs = client.get("/api/runs").json()
    assert runs[0]["run_id"] == run_id
    assert runs[0]["sample_count"] == 60
    assert runs[0]["alarm_count"] == 1


def test_replay_lifecycle(seeded):
    client, run_id, _ = seeded
    idle = client.get("/api/replay").json()
    assert idle["state"] == "idle"

    started = client.post("/api/replay", json={"run_id": run_id, "speed": 4.0})
    assert started.status_code == 200
    assert started.json()["state"] == "running"
    time.sleep(0.2)
    polled = client.get("/api/replay", params={"since": 0}).json()
    assert polled["index"] > 0
    assert polled["total"] == 60
    assert any(frame["topic"] == "/odom" for frame in polled["frames"])

    stopped = client.post("/api/replay/stop").json()
    assert stopped["state"] == "stopped"


def test_replay_errors(seeded):
    client, run_id, _ = seeded
    assert client.post("/api/replay", json={"run_id": "nope"}).status_code == 404


def test_replay_rejects_empty_run(seeded, tmp_path):
    client, _, _ = seeded
    store = client.app.state.store
    store.start_run("tb3-02", run_id="empty-run", started_at=time.time())
    store.finish_run("empty-run")
    response = client.post("/api/replay", json={"run_id": "empty-run"})
    assert response.status_code == 409


def test_closed_old_run_is_reported_as_recording(seeded):
    client, _, _ = seeded
    store = client.app.state.store
    old = time.time() - 3600
    store.start_run("tb3-old", source="sim", run_id="old-run", started_at=old)
    store.insert_sample(
        run_id="old-run", robot_id="tb3-old", topic="/odom", seq=1,
        source_ts=old, recv_ts=old, data={"x": 0.0, "y": 0.0},
    )
    store.finish_run("old-run", finished_at=old + 5)
    robots = {robot["robot_id"]: robot for robot in client.get("/api/robots").json()}
    assert robots["tb3-old"]["health"] == "recorded"


def test_dashboard_is_served(seeded):
    client, _, _ = seeded
    response = client.get("/")
    assert response.status_code == 200
    assert "ros2-fleet-ops" in response.text
    assert "Fleet Monitor" in response.text
    assert client.get("/static/index.html").status_code == 200
