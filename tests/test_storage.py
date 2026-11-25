from __future__ import annotations

import json

import pytest

from collector.storage import Store


def test_start_run_persists_metadata(store):
    run_id = store.start_run("tb3-01", source="sim", note="unit", run_id="r1", started_at=100.0)
    run = store.get_run(run_id)
    assert run["robot_id"] == "tb3-01"
    assert run["source"] == "sim"
    assert run["started_at"] == 100.0
    assert run["finished_at"] is None
    assert run["note"] == "unit"
    assert store.list_runs()[0]["open"] is True


def test_sample_roundtrip_writes_sqlite_and_jsonl(store):
    store.start_run("tb3-01", run_id="r1", started_at=100.0)
    store.insert_sample(
        run_id="r1",
        robot_id="tb3-01",
        topic="/odom",
        seq=7,
        source_ts=100.5,
        recv_ts=100.52,
        data={"x": 1.0, "y": 2.0},
    )
    rows = store.fetch_samples("r1")
    assert len(rows) == 1
    assert rows[0]["topic"] == "/odom"
    assert rows[0]["seq"] == 7
    assert rows[0]["data"] == {"x": 1.0, "y": 2.0}
    assert store.sample_count("r1") == 1

    lines = store.jsonl_path("r1").read_text(encoding="utf-8").strip().splitlines()
    assert len(lines) == 1
    record = json.loads(lines[0])
    assert record["record_type"] == "sample"
    assert record["data"] == {"x": 1.0, "y": 2.0}


def test_finish_run_closes_window(store):
    store.start_run("tb3-01", run_id="r1", started_at=100.0)
    store.insert_sample(run_id="r1", robot_id="tb3-01", topic="/odom", seq=1, source_ts=100.0, recv_ts=100.0)
    store.finish_run("r1", finished_at=110.0)
    run = store.get_run("r1")
    assert run["finished_at"] == 110.0
    listing = store.list_runs()[0]
    assert listing["open"] is False
    assert listing["duration_s"] == pytest.approx(10.0)
    assert listing["sample_count"] == 1


def test_list_runs_orders_by_start_and_counts(store):
    store.start_run("tb3-01", run_id="old", started_at=100.0)
    store.start_run("tb3-01", run_id="new", started_at=200.0)
    store.insert_sample(run_id="new", robot_id="tb3-01", topic="/odom", seq=1, source_ts=200.0, recv_ts=200.0)
    store.insert_alarm(
        {"run_id": "new", "robot_id": "tb3-01", "ts": 200.1, "kind": "topic_drop", "severity": "warning", "message": "x"}
    )
    runs = store.list_runs()
    assert [run["run_id"] for run in runs] == ["new", "old"]
    assert runs[0]["sample_count"] == 1
    assert runs[0]["alarm_count"] == 1
    assert store.latest_run()["run_id"] == "new"


def test_alarms_filter_and_counts(store):
    store.start_run("tb3-01", run_id="r1", started_at=100.0)
    for index, severity in enumerate(("warning", "critical", "warning")):
        store.insert_alarm(
            {
                "run_id": "r1",
                "robot_id": "tb3-01",
                "ts": 100.0 + index,
                "kind": "topic_silent" if index % 2 == 0 else "latency_high",
                "severity": severity,
                "topic": "/scan",
                "message": f"m{index}",
                "value": float(index),
                "threshold": 0.5,
            }
        )
    assert len(store.fetch_alarms(run_id="r1")) == 3
    assert [item["severity"] for item in store.fetch_alarms(severity="critical")] == ["critical"]
    assert store.alarm_counts("r1") == {"topic_silent": 2, "latency_high": 1}
    assert store.fetch_alarms(run_id="r1")[0]["ts"] == 102.0  # newest first


def test_trajectory_uses_odom_only(store):
    store.start_run("tb3-01", run_id="r1", started_at=100.0)
    for index in range(3):
        store.insert_sample(
            run_id="r1",
            robot_id="tb3-01",
            topic="/odom",
            seq=index + 1,
            source_ts=100.0 + index,
            recv_ts=100.0 + index,
            data={"x": float(index), "y": float(index) * 2, "yaw": 0.0},
        )
    store.insert_sample(
        run_id="r1", robot_id="tb3-01", topic="/scan", seq=1, source_ts=100.0, recv_ts=100.0, data={"point_count": 10}
    )
    points = store.trajectory("r1")
    assert [point["x"] for point in points] == [0.0, 1.0, 2.0]
    assert points[2]["y"] == 4.0


def test_trajectory_downsamples(store):
    store.start_run("tb3-01", run_id="r1", started_at=100.0)
    for index in range(100):
        store.insert_sample(
            run_id="r1", robot_id="tb3-01", topic="/odom", seq=index + 1,
            source_ts=100.0 + index, recv_ts=100.0 + index, data={"x": 0.01 * index, "y": 0.0},
        )
    points = store.trajectory("r1", max_points=20)
    assert len(points) <= 20
    assert points[-1]["x"] == pytest.approx(0.99)


def test_latest_by_topic_and_recent_samples(store):
    store.start_run("tb3-01", run_id="r1", started_at=100.0)
    for index in range(5):
        store.insert_sample(
            run_id="r1", robot_id="tb3-01", topic="/odom", seq=index + 1,
            source_ts=100.0 + index, recv_ts=100.0 + index, data={"x": float(index)},
        )
    store.insert_sample(
        run_id="r1", robot_id="tb3-01", topic="/scan", seq=1, source_ts=100.0, recv_ts=100.0, data={}
    )
    latest = {row["topic"]: row for row in store.latest_by_topic("tb3-01")}
    assert set(latest) == {"/odom", "/scan"}
    assert latest["/odom"]["data"]["x"] == 4.0

    recent = store.recent_samples("r1", limit=2, topics=["/odom"])
    assert [row["data"]["x"] for row in recent] == [3.0, 4.0]


def test_heartbeat_roundtrip_and_staleness(store):
    assert store.read_heartbeat() is None
    store.write_heartbeat({"samples": 3})
    heartbeat = store.read_heartbeat(max_age_s=60)
    assert heartbeat["samples"] == 3
    assert store.read_heartbeat(max_age_s=-1) is None


def test_store_reopens_existing_database(tmp_path):
    first = Store(tmp_path / "fleet.db", tmp_path / "jsonl")
    first.start_run("tb3-01", run_id="r1", started_at=1.0)
    first.close()

    second = Store(tmp_path / "fleet.db", tmp_path / "jsonl")
    try:
        assert second.get_run("r1") is not None
    finally:
        second.close()
