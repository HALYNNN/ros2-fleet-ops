from __future__ import annotations

import json

import pytest

from collector.analyzer import analyze_run, main, render_markdown, render_text
from collector.storage import Store


def seed_run(store: Store, run_id: str = "a1", *, rate_hz: float = 20.0, count: int = 40, start: float = 1000.0):
    store.start_run("tb3-01", source="sim", run_id=run_id, started_at=start)
    period = 1.0 / rate_hz
    for index in range(count):
        ts = start + index * period
        store.insert_sample(
            run_id=run_id,
            robot_id="tb3-01",
            topic="/odom",
            seq=index + 1,
            source_ts=ts,
            recv_ts=ts + 0.01,
            data={"x": 0.01 * index, "y": 0.0, "yaw": 0.0, "linear_x": 0.2},
        )
    store.finish_run(run_id, finished_at=start + count * period)


def test_healthy_run_has_no_findings(store):
    seed_run(store)
    report = analyze_run(store, "a1")
    assert report["verdict"] == "healthy"
    assert report["findings"] == []
    assert report["alarms"]["total"] == 0
    odom = next(topic for topic in report["topics"] if topic["topic"] == "/odom")
    assert odom["count"] == 40
    assert odom["measured_hz"] == pytest.approx(20.0, abs=0.5)
    assert odom["dropped"] == 0
    assert odom["p95_latency_ms"] == pytest.approx(10.0, abs=1.0)


def test_drop_and_latency_findings(store):
    seed_run(store)
    for index in range(3):
        store.insert_alarm(
            {
                "run_id": "a1",
                "robot_id": "tb3-01",
                "ts": 1001.0 + index,
                "kind": "topic_drop",
                "severity": "warning",
                "topic": "/odom",
                "message": "결번 2건",
                "value": 2.0,
                "threshold": 0.0,
            }
        )
    store.insert_alarm(
        {
            "run_id": "a1",
            "robot_id": "tb3-01",
            "ts": 1005.0,
            "kind": "latency_high",
            "severity": "critical",
            "topic": "/odom",
            "message": "지연 600ms",
            "value": 600.0,
            "threshold": 250.0,
        }
    )
    report = analyze_run(store, "a1")
    assert report["verdict"] == "critical"
    kinds = {finding["kind"] for finding in report["findings"]}
    assert kinds == {"topic_drop", "latency_high"}
    drop = next(finding for finding in report["findings"] if finding["kind"] == "topic_drop")
    assert drop["count"] == 3
    assert drop["severity"] == "warning"
    assert report["alarms"]["by_kind"]["topic_drop"] == 3
    assert len(report["timeline"]) == 4
    assert report["timeline"][0]["offset_s"] == pytest.approx(1.0)


def test_no_data_verdict(store):
    store.start_run("tb3-01", run_id="empty", started_at=100.0)
    store.finish_run("empty", finished_at=101.0)
    report = analyze_run(store, "empty")
    assert report["verdict"] == "no-data"
    assert report["topics"] == []


def test_unknown_run_raises(store):
    with pytest.raises(KeyError):
        analyze_run(store, "nope")


def test_render_text_and_markdown(store):
    seed_run(store)
    store.insert_alarm(
        {
            "run_id": "a1",
            "robot_id": "tb3-01",
            "ts": 1001.0,
            "kind": "topic_silent",
            "severity": "critical",
            "topic": "/scan",
            "message": "/scan 2.0s 동안 수신 없음",
            "value": 2.0,
            "threshold": 0.5,
        }
    )
    report = analyze_run(store, "a1")
    text = render_text(report)
    assert "ros2-fleet-ops 분석 리포트" in text
    assert "a1" in text
    assert "topic_silent" in text
    assert "조치:" in text
    markdown = render_markdown(report)
    assert markdown.startswith("# ros2-fleet-ops 분석 리포트")
    assert "| `" in markdown and "| 토픽 |" in markdown


def test_cli_json_and_fail_on(tmp_path, capsys):
    db = tmp_path / "a.db"
    store = Store(db, tmp_path / "jsonl")
    seed_run(store)
    store.insert_alarm(
        {
            "run_id": "a1",
            "robot_id": "tb3-01",
            "ts": 1002.0,
            "kind": "topic_drop",
            "severity": "warning",
            "topic": "/odom",
            "message": "결번",
            "value": 1.0,
            "threshold": 0.0,
        }
    )
    store.close()

    code = main(["--db", str(db), "--jsonl-dir", str(tmp_path / "jsonl"), "--run-id", "a1", "--json"])
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["verdict"] == "degraded"

    code = main(["--db", str(db), "--jsonl-dir", str(tmp_path / "jsonl"), "--latest", "--fail-on", "degraded"])
    assert code == 1


def test_cli_without_runs_returns_2(tmp_path):
    code = main(["--db", str(tmp_path / "empty.db"), "--jsonl-dir", str(tmp_path / "jsonl")])
    assert code == 2


def test_topic_stats_use_real_timestamps(store):
    store.start_run("tb3-01", run_id="t1", started_at=100.0)
    times = [100.0, 100.05, 100.2, 100.25]
    for index, ts in enumerate(times):
        store.insert_sample(
            run_id="t1", robot_id="tb3-01", topic="/odom", seq=index + 1,
            source_ts=ts, recv_ts=ts, data={"x": float(index)},
        )
    report = analyze_run(store, "t1")
    odom = report["topics"][0]
    assert odom["max_gap_ms"] == pytest.approx(150.0, abs=0.01)
    assert odom["duration_s"] == pytest.approx(0.25, abs=0.001)
