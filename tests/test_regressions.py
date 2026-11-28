"""Regression tests for the code-review fixes.

Each block names the review item it locks down, so a future change that
reintroduces the bug fails here with a message pointing at the cause.
"""
from __future__ import annotations

import sqlite3
import time

import pytest
from fastapi.testclient import TestClient

from collector.anomaly import AnomalyDetector
from collector.collector import CollectorCore, main
from collector.config import CollectorConfig, TopicRule
from collector.storage import Store
from tests.conftest import odom
from web_monitor.main import create_app, robot_status


def make_detector(**kwargs) -> AnomalyDetector:
    return AnomalyDetector(CollectorConfig(robot_id="tb3-test", **kwargs))


# --------------------------------------------------------------------------
# 1. A severity escalation must not be hidden by its own cooldown
# --------------------------------------------------------------------------


def test_latency_warning_then_critical_bypasses_cooldown():
    detector = make_detector()
    base = 100.0
    detector.check("/odom", seq=1, source_ts=base, recv_ts=base, data=odom())
    # 300 ms against a 250 ms budget -> warning
    warning = detector.check(
        "/odom", seq=2, source_ts=base + 0.05, recv_ts=base + 0.35, data=odom()
    )
    warning_alarms = [alarm for alarm in warning if alarm.kind == "latency_high"]
    assert warning_alarms and warning_alarms[0].severity == "warning"

    # 700 ms is critical, and it arrives well inside the 2 s cooldown.
    escalated = detector.check(
        "/odom", seq=3, source_ts=base + 0.10, recv_ts=base + 0.80, data=odom()
    )
    critical_alarms = [alarm for alarm in escalated if alarm.kind == "latency_high"]
    assert critical_alarms, "critical escalation was swallowed by the warning's cooldown"
    assert critical_alarms[0].severity == "critical"

    # Same severity again is still debounced.
    repeated = detector.check(
        "/odom", seq=4, source_ts=base + 0.15, recv_ts=base + 0.95, data=odom()
    )
    assert not [alarm for alarm in repeated if alarm.kind == "latency_high"]


def test_silence_warning_then_critical_bypasses_cooldown():
    detector = make_detector()
    base = 100.0
    for index in range(5):
        detector.check(
            "/odom", seq=index + 1, source_ts=base + index * 0.05,
            recv_ts=base + index * 0.05, data=odom(),
        )
    # /odom silence limit is max(3/20, 0.5) = 0.5 s; critical above 1.0 s.
    warning = detector.check_silence(base + 0.75)
    assert [(alarm.kind, alarm.severity) for alarm in warning] == [("topic_silent", "warning")]
    critical = detector.check_silence(base + 1.3)
    assert [(alarm.kind, alarm.severity) for alarm in critical] == [("topic_silent", "critical")]


def test_cooldown_expires_and_allows_a_new_alarm():
    detector = make_detector()
    base = 100.0
    detector.check("/odom", seq=1, source_ts=base, recv_ts=base, data=odom())
    first = detector.check("/odom", seq=2, source_ts=base + 0.05, recv_ts=base + 1.05, data=odom())
    assert any(alarm.kind == "latency_high" for alarm in first)
    # Past the cooldown a fresh alarm is allowed again.
    later = detector.check("/odom", seq=3, source_ts=base + 2.30, recv_ts=base + 3.30, data=odom())
    assert any(alarm.kind == "latency_high" for alarm in later)


# --------------------------------------------------------------------------
# 2. rate_low uses a bounded recent window, not the lifetime average
# --------------------------------------------------------------------------


def test_tracker_windows_are_bounded():
    detector = make_detector()
    ts = 100.0
    for index in range(250):
        ts = 100.0 + index * 0.05
        detector.check("/odom", seq=index + 1, source_ts=ts, recv_ts=ts, data=odom())
    tracker = detector.tracker("/odom")
    assert len(tracker.gaps) == detector.config.rate_window
    assert len(tracker.latencies) == detector.config.rate_window * 10


def test_rate_low_reflects_recent_slump_not_lifetime_average():
    detector = make_detector()
    alarms = []
    ts = 100.0
    # Long healthy stretch -> lifetime average stays near 20 Hz.
    for index in range(60):
        ts = 100.0 + index * 0.05
        alarms.extend(
            detector.check("/odom", seq=index + 1, source_ts=ts, recv_ts=ts, data=odom())
        )
    assert not [alarm for alarm in alarms if alarm.kind == "rate_low"]

    # Then the publisher drops to 5 Hz for a full window.
    fast_end = ts
    for index in range(1, 21):
        ts = fast_end + index * 0.2
        alarms.extend(
            detector.check("/odom", seq=60 + index, source_ts=ts, recv_ts=ts, data=odom())
        )
    rate_alarms = [alarm for alarm in alarms if alarm.kind == "rate_low"]
    assert rate_alarms, "recent 5 Hz window should raise rate_low"
    assert rate_alarms[0].value < 10.0
    assert detector.snapshot()["/odom"]["rate_hz"] == pytest.approx(5.0, abs=0.2)

    # The lifetime mean of the same stream is far above the warning threshold.
    lifetime_gap = (ts - 100.0) / 79
    assert 1.0 / lifetime_gap > 10.0


# --------------------------------------------------------------------------
# 3. estop_command needs a sustained zero command, not a normal brake
# --------------------------------------------------------------------------


def test_normal_brake_does_not_raise_estop():
    detector = make_detector()
    base = 100.0
    detector.check("/cmd_vel", seq=1, source_ts=base, recv_ts=base, data={"linear_x": 0.22, "angular_z": 0.0})
    detector.check("/cmd_vel", seq=2, source_ts=base + 0.1, recv_ts=base + 0.1, data={"linear_x": 0.15, "angular_z": 0.0})
    detector.check("/cmd_vel", seq=3, source_ts=base + 0.2, recv_ts=base + 0.2, data={"linear_x": 0.0, "angular_z": 0.0})
    alarms = []
    for offset, speed in ((0.25, 0.12), (0.4, 0.03), (0.55, 0.0)):
        alarms.extend(
            detector.check(
                "/odom", seq=10, source_ts=base + offset, recv_ts=base + offset,
                data=odom(linear_x=speed),
            )
        )
    assert [alarm for alarm in alarms if alarm.kind == "estop_command"] == []


def test_estop_timer_resets_when_command_resumes():
    detector = make_detector()
    base = 100.0
    detector.check("/cmd_vel", seq=1, source_ts=base, recv_ts=base, data={"linear_x": 0.2, "angular_z": 0.0})
    detector.check("/cmd_vel", seq=2, source_ts=base + 0.1, recv_ts=base + 0.1, data={"linear_x": 0.0, "angular_z": 0.0})
    assert detector.check(
        "/odom", seq=5, source_ts=base + 0.3, recv_ts=base + 0.3, data=odom(linear_x=0.18)
    ) == []
    detector.check("/cmd_vel", seq=3, source_ts=base + 0.4, recv_ts=base + 0.4, data={"linear_x": 0.2, "angular_z": 0.0})
    alarms = detector.check(
        "/odom", seq=9, source_ts=base + 0.9, recv_ts=base + 0.9, data=odom(linear_x=0.18)
    )
    assert [alarm for alarm in alarms if alarm.kind == "estop_command"] == []


def test_estop_debounce_is_configurable():
    detector = make_detector(estop_debounce_s=0.2)
    base = 100.0
    detector.check("/cmd_vel", seq=1, source_ts=base, recv_ts=base, data={"linear_x": 0.2, "angular_z": 0.0})
    detector.check("/cmd_vel", seq=2, source_ts=base + 0.05, recv_ts=base + 0.05, data={"linear_x": 0.0, "angular_z": 0.0})
    assert detector.check(
        "/odom", seq=5, source_ts=base + 0.1, recv_ts=base + 0.1, data=odom(linear_x=0.18)
    ) == []
    alarms = detector.check(
        "/odom", seq=9, source_ts=base + 0.3, recv_ts=base + 0.3, data=odom(linear_x=0.18)
    )
    assert [alarm.kind for alarm in alarms if alarm.kind == "estop_command"] == ["estop_command"]


# --------------------------------------------------------------------------
# 4. wall-clock timestamps alongside the node/sim clock
# --------------------------------------------------------------------------


def test_core_records_wall_clock_next_to_sim_clock(store, config):
    core = CollectorCore(
        store,
        config,
        source="ros2",
        run_id="r-wall",
        clock=lambda: 5.0,
        wall_clock=lambda: 1_700_000_000.0,
    )
    core.handle_sample("/odom", odom(), seq=1, source_ts=4.9, recv_ts=5.0)
    row = store.fetch_samples("r-wall")[0]
    assert row["recv_ts"] == 5.0
    assert row["wall_ts"] == 1_700_000_000.0


def test_core_alarm_rows_carry_wall_clock(store, config):
    core = CollectorCore(
        store,
        config,
        source="ros2",
        run_id="r-alarm-wall",
        clock=lambda: 5.0,
        wall_clock=lambda: 1_700_000_000.0,
    )
    core.handle_sample("/odom", odom(), seq=1, source_ts=5.0, recv_ts=5.0)
    core.handle_sample("/odom", odom(), seq=9, source_ts=5.05, recv_ts=5.05)
    alarm = store.fetch_alarms(run_id="r-alarm-wall")[0]
    assert alarm["kind"] == "topic_drop"
    assert alarm["wall_ts"] == 1_700_000_000.0


def test_robot_status_uses_wall_clock_under_sim_time(tmp_path):
    store = Store(tmp_path / "sim.db", tmp_path / "jsonl")
    now = time.time()
    try:
        run_id = store.start_run("tb3-sim", source="ros2", run_id="sim-run", started_at=now - 3.0)
        for index in range(30):
            ts = 0.1 + index * 0.05  # Gazebo /clock: starts near zero
            store.insert_sample(
                run_id=run_id,
                robot_id="tb3-sim",
                topic="/odom",
                seq=index + 1,
                source_ts=ts,
                recv_ts=ts,
                wall_ts=now - 2.0 + index * 0.05,
                data={"x": 0.01 * index, "y": 0.0},
            )
        store.insert_alarm(
            {
                "run_id": run_id,
                "robot_id": "tb3-sim",
                "ts": 0.5,  # sim clock
                "wall_ts": now - 1.0,  # one second ago in real life
                "kind": "topic_silent",
                "severity": "critical",
                "topic": "/scan",
                "message": "수신 없음",
            }
        )
        status = robot_status(store, "tb3-sim")
        assert status["health"] == "critical", "recent critical alarm must be seen through wall_ts"
        assert status["last_seen"] == pytest.approx(now - 2.0 + 29 * 0.05)
        topics = {topic["topic"]: topic for topic in status["topics"]}
        assert topics["/odom"]["last_recv_ts"] == pytest.approx(0.1 + 29 * 0.05)
    finally:
        store.close()


def test_robot_status_marks_stale_from_wall_clock(tmp_path):
    store = Store(tmp_path / "sim.db", tmp_path / "jsonl")
    now = time.time()
    try:
        run_id = store.start_run("tb3-sim", run_id="sim-run", started_at=now - 200)
        for index in range(5):
            store.insert_sample(
                run_id=run_id,
                robot_id="tb3-sim",
                topic="/odom",
                seq=index + 1,
                source_ts=now + index,  # looks fresh on the sim clock
                recv_ts=now + index,
                wall_ts=now - 120.0,  # but two minutes have passed in real life
                data={"x": 0.0, "y": 0.0},
            )
        assert robot_status(store, "tb3-sim")["health"] == "stale"
    finally:
        store.close()


LEGACY_SCHEMA = """
CREATE TABLE runs (
    run_id      TEXT PRIMARY KEY,
    robot_id    TEXT NOT NULL,
    source      TEXT NOT NULL,
    started_at  REAL NOT NULL,
    finished_at REAL,
    note        TEXT,
    jsonl_path  TEXT
);
CREATE TABLE samples (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id    TEXT NOT NULL,
    robot_id  TEXT NOT NULL,
    topic     TEXT NOT NULL,
    seq       INTEGER,
    source_ts REAL NOT NULL,
    recv_ts   REAL NOT NULL,
    data      TEXT NOT NULL
);
CREATE TABLE alarms (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id    TEXT NOT NULL,
    robot_id  TEXT NOT NULL,
    ts        REAL NOT NULL,
    kind      TEXT NOT NULL,
    severity  TEXT NOT NULL,
    topic     TEXT,
    message   TEXT NOT NULL,
    value     REAL,
    threshold REAL
);
"""


def test_existing_database_is_migrated_with_wall_ts(tmp_path):
    db = tmp_path / "legacy.db"
    conn = sqlite3.connect(db)
    conn.executescript(LEGACY_SCHEMA)
    conn.execute(
        "INSERT INTO runs (run_id, robot_id, source, started_at) VALUES ('legacy', 'tb3-01', 'sim', 100.0)"
    )
    conn.execute(
        "INSERT INTO samples (run_id, robot_id, topic, seq, source_ts, recv_ts, data)"
        " VALUES ('legacy', 'tb3-01', '/odom', 1, 5.0, 5.1, '{}')"
    )
    conn.execute(
        "INSERT INTO alarms (run_id, robot_id, ts, kind, severity, topic, message, value, threshold)"
        " VALUES ('legacy', 'tb3-01', 5.2, 'topic_drop', 'warning', '/odom', 'm', 1.0, 0.0)"
    )
    conn.commit()
    conn.close()

    store = Store(db, tmp_path / "jsonl")
    try:
        sample = store.fetch_samples("legacy")[0]
        assert sample["wall_ts"] == sample["recv_ts"] == 5.1  # fallback, not NULL
        alarm = store.fetch_alarms(run_id="legacy")[0]
        assert alarm["wall_ts"] == alarm["ts"] == 5.2

        store.insert_sample(
            run_id="legacy",
            robot_id="tb3-01",
            topic="/odom",
            seq=2,
            source_ts=5.2,
            recv_ts=5.2,
            data={},
            wall_ts=1234.5,
        )
        assert store.fetch_samples("legacy")[-1]["wall_ts"] == 1234.5
    finally:
        store.close()

    reopened = Store(db, tmp_path / "jsonl")
    try:
        columns = {row["name"] for row in reopened._conn.execute("PRAGMA table_info(samples)")}
        assert "wall_ts" in columns
    finally:
        reopened.close()


# --------------------------------------------------------------------------
# 5. recent_samples has an index for its run_id + id tail read
# --------------------------------------------------------------------------


def test_recent_samples_tail_read_uses_run_id_index(store):
    store.start_run("tb3-01", run_id="r1", started_at=100.0)
    for index in range(5):
        store.insert_sample(
            run_id="r1", robot_id="tb3-01", topic="/odom", seq=index + 1,
            source_ts=100.0 + index, recv_ts=100.0 + index, data={"x": float(index)},
        )
    indexes = {
        row["name"]
        for row in store._conn.execute(
            "SELECT name FROM sqlite_master WHERE type='index' AND tbl_name='samples'"
        )
    }
    assert "idx_samples_run_id" in indexes

    plan = store._conn.execute(
        "EXPLAIN QUERY PLAN SELECT * FROM samples WHERE run_id=? ORDER BY id DESC LIMIT ?",
        ("r1", 2),
    ).fetchall()
    details = " ".join(row["detail"] for row in plan)
    assert "idx_samples_run_id" in details, details


# --------------------------------------------------------------------------
# 6. /api/health uses a cheap COUNT(*) instead of list_runs(limit=1000)
# --------------------------------------------------------------------------


def test_health_does_not_list_runs(tmp_path, monkeypatch):
    db = tmp_path / "api.db"
    jsonl = tmp_path / "jsonl"
    store = Store(db, jsonl)
    store.start_run("tb3-01", source="sim", run_id="only-run", started_at=time.time())
    store.close()

    app = create_app(db, jsonl)
    client = TestClient(app)
    try:
        api_store = app.state.store
        assert api_store.count_runs() == 1

        def boom(*args, **kwargs):
            raise AssertionError("/api/health must not call list_runs()")

        monkeypatch.setattr(api_store, "list_runs", boom)
        body = client.get("/api/health").json()
        assert body["status"] == "ok"
        assert body["runs"] == 1
    finally:
        app.state.store.close()


# --------------------------------------------------------------------------
# 8. config validation rejects a non-positive expected_hz
# --------------------------------------------------------------------------


@pytest.mark.parametrize("bad", [0.0, -1.0, -0.001, float("nan")])
def test_topic_rule_rejects_non_positive_expected_hz(bad):
    with pytest.raises(ValueError):
        TopicRule("/odom", expected_hz=bad)


def test_config_from_yaml_rejects_zero_expected_hz(tmp_path):
    path = tmp_path / "zero.yaml"
    path.write_text("topics:\n  - topic: /odom\n    expected_hz: 0\n", encoding="utf-8")
    with pytest.raises(ValueError):
        CollectorConfig.from_yaml(path)


def test_cli_rejects_zero_expected_hz(tmp_path):
    path = tmp_path / "zero.yaml"
    path.write_text("topics:\n  - topic: /odom\n    expected_hz: 0\n", encoding="utf-8")
    code = main(
        [
            "--mode", "sim",
            "--db", str(tmp_path / "cli.db"),
            "--jsonl-dir", str(tmp_path / "jsonl"),
            "--config", str(path),
            "--duration", "0.2",
            "--quiet",
        ]
    )
    assert code == 2
