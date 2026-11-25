from __future__ import annotations

import math
import types

import pytest

from collector.collector import (
    CollectorCore,
    main,
    odom_to_data,
    resolve_mode,
    scan_to_data,
    stamp_to_sec,
    twist_to_data,
)
from collector.storage import Store


def test_core_persists_samples_and_alarms(store, config):
    core = CollectorCore(store, config, source="sim", run_id="r-core", clock=lambda: 500.0)
    core.handle_sample("/odom", {"x": 0.1, "y": 0.2}, seq=1, source_ts=500.0, recv_ts=500.0)
    core.handle_sample("/odom", {"x": 0.2, "y": 0.2}, seq=5, source_ts=500.05, recv_ts=500.05)
    assert store.sample_count("r-core") == 2
    alarms = store.fetch_alarms(run_id="r-core")
    assert [alarm["kind"] for alarm in alarms] == ["topic_drop"]
    assert core.summary()["alarms"] == {"topic_drop": 1}


def test_core_tick_records_silence(store, config):
    core = CollectorCore(store, config, source="sim", run_id="r-tick", clock=lambda: 100.0)
    core.handle_sample("/scan", {"point_count": 36}, seq=1, source_ts=100.0, recv_ts=100.0)
    assert core.tick(now=100.2) == []
    fired = core.tick(now=101.2)
    assert [alarm.kind for alarm in fired] == ["topic_silent"]
    stored = store.fetch_alarms(run_id="r-tick", severity="critical")
    assert len(stored) == 1
    assert stored[0]["topic"] == "/scan"


def test_core_finish_closes_run_and_summarises(store, config):
    core = CollectorCore(store, config, source="sim", run_id="r-fin", clock=lambda: 700.0)
    core.handle_sample("/odom", {"x": 0.0}, seq=1, source_ts=700.0, recv_ts=700.0)
    summary = core.finish()
    assert summary["samples"] == 1
    assert summary["topics"] == {"/odom": 1}
    assert store.get_run("r-fin")["finished_at"] == 700.0


def test_heartbeat_written_by_core(store, config):
    now = [10.0]
    core = CollectorCore(store, config, source="sim", run_id="r-hb", clock=lambda: now[0])
    now[0] = 11.5
    core.handle_sample("/odom", {"x": 0.0}, seq=1, source_ts=11.5, recv_ts=11.5)
    heartbeat = store.read_heartbeat(max_age_s=None)
    assert heartbeat["run_id"] == "r-hb"
    assert heartbeat["samples"] == 1


def test_resolve_mode_explicit_and_auto():
    assert resolve_mode("sim") == "sim"
    assert resolve_mode("ros2") == "ros2"
    assert resolve_mode("auto") in ("sim", "ros2")


def test_stamp_to_sec_handles_ros2_and_ros1_shapes():
    ros2 = types.SimpleNamespace(sec=12, nanosec=500_000_000)
    ros1 = types.SimpleNamespace(secs=12, nsecs=500_000_000)
    assert stamp_to_sec(ros2) == pytest.approx(12.5)
    assert stamp_to_sec(ros1) == pytest.approx(12.5)
    assert stamp_to_sec(types.SimpleNamespace(sec=0, nanosec=0)) is None
    assert stamp_to_sec(None) is None


def _odom_message(x=1.0, y=2.0, yaw=math.pi / 2, linear=0.22):
    orientation = types.SimpleNamespace(
        x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)
    )
    return types.SimpleNamespace(
        pose=types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=x, y=y, z=0.0), orientation=orientation
            )
        ),
        twist=types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=linear, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.3),
            )
        ),
    )


def test_odom_to_data_converts_pose_and_yaw():
    data = odom_to_data(_odom_message(), seq=4)
    assert data["x"] == 1.0
    assert data["y"] == 2.0
    assert data["yaw"] == pytest.approx(math.pi / 2, abs=1e-3)
    assert data["linear_x"] == 0.22
    assert data["angular_z"] == 0.3
    assert data["seq"] == 4


def test_scan_to_data_filters_invalid_ranges_and_finds_front():
    message = types.SimpleNamespace(
        ranges=[float("inf"), 1.2, 0.4, float("nan"), 2.0],
        angle_min=-1.57,
        angle_increment=0.785,
    )
    data = scan_to_data(message)
    assert data["point_count"] == 5
    assert data["min_range"] == 0.4
    # middle index ~0 rad is in the front sector; inf/nan are skipped
    assert data["front_min_range"] == 0.4


def test_twist_to_data_rounds_values():
    message = types.SimpleNamespace(
        linear=types.SimpleNamespace(x=0.123456, y=0.0, z=0.0),
        angular=types.SimpleNamespace(x=0.0, y=0.0, z=-0.111),
    )
    data = twist_to_data(message)
    assert data == {"linear_x": 0.1235, "angular_z": -0.111}


def test_cli_sim_mode_creates_run(tmp_path, capsys):
    db = tmp_path / "cli.db"
    code = main(
        [
            "--mode",
            "sim",
            "--db",
            str(db),
            "--jsonl-dir",
            str(tmp_path / "jsonl"),
            "--duration",
            "1",
            "--time-scale",
            "20",
            "--robot-id",
            "tb3-cli",
        ]
    )
    assert code == 0
    store = Store(db, tmp_path / "jsonl")
    try:
        runs = store.list_runs()
        assert runs and runs[0]["robot_id"] == "tb3-cli"
        assert runs[0]["sample_count"] > 10
        assert runs[0]["open"] is False
    finally:
        store.close()


def test_cli_rejects_bad_config(tmp_path, capsys):
    config_path = tmp_path / "bad.yaml"
    config_path.write_text("topics:\n  - topic: /odom\n", encoding="utf-8")
    code = main(["--mode", "sim", "--config", str(config_path), "--duration", "0.2"])
    assert code == 2
