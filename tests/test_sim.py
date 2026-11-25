from __future__ import annotations

import pytest

from collector.collector import CollectorCore
from collector.sim import FaultPlan, SimClock, SimRobot, run_simulation, scan_payload


def run(store, config, *, duration=2.0, time_scale=50.0, faults=None, seed=7):
    core = CollectorCore(store, config, source="sim", run_id="sim-run", clock=lambda: 0.0)
    result = run_simulation(
        core, duration_s=duration, time_scale=time_scale, faults=faults, seed=seed
    )
    summary = core.finish()
    return result, summary, core


def alarm_kinds(store, run_id="sim-run"):
    return {alarm["kind"] for alarm in store.fetch_alarms(run_id=run_id)}


def test_simulation_publishes_all_topics(store, config):
    result, summary, _ = run(store, config, duration=2.0)
    assert result["duration_s"] == pytest.approx(2.0, abs=0.15)
    assert summary["topics"]["/odom"] >= 30
    assert summary["topics"]["/scan"] >= 15
    assert summary["topics"]["/cmd_vel"] >= 15
    assert result["final_pose"]["x"] != pytest.approx(0.6, abs=0.01)


def test_simulation_records_odom_trajectory_and_scan(store, config):
    run(store, config, duration=3.0)
    points = store.trajectory("sim-run")
    assert len(points) > 20
    assert all(point["x"] is not None for point in points)
    scan = [row for row in store.fetch_samples("sim-run", topics=["/scan"])][0]
    assert scan["data"]["point_count"] == 36
    assert 0.1 < scan["data"]["min_range"] <= 3.5


def test_drop_fault_produces_sequence_gaps(store, config):
    plan = FaultPlan(drop_every=3, drop_topic="/scan")
    result, summary, _ = run(store, config, duration=4.0, faults=plan)
    assert result["dropped"]["/scan"] > 0
    assert "topic_drop" in alarm_kinds(store)
    assert summary["alarms"]["topic_drop"] >= 1


def test_latency_fault_is_detected(store, config):
    plan = FaultPlan(latency_ms=600, latency_topic="/odom")
    _, _, _ = run(store, config, duration=3.0, faults=plan)
    alarms = store.fetch_alarms(run_id="sim-run", severity="critical")
    assert any(alarm["kind"] == "latency_high" for alarm in alarms)


def test_silence_fault_is_detected_and_recovers(store, config):
    plan = FaultPlan(silence_topic="/scan", silence_at_s=1.0, silence_for_s=2.0)
    run(store, config, duration=3.5, faults=plan)
    kinds = alarm_kinds(store)
    assert "topic_silent" in kinds
    assert "topic_recovered" in kinds


def test_estop_fault_is_detected(store, config):
    plan = FaultPlan(estop_at_s=1.5, estop_for_s=2.0)
    result, _, _ = run(store, config, duration=4.0, faults=plan)
    assert "estop_command" in alarm_kinds(store)


def test_simulation_is_quiet_without_faults(store, config):
    run(store, config, duration=2.0)
    assert alarm_kinds(store) == set()


def test_fault_plan_validation_and_descriptions():
    plan = FaultPlan.from_names(["latency", "drop", "silence", "estop"])
    text = " / ".join(plan.describe())
    assert "latency" in text and "drop" in text and "silence" in text and "emergency stop" in text
    with pytest.raises(ValueError):
        FaultPlan.from_names(["bogus"])
    with pytest.raises(ValueError):
        FaultPlan.from_names(["latency"], bogus_option=1)


def test_scan_payload_respects_room_bounds():
    robot = SimRobot(x=0.06, y=0.05, yaw=0.0)
    payload = scan_payload(robot)
    # A robot hugging the corner must see a very close wall but never a negative range.
    assert payload["min_range"] >= 0.12
    assert len(payload["ranges_mm"]) == payload["point_count"]


def test_clock_time_scale_runs_faster_than_wall_clock():
    clock = SimClock(time_scale=10.0)
    start = clock.now()
    clock.sleep(0.5)  # 0.5 simulated seconds -> 0.05 real seconds
    assert clock.now() - start == pytest.approx(0.5, abs=0.05)
