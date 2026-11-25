from __future__ import annotations

import threading
import time

import pytest

from collector.replay import ReplayEngine, iter_timed, load_frames
from collector.storage import Store


def seed(store: Store, run_id: str = "rep", count: int = 30, period: float = 0.05, start: float = 500.0):
    store.start_run("tb3-01", run_id=run_id, started_at=start)
    for index in range(count):
        ts = start + index * period
        store.insert_sample(
            run_id=run_id,
            robot_id="tb3-01",
            topic="/odom",
            seq=index + 1,
            source_ts=ts,
            recv_ts=ts,
            data={"x": 0.02 * index, "y": 0.0, "yaw": 0.0},
        )
    store.finish_run(run_id, finished_at=start + count * period)


def test_load_frames_is_ordered(store):
    seed(store)
    frames = load_frames(store, "rep")
    assert len(frames) == 30
    assert [frame["index"] for frame in frames] == list(range(30))
    assert frames[0]["source_ts"] == 500.0
    assert frames[-1]["data"]["x"] == pytest.approx(0.02 * 29)
    assert all(frame["topic"] == "/odom" for frame in frames)


def test_load_frames_filters_topics(store):
    seed(store)
    store.insert_sample(
        run_id="rep", robot_id="tb3-01", topic="/scan", seq=1, source_ts=500.0, recv_ts=500.0, data={}
    )
    assert len(load_frames(store, "rep", topics=["/scan"])) == 1


def test_iter_timed_yields_all_frames_and_paces():
    frames = [{"source_ts": 10.0 + index * 0.02} for index in range(6)]
    started = time.monotonic()
    received = list(iter_timed(frames, speed=10.0))
    elapsed = time.monotonic() - started
    assert len(received) == 6
    assert elapsed == pytest.approx(0.01, abs=0.05)


def test_iter_timed_stops_on_event():
    frames = [{"source_ts": 10.0 + index * 0.1} for index in range(100)]
    stop = threading.Event()
    stop.set()
    assert list(iter_timed(frames, speed=1.0, stop_event=stop)) == []


def test_engine_plays_to_completion(store):
    seed(store, count=10, period=0.01)
    engine = ReplayEngine(load_frames(store, "rep"), run_id="rep", speed=4.0)
    assert engine.start() is True
    assert engine.wait(timeout=5.0) is True
    assert engine.state == "finished"
    assert engine.index == 10
    assert len(engine.emitted) == 10
    assert engine.emitted[0]["played_at"] <= engine.emitted[-1]["played_at"]


def test_engine_stop_early(store):
    seed(store, count=400, period=0.02)  # 8 s of recording
    engine = ReplayEngine(load_frames(store, "rep"), run_id="rep", speed=1.0)
    engine.start()
    time.sleep(0.15)
    engine.stop()
    assert engine.state == "stopped"
    assert 0 < engine.index < 400
    assert engine.status()["progress"] < 1.0


def test_engine_empty_frames():
    engine = ReplayEngine([], run_id="empty")
    assert engine.start() is False
    assert engine.state == "empty"
    assert engine.status()["total"] == 0


def test_status_window_and_current_frame(store):
    seed(store, count=12, period=0.01)
    engine = ReplayEngine(load_frames(store, "rep"), run_id="rep", speed=8.0)
    engine.start()
    engine.wait(timeout=5.0)
    status = engine.status(since_index=5, max_frames=3)
    assert status["frames_offset"] == 5
    assert len(status["frames"]) == 3
    assert status["current"]["topic"] == "/odom"
    assert status["progress"] == 1.0
    assert status["speed"] == 8.0
