"""Shared fixtures for the ros2-fleet-ops test suite."""
from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# Keep the default web app from writing into the repository's data/ during tests.
_TMP_DEFAULT = Path(tempfile.mkdtemp(prefix="fleet-import-"))
os.environ.setdefault("FLEET_DB", str(_TMP_DEFAULT / "import.db"))
os.environ.setdefault("FLEET_JSONL_DIR", str(_TMP_DEFAULT / "jsonl"))

from collector.collector import CollectorCore  # noqa: E402
from collector.config import CollectorConfig  # noqa: E402
from collector.storage import Store  # noqa: E402


@pytest.fixture()
def store(tmp_path):
    store = Store(tmp_path / "fleet.db", tmp_path / "jsonl")
    yield store
    store.close()


@pytest.fixture()
def config():
    return CollectorConfig(robot_id="tb3-test")


@pytest.fixture()
def core(store, config):
    core = CollectorCore(store, config, source="sim", run_id="run-test", clock=lambda: 1000.0)
    yield core
    try:
        store._close_jsonl(core.run_id)
    except Exception:
        pass


class SteadyClock:
    """Controllable clock for deterministic detector tests."""

    def __init__(self, start: float = 1000.0) -> None:
        self.value = start

    def __call__(self) -> float:
        return self.value

    def advance(self, seconds: float) -> float:
        self.value += seconds
        return self.value


@pytest.fixture()
def steady_clock():
    return SteadyClock()


def odom(x: float = 0.0, y: float = 0.0, yaw: float = 0.0, linear_x: float = 0.2) -> dict:
    return {"x": x, "y": y, "yaw": yaw, "linear_x": linear_x, "angular_z": 0.0}


def feed_stream(
    core: CollectorCore,
    *,
    topic: str = "/odom",
    count: int = 40,
    rate_hz: float = 20.0,
    start_ts: float = 1000.0,
    make_data=None,
    seq_start: int = 1,
    skip: set[int] | None = None,
) -> float:
    """Feed `count` evenly spaced samples; returns the last source timestamp.

    `skip` contains zero-based indices that are *not delivered* (sequence numbers
    still advance, producing the drop the detector should notice).
    """
    skip = skip or set()
    period = 1.0 / rate_hz
    ts = start_ts
    seq = seq_start - 1
    for index in range(count):
        seq += 1
        ts = start_ts + index * period
        if index in skip:
            continue
        data = make_data(index, seq) if make_data else odom(x=0.1 * index, y=0.0, linear_x=0.2)
        core.handle_sample(topic, data, seq=seq, source_ts=ts, recv_ts=ts)
    return ts
