"""Replay of a recorded run, at real time or accelerated.

Replay is what makes a field incident reproducible: the dashboard can re-draw a
run's trajectory exactly in recorded order, and the same frames can be fed to
another consumer (a simulator, a test assertion) without the robot.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Iterable, Sequence

from .storage import Store


def load_frames(store: Store, run_id: str, topics: Sequence[str] | None = None) -> list[dict[str, Any]]:
    """Load a run's samples as replay frames ordered by source timestamp."""
    rows = store.fetch_samples(run_id, topics=topics)
    return [
        {
            "index": index,
            "topic": row["topic"],
            "seq": row.get("seq"),
            "source_ts": row["source_ts"],
            "recv_ts": row["recv_ts"],
            "data": row["data"],
            "robot_id": row["robot_id"],
        }
        for index, row in enumerate(rows)
    ]


def iter_timed(
    frames: Iterable[dict[str, Any]],
    *,
    speed: float = 1.0,
    clock: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    stop_event: threading.Event | None = None,
) -> Iterable[dict[str, Any]]:
    """Yield frames with wall-clock pacing derived from their timestamps."""
    speed = max(0.01, float(speed))
    frames = list(frames)
    if not frames:
        return
    base = frames[0]["source_ts"]
    wall_start = clock()
    for frame in frames:
        if stop_event is not None and stop_event.is_set():
            return
        target = (frame["source_ts"] - base) / speed
        delay = target - (clock() - wall_start)
        if delay > 0:
            sleep(delay)
        yield frame


class ReplayEngine:
    """Background replay session with thread-safe status for the web API."""

    MAX_EMITTED = 50_000

    def __init__(
        self,
        frames: list[dict[str, Any]],
        *,
        run_id: str = "",
        speed: float = 1.0,
        max_gap_s: float = 2.0,
    ) -> None:
        self.frames = frames
        self.run_id = run_id
        self.speed = max(0.01, float(speed))
        self.max_gap_s = max_gap_s
        self.state = "idle" if frames else "empty"
        self.index = 0
        self.emitted: list[dict[str, Any]] = []
        self.started_at: float | None = None
        self.finished_at: float | None = None
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()

    # ------------------------------------------------------------------- run

    def start(self) -> bool:
        if not self.frames:
            self.state = "empty"
            return False
        if self._thread is not None and self._thread.is_alive():
            return False
        self._stop.clear()
        self.state = "running"
        self.started_at = time.time()
        self._thread = threading.Thread(target=self._play, name=f"replay-{self.run_id}", daemon=True)
        self._thread.start()
        return True

    def _play(self) -> None:
        try:
            base = self.frames[0]["source_ts"]
            wall_start = time.monotonic()
            for position, frame in enumerate(self.frames):
                target = (frame["source_ts"] - base) / self.speed
                delay = target - (time.monotonic() - wall_start)
                if delay > 0:
                    remaining = delay
                    while remaining > 0:
                        chunk = min(remaining, 0.1)
                        if self._stop.wait(chunk):
                            with self._lock:
                                self.state = "stopped"
                            return
                        remaining -= chunk
                elif delay < -self.max_gap_s:
                    # We fell far behind (paused host / huge gap): resync instead of bursting.
                    wall_start = time.monotonic() - target
                if self._stop.is_set():
                    with self._lock:
                        self.state = "stopped"
                    return
                frame = dict(frame, played_at=time.time())
                with self._lock:
                    if len(self.emitted) < self.MAX_EMITTED:
                        self.emitted.append(frame)
                    self.index = position + 1
            with self._lock:
                self.state = "finished"
                self.finished_at = time.time()
        except Exception as exc:  # pragma: no cover - defensive
            with self._lock:
                self.state = "error"
                self.error = str(exc)

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)
        if self.state == "running":
            with self._lock:
                self.state = "stopped"

    def wait(self, timeout: float | None = None) -> bool:
        thread = self._thread
        if thread is None:
            return True
        thread.join(timeout=timeout)
        return not thread.is_alive()

    # -------------------------------------------------------------- reporting

    def status(self, since_index: int = 0, max_frames: int = 2000) -> dict[str, Any]:
        with self._lock:
            frames = self.emitted[since_index : since_index + max_frames]
            total = len(self.frames)
            current = frames[-1] if frames else None
            return {
                "run_id": self.run_id,
                "state": self.state,
                "speed": self.speed,
                "index": self.index,
                "total": total,
                "progress": round(self.index / total, 4) if total else 0.0,
                "started_at": self.started_at,
                "finished_at": self.finished_at,
                "error": self.error,
                "frames": frames,
                "frames_offset": since_index,
                "current": (
                    {
                        "topic": current["topic"],
                        "source_ts": current["source_ts"],
                        "data": current["data"],
                    }
                    if current
                    else None
                ),
            }
