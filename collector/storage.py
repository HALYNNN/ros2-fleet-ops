"""Telemetry persistence: SQLite for queries, JSONL for raw per-run archives.

SQLite is the operational store the web monitor and analyzer read. Every sample
and alarm is also appended to `jsonl_dir/<run_id>.jsonl` so a run can be handed
to another tool (or re-imported) without the database.

Both writers are guarded by one re-entrant lock and the SQLite connection is
opened with `check_same_thread=False`; the collector thread and the watchdog
thread may write concurrently.
"""
from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Iterable, Sequence

SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id      TEXT PRIMARY KEY,
    robot_id    TEXT NOT NULL,
    source      TEXT NOT NULL,
    started_at  REAL NOT NULL,
    finished_at REAL,
    note        TEXT,
    jsonl_path  TEXT
);

CREATE TABLE IF NOT EXISTS samples (
    id        INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id    TEXT NOT NULL,
    robot_id  TEXT NOT NULL,
    topic     TEXT NOT NULL,
    seq       INTEGER,
    source_ts REAL NOT NULL,
    recv_ts   REAL NOT NULL,
    data      TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_samples_run_topic_ts ON samples(run_id, topic, source_ts);
CREATE INDEX IF NOT EXISTS idx_samples_run_recv ON samples(run_id, recv_ts);
CREATE INDEX IF NOT EXISTS idx_samples_robot_topic ON samples(robot_id, topic, id);

CREATE TABLE IF NOT EXISTS alarms (
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
CREATE INDEX IF NOT EXISTS idx_alarms_run_ts ON alarms(run_id, ts);
CREATE INDEX IF NOT EXISTS idx_alarms_kind ON alarms(kind);
"""


def _new_run_id(robot_id: str, now: float | None = None) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%S", time.localtime(now if now is not None else time.time()))
    return f"{robot_id}-{stamp}-{uuid.uuid4().hex[:6]}"


class Store:
    """SQLite + JSONL sink for samples and alarms."""

    def __init__(
        self,
        db_path: str | Path = "data/fleet.db",
        jsonl_dir: str | Path = "data/jsonl",
        heartbeat_path: str | Path | None = None,
    ) -> None:
        self.db_path = Path(db_path)
        self.jsonl_dir = Path(jsonl_dir)
        self.heartbeat_path = (
            Path(heartbeat_path) if heartbeat_path is not None else self.db_path.parent / "collector.heartbeat"
        )
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.jsonl_dir.mkdir(parents=True, exist_ok=True)
        self.heartbeat_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._jsonl_lock = threading.Lock()
        self._handles: dict[str, Any] = {}
        self._conn = sqlite3.connect(str(self.db_path), check_same_thread=False, timeout=30.0)
        self._conn.row_factory = sqlite3.Row
        self._conn.execute("PRAGMA journal_mode=WAL")
        self._conn.execute("PRAGMA synchronous=NORMAL")
        self._conn.executescript(SCHEMA)
        self._conn.commit()

    # ------------------------------------------------------------------ runs

    def start_run(
        self,
        robot_id: str,
        source: str = "sim",
        note: str | None = None,
        run_id: str | None = None,
        started_at: float | None = None,
    ) -> str:
        run_id = run_id or _new_run_id(robot_id, started_at)
        started_at = time.time() if started_at is None else float(started_at)
        jsonl_path = self.jsonl_dir / f"{run_id}.jsonl"
        with self._lock:
            self._conn.execute(
                "INSERT INTO runs (run_id, robot_id, source, started_at, finished_at, note, jsonl_path)"
                " VALUES (?, ?, ?, ?, NULL, ?, ?)",
                (run_id, robot_id, source, started_at, note, str(jsonl_path)),
            )
            self._conn.commit()
        return run_id

    def finish_run(self, run_id: str, finished_at: float | None = None) -> None:
        with self._lock:
            self._conn.execute(
                "UPDATE runs SET finished_at=? WHERE run_id=?",
                (time.time() if finished_at is None else float(finished_at), run_id),
            )
            self._conn.commit()
        self._close_jsonl(run_id)

    def get_run(self, run_id: str) -> dict[str, Any] | None:
        with self._lock:
            row = self._conn.execute("SELECT * FROM runs WHERE run_id=?", (run_id,)).fetchone()
        return dict(row) if row else None

    def list_runs(self, limit: int = 50, robot_id: str | None = None) -> list[dict[str, Any]]:
        sql = (
            "SELECT r.*,"
            " (SELECT COUNT(*) FROM samples s WHERE s.run_id=r.run_id) AS sample_count,"
            " (SELECT COUNT(*) FROM alarms a WHERE a.run_id=r.run_id) AS alarm_count,"
            " (SELECT MIN(source_ts) FROM samples s WHERE s.run_id=r.run_id) AS data_start,"
            " (SELECT MAX(source_ts) FROM samples s WHERE s.run_id=r.run_id) AS data_end"
            " FROM runs r"
        )
        params: list[Any] = []
        if robot_id:
            sql += " WHERE r.robot_id=?"
            params.append(robot_id)
        sql += " ORDER BY r.started_at DESC LIMIT ?"
        params.append(int(limit))
        with self._lock:
            rows = self._conn.execute(sql, params).fetchall()
        out = []
        for row in rows:
            item = dict(row)
            wall_end = item["finished_at"] if item["finished_at"] is not None else item["started_at"]
            item["wall_duration_s"] = round(max(0.0, wall_end - item["started_at"]), 3)
            if (
                item.get("data_start") is not None
                and item.get("data_end") is not None
                and item["data_end"] > item["data_start"]
            ):
                # Telemetry time is the honest window: in simulator mode the clock
                # can run faster than wall time, and a paused collector still has
                # a real data span.
                item["duration_s"] = round(max(0.0, item["data_end"] - item["data_start"]), 3)
            else:
                item["duration_s"] = item["wall_duration_s"]
            item.pop("data_start", None)
            item.pop("data_end", None)
            item["open"] = item["finished_at"] is None
            out.append(item)
        return out

    def latest_run(self, robot_id: str | None = None) -> dict[str, Any] | None:
        runs = self.list_runs(limit=1, robot_id=robot_id)
        return runs[0] if runs else None

    # --------------------------------------------------------------- samples

    def insert_sample(
        self,
        run_id: str,
        robot_id: str,
        topic: str,
        seq: int | None,
        source_ts: float,
        recv_ts: float,
        data: dict[str, Any] | None = None,
    ) -> None:
        payload = json.dumps(data or {}, separators=(",", ":"), sort_keys=True)
        record = {
            "record_type": "sample",
            "run_id": run_id,
            "robot_id": robot_id,
            "topic": topic,
            "seq": seq,
            "source_ts": round(float(source_ts), 6),
            "recv_ts": round(float(recv_ts), 6),
            "data": data or {},
        }
        with self._lock:
            self._conn.execute(
                "INSERT INTO samples (run_id, robot_id, topic, seq, source_ts, recv_ts, data)"
                " VALUES (?, ?, ?, ?, ?, ?, ?)",
                (run_id, robot_id, topic, seq, float(source_ts), float(recv_ts), payload),
            )
            self._conn.commit()
        self._append_jsonl(run_id, record)

    def insert_samples(self, records: Iterable[dict[str, Any]]) -> int:
        """Bulk insert records shaped like `insert_sample` arguments."""
        rows = []
        for record in records:
            rows.append(
                (
                    record["run_id"],
                    record["robot_id"],
                    record["topic"],
                    record.get("seq"),
                    float(record["source_ts"]),
                    float(record["recv_ts"]),
                    json.dumps(record.get("data") or {}, separators=(",", ":"), sort_keys=True),
                )
            )
        if not rows:
            return 0
        with self._lock:
            self._conn.executemany(
                "INSERT INTO samples (run_id, robot_id, topic, seq, source_ts, recv_ts, data)"
                " VALUES (?, ?, ?, ?, ?, ?, ?)",
                rows,
            )
            self._conn.commit()
        return len(rows)

    def fetch_samples(
        self,
        run_id: str,
        topics: Sequence[str] | None = None,
        limit: int | None = None,
        after_ts: float | None = None,
    ) -> list[dict[str, Any]]:
        sql = "SELECT * FROM samples WHERE run_id=?"
        params: list[Any] = [run_id]
        if topics:
            sql += f" AND topic IN ({','.join('?' * len(topics))})"
            params.extend(topics)
        if after_ts is not None:
            sql += " AND source_ts > ?"
            params.append(float(after_ts))
        sql += " ORDER BY source_ts ASC, id ASC"
        if limit:
            sql += " LIMIT ?"
            params.append(int(limit))
        with self._lock:
            rows = self._conn.execute(sql, params).fetchall()
        return [_sample_row(row) for row in rows]

    def recent_samples(
        self, run_id: str, limit: int = 300, topics: Sequence[str] | None = None
    ) -> list[dict[str, Any]]:
        """The most recent `limit` samples of a run, returned oldest-first."""
        inner = "SELECT * FROM samples WHERE run_id=?"
        params: list[Any] = [run_id]
        if topics:
            inner += f" AND topic IN ({','.join('?' * len(topics))})"
            params.extend(topics)
        inner += " ORDER BY id DESC LIMIT ?"
        params.append(int(limit))
        with self._lock:
            rows = self._conn.execute(
                f"SELECT * FROM ({inner}) ORDER BY id ASC", params
            ).fetchall()
        return [_sample_row(row) for row in rows]

    def sample_count(self, run_id: str) -> int:
        with self._lock:
            row = self._conn.execute(
                "SELECT COUNT(*) AS n FROM samples WHERE run_id=?", (run_id,)
            ).fetchone()
        return int(row["n"])

    def latest_by_topic(self, robot_id: str | None = None) -> list[dict[str, Any]]:
        sql = (
            "SELECT s.* FROM samples s JOIN ("
            "  SELECT robot_id, topic, MAX(id) AS mid FROM samples"
            + (" WHERE robot_id=?" if robot_id else "")
            + " GROUP BY robot_id, topic"
            ") m ON s.id = m.mid ORDER BY s.topic"
        )
        params: list[Any] = [robot_id] if robot_id else []
        with self._lock:
            rows = self._conn.execute(sql, params).fetchall()
        return [_sample_row(row) for row in rows]

    def trajectory(self, run_id: str, max_points: int = 2000) -> list[dict[str, Any]]:
        rows = self.fetch_samples(run_id, topics=["/odom"])
        points = [
            {"x": r["data"].get("x"), "y": r["data"].get("y"), "yaw": r["data"].get("yaw"), "ts": r["source_ts"]}
            for r in rows
            if r["data"].get("x") is not None and r["data"].get("y") is not None
        ]
        if len(points) > max_points > 0:
            stride = max(1, len(points) // max_points)
            last = points[-1]
            points = points[::stride][:max_points]
            if points[-1] != last:
                points = (points[: max_points - 1] + [last]) if len(points) >= max_points else points + [last]
        return points

    # ---------------------------------------------------------------- alarms

    def insert_alarm(self, alarm: dict[str, Any]) -> None:
        row = (
            alarm["run_id"],
            alarm.get("robot_id", ""),
            float(alarm["ts"]),
            alarm["kind"],
            alarm.get("severity", "warning"),
            alarm.get("topic"),
            alarm.get("message", ""),
            alarm.get("value"),
            alarm.get("threshold"),
        )
        with self._lock:
            self._conn.execute(
                "INSERT INTO alarms (run_id, robot_id, ts, kind, severity, topic, message, value, threshold)"
                " VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
                row,
            )
            self._conn.commit()
        self._append_jsonl(
            alarm["run_id"],
            {
                "record_type": "alarm",
                "ts": round(float(alarm["ts"]), 6),
                "robot_id": alarm.get("robot_id", ""),
                "kind": alarm["kind"],
                "severity": alarm.get("severity", "warning"),
                "topic": alarm.get("topic"),
                "message": alarm.get("message", ""),
                "value": alarm.get("value"),
                "threshold": alarm.get("threshold"),
            },
        )

    def fetch_alarms(
        self,
        run_id: str | None = None,
        severity: str | None = None,
        limit: int = 200,
        robot_id: str | None = None,
    ) -> list[dict[str, Any]]:
        sql = "SELECT * FROM alarms WHERE 1=1"
        params: list[Any] = []
        if run_id:
            sql += " AND run_id=?"
            params.append(run_id)
        if severity:
            sql += " AND severity=?"
            params.append(severity)
        if robot_id:
            sql += " AND robot_id=?"
            params.append(robot_id)
        sql += " ORDER BY ts DESC, id DESC LIMIT ?"
        params.append(int(limit))
        with self._lock:
            rows = self._conn.execute(sql, params).fetchall()
        return [dict(row) for row in rows]

    def alarm_counts(self, run_id: str) -> dict[str, int]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT kind, COUNT(*) AS n FROM alarms WHERE run_id=? GROUP BY kind", (run_id,)
            ).fetchall()
        return {row["kind"]: int(row["n"]) for row in rows}

    # ---------------------------------------------------------------- health

    def write_heartbeat(self, extra: dict[str, Any] | None = None) -> None:
        payload = {"ts": time.time(), "pid": os.getpid()}
        payload.update(extra or {})
        tmp = self.heartbeat_path.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload), encoding="utf-8")
        os.replace(tmp, self.heartbeat_path)

    def read_heartbeat(self, max_age_s: float | None = None) -> dict[str, Any] | None:
        if not self.heartbeat_path.exists():
            return None
        try:
            payload = json.loads(self.heartbeat_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        if max_age_s is not None and time.time() - float(payload.get("ts", 0)) > max_age_s:
            return None
        return payload

    # ------------------------------------------------------------------- io

    def _append_jsonl(self, run_id: str, record: dict[str, Any]) -> None:
        with self._jsonl_lock:
            handle = self._handles.get(run_id)
            if handle is None:
                path = self.jsonl_dir / f"{run_id}.jsonl"
                handle = open(path, "a", encoding="utf-8")
                self._handles[run_id] = handle
            handle.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            handle.flush()

    def _close_jsonl(self, run_id: str) -> None:
        with self._jsonl_lock:
            handle = self._handles.pop(run_id, None)
            if handle is not None:
                handle.close()

    def jsonl_path(self, run_id: str) -> Path:
        return self.jsonl_dir / f"{run_id}.jsonl"

    def count_jsonl_records(self, run_id: str) -> int:
        path = self.jsonl_path(run_id)
        if not path.exists():
            return 0
        with path.open("r", encoding="utf-8") as handle:
            return sum(1 for line in handle if line.strip())

    def close(self) -> None:
        with self._jsonl_lock:
            for handle in self._handles.values():
                handle.close()
            self._handles.clear()
        with self._lock:
            self._conn.commit()
            self._conn.close()

    def __enter__(self) -> "Store":
        return self

    def __exit__(self, *exc_info: Any) -> None:
        self.close()


def _sample_row(row: sqlite3.Row) -> dict[str, Any]:
    item = dict(row)
    try:
        item["data"] = json.loads(item["data"])
    except (TypeError, json.JSONDecodeError):
        item["data"] = {}
    return item
