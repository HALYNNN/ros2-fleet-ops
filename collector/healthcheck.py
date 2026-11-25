"""Container healthcheck: collector heartbeat is fresh and the DB is readable."""
from __future__ import annotations

import argparse
import sqlite3
import sys
import time
from pathlib import Path


def check(db_path: str, max_age_s: float = 30.0) -> tuple[bool, str]:
    db = Path(db_path)
    if not db.exists():
        return False, f"database not found: {db}"
    heartbeat = db.parent / "collector.heartbeat"
    if not heartbeat.exists():
        return False, f"heartbeat not found: {heartbeat}"
    try:
        import json

        payload = json.loads(heartbeat.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return False, f"heartbeat unreadable: {exc}"
    age = time.time() - float(payload.get("ts", 0))
    if age > max_age_s:
        return False, f"heartbeat stale: {age:.1f}s > {max_age_s:.0f}s"
    try:
        connection = sqlite3.connect(f"file:{db}?mode=ro", uri=True, timeout=5.0)
        connection.execute("SELECT 1").fetchone()
        connection.close()
    except sqlite3.Error as exc:
        return False, f"database unreadable: {exc}"
    return True, f"ok samples={payload.get('samples', '?')} age={age:.1f}s"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="collector.healthcheck")
    parser.add_argument("--db", default="data/fleet.db")
    parser.add_argument("--max-age", type=float, default=30.0)
    args = parser.parse_args(argv)
    ok, message = check(args.db, args.max_age)
    print(message)
    return 0 if ok else 1


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
