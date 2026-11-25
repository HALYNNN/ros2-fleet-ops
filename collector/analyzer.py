"""Post-run analysis: turn stored samples + alarms into an operator report.

The analyzer never invents numbers. Every value in the report is computed from
the run's stored samples (`source_ts`, `recv_ts`, `seq`) or counted from the
alarm table, and the report states the window it looked at.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

from .storage import Store

SEVERITY_ORDER = {"info": 1, "warning": 2, "critical": 3}

#: Alarm kind -> (cause title, operator action). Ordered by how specific it is.
CAUSE_LIBRARY: dict[str, tuple[str, str]] = {
    "estop_command": (
        "비상정지/정지 명령으로 로봇 제어가 끊김",
        "cmd_vel 발행자(관제·자율주행 노드)와 비상정지 입력을 확인하고, 정지 해제 후 재개 절차를 따른다.",
    ),
    "topic_silent": (
        "해당 토픽의 발행 노드 정지 또는 통신 단절",
        "노드 프로세스 상태(systemd/ros2 node list)와 네트워크 링크를 확인하고, 재시작 후 동일 구간을 Replay로 재검증한다.",
    ),
    "topic_drop": (
        "메시지 드롭(발행 측 큐 또는 수신 버퍼 포화)",
        "QoS depth·DDS 버퍼와 수신 측 처리 시간을 확인한다. 결번이 몰리는 시각의 CPU 사용률을 함께 본다.",
    ),
    "gap_spike": (
        "수신 간격 급증(순간 지연/스케줄링 밀림)",
        "해당 시각의 프로세스 부하와 무선 구간 품질을 확인한다. 단발이면 경고, 반복이면 노드 우선순위를 조정한다.",
    ),
    "latency_high": (
        "메시지 지연 예산 초과(처리 또는 전송 지연)",
        "수신 타임스탬프와 발행 타임스탬프 차이를 확인하고, 브리지·릴레이·네트워크 구간을 분리 측정한다.",
    ),
    "rate_low": (
        "발행 주기 저하(노드 부하 또는 설정 변경)",
        "노드 파라미터의 발행 주기와 CPU/메모리 사용률을 대조한다.",
    ),
    "topic_recovered": (
        "토픽 수신 재개",
        "재개 시점 이후 샘플로 정상 주기·지연이 회복됐는지 확인한다.",
    ),
}


@dataclass
class TopicStats:
    topic: str
    count: int = 0
    first_ts: float | None = None
    last_ts: float | None = None
    measured_hz: float | None = None
    max_gap_ms: float | None = None
    p95_latency_ms: float | None = None
    max_latency_ms: float | None = None
    dropped: int = 0
    alarms: dict[str, int] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "topic": self.topic,
            "count": self.count,
            "first_ts": self.first_ts,
            "last_ts": self.last_ts,
            "duration_s": (
                None if self.first_ts is None or self.last_ts is None else round(self.last_ts - self.first_ts, 3)
            ),
            "measured_hz": None if self.measured_hz is None else round(self.measured_hz, 2),
            "max_gap_ms": None if self.max_gap_ms is None else round(self.max_gap_ms, 1),
            "p95_latency_ms": None if self.p95_latency_ms is None else round(self.p95_latency_ms, 1),
            "max_latency_ms": None if self.max_latency_ms is None else round(self.max_latency_ms, 1),
            "dropped": self.dropped,
            "alarms": dict(sorted(self.alarms.items())),
        }


def compute_topic_stats(store: Store, run_id: str) -> list[TopicStats]:
    samples = store.fetch_samples(run_id)
    grouped: dict[str, list[dict[str, Any]]] = {}
    for sample in samples:
        grouped.setdefault(sample["topic"], []).append(sample)

    stats_list: list[TopicStats] = []
    for topic, rows in sorted(grouped.items()):
        rows.sort(key=lambda row: (row["source_ts"], row["id"]))
        stat = TopicStats(topic=topic, count=len(rows))
        stat.first_ts = rows[0]["source_ts"]
        stat.last_ts = rows[-1]["source_ts"]
        duration = stat.last_ts - stat.first_ts
        if duration > 0 and len(rows) > 1:
            stat.measured_hz = (len(rows) - 1) / duration
        gaps = [
            (rows[index]["source_ts"] - rows[index - 1]["source_ts"]) * 1000.0
            for index in range(1, len(rows))
        ]
        if gaps:
            stat.max_gap_ms = max(gaps)
        latencies = sorted((row["recv_ts"] - row["source_ts"]) * 1000.0 for row in rows)
        if latencies:
            stat.max_latency_ms = latencies[-1]
            index = min(len(latencies) - 1, max(0, round(0.95 * len(latencies)) - 1))
            stat.p95_latency_ms = latencies[index]
        dropped = 0
        for index in range(1, len(rows)):
            previous, current = rows[index - 1].get("seq"), rows[index].get("seq")
            if previous is not None and current is not None and current > previous + 1:
                dropped += current - previous - 1
        stat.dropped = dropped
        stats_list.append(stat)
    return stats_list


def analyze_run(store: Store, run_id: str) -> dict[str, Any]:
    run = store.get_run(run_id)
    if run is None:
        raise KeyError(f"run not found: {run_id}")

    topic_stats = compute_topic_stats(store, run_id)
    alarms = sorted(store.fetch_alarms(run_id=run_id, limit=5000), key=lambda a: (a["ts"], a["id"]))
    by_kind = Counter(alarm["kind"] for alarm in alarms)
    by_severity = Counter(alarm["severity"] for alarm in alarms)

    for alarm in alarms:
        for stat in topic_stats:
            if stat.topic == alarm.get("topic"):
                stat.alarms[alarm["kind"]] = stat.alarms.get(alarm["kind"], 0) + 1

    findings: list[dict[str, Any]] = []
    seen: set[tuple[str, str | None]] = set()
    # One finding per (kind, topic); the first occurrence is the head of the evidence.
    for alarm in alarms:
        key = (alarm["kind"], alarm.get("topic"))
        if key in seen or alarm["kind"] not in CAUSE_LIBRARY:
            seen.add(key)
            continue
        seen.add(key)
        title, action = CAUSE_LIBRARY[alarm["kind"]]
        occurrences = [
            item for item in alarms if item["kind"] == alarm["kind"] and item.get("topic") == alarm.get("topic")
        ]
        findings.append(
            {
                "kind": alarm["kind"],
                "topic": alarm.get("topic"),
                "severity": max((item["severity"] for item in occurrences), key=lambda s: SEVERITY_ORDER.get(s, 0)),
                "title": title,
                "detail": alarm["message"],
                "action": action,
                "first_ts": occurrences[0]["ts"],
                "count": len(occurrences),
                "evidence": {
                    "first_message": occurrences[0]["message"],
                    "value": occurrences[0].get("value"),
                    "threshold": occurrences[0].get("threshold"),
                },
            }
        )

    findings.sort(key=lambda item: (-SEVERITY_ORDER.get(item["severity"], 0), item["first_ts"]))
    if not alarms:
        verdict = "healthy" if topic_stats else "no-data"
    elif by_severity.get("critical"):
        verdict = "critical"
    else:
        verdict = "degraded"

    started_at = run["started_at"]
    data_start = min((topic.first_ts for topic in topic_stats if topic.first_ts), default=None)
    data_end = max((topic.last_ts for topic in topic_stats if topic.last_ts), default=None)
    duration = (
        round(data_end - data_start, 3)
        if data_start is not None and data_end is not None and data_end > data_start
        else round((run["finished_at"] or run["started_at"]) - run["started_at"], 3)
    )
    timeline = [
        {
            "offset_s": round(alarm["ts"] - (data_start if data_start is not None else started_at), 3),
            "kind": alarm["kind"],
            "severity": alarm["severity"],
            "topic": alarm.get("topic"),
            "message": alarm["message"],
        }
        for alarm in alarms[:200]
    ]

    return {
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "run": {
            "run_id": run["run_id"],
            "robot_id": run["robot_id"],
            "source": run["source"],
            "started_at": run["started_at"],
            "finished_at": run["finished_at"],
            "data_start_ts": data_start,
            "data_end_ts": data_end,
            "note": run.get("note"),
            "duration_s": duration,
            "wall_duration_s": round((run["finished_at"] or run["started_at"]) - run["started_at"], 3),
            "sample_count": store.sample_count(run_id),
        },
        "verdict": verdict,
        "alarms": {
            "total": len(alarms),
            "by_kind": dict(by_kind.most_common()),
            "by_severity": dict(by_severity.most_common()),
        },
        "topics": [stat.to_dict() for stat in topic_stats],
        "findings": findings,
        "timeline": timeline,
    }


def _fmt(value: Any, suffix: str = "") -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.1f}{suffix}"
    return f"{value}{suffix}"


def _fmt_time(ts: float | None) -> str:
    if not ts:
        return "-"
    return datetime.fromtimestamp(ts, tz=timezone.utc).astimezone().strftime("%Y-%m-%d %H:%M:%S")


def render_text(report: dict[str, Any]) -> str:
    run = report["run"]
    lines = [
        "=" * 78,
        " ros2-fleet-ops 분석 리포트",
        "=" * 78,
        f" run       {run['run_id']}",
        f" 로봇       {run['robot_id']} ({run['source']})",
        f" 구간       {_fmt_time(run.get('data_start_ts') or run['started_at'])} ~ "
        f"{_fmt_time(run.get('data_end_ts') or run['finished_at'])}  ({run['duration_s']}s)",
        f" 샘플       {run['sample_count']}건 / 알람 {report['alarms']['total']}건",
        f" 판정       {report['verdict'].upper()}",
    ]
    if run.get("note"):
        lines.append(f" 비고       {run['note']}")
    lines.append("")
    lines.append("[토픽 통계]")
    header = f" {'토픽':<10}{'건수':>7}{'측정Hz':>9}{'최대간격':>11}{'p95지연':>10}{'결번':>7}  알람"
    lines.append(header)
    for topic in report["topics"]:
        alarms = ", ".join(f"{kind}×{count}" for kind, count in topic["alarms"].items()) or "-"
        lines.append(
            f" {topic['topic']:<10}{topic['count']:>7}"
            f"{_fmt(topic['measured_hz']):>9}"
            f"{_fmt(topic['max_gap_ms'], 'ms'):>11}"
            f"{_fmt(topic['p95_latency_ms'], 'ms'):>10}"
            f"{topic['dropped']:>7}  {alarms}"
        )
    lines.append("")
    if report["findings"]:
        lines.append("[탐지된 이상]")
        for index, finding in enumerate(report["findings"], start=1):
            topic = f" {finding['topic']}" if finding["topic"] else ""
            lines.append(
                f" {index}. [{finding['severity']}] {finding['kind']}{topic} ×{finding['count']}"
                f" - {finding['title']}"
            )
            lines.append(f"    근거: {finding['detail']}")
            lines.append(f"    조치: {finding['action']}")
        lines.append("")
    else:
        lines.append("[탐지된 이상] 없음 - 모든 토픽이 설정된 주기·지연·결번 기준 안에 있습니다.")
        lines.append("")
    if report["timeline"]:
        lines.append("[타임라인] (run 시작 기준)")
        for event in report["timeline"][:20]:
            topic = f" {event['topic']}" if event["topic"] else ""
            lines.append(
                f" +{event['offset_s']:>7.2f}s  {event['severity']:<8} {event['kind']:<16}{topic} {event['message']}"
            )
        if len(report["timeline"]) > 20:
            lines.append(f" ... 외 {len(report['timeline']) - 20}건")
    return "\n".join(lines)


def render_markdown(report: dict[str, Any]) -> str:
    run = report["run"]
    lines = [
        f"# ros2-fleet-ops 분석 리포트 — `{run['run_id']}`",
        "",
        f"- 로봇: `{run['robot_id']}` (source: `{run['source']}`)",
        f"- 구간: {_fmt_time(run.get('data_start_ts') or run['started_at'])} ~ "
        f"{_fmt_time(run.get('data_end_ts') or run['finished_at'])} ({run['duration_s']}s)",
        f"- 샘플: {run['sample_count']}건 · 알람: {report['alarms']['total']}건",
        f"- 판정: **{report['verdict'].upper()}**",
        "",
        "## 토픽 통계",
        "",
        "| 토픽 | 건수 | 측정 Hz | 최대 간격 | p95 지연 | 결번 | 알람 |",
        "| --- | ---: | ---: | ---: | ---: | ---: | --- |",
    ]
    for topic in report["topics"]:
        alarms = ", ".join(f"{kind}×{count}" for kind, count in topic["alarms"].items()) or "-"
        lines.append(
            f"| `{topic['topic']}` | {topic['count']} | {_fmt(topic['measured_hz'])} |"
            f" {_fmt(topic['max_gap_ms'], 'ms')} | {_fmt(topic['p95_latency_ms'], 'ms')} |"
            f" {topic['dropped']} | {alarms} |"
        )
    lines += ["", "## 탐지된 이상", ""]
    if report["findings"]:
        lines += ["| # | 심각도 | 종류 | 토픽 | 발생 | 근거 |", "| --- | --- | --- | --- | ---: | --- |"]
        for index, finding in enumerate(report["findings"], start=1):
            lines.append(
                f"| {index} | {finding['severity']} | `{finding['kind']}` | `{finding['topic'] or '-'}` |"
                f" {finding['count']} | {finding['detail']} |"
            )
        lines.append("")
        for index, finding in enumerate(report["findings"], start=1):
            lines.append(f"{index}. **{finding['title']}**")
            lines.append(f"   - 조치: {finding['action']}")
    else:
        lines.append("설정된 주기·지연·결번 기준을 벗어난 항목이 없습니다.")
    return "\n".join(lines) + "\n"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="analyzer",
        description="Analyze a stored ros2-fleet-ops run and print likely causes",
    )
    parser.add_argument("--db", default="data/fleet.db")
    parser.add_argument("--jsonl-dir", default="data/jsonl")
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--latest", action="store_true", help="use the most recent run")
    parser.add_argument("--json", action="store_true", dest="as_json")
    parser.add_argument("--markdown", action="store_true")
    parser.add_argument("--out", default=None, help="write the report to this path")
    parser.add_argument(
        "--fail-on",
        choices=("never", "degraded", "critical"),
        default="never",
        help="exit 1 when the verdict reaches this level",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    store = Store(args.db, args.jsonl_dir)
    try:
        run_id = args.run_id
        if run_id is None:
            latest = store.latest_run()
            if latest is None:
                print("분석할 run이 없습니다.", file=sys.stderr)
                return 2
            run_id = latest["run_id"]
        report = analyze_run(store, run_id)
    except KeyError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    finally:
        store.close()

    if args.as_json:
        text = json.dumps(report, ensure_ascii=False, indent=2, default=str)
    elif args.markdown:
        text = render_markdown(report)
    else:
        text = render_text(report)
    print(text)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text + ("\n" if not text.endswith("\n") else ""), encoding="utf-8")

    verdict = report["verdict"]
    if args.fail_on == "degraded" and verdict in ("degraded", "critical"):
        return 1
    if args.fail_on == "critical" and verdict == "critical":
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
