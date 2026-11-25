"""Rule-based anomaly detection over the incoming telemetry stream.

Detectors are intentionally simple and explainable, because the point is not to
classify every failure but to give an operator a defensible line of evidence:

* `topic_silent`  - no message for `silence_factor / expected_hz` seconds
* `topic_recovered` - messages are flowing again after a silence alarm
* `topic_drop`    - a gap in the publisher sequence number (`seq`)
* `gap_spike`     - one receive interval much longer than the expected period
* `latency_high`  - `recv_ts - source_ts` above the topic budget
* `rate_low`      - measured rate below `expected_hz * rate_warn_ratio`
* `estop_command` - `cmd_vel` is zero while `odom` still reports motion

Every alarm passes through a per-(topic, kind) cooldown so a 5-second fault does
not produce 200 rows. All thresholds are constants from `CollectorConfig`; no
metric in an alarm is synthesised - `value` always comes from the message.
"""
from __future__ import annotations

import math
import statistics
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Deque, Iterable

from .config import CollectorConfig, TopicRule

SEVERITIES = ("info", "warning", "critical")
_SEVERITY_RANK = {name: index for index, name in enumerate(SEVERITIES)}


@dataclass(frozen=True)
class Alarm:
    kind: str
    severity: str
    message: str
    ts: float
    topic: str | None = None
    value: float | None = None
    threshold: float | None = None
    robot_id: str = ""
    run_id: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "severity": self.severity,
            "message": self.message,
            "ts": round(self.ts, 6),
            "topic": self.topic,
            "value": None if self.value is None else round(self.value, 4),
            "threshold": None if self.threshold is None else round(self.threshold, 4),
            "robot_id": self.robot_id,
            "run_id": self.run_id,
        }

    def with_context(self, *, run_id: str, robot_id: str) -> "Alarm":
        return Alarm(
            kind=self.kind,
            severity=self.severity,
            message=self.message,
            ts=self.ts,
            topic=self.topic,
            value=self.value,
            threshold=self.threshold,
            robot_id=robot_id,
            run_id=run_id,
        )


@dataclass
class TopicTracker:
    rule: TopicRule
    count: int = 0
    last_seq: int | None = None
    last_recv: float | None = None
    dropped: int = 0
    gaps: Deque[float] = field(default_factory=deque)
    latencies: Deque[float] = field(default_factory=deque)
    silent: bool = False

    def rate_hz(self) -> float | None:
        if not self.gaps:
            return None
        mean_gap = statistics.fmean(self.gaps)
        return 1.0 / mean_gap if mean_gap > 0 else None

    def p95_latency_ms(self) -> float | None:
        if not self.latencies:
            return None
        ordered = sorted(self.latencies)
        index = min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)
        return ordered[index] * 1000.0


class AnomalyDetector:
    """Stateful detector; one instance per collector run."""

    #: Cooldown override per alarm kind (seconds) when the rule has no better idea.
    KIND_COOLDOWN = {
        "topic_drop": 1.0,
        "gap_spike": 1.0,
        "latency_high": 2.0,
        "rate_low": 5.0,
        "topic_silent": 5.0,
        "topic_recovered": 0.0,
        "estop_command": 3.0,
    }

    def __init__(self, config: CollectorConfig | None = None) -> None:
        self.config = config or CollectorConfig()
        self._trackers: dict[str, TopicTracker] = {}
        self._last_alarm: dict[tuple[str, str], float] = {}
        self._cmd_linear_zero = False
        self._cmd_seen = False
        self._had_motion_cmd = False
        self._estop_zero_since: float | None = None

    # ------------------------------------------------------------ internals

    def tracker(self, topic: str) -> TopicTracker:
        tracker = self._trackers.get(topic)
        if tracker is None:
            tracker = TopicTracker(rule=self.config.rule(topic))
            self._trackers[topic] = tracker
        return tracker

    def _cooldown_for(self, kind: str, rule: TopicRule) -> float:
        if kind in ("latency_high", "gap_spike", "topic_drop"):
            return rule.cooldown_s
        return self.KIND_COOLDOWN.get(kind, 2.0)

    def _emit(
        self,
        kind: str,
        severity: str,
        message: str,
        now: float,
        topic: str | None,
        value: float | None = None,
        threshold: float | None = None,
    ) -> Alarm | None:
        key = (topic or "", kind)
        cooldown = self._cooldown_for(kind, self.config.rule(topic or ""))
        last = self._last_alarm.get(key)
        if last is not None and now - last < cooldown:
            return None
        self._last_alarm[key] = now
        return Alarm(
            kind=kind,
            severity=severity,
            message=message,
            ts=now,
            topic=topic,
            value=value,
            threshold=threshold,
        )

    # --------------------------------------------------------------- ingest

    def check(
        self,
        topic: str,
        *,
        seq: int | None,
        source_ts: float,
        recv_ts: float,
        data: dict[str, Any] | None = None,
        now: float | None = None,
    ) -> list[Alarm]:
        """Run all sample-triggered checks and return the alarms that fired."""
        data = data or {}
        now = recv_ts if now is None else now
        tracker = self.tracker(topic)
        rule = tracker.rule
        alarms: list[Alarm] = []

        if tracker.silent:
            tracker.silent = False
            recovered = self._emit(
                "topic_recovered",
                "info",
                f"{topic} 수신 재개 (마지막 수신 후 {now - (tracker.last_recv or now):.2f}s)",
                now,
                topic,
            )
            if recovered:
                alarms.append(recovered)

        period = 1.0 / rule.expected_hz if rule.expected_hz > 0 else 0.1

        if tracker.last_recv is not None:
            gap = recv_ts - tracker.last_recv
            if gap > 0:
                tracker.gaps.append(gap)
                if gap > period * self.config.gap_spike_factor and gap < rule.silence_after_s:
                    alarm = self._emit(
                        "gap_spike",
                        "warning",
                        f"{topic} 수신 간격 {gap * 1000:.0f}ms (기대 주기 {period * 1000:.0f}ms)",
                        now,
                        topic,
                        value=gap * 1000.0,
                        threshold=period * self.config.gap_spike_factor * 1000.0,
                    )
                    if alarm:
                        alarms.append(alarm)

        if seq is not None:
            if tracker.last_seq is not None and seq > tracker.last_seq + 1:
                dropped = seq - tracker.last_seq - 1
                tracker.dropped += dropped
                alarm = self._emit(
                    "topic_drop",
                    "warning",
                    f"{topic} 메시지 {dropped}건 결번 (seq {tracker.last_seq} → {seq})",
                    now,
                    topic,
                    value=float(dropped),
                    threshold=0.0,
                )
                if alarm:
                    alarms.append(alarm)
            tracker.last_seq = seq if tracker.last_seq is None else max(tracker.last_seq, seq)

        latency = recv_ts - source_ts
        if latency >= 0:
            tracker.latencies.append(latency)
            if latency > rule.max_latency_s:
                severity = "critical" if latency > rule.max_latency_s * 2 else "warning"
                alarm = self._emit(
                    "latency_high",
                    severity,
                    f"{topic} 지연 {latency * 1000:.0f}ms (예산 {rule.max_latency_s * 1000:.0f}ms)",
                    now,
                    topic,
                    value=latency * 1000.0,
                    threshold=rule.max_latency_s * 1000.0,
                )
                if alarm:
                    alarms.append(alarm)

        if len(tracker.gaps) >= min(5, max(3, self.config.rate_window // 4)):
            rate = tracker.rate_hz()
            if rate is not None and rate < rule.expected_hz * self.config.rate_warn_ratio:
                alarm = self._emit(
                    "rate_low",
                    "warning",
                    f"{topic} 발행 주기 {rate:.1f}Hz (기대 {rule.expected_hz:.0f}Hz)",
                    now,
                    topic,
                    value=rate,
                    threshold=rule.expected_hz * self.config.rate_warn_ratio,
                )
                if alarm:
                    alarms.append(alarm)

        alarms.extend(self._check_motion(topic, data, now))

        tracker.count += 1
        tracker.last_recv = recv_ts
        return alarms

    def _check_motion(self, topic: str, data: dict[str, Any], now: float) -> list[Alarm]:
        """Emergency-stop inference: zero velocity command while odom moves."""
        alarms: list[Alarm] = []
        if topic == "/cmd_vel":
            linear = float(data.get("linear_x", 0.0) or 0.0)
            angular = float(data.get("angular_z", 0.0) or 0.0)
            is_zero = abs(linear) < 1e-3 and abs(angular) < 1e-3
            if not is_zero:
                self._had_motion_cmd = True
                self._estop_zero_since = None
            elif self._had_motion_cmd and self._estop_zero_since is None:
                self._estop_zero_since = now
            self._cmd_linear_zero = is_zero
            self._cmd_seen = True
            return alarms

        if topic == "/odom" and self._cmd_seen and self._cmd_linear_zero:
            speed = data.get("linear_x")
            if speed is None:
                vx = float(data.get("vx", 0.0) or 0.0)
                vy = float(data.get("vy", 0.0) or 0.0)
                speed = math.hypot(vx, vy)
            speed = abs(float(speed))
            if speed > self.config.estop_speed_threshold:
                alarm = self._emit(
                    "estop_command",
                    "warning",
                    f"cmd_vel=0 인데 odom 속도 {speed:.2f} m/s - 비상정지/제어 명령 확인 필요",
                    now,
                    "/cmd_vel",
                    value=speed,
                    threshold=self.config.estop_speed_threshold,
                )
                if alarm:
                    alarms.append(alarm)
        return alarms

    def check_silence(self, now: float) -> list[Alarm]:
        """Call periodically; flags topics whose last message is too old."""
        alarms: list[Alarm] = []
        for topic, tracker in self._trackers.items():
            if tracker.last_recv is None:
                continue
            silence = now - tracker.last_recv
            limit = tracker.rule.silence_after_s
            if silence <= limit:
                continue
            severity = "critical" if silence > limit * 2 else "warning"
            alarm = self._emit(
                "topic_silent",
                severity,
                f"{topic} {silence:.1f}s 동안 수신 없음 (기준 {limit:.1f}s)",
                now,
                topic,
                value=silence,
                threshold=limit,
            )
            if alarm:
                tracker.silent = True
                alarms.append(alarm)
        return alarms

    # -------------------------------------------------------------- reporting

    def snapshot(self) -> dict[str, dict[str, Any]]:
        out: dict[str, dict[str, Any]] = {}
        for topic, tracker in self._trackers.items():
            out[topic] = {
                "topic": topic,
                "expected_hz": tracker.rule.expected_hz,
                "count": tracker.count,
                "dropped": tracker.dropped,
                "rate_hz": None if tracker.rate_hz() is None else round(tracker.rate_hz(), 2),
                "p95_latency_ms": (
                    None if tracker.p95_latency_ms() is None else round(tracker.p95_latency_ms(), 1)
                ),
                "last_recv_ts": tracker.last_recv,
                "silent": tracker.silent,
            }
        return out

    def reset(self) -> None:
        self._trackers.clear()
        self._last_alarm.clear()
        self._cmd_linear_zero = False
        self._cmd_seen = False
        self._had_motion_cmd = False
        self._estop_zero_since = None


def max_severity(alarms: Iterable[Alarm]) -> str:
    rank = 0
    for alarm in alarms:
        rank = max(rank, _SEVERITY_RANK.get(alarm.severity, 0))
    return SEVERITIES[rank]


def now_ts() -> float:
    return time.time()
