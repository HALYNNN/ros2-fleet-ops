"""Collector configuration: topic expectations and detection thresholds.

The collector is deliberately configured by plain data so the same rules can be
loaded from a YAML file in production and built in memory by tests.

Expected rates are *thresholds used for anomaly detection*, not measured values.
The operator sets them from the robot's real publish settings (TurtleBot3
burger defaults: /odom 20 Hz, /scan 10 Hz, /cmd_vel 10 Hz).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TopicRule:
    """Detection rule for one ROS 2 topic."""

    topic: str
    expected_hz: float
    max_latency_s: float = 0.30
    #: A topic is considered silent after `silence_factor / expected_hz` seconds.
    silence_factor: float = 3.0
    #: Minimum seconds between two alarms of the same kind for this topic.
    cooldown_s: float = 2.0

    def __post_init__(self) -> None:
        # A non-positive expectation would divide by zero in `silence_after_s`
        # and make every rate check meaningless, so reject it at construction.
        if not self.expected_hz > 0 or not math.isfinite(self.expected_hz):
            raise ValueError(
                f"expected_hz must be a positive number for topic {self.topic!r}, got {self.expected_hz!r}"
            )

    @property
    def silence_after_s(self) -> float:
        return max(self.silence_factor / self.expected_hz, 0.5)

    def to_dict(self) -> dict[str, Any]:
        return {
            "topic": self.topic,
            "expected_hz": self.expected_hz,
            "max_latency_s": self.max_latency_s,
            "silence_factor": self.silence_factor,
            "cooldown_s": self.cooldown_s,
        }


DEFAULT_RULES: dict[str, TopicRule] = {
    "/odom": TopicRule("/odom", expected_hz=20.0, max_latency_s=0.25),
    "/scan": TopicRule("/scan", expected_hz=10.0, max_latency_s=0.30),
    "/cmd_vel": TopicRule("/cmd_vel", expected_hz=10.0, max_latency_s=0.30, silence_factor=4.0),
}


@dataclass
class CollectorConfig:
    robot_id: str = "tb3-01"
    rules: dict[str, TopicRule] = field(default_factory=lambda: dict(DEFAULT_RULES))
    #: odom linear speed below this is "stopped" (m/s).
    estop_speed_threshold: float = 0.02
    #: zero `cmd_vel` must persist this long (s) before it counts as an e-stop.
    #: A normal brake also commands zero velocity; only a sustained zero command
    #: while odom still reports motion is treated as "control was cut".
    estop_debounce_s: float = 0.5
    #: rate_low fires when measured rate < expected_hz * this ratio.
    rate_warn_ratio: float = 0.5
    #: number of samples used for rate estimation.
    rate_window: int = 20
    heartbeat_interval_s: float = 1.0
    #: A receive gap above this multiple of the expected period is a gap_spike.
    gap_spike_factor: float = 2.5

    def rule(self, topic: str) -> TopicRule:
        return self.rules.get(topic, TopicRule(topic, expected_hz=10.0))

    def with_rules(self, rules: dict[str, TopicRule]) -> "CollectorConfig":
        return replace(self, rules=dict(rules))

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot_id": self.robot_id,
            "estop_speed_threshold": self.estop_speed_threshold,
            "estop_debounce_s": self.estop_debounce_s,
            "rate_warn_ratio": self.rate_warn_ratio,
            "rate_window": self.rate_window,
            "heartbeat_interval_s": self.heartbeat_interval_s,
            "gap_spike_factor": self.gap_spike_factor,
            "topics": [rule.to_dict() for rule in self.rules.values()],
        }

    @classmethod
    def from_yaml(cls, path: str | Path) -> "CollectorConfig":
        import yaml

        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
        rules: dict[str, TopicRule] = {}
        for item in raw.get("topics", []):
            rule = TopicRule(
                topic=item["topic"],
                expected_hz=float(item["expected_hz"]),
                max_latency_s=float(item.get("max_latency_s", 0.30)),
                silence_factor=float(item.get("silence_factor", 3.0)),
                cooldown_s=float(item.get("cooldown_s", 2.0)),
            )
            rules[rule.topic] = rule
        config = cls(rules=rules or dict(DEFAULT_RULES))
        for key in (
            "robot_id",
            "estop_speed_threshold",
            "estop_debounce_s",
            "rate_warn_ratio",
            "rate_window",
            "heartbeat_interval_s",
            "gap_spike_factor",
        ):
            if key in raw:
                setattr(config, key, type(getattr(config, key))(raw[key]))
        return config


DETECTION_DEFAULTS = CollectorConfig()
