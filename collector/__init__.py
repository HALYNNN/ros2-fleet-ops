"""ros2-fleet-ops collector package.

Modules
-------
storage    SQLite + JSONL persistence
anomaly    rule-based detectors (silence, drop, gap, latency, rate, estop)
collector  ingestion core, ROS 2 subscriptions, CLI entry point
sim        deterministic TurtleBot3-like simulator and fault plans
analyzer   post-run statistics and cause report
replay     recorded-run playback engine
"""

__version__ = "0.1.0"

__all__ = ["__version__"]
