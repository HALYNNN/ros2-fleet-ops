#!/usr/bin/env python3
"""Inject faults and verify that collection + analysis catch them.

Simulator mode (default, no ROS needed)
---------------------------------------
Creates its own run, plays the TurtleBot3-like simulator with the requested
faults, stores samples/alarms exactly like the collector does, then (optionally)
prints the analyzer report. Use it to validate thresholds after changing them::

    python scripts/inject_fault.py --faults latency,drop,silence,estop --duration 25 --analyze

ROS 2 mode (`--mode ros2`, requires rclpy)
------------------------------------------
* ``--fault estop``   publish zero Twist on /cmd_vel for --duration seconds
* ``--fault silence`` SIGSTOP a node process for --duration, then SIGCONT
* ``--fault delay``   relay --src-topic to --dst-topic with --delay-ms

Real ROS 2 has no publisher sequence number, so drop detection uses receive
gaps; the ``delay`` relay lets you produce real latency in a running stack.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from collector.analyzer import analyze_run, render_text  # noqa: E402
from collector.collector import CollectorCore  # noqa: E402
from collector.config import CollectorConfig  # noqa: E402
from collector.sim import FaultPlan, run_simulation  # noqa: E402
from collector.storage import Store  # noqa: E402

FAULT_NAMES = ("latency", "delay", "drop", "silence", "estop")


def normalize_faults(names: set[str]) -> set[str]:
    """`delay` is the ROS 2 relay name for the same latency fault."""
    return {"latency" if name == "delay" else name for name in names}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="inject_fault",
        description="Inject telemetry faults and check that the collector/analyzer detect them",
    )
    parser.add_argument("--mode", choices=("sim", "ros2"), default="sim")
    parser.add_argument("--faults", default="", help="comma separated: latency,drop,silence,estop")
    parser.add_argument("--fault", action="append", default=[], choices=FAULT_NAMES)
    parser.add_argument("--duration", type=float, default=25.0, help="scenario length (sim) or estop hold (ros2)")
    parser.add_argument("--time-scale", type=float, default=1.0, help="simulation acceleration")
    parser.add_argument("--db", default="data/fault-injection.db")
    parser.add_argument("--jsonl-dir", default="data/jsonl")
    parser.add_argument("--robot-id", default="tb3-sim-01")
    parser.add_argument("--latency-ms", type=float, default=0.0)
    parser.add_argument("--latency-topic", default=None)
    parser.add_argument("--drop-every", type=int, default=0)
    parser.add_argument("--drop-topic", default="/scan")
    parser.add_argument("--silence-topic", default=None)
    parser.add_argument("--silence-at", type=float, default=6.0)
    parser.add_argument("--silence-for", type=float, default=4.0)
    parser.add_argument("--estop-at", type=float, default=None)
    parser.add_argument("--estop-for", type=float, default=3.0)
    parser.add_argument("--analyze", action="store_true", help="print the analyzer report for this run")
    parser.add_argument("--report-out", default=None, help="write the analyzer markdown report here")
    parser.add_argument("--expect", default="", help="comma separated alarm kinds that must be present")
    parser.add_argument("--json", action="store_true", dest="as_json")
    # ROS 2 specific
    parser.add_argument("--target-pid", type=int, default=None, help="ros2 silence: PID to SIGSTOP/SIGCONT")
    parser.add_argument("--src-topic", default="/scan_raw", help="ros2 delay: relay source topic")
    parser.add_argument("--dst-topic", default="/scan", help="ros2 delay: relay destination topic")
    parser.add_argument("--delay-ms", type=float, default=None)
    return parser


def plan_from_args(args: argparse.Namespace) -> FaultPlan:
    names = {name.strip() for name in args.faults.split(",") if name.strip()}
    names.update(args.fault)
    names = normalize_faults(names)
    if not names and (args.delay_ms or 0) > 0 and args.mode == "ros2":
        names = {"latency"}
    overrides: dict[str, Any] = {
        "latency_ms": args.latency_ms or (args.delay_ms or 0.0),
        "latency_topic": args.latency_topic,
        "drop_every": args.drop_every,
        "drop_topic": args.drop_topic,
        "silence_topic": args.silence_topic,
        "silence_at_s": args.silence_at,
        "silence_for_s": args.silence_for,
        "estop_for_s": args.estop_for,
    }
    if args.estop_at is not None:
        overrides["estop_at_s"] = args.estop_at
    plan = FaultPlan.from_names(names, **overrides)
    if "estop" in names and args.estop_at is None and args.duration:
        plan.estop_at_s = max(1.0, args.duration / 2.0)
    return plan


# --------------------------------------------------------------- simulator


def run_sim(args: argparse.Namespace, plan: FaultPlan) -> tuple[int, dict[str, Any]]:
    config = CollectorConfig(robot_id=args.robot_id)
    store = Store(args.db, args.jsonl_dir)
    core = CollectorCore(
        store,
        config,
        source="sim",
        note="fault-injection: " + (", ".join(plan.describe()) or "none"),
    )
    try:
        result = run_simulation(
            core,
            duration_s=args.duration,
            time_scale=args.time_scale,
            faults=plan,
        )
        summary = core.finish()
        report = analyze_run(store, core.run_id)
        text = render_text(report)
    finally:
        store.close()

    if args.report_out:
        from collector.analyzer import render_markdown

        Path(args.report_out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report_out).write_text(render_markdown(report), encoding="utf-8")

    expected = {name.strip() for name in args.expect.split(",") if name.strip()}
    detected = set(report["alarms"]["by_kind"])
    missing = sorted(expected - detected)

    if args.as_json:
        print(
            json.dumps(
                {
                    "run_id": core.run_id,
                    "scenario": result,
                    "summary": summary,
                    "verdict": report["verdict"],
                    "alarms": report["alarms"],
                    "findings": report["findings"],
                    "missing_expected": missing,
                },
                ensure_ascii=False,
                indent=2,
                default=str,
            )
        )
    else:
        print(f"[inject_fault] run={core.run_id}")
        for note in plan.describe() or ["no fault injected"]:
            print(f"  - {note}")
        print(f"  samples={result['samples']} dropped={result['dropped']} verdict={report['verdict']}")
        print(f"  alarms={report['alarms']['by_kind'] or '{}'}")
        if args.analyze:
            print()
            print(text)
        if missing:
            print(f"[inject_fault] expected alarms not detected: {', '.join(missing)}", file=sys.stderr)

    return (1 if missing else 0), report


# ---------------------------------------------------------------- ROS 2 mode


def ros2_estop(duration_s: float, topic: str = "/cmd_vel") -> int:  # pragma: no cover - needs ROS 2
    import rclpy
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("fault_estop")
    publisher = node.create_publisher(Twist, topic, 10)
    message = Twist()
    deadline = time.monotonic() + max(0.1, duration_s)
    try:
        while time.monotonic() < deadline:
            publisher.publish(message)
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"[inject_fault] published zero Twist on {topic} for {duration_s:.1f}s")
    return 0


def ros2_silence(pid: int, duration_s: float) -> int:  # pragma: no cover - needs a live node
    if not shutil.which("kill"):
        print("kill not available", file=sys.stderr)
        return 2
    os.kill(pid, signal.SIGSTOP)
    print(f"[inject_fault] SIGSTOP pid {pid}; resuming in {duration_s:.1f}s")
    try:
        time.sleep(max(0.1, duration_s))
    finally:
        os.kill(pid, signal.SIGCONT)
    print(f"[inject_fault] SIGCONT pid {pid}")
    return 0


def ros2_delay(src_topic: str, dst_topic: str, delay_ms: float, duration_s: float) -> int:  # pragma: no cover
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    class DelayRelay(Node):
        def __init__(self) -> None:
            super().__init__("fault_delay_relay")
            self._buffer: list[tuple[float, Any]] = []
            self.publisher = self.create_publisher(LaserScan, dst_topic, 10)
            self.create_subscription(LaserScan, src_topic, self._on_scan, qos_profile_sensor_data)
            self.create_timer(0.01, self._flush)

        def _on_scan(self, message: Any) -> None:
            self._buffer.append((time.monotonic() + delay_ms / 1000.0, message))

        def _flush(self) -> None:
            now = time.monotonic()
            remaining = []
            for due, message in self._buffer:
                if due <= now:
                    self.publisher.publish(message)
                else:
                    remaining.append((due, message))
            self._buffer = remaining

    rclpy.init()
    node = DelayRelay()
    deadline = time.monotonic() + max(0.1, duration_s)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"[inject_fault] relayed {src_topic} -> {dst_topic} with {delay_ms:.0f}ms delay")
    return 0


# ----------------------------------------------------------------------- main


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    plan = plan_from_args(args)

    if args.mode == "sim":
        if not plan.describe():
            print("[inject_fault] no fault selected; pass --faults latency,drop,silence,estop", file=sys.stderr)
            return 2
        code, _ = run_sim(args, plan)
        return code

    names = normalize_faults(
        {name.strip() for name in args.faults.split(",") if name.strip()} | set(args.fault)
    )
    if "estop" in names:
        return ros2_estop(args.duration)
    if "silence" in names:
        if args.target_pid is None:
            print("--target-pid is required for ros2 silence", file=sys.stderr)
            return 2
        return ros2_silence(args.target_pid, args.duration)
    if "latency" in names:
        return ros2_delay(args.src_topic, args.dst_topic, args.delay_ms or args.latency_ms or 200.0, args.duration)
    print("[inject_fault] ros2 mode needs one of: estop, silence, delay", file=sys.stderr)
    return 2


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
