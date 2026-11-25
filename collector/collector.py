"""Collector entry point: subscribe (`/odom`, `/scan`, `/cmd_vel`) or simulate.

Two sources, one pipeline:

* ``--mode ros2``  real ROS 2 (rclpy) subscriptions; requires a Humble environment
* ``--mode sim``   deterministic TurtleBot3-like simulator (no ROS installation)
* ``--mode auto``  ROS 2 if rclpy imports, otherwise the simulator

Both feed the same :class:`CollectorCore`, so storage, anomaly detection, the
JSONL archive, the analyzer and the web monitor behave identically.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from typing import Any, Callable

from .anomaly import Alarm, AnomalyDetector
from .config import CollectorConfig
from .sim import FaultPlan, run_simulation
from .storage import Store

STOP = threading.Event()


# --------------------------------------------------------------------- core


class CollectorCore:
    """Source-independent ingestion pipeline."""

    def __init__(
        self,
        store: Store,
        config: CollectorConfig | None = None,
        *,
        source: str = "sim",
        run_id: str | None = None,
        note: str | None = None,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.store = store
        self.config = config or CollectorConfig()
        self.clock = clock or time.time
        self.source = source
        self.robot_id = self.config.robot_id
        self.detector = AnomalyDetector(self.config)
        if run_id:
            self.run_id = run_id
            if store.get_run(run_id) is None:
                store.start_run(
                    self.robot_id,
                    source=source,
                    note=note,
                    run_id=run_id,
                    started_at=self.clock(),
                )
        else:
            self.run_id = store.start_run(
                self.robot_id, source=source, note=note, started_at=self.clock()
            )
        self.samples = 0
        self.sample_counts: dict[str, int] = {}
        self.alarm_counts: dict[str, int] = {}
        self._last_heartbeat = 0.0
        self._heartbeat()

    # -- ingestion ---------------------------------------------------------

    def handle_sample(
        self,
        topic: str,
        data: dict[str, Any] | None = None,
        *,
        seq: int | None = None,
        source_ts: float | None = None,
        recv_ts: float | None = None,
    ) -> dict[str, Any]:
        recv = self.clock() if recv_ts is None else float(recv_ts)
        src = recv if source_ts is None else float(source_ts)
        data = data or {}
        self.store.insert_sample(
            run_id=self.run_id,
            robot_id=self.robot_id,
            topic=topic,
            seq=seq,
            source_ts=src,
            recv_ts=recv,
            data=data,
        )
        self.samples += 1
        self.sample_counts[topic] = self.sample_counts.get(topic, 0) + 1
        alarms = self.detector.check(
            topic, seq=seq, source_ts=src, recv_ts=recv, data=data, now=recv
        )
        self._record_alarms(alarms)
        self._heartbeat(recv)
        return {
            "run_id": self.run_id,
            "robot_id": self.robot_id,
            "topic": topic,
            "seq": seq,
            "source_ts": src,
            "recv_ts": recv,
            "data": data,
        }

    def tick(self, now: float | None = None) -> list[Alarm]:
        ts = self.clock() if now is None else float(now)
        alarms = self.detector.check_silence(ts)
        if alarms:
            self._record_alarms(alarms)
        self._heartbeat(ts)
        return alarms

    def _record_alarms(self, alarms: list[Alarm]) -> None:
        for alarm in alarms:
            contextual = alarm.with_context(run_id=self.run_id, robot_id=self.robot_id)
            self.store.insert_alarm(contextual.to_dict())
            self.alarm_counts[contextual.kind] = self.alarm_counts.get(contextual.kind, 0) + 1

    def _heartbeat(self, now: float | None = None) -> None:
        ts = self.clock() if now is None else now
        if ts - self._last_heartbeat >= self.config.heartbeat_interval_s:
            self._last_heartbeat = ts
            try:
                self.store.write_heartbeat(
                    {
                        "run_id": self.run_id,
                        "robot_id": self.robot_id,
                        "source": self.source,
                        "samples": self.samples,
                        "alarms": sum(self.alarm_counts.values()),
                    }
                )
            except OSError:
                pass

    # -- lifecycle ---------------------------------------------------------

    def finish(self) -> dict[str, Any]:
        self.store.finish_run(self.run_id, finished_at=self.clock())
        return self.summary()

    def summary(self) -> dict[str, Any]:
        return {
            "run_id": self.run_id,
            "robot_id": self.robot_id,
            "source": self.source,
            "samples": self.samples,
            "topics": dict(sorted(self.sample_counts.items())),
            "alarms": dict(sorted(self.alarm_counts.items())),
            "topics_state": self.detector.snapshot(),
        }


# ------------------------------------------------------------------ ROS 2 IO


def stamp_to_sec(stamp: Any) -> float | None:
    """Convert a `builtin_interfaces/Time` (or ROS 1 style stamp) to seconds."""
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", None)
    if sec is None:
        sec = getattr(stamp, "secs", None)
    if sec is None:
        return None
    nanosec = getattr(stamp, "nanosec", None)
    if nanosec is None:
        nanosec = getattr(stamp, "nsecs", 0)
    value = float(sec) + float(nanosec or 0) * 1e-9
    return value if value > 0 else None


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def odom_to_data(msg: Any, seq: int | None = None) -> dict[str, Any]:
    pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
    twist = msg.twist.twist if hasattr(msg.twist, "twist") else msg.twist
    orientation = pose.orientation
    return {
        "x": round(float(pose.position.x), 4),
        "y": round(float(pose.position.y), 4),
        "z": round(float(pose.position.z), 4),
        "yaw": round(
            quaternion_to_yaw(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
            4,
        ),
        "linear_x": round(float(twist.linear.x), 4),
        "angular_z": round(float(twist.angular.z), 4),
        "seq": seq,
    }


def scan_to_data(msg: Any) -> dict[str, Any]:
    ranges = [float(value) for value in msg.ranges]
    finite = [value for value in ranges if math.isfinite(value) and value > 0.0]
    angle_min = float(getattr(msg, "angle_min", 0.0))
    increment = float(getattr(msg, "angle_increment", 0.0))
    front: list[float] = []
    for index, value in enumerate(ranges):
        if not (math.isfinite(value) and value > 0.0):
            continue
        angle = angle_min + increment * index
        if abs(math.atan2(math.sin(angle), math.cos(angle))) <= math.radians(30.0):
            front.append(value)
    return {
        "point_count": len(ranges),
        "min_range": round(min(finite), 3) if finite else None,
        "front_min_range": round(min(front), 3) if front else None,
        "angle_min": angle_min,
        "angle_increment": increment,
    }


def twist_to_data(msg: Any) -> dict[str, Any]:
    return {
        "linear_x": round(float(msg.linear.x), 4),
        "angular_z": round(float(msg.angular.z), 4),
    }


def run_ros2(
    core: CollectorCore,
    *,
    duration_s: float = 0.0,
    node_name: str = "fleet_collector",
    use_sim_time: bool = False,
    topics: tuple[str, str, str] = ("/odom", "/scan", "/cmd_vel"),
) -> None:
    """Subscribe with rclpy and feed the core until stopped."""
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init(args=None)
    node = Node(
        node_name,
        parameter_overrides=[Parameter("use_sim_time", value=bool(use_sim_time))],
    )
    counters = {topic: 0 for topic in topics}
    started = node.get_clock().now().nanoseconds * 1e-9

    def make_callback(topic: str, convert: Callable[[Any], dict[str, Any]]):
        def callback(msg: Any) -> None:
            counters[topic] += 1
            recv = node.get_clock().now().nanoseconds * 1e-9
            source_ts = stamp_to_sec(getattr(msg.header, "stamp", None)) if hasattr(msg, "header") else None
            core.handle_sample(
                topic,
                convert(msg),
                seq=counters[topic],
                source_ts=recv if source_ts is None else source_ts,
                recv_ts=recv,
            )

        return callback

    odom_topic, scan_topic, cmd_topic = topics
    node.create_subscription(Odometry, odom_topic, make_callback(odom_topic, odom_to_data), qos_profile_sensor_data)
    node.create_subscription(LaserScan, scan_topic, make_callback(scan_topic, scan_to_data), qos_profile_sensor_data)
    node.create_subscription(Twist, cmd_topic, make_callback(cmd_topic, twist_to_data), 10)

    node.get_logger().info(
        f"collector run {core.run_id} listening on {odom_topic}, {scan_topic}, {cmd_topic}"
    )
    clock_warning_sent = False
    try:
        while rclpy.ok() and not STOP.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            if use_sim_time and now == 0.0 and not clock_warning_sent:
                node.get_logger().warning(
                    "use_sim_time is set but /clock has not arrived yet; "
                    "check ROS_DOMAIN_ID and that Gazebo is publishing /clock"
                )
                clock_warning_sent = True
            # Detector silence checks must run on the same clock as recv_ts.
            core.tick(now=now)
            if duration_s and now - started >= duration_s:
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ----------------------------------------------------------------------- CLI


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="collector",
        description="ros2-fleet-ops collector: /odom /scan /cmd_vel -> SQLite + JSONL with anomaly detection",
    )
    parser.add_argument("--mode", choices=("auto", "ros2", "sim"), default="auto")
    parser.add_argument("--db", default=os.environ.get("FLEET_DB", "data/fleet.db"))
    parser.add_argument("--jsonl-dir", default=os.environ.get("FLEET_JSONL_DIR", "data/jsonl"))
    parser.add_argument("--robot-id", default=os.environ.get("FLEET_ROBOT_ID", "tb3-01"))
    parser.add_argument(
        "--duration", type=float, default=0.0, help="seconds to run (0 = until SIGINT/SIGTERM)"
    )
    parser.add_argument("--time-scale", type=float, default=1.0, help="simulator acceleration (sim mode)")
    parser.add_argument("--config", default=None, help="YAML collector config (topic rules)")
    parser.add_argument(
        "--use-sim-time",
        action="store_true",
        help="ROS 2 mode: use /clock (Gazebo) time so header stamps and receive time match",
    )
    parser.add_argument("--note", default=None, help="free-form note stored with the run")
    parser.add_argument(
        "--faults",
        default="",
        help="sim mode only: comma separated latency,drop,silence,estop",
    )
    parser.add_argument("--quiet", action="store_true")
    return parser


def _install_signal_handlers() -> None:
    def handler(signum, frame):  # noqa: ANN001 - signal signature
        STOP.set()

    for name in ("SIGINT", "SIGTERM"):
        sig = getattr(signal, name, None)
        if sig is not None:
            try:
                signal.signal(sig, handler)
            except (ValueError, OSError):
                pass


def resolve_mode(requested: str) -> str:
    if requested != "auto":
        return requested
    try:
        import rclpy  # noqa: F401

        return "ros2"
    except Exception:  # pragma: no cover - depends on environment
        return "sim"


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _install_signal_handlers()

    try:
        config = CollectorConfig.from_yaml(args.config) if args.config else CollectorConfig()
    except (OSError, ValueError, KeyError) as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2
    if args.robot_id:
        config.robot_id = args.robot_id

    mode = resolve_mode(args.mode)
    store = Store(args.db, args.jsonl_dir)
    core = CollectorCore(store, config, source=mode, note=args.note)
    fault_plan = FaultPlan.from_names(
        [name.strip() for name in args.faults.split(",") if name.strip()]
    )
    if not args.quiet:
        print(f"[collector] run={core.run_id} robot={core.robot_id} mode={mode} db={store.db_path}")

    exit_code = 0
    try:
        if mode == "ros2":
            run_ros2(core, duration_s=args.duration, use_sim_time=args.use_sim_time)
        else:
            run_simulation(
                core,
                duration_s=args.duration,
                time_scale=args.time_scale,
                faults=fault_plan,
                should_stop=STOP.is_set,
            )
    except KeyboardInterrupt:  # pragma: no cover - interactive
        STOP.set()
    except Exception as exc:  # pragma: no cover - environment failures
        print(f"[collector] failed: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        summary = core.finish()
        store.close()
        if not args.quiet:
            print(json.dumps(summary, ensure_ascii=False, indent=2))
    if exit_code:
        return exit_code
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
