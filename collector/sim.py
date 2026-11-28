"""Deterministic TurtleBot3-like simulator used when ROS 2 is not available.

This is not a physics engine. It reproduces exactly what the collector cares
about: messages on `/odom`, `/scan`, `/cmd_vel`, their timestamps and sequence
numbers, and the ability to inject the four faults the analyzer knows about
(latency, drop, silence, emergency stop).

The simulator has its own clock with a `time_scale`, so a 30-second scenario can
be replayed in 1.5 seconds by tests while latency and silence keep the same
meaning in the recorded timestamps.
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

TOPICS = ("/odom", "/cmd_vel", "/scan")
DEFAULT_RATES = {"/odom": 20.0, "/cmd_vel": 10.0, "/scan": 10.0}
ARENA = (2.4, 2.0)  # metres, x by y
ROBOT_RADIUS = 0.105  # TurtleBot3 burger footprint radius


class SimClock:
    """Wall-clock paced clock with an acceleration factor."""

    def __init__(self, time_scale: float = 1.0, start: float | None = None) -> None:
        self.time_scale = max(0.01, float(time_scale))
        self._real_start = time.monotonic()
        self._start = time.time() if start is None else float(start)

    def now(self) -> float:
        return self._start + (time.monotonic() - self._real_start) * self.time_scale

    def real_elapsed(self) -> float:
        return time.monotonic() - self._real_start

    def sleep(self, sim_seconds: float) -> None:
        real = sim_seconds / self.time_scale
        if real > 0:
            time.sleep(real)


@dataclass
class FaultPlan:
    """What to break, and when (all times in simulated seconds)."""

    latency_ms: float = 0.0
    latency_topic: str | None = None  # None = every topic
    drop_every: int = 0
    drop_topic: str = "/scan"
    silence_topic: str | None = None
    silence_at_s: float = 6.0
    silence_for_s: float = 4.0
    estop_at_s: float | None = None
    estop_for_s: float = 3.0

    def latency_for(self, topic: str) -> float:
        if self.latency_ms <= 0:
            return 0.0
        if self.latency_topic is None or self.latency_topic == topic:
            return self.latency_ms / 1000.0
        return 0.0

    def drop_for(self, topic: str, seq: int) -> bool:
        if self.drop_every <= 0 or not seq:
            return False
        return topic == self.drop_topic and seq % int(self.drop_every) == 0

    def silenced_for(self, topic: str, elapsed: float) -> bool:
        if self.silence_topic != topic:
            return False
        return self.silence_at_s <= elapsed < self.silence_at_s + self.silence_for_s

    def estop(self, elapsed: float) -> bool:
        if self.estop_at_s is None:
            return False
        return self.estop_at_s <= elapsed < self.estop_at_s + self.estop_for_s

    def describe(self) -> list[str]:
        notes: list[str] = []
        if self.latency_ms > 0:
            scope = self.latency_topic or "all topics"
            notes.append(f"latency {self.latency_ms:.0f}ms on {scope}")
        if self.drop_every > 0:
            notes.append(f"drop every {self.drop_every}th {self.drop_topic} message")
        if self.silence_topic:
            notes.append(
                f"silence {self.silence_topic} for {self.silence_for_s:.0f}s from {self.silence_at_s:.0f}s"
            )
        if self.estop_at_s is not None:
            notes.append(f"emergency stop {self.estop_for_s:.0f}s from {self.estop_at_s:.0f}s")
        return notes

    @classmethod
    def from_names(cls, names: Iterable[str], **overrides: Any) -> "FaultPlan":
        """Build a plan from a set of fault names (`latency`, `drop`, `silence`, `estop`)."""
        known = {"latency", "drop", "silence", "estop"}
        unknown = set(names) - known
        if unknown:
            raise ValueError(f"unknown fault(s): {', '.join(sorted(unknown))}")
        plan = cls()
        for key, value in overrides.items():
            if not hasattr(plan, key):
                raise ValueError(f"unknown fault option: {key}")
            setattr(plan, key, value)
        if "latency" in names and plan.latency_ms <= 0:
            plan.latency_ms = 400.0
        if "drop" in names and plan.drop_every <= 0:
            plan.drop_every = 4
        if "silence" in names and plan.silence_topic is None:
            plan.silence_topic = "/scan"
        if "estop" in names and plan.estop_at_s is None:
            plan.estop_at_s = 12.0
        return plan


@dataclass
class SimRobot:
    """Waypoint patrol with a unicycle model."""

    waypoints: list[tuple[float, float]] = field(
        default_factory=lambda: [(0.6, 0.5), (1.8, 0.5), (1.8, 1.4), (0.6, 1.4)]
    )
    x: float = 0.6
    y: float = 0.5
    yaw: float = 0.0
    speed: float = 0.0
    angular: float = 0.0
    max_speed: float = 0.22
    max_turn: float = 1.2
    _target: int = 1

    def step(self, dt: float, estop: bool = False) -> None:
        target = self.waypoints[self._target % len(self.waypoints)]
        dx, dy = target[0] - self.x, target[1] - self.y
        distance = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        error = _normalize_angle(desired_yaw - self.yaw)

        if estop:
            # Emergency stop: the velocity command is cut, the base coasts.
            # The decay is deliberately slow (0.98 per 50 Hz tick, so the speed
            # needs ~2.4 s to fall from 0.22 m/s to the 0.02 m/s standstill
            # threshold): a genuine control cut must stay visible longer than the
            # detector's `estop_debounce_s` window, or the alarm could only be
            # raised by a same-tick cmd_vel/odom ordering accident.
            self.speed *= 0.98
            self.angular = 0.0
            if abs(self.speed) < 0.01:
                self.speed = 0.0
            self.x += self.speed * math.cos(self.yaw) * dt
            self.y += self.speed * math.sin(self.yaw) * dt
            return

        self.angular = max(-self.max_turn, min(self.max_turn, 2.5 * error))
        if abs(error) > 0.7:
            self.speed = 0.04
        else:
            self.speed = self.max_speed * max(0.35, 1.0 - abs(error))
        self.yaw = _normalize_angle(self.yaw + self.angular * dt)
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt

        if distance < 0.12:
            self._target += 1


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _ray_distance(x: float, y: float, angle: float, arena: tuple[float, float]) -> float:
    """Distance from (x, y) along `angle` to the arena wall."""
    width, height = arena
    dx, dy = math.cos(angle), math.sin(angle)
    candidates: list[float] = []
    if dx > 1e-9:
        candidates.append((width - x) / dx)
    elif dx < -1e-9:
        candidates.append(-x / dx)
    if dy > 1e-9:
        candidates.append((height - y) / dy)
    elif dy < -1e-9:
        candidates.append(-y / dy)
    positive = [value for value in candidates if value > 0]
    return min(positive) if positive else 0.0


def scan_payload(robot: SimRobot, beams: int = 36) -> dict[str, Any]:
    ranges: list[float] = []
    half_fov = math.radians(180.0)
    for index in range(beams):
        angle = robot.yaw - half_fov + (2 * half_fov) * index / (beams - 1)
        distance = _ray_distance(robot.x, robot.y, angle, ARENA) - ROBOT_RADIUS
        ranges.append(round(max(0.12, min(3.5, distance)), 3))
    front = [value for index, value in enumerate(ranges) if abs(index - beams // 2) <= 2]
    return {
        "point_count": beams,
        "min_range": min(ranges),
        "front_min_range": min(front) if front else min(ranges),
        "ranges_mm": [int(value * 1000) for value in ranges],
    }


def odom_payload(robot: SimRobot, seq: int) -> dict[str, Any]:
    return {
        "x": round(robot.x, 4),
        "y": round(robot.y, 4),
        "yaw": round(robot.yaw, 4),
        "linear_x": round(robot.speed, 4),
        "angular_z": round(robot.angular, 4),
        "seq": seq,
    }


def cmd_vel_payload(robot: SimRobot, estop: bool) -> dict[str, Any]:
    if estop:
        return {"linear_x": 0.0, "angular_z": 0.0, "estop": True}
    linear = min(robot.max_speed, max(0.0, robot.speed))
    return {"linear_x": round(linear, 4), "angular_z": round(robot.angular, 4), "estop": False}


def run_simulation(
    core: Any,
    *,
    duration_s: float = 30.0,
    time_scale: float = 1.0,
    faults: FaultPlan | None = None,
    seed: int = 7,
    tick_hz: float = 50.0,
    clock: SimClock | None = None,
    realtime: bool = True,
    on_tick: Callable[[float], None] | None = None,
    should_stop: Callable[[], bool] | None = None,
) -> dict[str, Any]:
    """Drive the simulator and feed `core.handle_sample` / `core.tick`.

    `core` must expose `handle_sample(topic, data, seq=, source_ts=, recv_ts=)`
    and `tick(now=)`. `duration_s=0` runs until stopped (Ctrl+C in the caller).
    """
    faults = faults or FaultPlan()
    clock = clock or SimClock(time_scale=time_scale)
    rng = random.Random(seed)
    robot = SimRobot()
    seqs = {topic: 0 for topic in TOPICS}
    rates = {topic: DEFAULT_RATES.get(topic, 10.0) for topic in TOPICS}
    tick_period = 1.0 / tick_hz
    # Simulated timestamps come from a fixed step grid, never from the wall clock.
    # Real time only paces the loop, so a slow CI machine cannot create fake gaps.
    step = min(tick_period, min(1.0 / rates[topic] for topic in TOPICS) / 2.0)
    start = clock.now()
    due = {topic: start for topic in TOPICS}
    samples = 0
    published = {topic: 0 for topic in TOPICS}
    dropped = {topic: 0 for topic in TOPICS}
    step_index = 0

    while True:
        if should_stop is not None and should_stop():
            break
        elapsed = step_index * step
        if duration_s and elapsed >= duration_s:
            break
        sim_now = start + elapsed
        estop = faults.estop(elapsed)
        robot.step(step, estop=estop)

        for topic in TOPICS:
            while sim_now >= due[topic]:
                due[topic] += 1.0 / rates[topic]
                if faults.silenced_for(topic, elapsed):
                    # A stopped publisher produces nothing, and its sequence freezes too.
                    continue
                seqs[topic] += 1
                seq = seqs[topic]
                if faults.drop_for(topic, seq):
                    dropped[topic] += 1
                    continue
                if topic == "/odom":
                    data = odom_payload(robot, seq)
                elif topic == "/cmd_vel":
                    data = cmd_vel_payload(robot, estop)
                else:
                    data = scan_payload(robot)
                latency = faults.latency_for(topic)
                jitter = rng.uniform(0.0, 0.002)
                recv = due[topic] - 1.0 / rates[topic]
                core.handle_sample(
                    topic,
                    data,
                    seq=seq,
                    source_ts=recv - latency - jitter,
                    recv_ts=recv,
                )
                samples += 1
                published[topic] += 1

        core.tick(now=sim_now)
        if on_tick is not None:
            on_tick(elapsed)
        step_index += 1
        if realtime:
            ahead = start + step_index * step - clock.now()
            if ahead > 0:
                clock.sleep(ahead)

    duration_actual = min(step_index * step, duration_s) if duration_s else step_index * step
    return {
        "duration_s": round(duration_actual, 3),
        "samples": samples,
        "published": published,
        "dropped": dropped,
        "seq": seqs,
        "faults": faults.describe(),
        "final_pose": {"x": round(robot.x, 3), "y": round(robot.y, 3), "yaw": round(robot.yaw, 3)},
    }
