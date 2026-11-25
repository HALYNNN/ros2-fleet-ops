from __future__ import annotations

import pytest

from collector.anomaly import AnomalyDetector, max_severity
from collector.config import CollectorConfig, TopicRule
from tests.conftest import odom


def make_detector(**kwargs) -> AnomalyDetector:
    return AnomalyDetector(CollectorConfig(robot_id="tb3-test", **kwargs))


def feed(detector, topic, times, data=None, seq_start=1, rates=None):
    alarms = []
    seq = seq_start
    for index, ts in enumerate(times):
        payload = data(index) if callable(data) else (data or odom(x=0.1 * index))
        alarms.extend(
            detector.check(topic, seq=seq, source_ts=ts, recv_ts=ts, data=payload, now=ts)
        )
        seq += 1
    return alarms


def test_steady_stream_has_no_alarms():
    detector = make_detector()
    times = [100.0 + index * 0.05 for index in range(60)]  # 20 Hz, seq intact
    alarms = feed(detector, "/odom", times)
    assert alarms == []


def test_sequence_gap_raises_topic_drop():
    detector = make_detector()
    times = [100.0 + index * 0.05 for index in range(10)]
    feed(detector, "/odom", times[:5])
    alarms = detector.check(
        "/odom", seq=13, source_ts=times[9], recv_ts=times[9], data=odom()
    )
    kinds = [alarm.kind for alarm in alarms]
    assert "topic_drop" in kinds
    drop = next(alarm for alarm in alarms if alarm.kind == "topic_drop")
    assert drop.value == 7  # seq 6..12 were never delivered
    assert "결번" in drop.message


def test_latency_alarm_and_escalation():
    detector = make_detector()
    base = 100.0
    alarms = detector.check("/odom", seq=1, source_ts=base, recv_ts=base, data=odom())
    assert alarms == []
    # 400 ms is above the 250 ms budget but below 2x
    alarms = detector.check("/odom", seq=2, source_ts=base + 0.45, recv_ts=base + 0.85, data=odom())
    latency = [alarm for alarm in alarms if alarm.kind == "latency_high"]
    assert latency and latency[0].severity == "warning"

    detector.reset()
    detector.check("/odom", seq=1, source_ts=base, recv_ts=base, data=odom())
    alarms = detector.check("/odom", seq=2, source_ts=base + 0.05, recv_ts=base + 1.05, data=odom())
    latency = [alarm for alarm in alarms if alarm.kind == "latency_high"]
    assert latency and latency[0].severity == "critical"


def test_latency_alarm_respects_cooldown():
    detector = make_detector()
    base = 100.0
    feed(detector, "/odom", [base + index * 0.05 for index in range(3)])
    first = detector.check("/odom", seq=10, source_ts=base + 0.2, recv_ts=base + 1.0, data=odom())
    second = detector.check("/odom", seq=11, source_ts=base + 0.25, recv_ts=base + 1.05, data=odom())
    assert any(alarm.kind == "latency_high" for alarm in first)
    assert not any(alarm.kind == "latency_high" for alarm in second)


def test_silence_then_recovery():
    detector = make_detector()
    feed(detector, "/odom", [100.0 + index * 0.05 for index in range(5)])
    # silence limit for /odom is max(3/20, 0.5) = 0.5 s
    assert detector.check_silence(100.4) == []
    silent = detector.check_silence(100.75)
    assert [alarm.kind for alarm in silent] == ["topic_silent"]
    assert silent[0].severity == "warning"
    assert detector.check_silence(100.9) == []  # still silent, but cooldown holds

    recovered = detector.check(
        "/odom", seq=20, source_ts=101.1, recv_ts=101.1, data=odom()
    )
    assert any(alarm.kind == "topic_recovered" for alarm in recovered)


def test_gap_spike_detected_below_silence_limit():
    detector = make_detector()
    base = 100.0
    feed(detector, "/odom", [base, base + 0.05])
    alarms = detector.check("/odom", seq=3, source_ts=base + 0.19, recv_ts=base + 0.19, data=odom())
    assert any(alarm.kind == "gap_spike" for alarm in alarms)


def test_rate_low_when_publisher_slows_down():
    detector = make_detector()
    times = [100.0 + index * 0.12 for index in range(10)]  # ~8 Hz vs 20 Hz expected
    alarms = feed(detector, "/odom", times)
    assert any(alarm.kind == "rate_low" for alarm in alarms)
    rate_alarm = next(alarm for alarm in alarms if alarm.kind == "rate_low")
    assert rate_alarm.value == pytest.approx(8.0, abs=0.5)


def test_estop_inferred_from_zero_cmd_while_moving():
    detector = make_detector()
    base = 100.0
    detector.check("/cmd_vel", seq=1, source_ts=base, recv_ts=base, data={"linear_x": 0.2, "angular_z": 0.0})
    detector.check("/cmd_vel", seq=2, source_ts=base + 0.1, recv_ts=base + 0.1, data={"linear_x": 0.0, "angular_z": 0.0})
    alarms = detector.check(
        "/odom", seq=9, source_ts=base + 0.15, recv_ts=base + 0.15, data=odom(linear_x=0.18)
    )
    assert [alarm.kind for alarm in alarms] == ["estop_command"]
    assert alarms[0].topic == "/cmd_vel"


def test_no_estop_alarm_when_robot_stopped():
    detector = make_detector()
    base = 100.0
    detector.check("/cmd_vel", seq=1, source_ts=base, recv_ts=base, data={"linear_x": 0.2, "angular_z": 0.0})
    detector.check("/cmd_vel", seq=2, source_ts=base + 0.1, recv_ts=base + 0.1, data={"linear_x": 0.0, "angular_z": 0.0})
    alarms = detector.check(
        "/odom", seq=9, source_ts=base + 0.15, recv_ts=base + 0.15, data=odom(linear_x=0.0)
    )
    assert alarms == []


def test_custom_rule_is_used():
    config = CollectorConfig(
        rules={"/imu": TopicRule("/imu", expected_hz=50.0, max_latency_s=0.02)}
    )
    detector = AnomalyDetector(config)
    detector.check("/imu", seq=1, source_ts=100.0, recv_ts=100.0, data={})
    alarms = detector.check("/imu", seq=2, source_ts=100.02, recv_ts=100.08, data={})
    assert any(alarm.kind == "latency_high" for alarm in alarms)
    assert detector.snapshot()["/imu"]["expected_hz"] == 50.0


def test_snapshot_reports_measured_rate_and_dropped():
    detector = make_detector()
    times = [100.0 + index * 0.05 for index in range(21)]
    feed(detector, "/odom", times, seq_start=1)
    detector.check("/odom", seq=30, source_ts=101.05, recv_ts=101.05, data=odom())
    snapshot = detector.snapshot()["/odom"]
    assert snapshot["count"] == 22
    assert snapshot["dropped"] == 8
    assert snapshot["rate_hz"] == pytest.approx(20.0, abs=1.0)


def test_max_severity_helper():
    from collector.anomaly import Alarm

    alarms = [
        Alarm(kind="a", severity="info", message="", ts=0),
        Alarm(kind="b", severity="critical", message="", ts=0),
        Alarm(kind="c", severity="warning", message="", ts=0),
    ]
    assert max_severity(alarms) == "critical"
    assert max_severity([]) == "info"
