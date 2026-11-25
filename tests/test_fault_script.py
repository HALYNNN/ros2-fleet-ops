from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from inject_fault import build_parser, plan_from_args  # noqa: E402


def run_script(tmp_path, *args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "scripts/inject_fault.py", "--db", str(tmp_path / "f.db"),
         "--jsonl-dir", str(tmp_path / "jsonl"), *args],
        cwd=ROOT,
        capture_output=True,
        text=True,
        timeout=120,
    )


def test_inject_drop_silence_estop_end_to_end(tmp_path):
    proc = run_script(
        tmp_path,
        "--faults", "drop,silence,estop",
        "--duration", "12",
        "--time-scale", "50",
        "--drop-every", "3",
        "--silence-at", "6", "--silence-for", "2",
        "--estop-at", "9", "--estop-for", "1.5",
        "--expect", "topic_drop,topic_silent,estop_command",
        "--json",
    )
    assert proc.returncode == 0, proc.stderr + proc.stdout
    payload = json.loads(proc.stdout)
    assert payload["missing_expected"] == []
    assert payload["verdict"] in ("degraded", "critical")
    assert payload["alarms"]["by_kind"]["topic_drop"] >= 1
    assert payload["alarms"]["by_kind"]["topic_silent"] >= 1
    assert payload["alarms"]["by_kind"]["estop_command"] >= 1
    assert payload["scenario"]["samples"] > 100


def test_inject_latency_report_out(tmp_path):
    report = tmp_path / "reports" / "latency.md"
    proc = run_script(
        tmp_path,
        "--faults", "latency",
        "--latency-ms", "500",
        "--duration", "6",
        "--time-scale", "50",
        "--expect", "latency_high",
        "--report-out", str(report),
    )
    assert proc.returncode == 0, proc.stderr
    assert "latency_high" in proc.stdout
    assert report.exists()
    assert report.read_text(encoding="utf-8").startswith("# ros2-fleet-ops 분석 리포트")


def test_expect_mismatch_exits_nonzero(tmp_path):
    proc = run_script(
        tmp_path, "--faults", "drop", "--duration", "6", "--time-scale", "50",
        "--expect", "topic_silent",
    )
    assert proc.returncode == 1
    assert "not detected" in proc.stderr


def test_no_fault_selected_returns_2(tmp_path):
    proc = run_script(tmp_path, "--duration", "2")
    assert proc.returncode == 2


def test_ros2_mode_requires_a_fault(tmp_path):
    proc = run_script(tmp_path, "--mode", "ros2", "--duration", "1")
    assert proc.returncode == 2


def test_plan_from_args_defaults():
    parser = build_parser()
    args = parser.parse_args(["--faults", "latency,drop", "--duration", "10"])
    plan = plan_from_args(args)
    assert plan.latency_ms == 400.0
    assert plan.drop_every == 4
    assert plan.estop_at_s is None


def test_plan_from_args_overrides_and_estop_placement():
    args = build_parser().parse_args(
        [
            "--fault", "estop", "--duration", "10", "--estop-for", "2",
            "--latency-ms", "250", "--drop-every", "5", "--silence-topic", "/odom",
        ]
    )
    plan = plan_from_args(args)
    assert plan.estop_at_s == 5.0
    assert plan.estop_for_s == 2.0
    assert plan.latency_ms == 250
    assert plan.drop_topic == "/scan"
    assert plan.silence_topic == "/odom"


def test_unknown_fault_name_is_rejected():
    args = build_parser().parse_args(["--faults", "explode"])
    with pytest.raises(ValueError):
        plan_from_args(args)
