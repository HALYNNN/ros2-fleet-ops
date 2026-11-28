"""Guard rails for the delivery artefacts: compose, Dockerfiles, systemd, CI, README."""
from __future__ import annotations

from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_yaml(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_compose_defines_services_with_healthchecks():
    compose = load_yaml(ROOT / "docker-compose.yml")
    services = compose["services"]
    assert {"sim-ros2", "collector", "web_monitor"} <= set(services)
    for name in ("sim-ros2", "collector", "web_monitor"):
        service = services[name]
        assert "healthcheck" in service, f"{name} has no healthcheck"
        assert service["healthcheck"].get("test"), f"{name} healthcheck has no test"
        assert service.get("build") or service.get("image"), f"{name} cannot be built"
    assert services["collector"]["depends_on"]["sim-ros2"]["condition"] == "service_healthy"
    assert "8000:8000" in services["web_monitor"]["ports"]
    assert "./data:/data" in services["collector"]["volumes"]


def test_compose_fault_injector_is_opt_in():
    services = load_yaml(ROOT / "docker-compose.yml")["services"]
    injector = services["fault-injector"]
    assert "tools" in injector["profiles"]


def test_dockerfiles_exist_and_install():
    for path, base in (
        (ROOT / "sim-ros2" / "Dockerfile", "ros:humble"),
        (ROOT / "collector" / "Dockerfile", "ros:humble"),
        (ROOT / "web_monitor" / "Dockerfile", "python:"),
    ):
        text = path.read_text(encoding="utf-8")
        assert text.startswith("# syntax") or text.lstrip().upper().startswith("FROM") or "FROM " in text
        assert base in text, f"{path} should build from {base}"


def test_sim_entrypoint_launches_gazebo():
    script = (ROOT / "sim-ros2" / "entrypoint.sh").read_text(encoding="utf-8")
    assert "turtlebot3_world.launch.py" in script
    assert "TURTLEBOT3_MODEL" in script


def test_collector_entrypoint_sources_ros_when_present():
    script_path = ROOT / "collector" / "docker-entrypoint.sh"
    script = script_path.read_text(encoding="utf-8")
    # ROS is only sourced if the base image provides it, so `--mode sim` on
    # python:*-slim keeps working.
    assert "/opt/ros/" in script and "setup.bash" in script
    assert "-f " in script
    assert 'exec python3 -m collector.collector "$@"' in script
    assert script_path.stat().st_mode & 0o111, "entrypoint must be executable"

    dockerfile = (ROOT / "collector" / "Dockerfile").read_text(encoding="utf-8")
    assert 'ENTRYPOINT ["/app/collector/docker-entrypoint.sh"]' in dockerfile
    assert "chmod +x /app/collector/docker-entrypoint.sh" in dockerfile


def test_systemd_unit_is_installable():
    text = (ROOT / "deploy" / "ros2-fleet-ops.service").read_text(encoding="utf-8")
    assert "[Unit]" in text and "[Service]" in text and "[Install]" in text
    assert "ExecStart=" in text and "docker compose" in text
    assert "Restart=" in text
    assert "WantedBy=multi-user.target" in text


def test_ci_workflow_runs_tests_and_validates_compose():
    workflow = load_yaml(ROOT / ".github" / "workflows" / "ci.yml")
    jobs = workflow["jobs"]
    assert "test" in jobs and "compose" in jobs
    steps = jobs["test"]["steps"]
    assert any("pytest" in (step.get("run") or "") for step in steps)
    compose_steps = jobs["compose"]["steps"]
    assert any("docker compose" in (step.get("run") or "") for step in compose_steps)


def test_requirements_and_pytest_config():
    assert (ROOT / "requirements-dev.txt").exists()
    assert (ROOT / "pyproject.toml").exists()
    pyproject = (ROOT / "pyproject.toml").read_text(encoding="utf-8")
    assert "testpaths" in pyproject


def test_readme_documents_run_and_limits():
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    for needle in (
        "ros2-fleet-ops",
        "docker compose up",
        "inject_fault.py",
        "collector/analyzer.py",
        "pytest",
    ):
        assert needle in readme, f"README is missing {needle!r}"
    assert "검증 범위" in readme
    assert "실행하지 못한 것" in readme


def test_dashboard_has_no_external_assets():
    html = (ROOT / "web_monitor" / "static" / "index.html").read_text(encoding="utf-8")
    assert "http://" not in html
    assert "https://cdn" not in html
    assert "<canvas" in html
    assert "/api/replay" in html
