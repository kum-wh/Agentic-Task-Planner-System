import json
import sys
from pathlib import Path

import pytest

from flight_planner import cli

ROOT = Path(__file__).parents[1]
KML = ROOT / "ops" / "SAFTI city.kml"
CONFIG = ROOT / "config.example.json"


def _run_cli(monkeypatch, argv):
    monkeypatch.setattr(sys, "argv", ["flight-planner", *argv])
    cli.main()


def test_cli_writes_baseline_plan_and_per_drone_folder(tmp_path, monkeypatch):
    output = tmp_path / "mission.plan"
    _run_cli(monkeypatch, [str(KML), "--config", str(CONFIG), "--output", str(output)])

    baseline = json.loads((tmp_path / "mission.baseline.plan").read_text())
    plan = json.loads(output.read_text())

    assert baseline["fileType"] == "Plan"
    lat, lon, _alt = plan["mission"]["plannedHomePosition"]
    assert 0 < lat < 90  # latitude first, not the ~103 degree longitude value
    assert lon > 90
    assert plan["planner"]["guardrailPassed"]

    drones_dir = tmp_path / "mission_drones"
    assert drones_dir.is_dir()
    drone_files = sorted(drones_dir.glob("drone-*.plan"))
    assert len(drone_files) == plan["x-multiDrone"]["droneCount"]
    for drone_file in drone_files:
        drone_plan = json.loads(drone_file.read_text())
        assert drone_plan["mission"]["items"]
        assert "x-multiDrone" not in drone_plan


def test_cli_scan_type_override(tmp_path, monkeypatch):
    output = tmp_path / "mission.plan"
    _run_cli(monkeypatch, [str(KML), "--config", str(CONFIG), "--output", str(output), "--scan-type", "voronoi"])
    plan = json.loads(output.read_text())
    assert plan["x-multiDrone"]["droneCount"] == 3
    assert plan["planner"]["guardrailPassed"]


def test_cli_custom_drone_plans_dir(tmp_path, monkeypatch):
    output = tmp_path / "mission.plan"
    custom_dir = tmp_path / "per_drone_plans"
    _run_cli(
        monkeypatch,
        [str(KML), "--config", str(CONFIG), "--output", str(output), "--drone-plans-dir", str(custom_dir)],
    )
    assert custom_dir.is_dir()
    assert sorted(p.name for p in custom_dir.glob("*.plan")) == ["drone-1.plan", "drone-2.plan", "drone-3.plan"]


@pytest.mark.parametrize("number_of_drones", [1, 2, 4])
@pytest.mark.parametrize("scan_type", ["lawnmower", "voronoi"])
def test_cli_runs_for_various_drone_counts_and_scan_types(tmp_path, monkeypatch, number_of_drones, scan_type):
    config_path = tmp_path / "config.json"
    config_data = json.loads(CONFIG.read_text())
    config_data["number_of_drones"] = number_of_drones
    config_data["scan_type"] = scan_type
    config_path.write_text(json.dumps(config_data))

    output = tmp_path / "mission.plan"
    _run_cli(monkeypatch, [str(KML), "--config", str(config_path), "--output", str(output)])

    plan = json.loads(output.read_text())
    assert plan["planner"]["guardrailPassed"], plan["planner"]["warnings"]
    assert plan["x-multiDrone"]["droneCount"] == number_of_drones

    drones_dir = tmp_path / "mission_drones"
    drone_files = list(drones_dir.glob("drone-*.plan"))
    assert 0 < len(drone_files) <= number_of_drones
