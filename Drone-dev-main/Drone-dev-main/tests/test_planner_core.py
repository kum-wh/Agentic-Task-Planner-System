import json
from pathlib import Path

from flight_planner.config import load_config
from flight_planner.kml_parser import parse_kml


KML = Path(__file__).parents[1] / "ops" / "SAFTI city.kml"


def test_config_has_valid_scan_parameters():
    config = load_config(Path(__file__).parents[1] / "config.example.json")
    assert config.number_of_drones == 3
    assert config.lane_spacing_m > 0


def test_kml_contains_expected_regions():
    mission = parse_kml(KML)
    assert mission.gcs_location.longitude == 103.6810782816786
    assert len(mission.geo_fence.coordinates) == 5
    assert len(mission.ops_area.coordinates) == 12
