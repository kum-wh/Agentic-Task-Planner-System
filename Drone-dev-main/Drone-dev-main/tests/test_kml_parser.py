from pathlib import Path

import pytest

from flight_planner.kml_parser import parse_kml

FIXTURE = Path(__file__).parent / "fixtures" / "generic_mission.kml"


def test_parses_generic_kml_regardless_of_folder_nesting_and_case():
    mission = parse_kml(FIXTURE)
    assert mission.gcs_location.longitude == 10.0
    assert mission.gcs_location.latitude == 50.0
    assert mission.gcs_location.altitude == 10.0


def test_finds_geo_fence_wrapped_in_multi_geometry():
    mission = parse_kml(FIXTURE)
    assert len(mission.geo_fence.coordinates) == 5
    assert mission.geo_fence.coordinates[0].longitude == 9.995
    assert mission.geo_fence.coordinates[0].latitude == 49.995


def test_finds_ops_area_ignoring_unrelated_placemarks():
    mission = parse_kml(FIXTURE)
    assert len(mission.ops_area.coordinates) == 5
    assert mission.ops_area.coordinates[0].longitude == 9.998
    assert mission.ops_area.coordinates[0].latitude == 49.998


def test_missing_placemark_raises_with_available_names_listed():
    with pytest.raises(ValueError, match="GeoFence"):
        parse_kml(Path(__file__).parent / "fixtures" / "missing_geofence.kml")
