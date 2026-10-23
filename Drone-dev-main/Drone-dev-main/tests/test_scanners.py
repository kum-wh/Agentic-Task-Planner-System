import math
from dataclasses import replace
from pathlib import Path

import pytest

from flight_planner.agents import MultiAgentPlanner, _missing_waypoint_warnings
from flight_planner.config import load_config
from flight_planner.kml_parser import parse_kml
from flight_planner.models import Coordinate, DronePlan, MissionRegion, PlannerConfig
from flight_planner.geometry import Point, rotate
from flight_planner.scanner_tools import _lawnmower_points, _seed_points, lawnmower_scan

ROOT = Path(__file__).parents[1]


@pytest.mark.parametrize("scan_type", ["lawnmower", "voronoi"])
@pytest.mark.parametrize("number_of_drones", [1, 2, 3, 4])
def test_multi_agent_plan_stays_inside_fence(scan_type, number_of_drones):
    mission = parse_kml(ROOT / "ops" / "SAFTI city.kml")
    config = replace(load_config(ROOT / "config.example.json"), scan_type=scan_type, number_of_drones=number_of_drones)
    result = MultiAgentPlanner().run(mission, config)
    assert result.guardrail_passed, result.warnings
    assert len(result.drones) == config.number_of_drones


def test_voronoi_mode_creates_one_partition_per_drone():
    mission = parse_kml(ROOT / "ops" / "SAFTI city.kml")
    config = replace(load_config(ROOT / "config.example.json"), scan_type="voronoi")
    result = MultiAgentPlanner().run(mission, config)
    assert result.guardrail_passed, result.warnings
    assert len(result.drones) == config.number_of_drones


def test_lawnmower_scan_partitions_into_contiguous_non_overlapping_strips():
    region = MissionRegion(
        "rect",
        [Coordinate(0.0, 0.0), Coordinate(0.02, 0.0), Coordinate(0.02, 0.01), Coordinate(0.0, 0.01)],
    )
    origin = Coordinate(0.0, 0.0)
    config = PlannerConfig(number_of_drones=2, camera_fov_degrees=70.0, scan_altitude_m=50.0, scan_overlap_percent=20.0, scan_type="lawnmower")

    paths = lawnmower_scan(region, origin, config)

    assert len(paths) == 2
    assert all(path for path in paths)
    max_lon_drone_1 = max(point.longitude for point in paths[0])
    min_lon_drone_2 = min(point.longitude for point in paths[1])
    assert max_lon_drone_1 <= min_lon_drone_2 + 1e-9


def test_lawnmower_drones_each_cover_only_a_fraction_of_the_ops_area():
    """Regression test: drones used to interleave every Nth point of one global sweep instead of
    getting a spatially contiguous strip, so every drone's bounding box spanned the whole area."""
    mission = parse_kml(ROOT / "ops" / "SAFTI city.kml")
    config = load_config(ROOT / "config.example.json")
    result = MultiAgentPlanner().run(mission, config)

    ops_lons = [c.longitude for c in mission.ops_area.coordinates]
    ops_lats = [c.latitude for c in mission.ops_area.coordinates]
    total_lon_span = max(ops_lons) - min(ops_lons)
    total_lat_span = max(ops_lats) - min(ops_lats)

    for drone in result.drones:
        if len(drone.waypoints) < 2:
            continue
        lons = [w.longitude for w in drone.waypoints]
        lats = [w.latitude for w in drone.waypoints]
        lon_ratio = (max(lons) - min(lons)) / total_lon_span
        lat_ratio = (max(lats) - min(lats)) / total_lat_span
        assert min(lon_ratio, lat_ratio) < 0.6


def test_lawnmower_handles_more_drones_than_the_area_can_usefully_split():
    region = MissionRegion(
        "thin",
        [Coordinate(0.0, 0.0), Coordinate(0.0001, 0.0), Coordinate(0.0001, 0.0001), Coordinate(0.0, 0.0001)],
    )
    origin = Coordinate(0.0, 0.0)
    config = PlannerConfig(number_of_drones=5, camera_fov_degrees=70.0, scan_altitude_m=50.0, scan_overlap_percent=20.0, scan_type="lawnmower")

    paths = lawnmower_scan(region, origin, config)

    assert len(paths) == 5


def test_missing_waypoint_warning_for_empty_drone():
    drones = [DronePlan(1, [Coordinate(0.0, 0.0)]), DronePlan(2, [])]
    warnings = _missing_waypoint_warnings(drones)
    assert warnings == ["drone 2 received no waypoints for this OpsArea/scan_type"]


# --- orientation-aware lanes -------------------------------------------------

def _lanes(points):
    """Consecutive point pairs are one lane each; return their lengths in metres."""
    return [math.dist(a, b) for a, b in zip(points[::2], points[1::2])]


def test_lanes_run_along_the_long_axis_of_a_thin_area():
    """Regression: the export always scanned east-west, so a north-south strip got
    many short turn-heavy lanes instead of a few full-length passes."""
    polygon = [Point(0, 0), Point(200, 0), Point(200, 1000), Point(0, 1000)]

    lanes = _lanes(_lawnmower_points(polygon, 25.0))

    assert len(lanes) == 8                       # 200 m across / 25 m pitch
    assert all(length > 950 for length in lanes)  # each pass runs the 1000 m axis


def test_lane_direction_is_not_an_axis_aligned_accident():
    polygon = [Point(0, 0), Point(200, 0), Point(200, 1000), Point(0, 1000)]
    rotated = [rotate(point, math.radians(37)) for point in polygon]

    straight = _lanes(_lawnmower_points(polygon, 25.0))
    diagonal = _lanes(_lawnmower_points(rotated, 25.0))

    assert len(diagonal) == len(straight)
    assert sum(diagonal) == pytest.approx(sum(straight), rel=0.02)


def test_lanes_are_centred_so_the_boundary_is_still_covered():
    polygon = [Point(0, 0), Point(1000, 0), Point(1000, 200), Point(0, 200)]
    spacing = 25.0

    lane_ys = sorted({round(point.y, 6) for point in _lawnmower_points(polygon, spacing)})

    assert min(lane_ys) > 0 and max(lane_ys) < 200        # no lane sits on the edge
    assert min(lane_ys) == pytest.approx(200 - max(lane_ys))  # symmetric within the area
    assert min(lane_ys) < spacing / 2 + 1e-6              # within one half-swath of the edge


def test_an_area_narrower_than_one_lane_still_gets_a_centre_pass():
    polygon = [Point(0, 0), Point(400, 0), Point(400, 10), Point(0, 10)]

    points = _lawnmower_points(polygon, 25.0)

    assert len(_lanes(points)) == 1
    assert points[0].y == pytest.approx(5.0)


# --- voronoi seeding ---------------------------------------------------------

def test_voronoi_seeds_are_distinct_on_an_awkward_shape():
    """Regression: too few in-polygon seeds used to be padded with a duplicate, and
    the half-plane clip between identical seeds is a no-op -- two drones would both
    inherit the whole OpsArea and fly the same mission."""
    diagonal_band = [Point(0, 0), Point(1000, 900), Point(1000, 1000), Point(0, 100)]

    for count in range(1, 7):
        seeds = _seed_points(diagonal_band, count)
        assert len(seeds) == count
        assert len(set(seeds)) == count


def test_voronoi_gives_each_drone_a_different_cell():
    mission = parse_kml(ROOT / "ops" / "SAFTI city.kml")
    config = replace(load_config(ROOT / "config.example.json"), scan_type="voronoi", number_of_drones=4)

    result = MultiAgentPlanner().run(mission, config)

    footprints = [frozenset((w.longitude, w.latitude) for w in drone.waypoints) for drone in result.drones]
    non_empty = [footprint for footprint in footprints if footprint]
    assert len(non_empty) == len(set(non_empty))


def test_multi_agent_planner_does_not_share_agent_instances():
    """Mutable dataclass defaults raise ValueError at import on Python >= 3.11."""
    first, second = MultiAgentPlanner(), MultiAgentPlanner()
    assert first.flight_planner is not second.flight_planner
    assert first.guardrail_planner is not second.guardrail_planner
