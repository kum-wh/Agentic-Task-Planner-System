from dataclasses import replace
from pathlib import Path

from flight_planner.agents import MultiAgentPlanner
from flight_planner.config import load_config
from flight_planner.kml_parser import parse_kml
from flight_planner.models import Coordinate, DronePlan, PlanningResult
from flight_planner.qgc_plan import build_baseline_plan, build_qgc_plan, split_plan_by_drone

ROOT = Path(__file__).parents[1]


def _mission():
    return parse_kml(ROOT / "ops" / "SAFTI city.kml")


def test_planned_home_position_is_lat_lon_alt_order():
    mission = _mission()
    plan = build_baseline_plan(mission)
    lat, lon, alt = plan["mission"]["plannedHomePosition"]
    assert lat == mission.gcs_location.latitude
    assert lon == mission.gcs_location.longitude
    assert alt == mission.gcs_location.altitude


def test_geo_fence_polygon_is_lat_lon_pairs():
    mission = _mission()
    plan = build_baseline_plan(mission)
    polygon = plan["geoFence"]["polygons"][0]["polygon"]
    assert len(polygon) == len(mission.geo_fence.coordinates)
    for (lat, lon), coordinate in zip(polygon, mission.geo_fence.coordinates):
        assert lat == coordinate.latitude
        assert lon == coordinate.longitude


def test_mission_items_tag_the_correct_drone_and_param_order():
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    result = MultiAgentPlanner().run(mission, config)
    plan = build_qgc_plan(result)
    items = plan["mission"]["items"]

    flying = [drone for drone in result.drones if drone.waypoints]
    # Each drone contributes a takeoff, a speed change and an RTL on top of its waypoints.
    assert len(items) == sum(len(drone.waypoints) + 3 for drone in flying)
    index = 0
    for drone in flying:
        assert items[index]["command"] == 178         # DO_CHANGE_SPEED, before takeoff
        assert items[index]["comment"] == f"drone-{drone.drone_id}"
        index += 1
        assert items[index]["command"] == 22          # NAV_TAKEOFF
        assert items[index]["comment"] == f"drone-{drone.drone_id}"
        index += 1
        for waypoint in drone.waypoints:
            item = items[index]
            assert item["command"] == 16              # NAV_WAYPOINT
            assert item["comment"] == f"drone-{drone.drone_id}"
            assert item["params"][4] == waypoint.latitude
            assert item["params"][5] == waypoint.longitude
            assert item["params"][6] == waypoint.altitude
            index += 1
        assert items[index]["command"] == 20          # NAV_RETURN_TO_LAUNCH
        assert items[index]["comment"] == f"drone-{drone.drone_id}"
        index += 1
    assert plan["x-multiDrone"]["droneCount"] == len(result.drones)


def test_every_drone_gets_a_complete_takeoff_lawnmower_rtl_mission():
    """The export emitted only NAV_WAYPOINT items -- no takeoff, no return-to-launch."""
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    result = MultiAgentPlanner().run(mission, config)

    for drone_id, drone_plan in split_plan_by_drone(build_qgc_plan(result)).items():
        items = drone_plan["mission"]["items"]
        assert items[0]["command"] == 178, f"drone {drone_id} has no speed command"
        assert items[1]["command"] == 22, f"drone {drone_id} has no takeoff"
        assert items[-1]["command"] == 20, f"drone {drone_id} has no RTL"
        assert all(item["command"] == 16 for item in items[2:-1])


def test_rtl_item_uses_frame_2():
    """Frame 3 on an RTL makes PX4 reject the whole mission ("Invalid item")."""
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    plan = build_qgc_plan(MultiAgentPlanner().run(mission, config))

    rtl_items = [item for item in plan["mission"]["items"] if item["command"] == 20]
    assert rtl_items
    for item in rtl_items:
        assert item["frame"] == 2
        assert item["params"] == [0, 0, 0, 0, 0, 0, 0]


def test_cruise_speed_comes_from_the_camera_model():
    """The export hardcoded cruiseSpeed 15 with no link to optics."""
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    plan = build_qgc_plan(MultiAgentPlanner().run(mission, config))

    assert plan["mission"]["cruiseSpeed"] == config.cruise_speed_m_s
    assert plan["planner"]["imaging"]["gsd_cm_px"] == round(config.profile.gsd_cm_px, 2)

    faster = build_qgc_plan(MultiAgentPlanner().run(mission, replace(config, scan_altitude_m=60.0)))
    assert faster["mission"]["cruiseSpeed"] > plan["mission"]["cruiseSpeed"]


def test_a_drone_with_no_waypoints_contributes_no_items():
    mission = _mission()
    result = PlanningResult(mission, [DronePlan(1, []), DronePlan(2, [])], True)
    assert build_qgc_plan(result)["mission"]["items"] == []


def test_split_plan_by_drone_produces_one_single_vehicle_plan_per_drone():
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    result = MultiAgentPlanner().run(mission, config)
    plan = build_qgc_plan(result)

    split = split_plan_by_drone(plan)
    assert set(split.keys()) == {drone.drone_id for drone in result.drones}

    for drone in result.drones:
        drone_plan = split[drone.drone_id]
        items = drone_plan["mission"]["items"]
        assert len(items) == len(drone.waypoints) + 3   # + takeoff + speed + RTL
        assert all(item["comment"] == f"drone-{drone.drone_id}" for item in items)
        assert [item["doJumpId"] for item in items] == list(range(1, len(items) + 1))
        assert drone_plan["mission"]["plannedHomePosition"] == plan["mission"]["plannedHomePosition"]
        assert drone_plan["geoFence"] == plan["geoFence"]
        assert "x-multiDrone" not in drone_plan
        assert drone_plan["planner"]["droneId"] == drone.drone_id


def test_cruise_speed_is_pushed_to_the_vehicle_not_just_the_planning_field():
    """A .plan's cruiseSpeed never reaches PX4 -- it flies MPC_XY_CRUISE (5 m/s) unless
    a DO_CHANGE_SPEED item says otherwise. Measured on the Pulau Ubin flight."""
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    plan = build_qgc_plan(MultiAgentPlanner().run(mission, config))

    speed_items = [i for i in plan["mission"]["items"] if i["command"] == 178]
    flying = [d for d in MultiAgentPlanner().run(mission, config).drones if d.waypoints]
    assert len(speed_items) == len(flying)
    for item, drone in zip(speed_items, flying):
        # Each drone flies its own altitude layer, so its blur-safe speed comes from that layer.
        expected = replace(config, scan_altitude_m=drone.altitude_m).cruise_speed_m_s
        assert item["frame"] == 2                                  # MAV_FRAME_MISSION
        assert item["params"][0] == 1                              # ground speed
        assert item["params"][1] == expected                       # PX4 reads param2
        assert item["params"][2] == -1                             # throttle unchanged
    assert plan["mission"]["cruiseSpeed"] == config.cruise_speed_m_s


def test_the_speed_item_comes_before_takeoff_and_any_waypoint():
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    plan = build_qgc_plan(MultiAgentPlanner().run(mission, config))
    commands = [i["command"] for i in plan["mission"]["items"]]
    assert commands.index(178) < commands.index(22) < commands.index(16)


def test_speed_scales_with_altitude_in_the_emitted_item():
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    low = build_qgc_plan(MultiAgentPlanner().run(mission, config))
    high = build_qgc_plan(MultiAgentPlanner().run(mission, replace(config, scan_altitude_m=60.0)))
    low_speed = next(i["params"][1] for i in low["mission"]["items"] if i["command"] == 178)
    high_speed = next(i["params"][1] for i in high["mission"]["items"] if i["command"] == 178)
    assert high_speed > low_speed


def test_a_cruise_speed_above_the_vehicle_ceiling_is_warned_about():
    """Above ~46 m the blur-safe speed exceeds MPC_XY_VEL_MAX and the plan is optimistic."""
    mission = _mission()
    config = replace(load_config(ROOT / "config.example.json"), scan_altitude_m=80.0)
    result = MultiAgentPlanner().run(mission, config)
    assert any("exceeds the vehicle ceiling" in w for w in result.warnings)
    assert not replace(config, scan_altitude_m=30.0).speed_warnings


def test_drone_count_reports_plans_that_exist_not_drones_requested():
    """Regression: droneCount counted every drone while _mission_items skips drones with
    empty cells, so a plan could advertise more drones than there are .plan files."""
    mission = _mission()
    result = PlanningResult(
        mission,
        [DronePlan(1, [Coordinate(103.68, 1.37, 30.0), Coordinate(103.681, 1.371, 30.0)]),
         DronePlan(2, []),
         DronePlan(3, [])],
        True,
    )
    plan = build_qgc_plan(result)

    assert plan["x-multiDrone"]["droneCount"] == 1              # only drone 1 produced items
    assert plan["x-multiDrone"]["requestedDroneCount"] == 3     # the discrepancy stays visible
    assert set(split_plan_by_drone(plan)) == {1}


def test_drone_count_matches_the_number_of_per_drone_plans_when_a_cell_is_starved():
    """A zero-area OpsArea starves some drones -- the counts must still agree."""
    from flight_planner.models import MissionInput, MissionRegion, PlannerConfig

    fence = MissionRegion("f", [Coordinate(-0.01, -0.01), Coordinate(0.02, -0.01),
                                Coordinate(0.02, 0.02), Coordinate(-0.01, 0.02)])
    degenerate = MissionRegion("o", [Coordinate(0, 0), Coordinate(0.001, 0.001),
                                     Coordinate(0.002, 0.002)])
    mission = MissionInput(Coordinate(0.001, 0.0005), fence, degenerate)
    result = MultiAgentPlanner().run(mission, PlannerConfig(number_of_drones=8))
    plan = build_qgc_plan(result)

    flying = [d for d in result.drones if d.waypoints]
    assert len(flying) < 8, "expected this shape to starve some drones"
    assert plan["x-multiDrone"]["droneCount"] == len(flying)
    assert plan["x-multiDrone"]["requestedDroneCount"] == 8
    assert len(split_plan_by_drone(plan)) == plan["x-multiDrone"]["droneCount"]
    starved = [w for w in result.warnings if "received no waypoints" in w]
    assert len(starved) == 8 - len(flying)


def test_drone_count_equals_requested_when_nothing_is_starved():
    mission = _mission()
    config = load_config(ROOT / "config.example.json")
    plan = build_qgc_plan(MultiAgentPlanner().run(mission, config))
    assert plan["x-multiDrone"]["droneCount"] == plan["x-multiDrone"]["requestedDroneCount"] == 3
