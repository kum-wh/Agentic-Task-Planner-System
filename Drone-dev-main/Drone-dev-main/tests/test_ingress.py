"""Group ingress: who flies which cell, and where the swarm splits up.

Two claims carry the feature and both are tested here as invariants rather than as
examples, because they are the reasons any of this is worth doing:

  * handing the cells out differently cannot change the total distance flown, only who
    pays it -- which is why energy assignment is free, and
  * a gate can never shorten anyone's ingress -- which is why it buys formation flying
    and a simultaneous start, and never endurance.

The energy model behind the assignment is a guess (see energy.py), so nothing here pins
its constants. The tests check its *shape* -- heavier costs more, emptier costs more --
which is all the planner actually relies on.
"""

import math
import random
from dataclasses import replace
from pathlib import Path

import pytest

from flight_planner.agents import (MultiAgentPlanner, _flight_distance, _flight_legs,
                                   _ingress_warnings, _local)
from flight_planner.energy import DroneSpec, assign_cells, worst_load
from flight_planner.geometry import Point, circumcentre, project, search_in_polygon
from flight_planner.ingress import CellEntry, choose_gate
from flight_planner.kml_parser import parse_kml
from flight_planner.models import Coordinate, DronePlan, MissionInput, PlannerConfig
from flight_planner.qgc_plan import build_qgc_plan

ROOT = Path(__file__).parents[1]
SECTORS = ["toa-payoh-urban.kml", "pulau-ubin-west.kml", "SAFTI city.kml"]
FLEET = (DroneSpec(2.3, 1.00, "light"), DroneSpec(2.6, 0.90, "medium"), DroneSpec(3.0, 0.75, "heavy"))
SQUARE = [Point(-800.0, -800.0), Point(800.0, -800.0), Point(800.0, 800.0), Point(-800.0, 800.0)]


def _mission(name: str) -> MissionInput:
    return parse_kml(ROOT / "ops" / name)


def _gated(**overrides) -> PlannerConfig:
    return PlannerConfig(ingress_style="gate", assignment="energy", drone_specs=FLEET, **overrides)


def _entries(starts, survey_m=1000.0):
    """Cells whose two ends are effectively the same point, so entry choice cannot cheat."""
    return [CellEntry((start, start), survey_m) for start in starts]


# --- the geometry, against answers worked out by hand -------------------------------

def test_circumcentre_of_an_equilateral_triangle_is_its_centre():
    side = 10.0
    triangle = (Point(0.0, 0.0), Point(side, 0.0), Point(side / 2, side * math.sqrt(3) / 2))
    centre = circumcentre(*triangle)
    assert centre.x == pytest.approx(side / 2)
    assert centre.y == pytest.approx(side / (2 * math.sqrt(3)))


def test_circumcentre_of_a_right_triangle_is_the_midpoint_of_its_hypotenuse():
    centre = circumcentre(Point(0.0, 0.0), Point(6.0, 0.0), Point(0.0, 8.0))
    assert (centre.x, centre.y) == pytest.approx((3.0, 4.0))


def test_collinear_starts_have_no_equidistant_point():
    assert circumcentre(Point(0.0, 0.0), Point(1.0, 1.0), Point(2.0, 2.0)) is None


def test_search_returns_none_when_nothing_is_inside_the_polygon():
    assert search_in_polygon([], lambda point: (0, 0.0)) is None


def test_gate_for_three_symmetric_starts_lands_at_their_centre():
    centre = Point(100.0, -50.0)
    starts = [Point(centre.x + 200 * math.cos(a), centre.y + 200 * math.sin(a))
              for a in (0.0, 2 * math.pi / 3, 4 * math.pi / 3)]
    gate, reverse = choose_gate(Point(0.0, 0.0), _entries(starts), SQUARE, [13.0] * 3,
                                [DroneSpec()] * 3, "distance")
    assert (gate.x, gate.y) == pytest.approx((centre.x, centre.y), abs=0.5)
    assert [math.dist(gate, start) for start in starts] == pytest.approx([200.0] * 3, abs=0.5)
    assert reverse == (False, False, False)


# --- invariants ---------------------------------------------------------------------

def test_a_gate_can_never_shorten_anyones_ingress():
    """The whole economics of the feature: grouping up costs distance, always.

    If this ever fails, the objective is measuring something other than what is flown.
    """
    home = Point(0.0, 0.0)
    rng = random.Random(20260916)
    for _ in range(20):
        starts = [Point(rng.uniform(-700, 700), rng.uniform(-700, 700)) for _ in range(3)]
        gate, _ = choose_gate(home, _entries(starts), SQUARE, [13.0] * 3, [DroneSpec()] * 3, "arrival")
        assert gate is not None
        for start in starts:
            assert math.dist(home, gate) + math.dist(gate, start) >= math.dist(home, start) - 1e-6


def test_handing_the_cells_out_differently_never_changes_the_total_distance():
    """Assignment is free: a permutation cannot change the sum, only who pays it.

    Exactly true when every drone would fly a given cell the same distance. In the planner
    the drones differ slightly -- each scans at its own layer, so its lane spacing and path
    length differ -- which is why this is stated over one shared set of cell costs.
    """
    cells = [4420.0, 5214.0, 5322.0]
    costs = [cells[:] for _ in range(3)]
    order = assign_cells(costs, list(FLEET))
    assert sorted(order) == [0, 1, 2]                    # a permutation, nothing dropped
    assert sum(costs[drone][cell] for drone, cell in enumerate(order)) == pytest.approx(sum(cells))
    assert order != (0, 1, 2)                            # and it did move, for this fleet


def test_the_worst_drone_is_better_off_after_assignment():
    """The measured win: -16.9 % on Toa Payoh lawnmower, for zero extra metres."""
    costs = [[4420.0, 5214.0, 5322.0]] * 3
    order = assign_cells(costs, list(FLEET))
    improved = worst_load(costs, list(FLEET), order)
    identity = worst_load(costs, list(FLEET), (0, 1, 2))
    assert improved < identity
    assert improved / identity == pytest.approx(0.831, abs=0.005)


def test_assignment_matches_brute_force():
    import itertools
    rng = random.Random(7)
    for _ in range(25):
        specs = [DroneSpec(rng.uniform(2.0, 3.5), rng.uniform(0.4, 1.0)) for _ in range(4)]
        costs = [[rng.uniform(1000, 6000) for _ in range(4)] for _ in range(4)]
        best = min(itertools.permutations(range(4)), key=lambda o: worst_load(costs, specs, o))
        assert worst_load(costs, specs, assign_cells(costs, specs)) == pytest.approx(
            worst_load(costs, specs, best))


def test_a_gate_is_always_inside_the_fence():
    gate, _ = choose_gate(Point(0.0, 0.0), _entries([Point(2000.0, 2000.0), Point(-2000.0, 1500.0),
                                                     Point(500.0, -3000.0)]),
                          SQUARE, [13.0] * 3, [DroneSpec()] * 3, "arrival")
    assert gate is not None
    assert -800.0 <= gate.x <= 800.0 and -800.0 <= gate.y <= 800.0


def test_planning_twice_gives_exactly_the_same_plan():
    """Guards against set or dict iteration order leaking into a flight plan."""
    mission = _mission("toa-payoh-urban.kml")
    first = build_qgc_plan(MultiAgentPlanner().run(mission, _gated()))
    second = build_qgc_plan(MultiAgentPlanner().run(mission, _gated()))
    assert first == second


# --- the energy model: shape only, never its constants ------------------------------

def test_a_heavier_drone_costs_more_per_metre():
    assert DroneSpec(3.0).energy_per_metre > DroneSpec(2.0).energy_per_metre


def test_a_flatter_battery_makes_the_same_flight_cost_more():
    assert DroneSpec(2.5, 0.5).load(1000) > DroneSpec(2.5, 1.0).load(1000)


def test_a_longer_flight_costs_more():
    assert DroneSpec(2.5).load(2000) > DroneSpec(2.5).load(1000)


def test_reversing_a_route_is_energy_neutral():
    """Reversal swaps the transit out with the return, so the sum cannot move.

    That is what makes entry-end choice a free lever for placing the gate.
    """
    mission = _mission("toa-payoh-urban.kml")
    config = PlannerConfig()
    forward = MultiAgentPlanner().run(mission, config).drones[0]
    local = [project(waypoint, mission.gcs_location) for waypoint in forward.waypoints]
    home = project(mission.gcs_location, mission.gcs_location)
    survey = sum(math.dist(a, b) for a, b in zip(local, local[1:]))
    out_and_back = math.dist(home, local[0]) + survey + math.dist(local[-1], home)
    reversed_way = math.dist(home, local[-1]) + survey + math.dist(local[0], home)
    assert out_and_back == pytest.approx(reversed_way)


# --- end to end ---------------------------------------------------------------------

@pytest.mark.parametrize("sector", SECTORS)
@pytest.mark.parametrize("scan", ["lawnmower", "voronoi"])
def test_gated_plans_still_pass_the_guardrail(sector, scan):
    result = MultiAgentPlanner().run(_mission(sector), _gated(scan_type=scan))
    assert result.guardrail_passed, result.warnings
    flying = [drone for drone in result.drones if drone.waypoints]
    assert all(drone.ingress_gate is not None for drone in flying)
    # One shared point, one transit layer each.
    gates = {(drone.ingress_gate.latitude, drone.ingress_gate.longitude) for drone in flying}
    assert len(gates) == 1
    assert len({drone.ingress_gate.altitude for drone in flying}) == len(flying)


def test_the_arrival_objective_gets_them_there_together():
    mission = _mission("toa-payoh-urban.kml")
    home = project(mission.gcs_location, mission.gcs_location)
    arrivals = []
    for drone in MultiAgentPlanner().run(mission, _gated(ingress_objective="arrival")).drones:
        gate = project(drone.ingress_gate, mission.gcs_location)
        start = project(drone.waypoints[0], mission.gcs_location)
        arrivals.append((math.dist(home, gate) + math.dist(gate, start)) / drone.cruise_speed_m_s)
    assert max(arrivals) - min(arrivals) < 1.0          # against 39 s flying out separately


def test_the_distance_objective_gives_every_drone_the_same_ingress():
    mission = _mission("toa-payoh-urban.kml")
    home = project(mission.gcs_location, mission.gcs_location)
    lengths = []
    for drone in MultiAgentPlanner().run(mission, _gated(ingress_objective="distance")).drones:
        gate = project(drone.ingress_gate, mission.gcs_location)
        start = project(drone.waypoints[0], mission.gcs_location)
        lengths.append(math.dist(home, gate) + math.dist(gate, start))
    assert max(lengths) - min(lengths) < 2.0


def test_the_energy_objective_puts_the_gate_at_home_and_says_so():
    """Correct, and worth a warning: the cheapest gate is no gate at all."""
    result = MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"), _gated(ingress_objective="energy"))
    assert any("splits up where it launched" in warning for warning in result.warnings)


def test_takeoff_aims_at_the_gate_at_the_transit_layer():
    """Takeoff flies the vehicle to its target, so aiming it at the survey would send the
    drone to its own cell before the rendezvous."""
    result = MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"), _gated())
    plan = build_qgc_plan(result)
    drone = next(d for d in result.drones if d.drone_id == 3)
    takeoff = next(item for item in plan["mission"]["items"]
                   if item["command"] == 22 and item["comment"] == "drone-3")
    assert takeoff["params"][4] == pytest.approx(drone.ingress_gate.latitude)
    assert takeoff["params"][5] == pytest.approx(drone.ingress_gate.longitude)
    # The transit layer belongs to the slot the drone was given, not to its number: with
    # energy assignment drone 3 need not be on the third layer.
    assert takeoff["params"][6] == pytest.approx(PlannerConfig().transit_altitude_m(drone.cell_index))


def test_the_default_plan_carries_no_ingress_block_at_all():
    """Defaults must stay byte-for-byte what they were before any of this existed."""
    plan = build_qgc_plan(MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"), PlannerConfig()))
    assert "ingress" not in plan["planner"]
    assert all(drone.ingress_gate is None
               for drone in MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"), PlannerConfig()).drones)


def test_a_single_drone_gets_no_rendezvous():
    result = MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"),
                                     replace(_gated(), number_of_drones=1, drone_specs=FLEET[:1]))
    assert all(drone.ingress_gate is None for drone in result.drones)


def test_no_legal_gate_means_no_gate_rather_than_a_guess():
    """Nowhere legal to meet must fall back to flying out directly, not to an unchecked point."""
    gate, reverse = choose_gate(Point(0.0, 0.0), _entries([Point(100.0, 0.0), Point(0.0, 100.0),
                                                           Point(-100.0, 0.0)]),
                                [], [13.0] * 3, [DroneSpec()] * 3, "arrival")
    assert gate is None
    assert reverse == (False, False, False)


def test_a_gate_that_could_not_be_placed_is_reported():
    """Silence would leave the operator believing the swarm flies in as a formation."""
    drones = [DronePlan(index + 1, [Coordinate(0.0, 0.0)]) for index in range(3)]
    warnings = _ingress_warnings(_mission("toa-payoh-urban.kml"), drones, _gated())
    assert any("no legal one was found" in warning for warning in warnings)


@pytest.mark.parametrize("bad", [
    {"ingress_style": "convoy"},
    {"ingress_objective": "vibes"},
    {"assignment": "random"},
    {"transit_layer_spacing_m": -1.0},
    {"drone_specs": (DroneSpec(2.0),)},          # one spec, three drones
    {"drone_specs": (DroneSpec(0.0),) * 3},      # massless
    {"drone_specs": (DroneSpec(2.0, 0.0),) * 3},  # flat battery
])
def test_nonsense_configuration_is_refused(bad):
    with pytest.raises(ValueError):
        replace(PlannerConfig(), **bad).validate()


def test_the_altitude_layer_travels_with_the_cell():
    """A slot is a cell AND its layer. The lowest layer flies the tightest lanes and so the
    longest survey -- at Toa Payoh 5322 m against 4420 m for the same ground -- so leaving
    the layer pinned to the drone number strands the weakest aircraft on the dearest slot.
    """
    mission = _mission("toa-payoh-urban.kml")
    home = project(mission.gcs_location, mission.gcs_location)
    plain = MultiAgentPlanner().run(mission, PlannerConfig(drone_specs=FLEET))
    sorted_out = MultiAgentPlanner().run(mission, PlannerConfig(assignment="energy", drone_specs=FLEET))

    def distances(result):
        return [_flight_distance(home, _local(drone.waypoints, mission.gcs_location))
                for drone in result.drones]

    # Same work, in a different pair of hands: the total cannot move.
    assert sum(distances(sorted_out)) == pytest.approx(sum(distances(plain)))
    assert sorted([d.altitude_m for d in sorted_out.drones]) == sorted([d.altitude_m for d in plain.drones])
    # ...and the drone that was worst off is better off.
    def worst(result):
        return max(FLEET[index].load(value) for index, value in enumerate(distances(result)) if value)
    assert worst(sorted_out) < worst(plain)


# --- the patches of 16 Sep: layer clashes, resolution, the gate floor, coverage ---------

def test_two_ladders_of_different_spacing_clash_once_the_fleet_is_big_enough():
    """8 drones on 5 m transit layers puts one drone's transit at 25 m and another's survey
    at 26 m, and the ingress crosses the survey area. Caught at configuration time, with the
    numbers in the message, rather than as a wall of separation failures at the guardrail.
    """
    small = replace(PlannerConfig(), ingress_style="gate", number_of_drones=3,
                    transit_layer_spacing_m=5.0)
    small.validate()                                   # 3 drones fit: 25 / 30 / 35 clears 28 / 30 / 32
    assert small.layer_clash is None

    big = replace(small, number_of_drones=8, drone_specs=())
    assert big.layer_clash is not None
    with pytest.raises(ValueError, match="transit_layer_spacing_m"):
        big.validate()
    # The same fleet is fine once both ladders share a spacing.
    replace(big, transit_layer_spacing_m=2.0).validate()


def test_a_clash_only_matters_when_the_swarm_actually_groups_up():
    """Without a gate there is no transit layer to clash with."""
    replace(PlannerConfig(), number_of_drones=8, transit_layer_spacing_m=5.0).validate()


def test_a_tall_stack_warns_that_the_top_layers_lose_resolution():
    warnings = replace(PlannerConfig(), number_of_drones=8).imaging_warnings
    assert warnings and "3 cm/px" in warnings[0]
    assert "38" in warnings[0]                         # the top layer, 3.30 cm/px
    assert not PlannerConfig().imaging_warnings        # three drones stay inside it


def test_the_gate_floor_buys_a_formation_without_costing_fairness():
    """Without a floor the cost tie-break drags the gate home: fair, but nobody flies together."""
    mission = _mission("toa-payoh-urban.kml")
    home = project(mission.gcs_location, mission.gcs_location)

    def gate_distance(floor):
        result = MultiAgentPlanner().run(mission, _gated(ingress_min_gate_m=floor))
        gate = result.drones[0].ingress_gate
        return math.dist(home, project(gate, mission.gcs_location)) if gate else None

    assert gate_distance(300.0) >= 300.0


def test_nowhere_far_enough_out_falls_back_to_flying_direct():
    result = MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"), _gated(ingress_min_gate_m=50_000.0))
    assert all(drone.ingress_gate is None for drone in result.drones)
    assert any("no legal one was found" in warning for warning in result.warnings)


def test_the_drop_to_the_survey_layer_waits_until_the_swarm_has_opened_up():
    """Descending at the gate would stack every altitude change where they are all together."""
    result = MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"), _gated(ingress_min_gate_m=300.0))
    drone = next(d for d in result.drones if d.ingress_descent is not None)
    assert drone.ingress_descent.altitude == pytest.approx(drone.ingress_gate.altitude)
    assert drone.ingress_descent.altitude != pytest.approx(drone.altitude_m)
    # ...and it is a real waypoint, between the takeoff and the survey.
    plan = build_qgc_plan(result)
    tag = f"drone-{drone.drone_id}"
    commands = [item["command"] for item in plan["mission"]["items"] if item.get("comment") == tag]
    assert commands[:3] == [178, 22, 16]               # speed, takeoff at the gate, then the descent point


def test_separation_is_judged_leg_by_leg_not_over_the_whole_sortie():
    """A drone transits on one layer and surveys on another; banding the two together says it
    could be at any altitude in between, anywhere along the flight, which is false."""
    mission = _mission("toa-payoh-urban.kml")
    home = project(mission.gcs_location, mission.gcs_location)
    result = MultiAgentPlanner().run(mission, _gated(ingress_min_gate_m=300.0))
    drone = result.drones[0]
    legs = _flight_legs(drone, [project(w, mission.gcs_location) for w in drone.waypoints],
                        home, mission.gcs_location)
    assert legs[0][2] == pytest.approx(drone.ingress_gate.altitude)      # out to the gate, transit layer
    assert legs[-1][2] == pytest.approx(drone.altitude_m)                # home again, survey layer
    assert {leg[2] for leg in legs} >= {drone.ingress_gate.altitude, drone.altitude_m}


def test_coverage_is_measured_and_reported():
    """The urban sector leaves slivers of its own edge unimaged; the flown sector does not."""
    urban = MultiAgentPlanner().run(_mission("safti-urban.kml"), PlannerConfig())
    assert any(w.startswith("coverage:") for w in urban.warnings)
    flown = MultiAgentPlanner().run(_mission("toa-payoh-urban.kml"), PlannerConfig())
    assert not [w for w in flown.warnings if w.startswith("coverage:")]
