from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from itertools import combinations

from .energy import assign_cells
from .geometry import (
    Point,
    point_in_polygon,
    point_within,
    project,
    rounded_corner,
    segment_distance,
    segment_in_polygon,
    segment_within,
    unproject,
)
from .ingress import CellEntry, choose_gate
from .models import (
    CORNER_ACCEPTANCE_RADIUS_M,
    MIN_HORIZONTAL_SEPARATION_M,
    MIN_VERTICAL_SEPARATION_M,
    DronePlan,
    MissionInput,
    PlanningResult,
    PlannerConfig,
)
from .scanner_tools import partition_cells, scan_cell


# A fan-out leg shorter than this keeps its altitude change on the leg out of the gate:
# there is no room to hold the transit layer and still descend gently.
MIN_DESCENT_RUN_M = 150.0


def _local(waypoints: list, origin) -> list[Point]:
    return [project(waypoint, origin) for waypoint in waypoints]


def _path_length(local: list[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(local, local[1:]))


def _flight_distance(home: Point, local: list[Point]) -> float:
    """Everything one drone flies on a direct sortie: out, survey, and back."""
    if not local:
        return 0.0
    return math.dist(home, local[0]) + _path_length(local) + math.dist(local[-1], home)


@dataclass
class FlightPlannerAgent:
    """Partitions the OpsArea, decides who flies what, and generates the scan waypoints.

    Three decisions, in the order they have to be made:

    1. **Cut the OpsArea into cells** -- geometry only, independent of which drone flies which.
    2. **Hand the work out.** A slot is a cell *and* the altitude layer it is flown at --
       the two travel together, because the layer sets the lane spacing and so the length
       of the survey. The lowest layer flies the tightest lanes and is therefore the most
       expensive slot: at Toa Payoh, 5322 m against 4420 m for the same area, a fifth more
       flying. With `assignment: "energy"` the slots go to whichever drone can best afford
       them, so the worst-off aircraft keeps the most margin. This is free -- a permutation
       changes who pays, not the total -- and it is why the layer has to be part of the
       slot: leaving it pinned to the drone index strands a heavy, half-charged aircraft on
       the most expensive layer, which is exactly the case worth fixing. Costs are priced
       on a direct sortie; a gate then adds a leg common to everyone.
    3. **Place the dispersal gate**, if the swarm is to fly in as a group, along with which
       end each drone enters its cell from (see ingress.py).
    """

    def plan(self, mission: MissionInput, config: PlannerConfig) -> list[DronePlan]:
        origin = mission.gcs_location
        home = project(origin, origin)
        cells, polygon = partition_cells(mission.ops_area, origin, config)
        count = len(cells)
        specs = [config.spec_for(index) for index in range(count)]
        cache: dict[tuple[int, int, bool], list] = {}

        def route(slot: int, reverse: bool = False) -> list:
            """Slot `slot`: its cell, scanned at its own altitude layer."""
            key = (slot, reverse)
            if key not in cache:
                cache[key] = scan_cell(cells[slot], polygon, origin, config, slot, reverse)
            return cache[key]

        order = tuple(range(count))
        if config.assignment == "energy" and count > 1 and count == config.number_of_drones:
            # A slot costs what it costs, whoever flies it -- cell and layer travel together --
            # so every drone prices the same row and only the aircraft differ.
            slot_costs = [_flight_distance(home, _local(route(slot), origin)) for slot in range(count)]
            order = assign_cells([slot_costs[:] for _ in range(count)], specs)

        reverse = tuple(False for _ in range(count))
        gate = None
        flying = [drone for drone in range(count) if route(order[drone])]
        if config.ingress_style == "gate" and len(flying) > 1:
            fence = [project(point, origin) for point in mission.geo_fence.coordinates]
            entries, speeds = [], []
            for drone in flying:
                local = _local(route(order[drone]), origin)
                entries.append(CellEntry((local[0], local[-1]), _path_length(local)))
                speeds.append(config.for_drone(order[drone]).cruise_speed_m_s)
            gate, chosen = choose_gate(home, entries, fence, speeds, [specs[d] for d in flying],
                                       config.ingress_objective, min_gate_m=config.ingress_min_gate_m)
            if gate is not None:
                picked = dict(zip(flying, chosen))
                reverse = tuple(picked.get(drone, False) for drone in range(count))

        plans = []
        for drone, slot in enumerate(order):
            waypoints = route(slot, reverse[drone])
            # One shared point; each drone holds the transit layer belonging to its own slot.
            transit_altitude = config.transit_altitude_m(slot)
            gate_coordinate = descent_coordinate = None
            if gate is not None and waypoints:
                gate_coordinate = unproject(gate, origin, transit_altitude)
                first = project(waypoints[0], origin)
                # Hold the transit layer until the fan-out has opened the drones up, then
                # descend. Descending at the gate itself would stack every altitude change in
                # the one place they are all on top of each other.
                if math.dist(gate, first) > MIN_DESCENT_RUN_M:
                    midpoint = Point((gate.x + first.x) / 2, (gate.y + first.y) / 2)
                    descent_coordinate = unproject(midpoint, origin, transit_altitude)
            plans.append(DronePlan(drone + 1, waypoints, config.drone_altitude_m(slot),
                                   config.for_drone(slot).cruise_speed_m_s, slot, gate_coordinate,
                                   descent_coordinate))
        return plans


@dataclass
class GuardrailPlannerAgent:
    """Checks the whole flown path against the KML GeoFence.

    "Whole" means every survey waypoint, every leg between them, AND the two legs the
    planner does not choose but the vehicle still flies: the takeoff transit out to the
    first waypoint, and the RTL return from the last. Those are not decorative -- on the
    Pulau Ubin sortie they were 269, 585 and 664 m, a third of the shortest drone's
    flight, and until 09 Sep 2026 nothing validated them.

    The survey itself is also held to the OpsArea. A notch cut out of it -- a road, a
    rail corridor -- is where the search must not go, and until 10 Sep 2026 the lanes
    were cut from the OpsArea but never checked against it: 1208 m of the Toa Payoh
    survey flew straight across the excluded corridor with the guardrail reporting a
    pass. The two transits are exempt: home is normally outside the OpsArea, so the legs
    out to the survey and back from it cross its boundary by design.

    Two more checks since 11 Sep 2026. Corners are checked as flown, not as drawn: PX4
    rounds each one, and the rounded turn must stay inside the OpsArea too. And no two
    drones may be able to meet: drones on altitude layers at least
    MIN_VERTICAL_SEPARATION_M apart are separated; drones closer than that must keep
    MIN_HORIZONTAL_SEPARATION_M along their whole flown paths, whatever the timing.
    Before layering, Pulau Ubin drones 1 and 2 flew edge lanes 0 m apart at one altitude.
    """

    def validate(self, mission: MissionInput, drones: list[DronePlan]) -> tuple[bool, list[str]]:
        fence = [project(point, mission.gcs_location) for point in mission.geo_fence.coordinates]
        ops_area = [project(point, mission.gcs_location) for point in mission.ops_area.coordinates]
        home = project(mission.gcs_location, mission.gcs_location)
        violations = []

        # Checked once, and named for what it is. Every drone launches from and returns to
        # this point, so a fence excluding it fails the sortie, not one aircraft -- and
        # reporting it as 2N leg failures would bury the actual cause.
        home_inside = point_in_polygon(home, fence)
        if not home_inside:
            violations.append("GCS location is outside the GeoFence")

        tracks = []
        for drone in drones:
            local = [project(waypoint, mission.gcs_location) for waypoint in drone.waypoints]
            for waypoint_index, waypoint in enumerate(local):
                if not point_in_polygon(waypoint, fence):
                    violations.append(f"drone {drone.drone_id} waypoint {waypoint_index} is outside GeoFence")
            # Corners alone are not enough: across a concave fence the straight line
            # between two in-fence waypoints can still leave the fence and come back.
            for leg_index, (start, end) in enumerate(zip(local, local[1:])):
                if not segment_in_polygon(start, end, fence):
                    violations.append(
                        f"drone {drone.drone_id} leg {leg_index}->{leg_index + 1} leaves GeoFence"
                    )
            # Lanes end ON the OpsArea boundary, so on-boundary counts as inside here.
            for waypoint_index, waypoint in enumerate(local):
                if not point_within(waypoint, ops_area):
                    violations.append(f"drone {drone.drone_id} waypoint {waypoint_index} is outside the OpsArea")
            for leg_index, (start, end) in enumerate(zip(local, local[1:])):
                if not segment_within(start, end, ops_area):
                    violations.append(
                        f"drone {drone.drone_id} leg {leg_index}->{leg_index + 1} leaves the OpsArea"
                    )
            # ...and so must the corners as actually flown: PX4 starts each turn up to the
            # waypoint's acceptance radius out and rounds it.
            for index in range(1, len(local) - 1):
                radius = getattr(drone.waypoints[index], "acceptance_radius_m", 0.0) or CORNER_ACCEPTANCE_RADIUS_M
                arc = rounded_corner(local[index - 1], local[index], local[index + 1], radius)
                if not all(segment_within(a, b, ops_area) for a, b in zip(arc, arc[1:])):
                    violations.append(f"drone {drone.drone_id} corner at waypoint {index} is rounded outside the OpsArea")
            gate = project(drone.ingress_gate, mission.gcs_location) if drone.ingress_gate else None
            if local:
                tracks.append((drone.drone_id, _flight_legs(drone, local, home, mission.gcs_location)))
            if gate is not None and not point_in_polygon(gate, fence):
                violations.append(f"drone {drone.drone_id} dispersal gate is outside GeoFence")
            if local and home_inside:
                if not segment_in_polygon(home, gate if gate is not None else local[0], fence):
                    violations.append(
                        f"drone {drone.drone_id} takeoff transit (home -> "
                        f"{'gate' if gate is not None else 'waypoint 0'}) leaves GeoFence"
                    )
                if gate is not None and not segment_in_polygon(gate, local[0], fence):
                    violations.append(
                        f"drone {drone.drone_id} ingress (gate -> waypoint 0) leaves GeoFence"
                    )
                if not segment_in_polygon(local[-1], home, fence):
                    violations.append(
                        f"drone {drone.drone_id} RTL return (waypoint {len(local) - 1} -> home) "
                        "leaves GeoFence"
                    )
        violations += _separation_violations(tracks)
        return not violations, violations


def _flight_legs(drone: DronePlan, local: list[Point], home: Point,
                 origin) -> list[tuple[Point, Point, float, float]]:
    """Every leg the drone flies, each with the altitude band it is flown at.

    Legs, not one band per flight. A drone transits on one layer and surveys on another, and
    banding the whole sortie together says it might be at any altitude in between anywhere
    along it. That is false, and expensively so: with 8 drones the 5 m transit ladder and the
    2 m survey ladder interleave, every band overlaps its neighbour, and since all of them
    pass through the one gate the guardrail refuses a plan that is actually separated.
    """
    survey = drone.altitude_m if drone.altitude_m is not None else drone.waypoints[0].altitude
    legs: list[tuple[Point, Point, float, float]] = []
    if drone.ingress_gate is not None:
        transit = drone.ingress_gate.altitude
        gate = project(drone.ingress_gate, origin)
        legs.append((home, gate, transit, transit))
        if drone.ingress_descent is not None:
            descent = project(drone.ingress_descent, origin)
            legs.append((gate, descent, transit, transit))
            legs.append((descent, local[0], min(transit, survey), max(transit, survey)))
        else:
            legs.append((gate, local[0], min(transit, survey), max(transit, survey)))
    else:
        legs.append((home, local[0], survey, survey))
    legs += [(a, b, survey, survey) for a, b in zip(local, local[1:])]
    legs.append((local[-1], home, survey, survey))
    return legs


def _separation_violations(tracks: list[tuple[int, list[tuple[Point, Point, float, float]]]]) -> list[str]:
    """Pairs of drones that could meet: two legs whose altitudes are close AND which pass
    within reach of each other.

    Checked leg against leg over the whole flight -- transit out, survey and RTL -- and
    regardless of timing, because timing drifts. A shared launch point is a meeting point by
    definition, so two drones on one layer from one home always fail: that is exactly the
    case altitude layering exists to prevent.
    """
    violations = []
    for (id_a, legs_a), (id_b, legs_b) in combinations(tracks, 2):
        worst = None
        for a1, a2, low_a, high_a in legs_a:
            for b1, b2, low_b, high_b in legs_b:
                vertical = max(low_b - high_a, low_a - high_b)
                if vertical >= MIN_VERTICAL_SEPARATION_M - 1e-6:
                    continue
                horizontal = segment_distance(a1, a2, b1, b2)
                if horizontal < MIN_HORIZONTAL_SEPARATION_M and (worst is None or horizontal < worst[0]):
                    worst = (horizontal, max(vertical, 0.0))
        if worst is not None:
            violations.append(
                f"drones {id_a} and {id_b} are not separated: {worst[1]:.1f} m apart vertically "
                f"and their paths come within {worst[0]:.1f} m (need {MIN_VERTICAL_SEPARATION_M:.0f} m "
                f"vertical or {MIN_HORIZONTAL_SEPARATION_M:.0f} m horizontal)"
            )
    return violations


def _shared_home_warnings(drones: list[DronePlan]) -> list[str]:
    flying = [drone for drone in drones if drone.waypoints]
    if len(flying) < 2:
        return []
    return [
        f"{len(flying)} drones launch from and land at one GCS point: their altitude layers separate "
        "them in the air, but climbs and descents at home cross every lower layer -- use separate "
        "pads and stagger launch and landing"
    ]


# A gate this close to home is not a rendezvous: the swarm splits where it launched.
GATE_AT_HOME_M = 50.0


def _ingress_warnings(mission: MissionInput, drones: list[DronePlan], config: PlannerConfig) -> list[str]:
    """Say so when a requested gate did not happen, or happened somewhere useless."""
    if config.ingress_style != "gate":
        return []
    flying = [drone for drone in drones if drone.waypoints]
    gates = [drone.ingress_gate for drone in flying if drone.ingress_gate is not None]
    if len(flying) < 2:
        return []
    if not gates:
        return ["ingress_style 'gate' asked for a dispersal point but no legal one was found "
                "inside the GeoFence; the swarm flies out directly instead"]
    home = project(mission.gcs_location, mission.gcs_location)
    distance = math.dist(home, project(gates[0], mission.gcs_location))
    if distance < GATE_AT_HOME_M:
        return [
            f"the dispersal gate came out {distance:.0f} m from home, so the swarm splits up "
            "where it launched and never flies as a formation. With ingress_objective "
            "'energy' this is the correct answer -- any gate costs every drone distance -- "
            "so use 'arrival' or 'distance' if a formation is the point"
        ]
    return []


# The OpsArea edge is walked at this pitch to check the scan actually reaches it.
COVERAGE_SAMPLE_M = 8.0
# Slivers below this are not worth a warning: they are inside the tracking error we already
# fly with (measured cross-track mean 0.13-0.46 m, max 4.95 m).
COVERAGE_TOLERANCE_M = 1.0


def _boundary_samples(polygon: list[Point], pitch: float) -> list[Point]:
    samples = []
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        length = math.dist(start, end)
        for step in range(max(1, int(length / pitch))):
            fraction = step / max(1, int(length / pitch))
            samples.append(Point(start.x + fraction * (end.x - start.x),
                                 start.y + fraction * (end.y - start.y)))
    return samples


def _point_to_segment(point: Point, start: Point, end: Point) -> float:
    dx, dy = end.x - start.x, end.y - start.y
    length = dx * dx + dy * dy
    if length == 0:
        return math.dist(point, start)
    along = max(0.0, min(1.0, ((point.x - start.x) * dx + (point.y - start.y) * dy) / length))
    return math.hypot(point.x - start.x - along * dx, point.y - start.y - along * dy)


def _coverage_warnings(mission: MissionInput, drones: list[DronePlan], config: PlannerConfig) -> list[str]:
    """Does the scan actually reach the edges of the area it was asked to search?

    Lanes are centred inside each cell, so the outermost swath can stop short of a boundary,
    and a concave corner can fall between two cells' lanes. Measured on the SAFTI urban
    sector (16 Sep 2026): up to 4.5 m of the edge lay outside every camera swath. The
    interior is covered by construction -- lane pitch is narrower than the swath -- so only
    the boundary is walked, which is also what makes this cheap enough to run on every plan.
    """
    ops_area = [project(point, mission.gcs_location) for point in mission.ops_area.coordinates]
    if len(ops_area) < 3:
        return []
    legs: list[tuple[Point, Point, float]] = []
    for drone in drones:
        if not drone.waypoints:
            continue
        altitude = drone.altitude_m if drone.altitude_m is not None else config.scan_altitude_m
        half_swath = replace(config, scan_altitude_m=altitude).profile.swath_w_m / 2
        local = [project(waypoint, mission.gcs_location) for waypoint in drone.waypoints]
        legs += [(a, b, half_swath) for a, b in zip(local, local[1:])]
    if not legs:
        return []

    # Bucket the legs so each sample only measures against what is near it.
    cell_size = max(half for _, _, half in legs) * 2
    index: dict[tuple[int, int], list[int]] = {}
    for position, (start, end, half) in enumerate(legs):
        lo_x, hi_x = sorted((start.x, end.x)); lo_y, hi_y = sorted((start.y, end.y))
        for cx in range(int((lo_x - half) // cell_size), int((hi_x + half) // cell_size) + 1):
            for cy in range(int((lo_y - half) // cell_size), int((hi_y + half) // cell_size) + 1):
                index.setdefault((cx, cy), []).append(position)

    worst, uncovered, samples = 0.0, 0, _boundary_samples(ops_area, COVERAGE_SAMPLE_M)
    for sample in samples:
        nearby = index.get((int(sample.x // cell_size), int(sample.y // cell_size)))
        candidates = (legs[position] for position in nearby) if nearby else legs
        gap = min(_point_to_segment(sample, start, end) - half for start, end, half in candidates)
        if gap > COVERAGE_TOLERANCE_M:
            uncovered += 1
            worst = max(worst, gap)
    if not uncovered:
        return []
    return [
        f"coverage: {uncovered} of {len(samples)} points along the OpsArea edge "
        f"({100 * uncovered / len(samples):.1f} %) fall outside every camera swath, the worst by "
        f"{worst:.1f} m -- lanes are centred in each cell, so the outermost pass can stop short. "
        f"Raise scan_overlap_percent or lower scan_altitude_m if the edges matter"
    ]


def _missing_waypoint_warnings(drones: list[DronePlan]) -> list[str]:
    return [f"drone {drone.drone_id} received no waypoints for this OpsArea/scan_type" for drone in drones if not drone.waypoints]


@dataclass
class MultiAgentPlanner:
    # default_factory, not a shared instance: a mutable dataclass default is a
    # ValueError at class-creation time on Python >= 3.11.
    flight_planner: FlightPlannerAgent = field(default_factory=FlightPlannerAgent)
    guardrail_planner: GuardrailPlannerAgent = field(default_factory=GuardrailPlannerAgent)

    def run(self, mission: MissionInput, config: PlannerConfig) -> PlanningResult:
        drones = self.flight_planner.plan(mission, config)
        passed, warnings = self.guardrail_planner.validate(mission, drones)
        warnings = (warnings + _missing_waypoint_warnings(drones) + _shared_home_warnings(drones)
                    + _ingress_warnings(mission, drones, config)
                    + config.speed_warnings + config.imaging_warnings
                    + _coverage_warnings(mission, drones, config))
        return PlanningResult(mission, drones, passed, warnings, config)
