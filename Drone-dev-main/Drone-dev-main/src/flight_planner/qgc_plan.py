"""QGroundControl .plan writer.

Every mission is a complete flight — NAV_TAKEOFF, the lawnmower, NAV_RETURN_TO_LAUNCH
— and its cruise speed comes from the camera model rather than a constant. Both of
those, and the frame-2 requirement on the RTL item, are carried over from drone-dev's
`swaths_to_plan.py`, where they were established against PX4 SITL.
"""

from __future__ import annotations

import copy
from dataclasses import replace
from typing import Any

from .models import Coordinate, MissionInput, PlanningResult

NAV_WAYPOINT = 16
NAV_TAKEOFF = 22
NAV_RETURN_TO_LAUNCH = 20
DO_CHANGE_SPEED = 178

FRAME_GLOBAL_RELATIVE_ALT = 3
FRAME_MISSION = 2


def _simple_item(sequence: int, command: int, coordinate: Coordinate) -> dict[str, Any]:
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": coordinate.altitude,
        "AltitudeMode": 1,                      # relative to home
        "autoContinue": True,
        "command": command,
        "doJumpId": sequence,
        "frame": FRAME_GLOBAL_RELATIVE_ALT,
        # param4 is yaw: null leaves it to MPC_YAW_MODE (see sar_turn_params.params).
        "params": [0.0, 0.0, 0.0, None, coordinate.latitude, coordinate.longitude, coordinate.altitude],
        "type": "SimpleItem",
    }


def _return_to_launch_item(sequence: int) -> dict[str, Any]:
    """RTL must be frame 2 (MAV_FRAME_MISSION).

    On frame 3 PX4 rejects the whole mission with "Command not supported" /
    "IGN MISSION_ITEM: Invalid item" — found the hard way in drone-dev.
    """
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": 0,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": NAV_RETURN_TO_LAUNCH,
        "doJumpId": sequence,
        "frame": FRAME_MISSION,
        "params": [0, 0, 0, 0, 0, 0, 0],
        "type": "SimpleItem",
    }


def _change_speed_item(sequence: int, speed_m_s: float) -> dict[str, Any]:
    """Push the blur-safe cruise speed onto the vehicle.

    The `cruiseSpeed` field of a .plan is a ground-station planning value used for time
    estimates -- it never reaches the aircraft, which otherwise flies MPC_XY_CRUISE
    (5 m/s by default). Measured on the Pulau Ubin flight: the plan asked for 13.0 m/s
    and the vehicle held 5.00 m/s for the whole survey.

    PX4 reads the speed from param2 and ignores param1's speed type
    ("XXX not differentiating ground and airspeed yet", navigator_main.cpp), and it
    caches this item and replays it after a mission resume, so one item after takeoff
    governs the whole flight.
    """
    return {
        "AMSLAltAboveTerrain": None,
        "Altitude": 0,
        "AltitudeMode": 1,
        "autoContinue": True,
        "command": DO_CHANGE_SPEED,
        "doJumpId": sequence,
        "frame": FRAME_MISSION,                 # no position of its own
        # param1 = 1 (ground speed), param2 = speed m/s, param3 = -1 (throttle unchanged)
        "params": [1, round(speed_m_s, 1), -1, 0, 0, 0, 0],
        "type": "SimpleItem",
    }


def _waypoint_item(coordinate: Coordinate, sequence: int) -> dict[str, Any]:
    item = _simple_item(sequence, NAV_WAYPOINT, coordinate)
    # param2 = acceptance radius. 0 leaves the corner to NAV_ACC_RAD; a corner that must
    # be flown tight carries its own, which PX4 honours for multicopters both for
    # "reached" and for how fast it rounds the corner (mission_block.cpp).
    item["params"][1] = getattr(coordinate, "acceptance_radius_m", 0.0)
    # param4 = heading to hold on the way to this waypoint (turn_style "pre_yaw"). None
    # leaves it to MPC_YAW_MODE. A heading also makes PX4 wait at the waypoint until the
    # nose is within MIS_YAW_ERR of it (mission_block.cpp).
    yaw = getattr(coordinate, "yaw_deg", None)
    if yaw is not None:
        item["params"][3] = round(yaw, 2)
    return item


def _planned_home_position(coordinate: Coordinate) -> list[float]:
    return [coordinate.latitude, coordinate.longitude, coordinate.altitude]


def _mission_items(result: PlanningResult) -> list[dict[str, Any]]:
    """Takeoff -> lawnmower -> RTL for each drone, every item tagged with its drone.

    The tag is what `split_plan_by_drone` cuts on, so takeoff and RTL carry it too and
    each per-drone file comes out as a complete, self-contained mission.
    """
    items: list[dict[str, Any]] = []
    sequence = 1
    for drone in result.drones:
        if not drone.waypoints:
            continue
        tag = f"drone-{drone.drone_id}"
        # Speed first, BEFORE takeoff. NAV_TAKEOFF flies the vehicle all the way to the
        # first waypoint, so a speed item placed after it leaves that transit at PX4's
        # default. Measured on drone-3 at Pulau Ubin: 155 s of a 443 s mission crawling
        # to the survey area at 5 m/s.
        # Each drone's own layer sets its blur-safe speed; hand-built plans fall back to the config's.
        speed = drone.cruise_speed_m_s if drone.cruise_speed_m_s is not None else result.config.cruise_speed_m_s
        items.append({**_change_speed_item(sequence, speed), "comment": tag})
        sequence += 1
        # With a dispersal gate the climb is aimed at the gate, not at the survey: takeoff
        # flies the vehicle all the way to its target, so aiming it at waypoint 0 would send
        # the drone to its own cell first and only then back out to the rendezvous. The
        # gate carries the transit layer, so the descent to the survey layer happens on the
        # leg after it -- once the swarm has already split.
        takeoff_target = drone.ingress_gate if drone.ingress_gate is not None else drone.waypoints[0]
        items.append({**_simple_item(sequence, NAV_TAKEOFF, takeoff_target), "comment": tag})
        sequence += 1
        if drone.ingress_descent is not None:
            # Still at the transit layer, but clear of the gate: the descent to the survey
            # layer happens on the leg after this one, once the swarm has opened up.
            items.append({**_waypoint_item(drone.ingress_descent, sequence), "comment": tag})
            sequence += 1
        for waypoint in drone.waypoints:
            items.append({**_waypoint_item(waypoint, sequence), "comment": tag})
            sequence += 1
        items.append({**_return_to_launch_item(sequence), "comment": tag})
        sequence += 1
    return items


def _layers(result: PlanningResult) -> list[dict[str, Any]]:
    """Each flying drone's altitude layer and the imaging numbers that layer gives it."""
    layers = []
    for drone in result.drones:
        if not drone.waypoints:
            continue
        altitude = drone.altitude_m if drone.altitude_m is not None else result.config.scan_altitude_m
        config = replace(result.config, scan_altitude_m=altitude)
        profile = config.profile
        layers.append({
            "droneId": drone.drone_id,
            "altitude_m": round(altitude, 1),
            "lane_spacing_m": round(profile.line_spacing_m, 1),
            "gsd_cm_px": round(profile.gsd_cm_px, 2),
            "cruise_speed_m_s": config.cruise_speed_m_s,
        })
    return layers


def _ingress(result: PlanningResult) -> dict[str, Any] | None:
    """Where the swarm splits up, and who ended up flying which cell.

    Recorded so a plan can be read back without re-running the planner -- which cell a
    drone was given is no longer simply its own index once assignment is by energy.
    """
    flying = [drone for drone in result.drones if drone.waypoints]
    gates = [drone.ingress_gate for drone in flying if drone.ingress_gate is not None]
    # Default ingress says nothing a reader cannot already infer (cell N is drone N), so it
    # is left out entirely -- a plan built with the defaults stays byte-for-byte what it was
    # before any of this existed.
    if not flying or (result.config.ingress_style == "direct" and result.config.assignment == "index"):
        return None
    block: dict[str, Any] = {
        "style": result.config.ingress_style,
        "assignment": result.config.assignment,
        "drones": [
            {"droneId": drone.drone_id, "cell": drone.cell_index,
             "transit_altitude_m": round(drone.ingress_gate.altitude, 1) if drone.ingress_gate else None}
            for drone in flying
        ],
    }
    if gates:
        block["objective"] = result.config.ingress_objective
        block["gate"] = {"latitude": gates[0].latitude, "longitude": gates[0].longitude}
    return block


def build_qgc_plan(result: PlanningResult, planner_name: str = "multi-agent-flight-planner") -> dict[str, Any]:
    fence = [coordinate.as_qgc()[:2][::-1] for coordinate in result.mission.geo_fence.coordinates]
    profile = result.config.profile
    return {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "version": 1,
        "geoFence": {"circles": [], "polygons": [{"inclusion": True, "polygon": fence}], "version": 2},
        "rallyPoints": {"points": [], "version": 2},
        "mission": {
            "cruiseSpeed": result.config.cruise_speed_m_s,
            "firmwareType": 12,                 # PX4
            "globalPlanAltitudeMode": 1,
            "hoverSpeed": 5,
            "vehicleType": 2,                   # multirotor
            "items": _mission_items(result),
            "plannedHomePosition": _planned_home_position(result.mission.gcs_location),
            "version": 2,
        },
        "planner": {
            "name": planner_name,
            "guardrailPassed": result.guardrail_passed,
            "warnings": result.warnings,
            # Recorded so a plan can be traced back to the altitude it came from.
            "imaging": {
                "altitude_m": round(profile.altitude_m, 1),
                "lane_spacing_m": round(profile.line_spacing_m, 1),
                "gsd_cm_px": round(profile.gsd_cm_px, 2),
                "cruise_speed_m_s": result.config.cruise_speed_m_s,
            },
            "layers": _layers(result),
            **({"ingress": ingress} if (ingress := _ingress(result)) else {}),
        },
        # droneCount is what every consumer assumes: the number of drone-N.plan files that
        # split_plan_by_drone will produce. Drones whose cell came out empty contribute no
        # items and so no file -- counting them here would advertise plans that do not
        # exist. The requested figure is kept so the discrepancy stays visible rather than
        # being quietly rounded away; _missing_waypoint_warnings names which drones missed out.
        "x-multiDrone": {
            "droneCount": len([drone for drone in result.drones if drone.waypoints]),
            "requestedDroneCount": len(result.drones),
            "assignment": "comment field on each mission item",
        },
    }


def split_plan_by_drone(plan: dict[str, Any]) -> dict[int, dict[str, Any]]:
    """Split a combined multi-drone Plan into one single-vehicle Plan per drone, keyed by drone id.

    Each drone's file reuses the shared geoFence, rallyPoints, and plannedHomePosition, and
    renumbers doJumpId locally so it can be uploaded to QGroundControl as its own mission.
    """
    items_by_drone: dict[int, list[dict[str, Any]]] = {}
    for item in plan.get("mission", {}).get("items", []):
        comment = item.get("comment", "")
        if not comment.startswith("drone-"):
            continue
        drone_id = int(comment.split("-", 1)[1])
        items_by_drone.setdefault(drone_id, []).append(item)

    plans: dict[int, dict[str, Any]] = {}
    for drone_id, items in items_by_drone.items():
        drone_plan = copy.deepcopy(plan)
        drone_plan["mission"]["items"] = [{**item, "doJumpId": sequence} for sequence, item in enumerate(items, start=1)]
        drone_plan["planner"] = {**plan.get("planner", {}), "droneId": drone_id}
        drone_plan.pop("x-multiDrone", None)
        plans[drone_id] = drone_plan
    return plans


def build_baseline_plan(mission: MissionInput) -> dict[str, Any]:
    """Create a minimal single-vehicle QGC plan from the KML home and fence."""
    empty_result = PlanningResult(mission, [], True)
    plan = build_qgc_plan(empty_result, "kml-baseline")
    plan["mission"]["plannedHomePosition"] = _planned_home_position(mission.gcs_location)
    return plan
