"""The GeoFence guardrail: waypoints, the legs between them, and the legs the vehicle
flies but the planner never chose -- the takeoff transit and the RTL return."""

from pathlib import Path

from flight_planner.agents import GuardrailPlannerAgent, MultiAgentPlanner
from flight_planner.config import load_config
from flight_planner.kml_parser import parse_kml
from flight_planner.models import Coordinate, DronePlan, MissionInput, MissionRegion

ROOT = Path(__file__).parents[1]

# A C-shaped fence: two arms either side of a notch that is outside the fence.
#
#   0.010  +----+    +----+
#          |    |    |    |
#          | L  |    | R  |     <- the notch between the arms is OUTSIDE
#   0.003  |    +----+    |
#          |              |
#   0.000  +--------------+
#          0   .003  .007  .010
CONCAVE_FENCE = MissionRegion(
    "GeoFence",
    [
        Coordinate(0.000, 0.000),
        Coordinate(0.010, 0.000),
        Coordinate(0.010, 0.010),
        Coordinate(0.007, 0.010),
        Coordinate(0.007, 0.003),
        Coordinate(0.003, 0.003),
        Coordinate(0.003, 0.010),
        Coordinate(0.000, 0.010),
    ],
)

LEFT_ARM_LOW = Coordinate(0.001, 0.001)     # in the base, below the notch
LEFT_ARM_TOP = Coordinate(0.001, 0.008)
RIGHT_ARM_TOP = Coordinate(0.009, 0.008)


def _mission(home: Coordinate) -> MissionInput:
    return MissionInput(home, CONCAVE_FENCE, CONCAVE_FENCE)


def _containment_violations(violations: list[str]) -> list[str]:
    return [v for v in violations if "is outside GeoFence" in v]


# --- waypoints and survey legs -----------------------------------------------

def test_a_leg_between_two_in_fence_waypoints_can_still_breach_the_fence():
    """Regression: the export validated corners only, so a leg across a concave
    notch passed the guardrail while the vehicle would fly outside the fence."""
    mission = _mission(LEFT_ARM_LOW)

    passed, violations = GuardrailPlannerAgent().validate(
        mission, [DronePlan(1, [LEFT_ARM_TOP, RIGHT_ARM_TOP])]
    )

    assert not passed
    assert any("leg 0->1" in v for v in violations)
    assert not _containment_violations(violations)   # both corners are genuinely inside


def test_a_leg_that_stays_inside_one_arm_is_accepted():
    mission = _mission(LEFT_ARM_LOW)
    passed, violations = GuardrailPlannerAgent().validate(
        mission, [DronePlan(1, [Coordinate(0.001, 0.004), LEFT_ARM_TOP])]
    )
    assert passed, violations


def test_a_waypoint_outside_the_fence_is_still_caught():
    mission = _mission(LEFT_ARM_LOW)
    passed, violations = GuardrailPlannerAgent().validate(
        mission, [DronePlan(1, [Coordinate(0.001, 0.004), Coordinate(0.050, 0.004)])]
    )
    assert not passed
    assert any("waypoint 1 is outside GeoFence" in v for v in violations)


def test_a_waypoint_on_the_fence_boundary_is_not_a_breach():
    """Touching the boundary is not leaving it; rejecting it would fail valid missions."""
    fence = MissionRegion("GeoFence", [Coordinate(0.0, 0.0), Coordinate(0.01, 0.0),
                                       Coordinate(0.01, 0.01), Coordinate(0.0, 0.01)])
    mission = MissionInput(Coordinate(0.005, 0.005), fence, fence)
    passed, violations = GuardrailPlannerAgent().validate(
        mission, [DronePlan(1, [Coordinate(0.002, 0.005), Coordinate(0.008, 0.005)])]
    )
    assert passed, violations


# --- the legs the planner never chose ----------------------------------------

def test_the_takeoff_transit_is_checked():
    """The vehicle flies home -> waypoint 0 before any survey leg. Until 09 Sep 2026
    that leg was never validated; on Ubin it was up to 664 m long."""
    mission = _mission(LEFT_ARM_TOP)          # home in the left arm...

    passed, violations = GuardrailPlannerAgent().validate(
        mission, [DronePlan(1, [RIGHT_ARM_TOP])]   # ...single waypoint in the right arm
    )

    assert not passed
    assert any("takeoff transit (home -> waypoint 0) leaves GeoFence" in v for v in violations)


def test_the_rtl_return_is_checked():
    mission = _mission(LEFT_ARM_TOP)

    passed, violations = GuardrailPlannerAgent().validate(
        mission, [DronePlan(1, [RIGHT_ARM_TOP])]
    )

    assert not passed
    assert any("RTL return (waypoint 0 -> home) leaves GeoFence" in v for v in violations)


def test_a_transit_that_stays_inside_the_fence_is_accepted():
    mission = _mission(LEFT_ARM_LOW)
    passed, violations = GuardrailPlannerAgent().validate(
        mission, [DronePlan(1, [Coordinate(0.001, 0.004), LEFT_ARM_TOP])]
    )
    assert passed, violations


def test_home_outside_the_fence_is_reported_once_not_per_leg():
    """A fence excluding the launch point fails the sortie, not one aircraft. Reporting
    it as 2N leg failures would bury the cause."""
    mission = _mission(Coordinate(0.005, 0.006))    # dead in the notch, outside the fence
    drones = [DronePlan(1, [LEFT_ARM_TOP]), DronePlan(2, [RIGHT_ARM_TOP])]

    passed, violations = GuardrailPlannerAgent().validate(mission, drones)

    assert not passed
    assert violations.count("GCS location is outside the GeoFence") == 1
    assert not any("transit" in v or "RTL" in v for v in violations)


def test_a_drone_with_no_waypoints_contributes_no_transit_violations():
    mission = _mission(LEFT_ARM_LOW)
    passed, violations = GuardrailPlannerAgent().validate(mission, [DronePlan(1, [])])
    assert passed, violations


# --- regression against the real sectors -------------------------------------

def test_the_real_safti_mission_still_passes():
    mission = parse_kml(ROOT / "ops" / "SAFTI city.kml")
    result = MultiAgentPlanner().run(mission, load_config(ROOT / "config.example.json"))
    assert result.guardrail_passed, result.warnings


def test_the_pulau_ubin_mission_still_passes_with_transits_checked():
    """Its transits are 269 / 585 / 664 m -- now actually validated, not assumed."""
    mission = parse_kml(ROOT / "ops" / "pulau-ubin-west.kml")
    result = MultiAgentPlanner().run(mission, load_config(ROOT / "config.example.json"))
    assert result.guardrail_passed, result.warnings
