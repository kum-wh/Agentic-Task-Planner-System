from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any

from .camera import MissionProfile, camera_with_hfov, compute
from .energy import DroneSpec

# Must match MPC_XY_VEL_MAX in sar_turn_params.params. PX4's own default is 12.0,
# which would clamp the blur-safe cruise speed at any altitude above ~28 m.
VEHICLE_MAX_SPEED_M_S = 20.0

# Must match NAV_ACC_RAD in sar_turn_params.params. PX4 uses it for every waypoint whose
# own acceptance radius (param2) is 0, and for a multicopter it sets how far out a corner
# may start to be rounded: at 15 deg of bank a 90 deg lane corner is flown at ~3.6 m/s
# instead of from a standstill (TrajMath::computeMaxSpeedInWaypoint).
CORNER_ACCEPTANCE_RADIUS_M = 5.0

# Written as param2 on the corners that must stay tight: the router's corners beside an
# excluded notch, and any corner whose NAV_ACC_RAD rounding would leave the OpsArea. It
# cuts 0.83 m inside a 90 deg corner, which ROUTE_CLEARANCE_M allows for.
TIGHT_CORNER_RADIUS_M = 2.0

# How deep a drawn turn may cut inside its corner, as a fraction of the camera's along-track
# half-footprint (geometry.rounded_turns). The corner the turn cuts off must still be imaged;
# the remaining 20 % is margin for tracking error. Binds only on a large turn_radius_m.
TURN_CUT_FOOTPRINT_FRACTION = 0.8

# A corner whose circle would be tighter than this (a hairpin, or legs too short) is left as
# a plain corner: drawn, its vertices would carry a near-zero acceptance radius and PX4 would
# crawl round. PX4's own rounding of a sharp corner is no slower -- 2.4 m/s on 11 Sep, about
# what a 3 m circle allows at 15 deg of bank (sqrt(2.6 * 3) = 2.8 m/s).
MIN_TURN_RADIUS_M = 3.0

# Two drones count as separated when their altitude layers are at least this far apart,
# or, failing that, when their whole flown paths never come within the horizontal
# minimum -- twice (GNSS ~3 m + measured tracking 5 m) plus the airframe. The vertical
# figure is the 2 m layer spacing chosen on 11 Sep 2026.
MIN_VERTICAL_SEPARATION_M = 2.0
MIN_HORIZONTAL_SEPARATION_M = 20.0

# The ground resolution a search is expected to hold: the `--ceiling` query answers
# "how high can I fly and still get this?", and 3 cm/px is what it has always been asked
# for -- roughly 17 px across a person. A stacked swarm puts its top layers above the
# altitude that delivers it, so the planner says so rather than quietly losing detail.
GSD_STANDARD_CM_PX = 3.0


@dataclass(frozen=True)
class Coordinate:
    longitude: float
    latitude: float
    altitude: float = 0.0

    def as_qgc(self) -> list[float]:
        return [self.longitude, self.latitude, self.altitude]


@dataclass(frozen=True)
class Waypoint(Coordinate):
    """A survey waypoint that can carry its own acceptance radius (the .plan's param2).

    0 leaves the corner to NAV_ACC_RAD (CORNER_ACCEPTANCE_RADIUS_M). PX4 honours a
    per-item radius for multicopters (mission_block.cpp), so a corner that must be flown
    tight says so here instead of forcing every corner in the survey to be flown that way.
    """

    acceptance_radius_m: float = 0.0
    # Heading to hold while flying to this waypoint (the .plan's param4), degrees clockwise
    # from north. None leaves heading to MPC_YAW_MODE. With a heading set, PX4 also refuses
    # to count the waypoint reached until the nose is within MIS_YAW_ERR of it.
    yaw_deg: float | None = None


@dataclass(frozen=True)
class MissionRegion:
    name: str
    coordinates: list[Coordinate]


@dataclass(frozen=True)
class MissionInput:
    gcs_location: Coordinate
    geo_fence: MissionRegion
    ops_area: MissionRegion


@dataclass(frozen=True)
class PlannerConfig:
    number_of_drones: int = 3
    camera_fov_degrees: float = 58.1     # SIYI ZR30 @ 1x, 720p
    scan_altitude_m: float = 30.0        # the one knob everything else derives from
    scan_overlap_percent: float = 25.0
    scan_type: str = "lawnmower"
    # Vertical stagger between drones: the base altitude, then one layer up, one down, ...
    # Each drone's lanes, GSD and cruise speed re-derive from its own layer.
    layer_spacing_m: float = 2.0
    # Radius every turn is flown at, in one steady bank. 0 = automatic: half the drone's own
    # lane spacing, the widest turn that fits between neighbouring lanes. Capped by the
    # geometry, and so the camera still sees every corner a turn cuts off.
    turn_radius_m: float = 0.0
    # How corners are flown. "circle" (default): drawn as circles for PX4 to round.
    # "pre_yaw": sharp corners, with the nose turned to the next leg's heading on the
    # approach, so the heading change is done before the corner -- the "anti-centripetal"
    # tactic, tried 14 Sep 2026.
    turn_style: str = "circle"
    # pre_yaw only: how long before each corner the nose starts turning, in seconds at
    # cruise speed. Must cover the rotation at MPC_YAWRAUTO_MAX or PX4 hovers at the corner.
    pre_yaw_lead_s: float = 3.0
    # How the swarm leaves home. "direct" (default): every drone flies straight to its own
    # first waypoint, as it always has. "gate": they fly as one group to a shared dispersal
    # point and split there. A gate always costs distance -- home -> gate -> start is never
    # shorter than home -> start -- so it buys formation flying and a simultaneous start,
    # never endurance (see energy.py).
    ingress_style: str = "direct"
    # What the gate is placed to equalise. "arrival": every drone reaches its cell at the
    # same moment (the layers cruise at different speeds, so this is NOT the same as equal
    # distance). "distance": equal metres. "energy": lowest worst-case battery load.
    ingress_objective: str = "arrival"
    # Vertical spacing between drones while they transit as a group, before they split.
    # Wider than the survey layers because formation flight wants more room; it costs
    # nothing in image quality, as nothing is being imaged in transit.
    transit_layer_spacing_m: float = 5.0
    # How far out the gate must be for the formation to be worth flying. 0 = no constraint,
    # and then "fairest, then cheapest" will put the gate at home whenever entry-end choice
    # alone can equalise the ingress -- fair, but the swarm splits where it launched.
    ingress_min_gate_m: float = 0.0
    # Which drone flies which cell. "index" (default): drone i flies cell i, as before.
    # "energy": cells are handed out so the worst-off drone keeps the most margin. Free --
    # a permutation cannot change the total distance flown, only who pays it.
    assignment: str = "index"
    # Per-drone mass and state of charge, in drone order. Empty means "all identical", in
    # which case energy assignment has nothing to work with and leaves the order alone.
    drone_specs: tuple[DroneSpec, ...] = ()

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "PlannerConfig":
        config = cls(
            number_of_drones=int(data.get("number_of_drones", cls.number_of_drones)),
            camera_fov_degrees=float(data.get("camera_fov_degrees", cls.camera_fov_degrees)),
            scan_altitude_m=float(data.get("scan_altitude_m", cls.scan_altitude_m)),
            scan_overlap_percent=float(data.get("scan_overlap_percent", cls.scan_overlap_percent)),
            scan_type=str(data.get("scan_type", cls.scan_type)).lower(),
            layer_spacing_m=float(data.get("layer_spacing_m", cls.layer_spacing_m)),
            turn_radius_m=float(data.get("turn_radius_m", cls.turn_radius_m)),
            turn_style=str(data.get("turn_style", cls.turn_style)).lower(),
            pre_yaw_lead_s=float(data.get("pre_yaw_lead_s", cls.pre_yaw_lead_s)),
            ingress_style=str(data.get("ingress_style", cls.ingress_style)).lower(),
            ingress_objective=str(data.get("ingress_objective", cls.ingress_objective)).lower(),
            transit_layer_spacing_m=float(data.get("transit_layer_spacing_m", cls.transit_layer_spacing_m)),
            ingress_min_gate_m=float(data.get("ingress_min_gate_m", cls.ingress_min_gate_m)),
            assignment=str(data.get("assignment", cls.assignment)).lower(),
            drone_specs=tuple(
                DroneSpec(
                    mass_kg=float(spec.get("mass_kg", DroneSpec.mass_kg)),
                    battery_fraction=float(spec.get("battery_fraction", DroneSpec.battery_fraction)),
                    payload_class=str(spec.get("payload_class", DroneSpec.payload_class)),
                )
                for spec in data.get("drones", [])
            ),
        )
        config.validate()
        return config

    def validate(self) -> None:
        if self.number_of_drones < 1:
            raise ValueError("number_of_drones must be at least 1")
        if not 1 <= self.camera_fov_degrees < 180:
            raise ValueError("camera_fov_degrees must be between 1 and 179")
        if self.scan_altitude_m <= 0:
            raise ValueError("scan_altitude_m must be positive")
        if not 0 <= self.scan_overlap_percent < 100:
            raise ValueError("scan_overlap_percent must be in [0, 100)")
        if self.scan_type not in {"lawnmower", "voronoi"}:
            raise ValueError("scan_type must be 'lawnmower' or 'voronoi'")
        if self.layer_spacing_m < 0:
            raise ValueError("layer_spacing_m must be >= 0")
        if self.turn_radius_m < 0:
            raise ValueError("turn_radius_m must be >= 0 (0 = automatic)")
        if self.turn_style not in {"circle", "pre_yaw"}:
            raise ValueError("turn_style must be 'circle' or 'pre_yaw'")
        if self.pre_yaw_lead_s <= 0:
            raise ValueError("pre_yaw_lead_s must be positive")
        if self.ingress_style not in {"direct", "gate"}:
            raise ValueError("ingress_style must be 'direct' or 'gate'")
        if self.ingress_objective not in {"arrival", "distance", "energy"}:
            raise ValueError("ingress_objective must be 'arrival', 'distance' or 'energy'")
        if self.transit_layer_spacing_m < 0:
            raise ValueError("transit_layer_spacing_m must be >= 0")
        if self.ingress_min_gate_m < 0:
            raise ValueError("ingress_min_gate_m must be >= 0 (0 = no constraint)")
        if self.assignment not in {"index", "energy"}:
            raise ValueError("assignment must be 'index' or 'energy'")
        if self.drone_specs and len(self.drone_specs) != self.number_of_drones:
            raise ValueError(
                f"drones: {len(self.drone_specs)} specs for {self.number_of_drones} drones -- "
                "give one per drone, or none at all"
            )
        for spec in self.drone_specs:
            spec.validate()
        if min(self.transit_altitude_m(index) for index in range(self.number_of_drones)) <= 0:
            raise ValueError("the lowest transit layer is at or below the ground; "
                             "lower transit_layer_spacing_m or raise scan_altitude_m")
        clash = self.layer_clash
        if clash is not None and self.ingress_style == "gate":
            transit_drone, transit_alt, survey_drone, survey_alt = clash
            raise ValueError(
                f"transit_layer_spacing_m {self.transit_layer_spacing_m:g} puts drone "
                f"{transit_drone + 1}'s transit layer ({transit_alt:g} m) only "
                f"{abs(transit_alt - survey_alt):g} m from drone {survey_drone + 1}'s survey layer "
                f"({survey_alt:g} m), and the ingress crosses the survey area. Two ladders of "
                f"different spacing interleave once the fleet is large enough: with "
                f"{self.number_of_drones} drones the survey stack already spans "
                f"{min(self.drone_altitude_m(i) for i in range(self.number_of_drones)):g}-"
                f"{max(self.drone_altitude_m(i) for i in range(self.number_of_drones)):g} m. "
                f"Set transit_layer_spacing_m to {self.layer_spacing_m:g} (the survey spacing), "
                f"or separate the formation horizontally instead of vertically"
            )
        if min(self.drone_altitude_m(index) for index in range(self.number_of_drones)) <= 0:
            raise ValueError("the lowest altitude layer is at or below the ground; "
                             "lower layer_spacing_m or raise scan_altitude_m")

    def drone_altitude_m(self, index: int) -> float:
        """Altitude layer of the drone at 0-based `index`: the base, one up, one down, two up, ..."""
        step = (index + 1) // 2
        return self.scan_altitude_m + (step if index % 2 else -step) * self.layer_spacing_m

    @property
    def layer_clash(self) -> tuple[int, float, int, float] | None:
        """A transit layer sitting too close to a *different* drone's survey layer, if any.

        The two ladders have different spacings, so for a big enough fleet they interleave:
        at 8 drones and 5 m transit spacing, one drone transits at 25 m while another surveys
        at 26 m. The ingress runs across the survey area, so that is a real conflict, not a
        bookkeeping one -- and the altitude budget cannot fix it, since clearing a 14 m survey
        stack would put the top of the transit stack above 50 m.
        """
        for transit_drone in range(self.number_of_drones):
            transit = self.transit_altitude_m(transit_drone)
            for survey_drone in range(self.number_of_drones):
                if survey_drone == transit_drone:
                    continue                      # one aircraft cannot conflict with itself
                survey = self.drone_altitude_m(survey_drone)
                if abs(transit - survey) < MIN_VERTICAL_SEPARATION_M - 1e-6:
                    return transit_drone, transit, survey_drone, survey
        return None

    @property
    def imaging_warnings(self) -> list[str]:
        """Layers flown too high to meet the ground-resolution standard.

        Stacking drones costs resolution at the top of the stack, and nothing said so: at
        8 drones the top two layers are 36 and 38 m, where GSD is 3.12 and 3.30 cm/px.
        """
        over = [(index, self.for_drone(index).gsd_cm_px)
                for index in range(self.number_of_drones)
                if self.for_drone(index).gsd_cm_px > GSD_STANDARD_CM_PX + 1e-9]
        if not over:
            return []
        worst = max(over, key=lambda entry: entry[1])
        return [
            f"{len(over)} of {self.number_of_drones} drones fly above the {GSD_STANDARD_CM_PX:g} cm/px "
            f"standard -- drone {worst[0] + 1} at {self.drone_altitude_m(worst[0]):g} m gives "
            f"{worst[1]:.2f} cm/px. Lower scan_altitude_m or layer_spacing_m, or fly fewer drones"
        ]

    def transit_altitude_m(self, index: int) -> float:
        """Altitude this drone holds while the swarm transits together, before the split."""
        step = (index + 1) // 2
        return self.scan_altitude_m + (step if index % 2 else -step) * self.transit_layer_spacing_m

    def spec_for(self, index: int) -> DroneSpec:
        """This drone's mass and charge, or the reference airframe if none was given."""
        return self.drone_specs[index] if index < len(self.drone_specs) else DroneSpec()

    def for_drone(self, index: int) -> "PlannerConfig":
        """This configuration at one drone's own layer, so its imaging numbers re-derive from it."""
        return replace(self, scan_altitude_m=self.drone_altitude_m(index), layer_spacing_m=0.0)

    @property
    def profile(self) -> MissionProfile:
        """The imaging numbers this configuration implies (see camera.py).

        Altitude, FOV and overlap are the only inputs; spacing, GSD and cruise speed
        are read off this one profile so they can never drift out of agreement.
        """
        overlap = self.scan_overlap_percent / 100
        return compute(
            self.scan_altitude_m,
            cam=camera_with_hfov(self.camera_fov_degrees),
            side_overlap=overlap,
            fwd_overlap=overlap,
        )

    @property
    def lane_spacing_m(self) -> float:
        """Scanner lane pitch: the camera swath minus the requested side overlap."""
        return max(self.profile.line_spacing_m, 0.1)

    @property
    def cruise_speed_m_s(self) -> float:
        """Blur-safe cruise speed. Written into the .plan; never a hardcoded constant."""
        return round(self.profile.v_recommended_ms, 1)

    @property
    def gsd_cm_px(self) -> float:
        return self.profile.gsd_cm_px

    @property
    def speed_warnings(self) -> list[str]:
        """Flag the case where the airframe, not the optics, becomes the limit.

        Cruise speed scales with altitude, so above roughly 46 m the blur-safe speed
        exceeds what MPC_XY_VEL_MAX permits and the vehicle silently flies slower than
        planned -- the coverage estimate would then be optimistic.
        """
        # The highest layer flies fastest, so it is the one that meets the ceiling first.
        cruise = max((self.for_drone(index).cruise_speed_m_s for index in range(self.number_of_drones)),
                     default=self.cruise_speed_m_s)
        if cruise > VEHICLE_MAX_SPEED_M_S:
            return [
                f"derived cruise {cruise:.1f} m/s exceeds the vehicle ceiling "
                f"{VEHICLE_MAX_SPEED_M_S:.1f} m/s (MPC_XY_VEL_MAX); the vehicle will fly "
                f"slower than planned -- raise MPC_XY_VEL_MAX or lower the altitude"
            ]
        return []


@dataclass
class DronePlan:
    drone_id: int
    waypoints: list[Coordinate] = field(default_factory=list)
    # This drone's altitude layer, and the cruise speed its camera model gives there.
    # None falls back to the mission-wide config (hand-built plans, older callers).
    altitude_m: float | None = None
    cruise_speed_m_s: float | None = None
    # Which cell of the partition this drone was given. Only interesting when assignment
    # is "energy", where it is no longer the same as drone_id - 1.
    cell_index: int | None = None
    # The shared dispersal point, when ingress_style is "gate": flown at the transit layer
    # before the survey starts. A transit waypoint, not a survey one -- it is held to the
    # GeoFence but not to the OpsArea.
    ingress_gate: Coordinate | None = None
    # Where this drone leaves the transit layer for its survey layer, part-way down the
    # fan-out leg. Descending at the gate would put every drone's altitude change in the one
    # place they are all stacked; by here they have diverged horizontally.
    ingress_descent: Coordinate | None = None


@dataclass
class PlanningResult:
    mission: MissionInput
    drones: list[DronePlan]
    guardrail_passed: bool
    warnings: list[str] = field(default_factory=list)
    # Carried so the plan writer can take cruise speed and altitude from the camera
    # model instead of inventing them.
    config: PlannerConfig = field(default_factory=PlannerConfig)
