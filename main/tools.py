"""Mock drone-fleet tools: they return canned strings and don't control anything."""
import math

from langchain_core.tools import tool

M_PER_DEG_LAT = 111_320.0
CRUISE_MPS = 5.0
VALID_PATTERNS = ("lawnmower", "spiral")


def _lawnmower_xy(radius: float, spacing: float) -> list[tuple[float, float]]:
    """Boustrophedon: east-west sweeps clipped to the circle, alternating direction."""
    n = int(radius // spacing)
    pts, west_to_east = [], True
    for i in range(-n, n + 1):
        y = i * spacing
        half = math.sqrt(max(radius**2 - y**2, 0.0))
        line = [(-half, y), (half, y)]
        pts += line if west_to_east else line[::-1]
        west_to_east = not west_to_east
    return pts


def _spiral_xy(radius: float, spacing: float, points_per_turn: int = 6) -> list[tuple[float, float]]:
    """Archimedean spiral from the centre outwards; rings are `spacing` apart."""
    pts = [(0.0, 0.0)]
    step = 2 * math.pi / points_per_turn
    theta = step
    while True:
        r = min(spacing * theta / (2 * math.pi), radius)
        pts.append((r * math.cos(theta), r * math.sin(theta)))
        if r >= radius:
            return pts
        theta += step


@tool
def plan_path(latitude: float, longitude: float, radius_m: float, pattern: str = "lawnmower") -> str:
    """Plan a search path that covers a circular region and return its waypoints.

    Use this before navigate: it only plans, it does not move any drone.
    Call it once per region, then send the returned waypoints to a drone
    with navigate, in order.

    Args:
        latitude: Centre of the region in decimal degrees (-90 to 90).
        longitude: Centre of the region in decimal degrees (-180 to 180).
        radius_m: Radius of the region in metres (typically 50 to 2000).
        pattern: Sweep style: 'lawnmower' (parallel lines, best for open
            ground) or 'spiral' (outward from the centre, best when the
            last known position is near the centre).
    """
    pattern = pattern.strip().lower()
    if pattern not in VALID_PATTERNS:
        return f"error: unknown pattern '{pattern}', use one of {', '.join(VALID_PATTERNS)}"
    if not (-90 <= latitude <= 90 and -180 <= longitude <= 180):
        return "error: latitude must be in [-90, 90] and longitude in [-180, 180]"
    if radius_m <= 0:
        return "error: radius_m must be positive"

    spacing = radius_m / 3  # sweep width between adjacent lines / rings
    xy = _lawnmower_xy(radius_m, spacing) if pattern == "lawnmower" else _spiral_xy(radius_m, spacing)

    m_per_deg_lon = M_PER_DEG_LAT * max(math.cos(math.radians(latitude)), 1e-6)
    waypoints = [(round(latitude + y / M_PER_DEG_LAT, 6), round(longitude + x / m_per_deg_lon, 6))
                 for x, y in xy]
    length = sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
    minutes = length / CRUISE_MPS / 60

    lines = [f"{i}. ({la}, {lo})" for i, (la, lo) in enumerate(waypoints, 1)]
    return (
        f"path planned ({pattern}) over ({latitude}, {longitude}) radius {radius_m} m: "
        f"{len(waypoints)} waypoints, total length {length / 1000:.2f} km, "
        f"est. flight time {minutes:.0f} min\n" + "\n".join(lines)
    )


@tool
def list_drones() -> str:
    """List every drone in the fleet with its id, state, position and battery.

    Call this first to discover which drone ids exist and which are free
    (state 'idle') before giving any drone work. Drones with battery below
    25% should not be sent on new searches.
    """
    return (
        "alpha-1: idle, at base, battery 92%\n"
        "alpha-2: searching, lat 1.3521 lon 103.8198, battery 61%\n"
        "bravo-1: returning to base, battery 18%"
    )


@tool
def navigate(drone_id: str, latitude: float, longitude: float, altitude_m: float = 60.0) -> str:
    """Command one drone to fly to a single point. Call it repeatedly, once
    per waypoint, to fly a path from plan_path. The drone must exist
    (see list_drones) and have enough battery to get there and back.

    Args:
        drone_id: Drone identifier as returned by list_drones, e.g. 'alpha-1'.
        latitude: Destination in decimal degrees (-90 to 90).
        longitude: Destination in decimal degrees (-180 to 180).
        altitude_m: Flight altitude in metres above ground (10 to 120).
    """
    return f"ok: {drone_id} flying to ({latitude}, {longitude}) at {altitude_m} m, ETA 4 min"
