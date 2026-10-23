from __future__ import annotations

import math

from .geometry import (
    BoundaryRouter,
    Point,
    bounds,
    clip_polygon,
    corner_radii,
    min_width_angle,
    point_in_polygon,
    polygon_segments,
    pre_yaw_turns,
    project,
    rotate,
    rounded_turns,
    unproject,
)
from .models import (
    MIN_TURN_RADIUS_M,
    TIGHT_CORNER_RADIUS_M,
    TURN_CUT_FOOTPRINT_FRACTION,
    Coordinate,
    MissionRegion,
    PlannerConfig,
    Waypoint,
)


def _overlap(first: tuple[float, float], second: tuple[float, float]) -> bool:
    return max(first[0], second[0]) < min(first[1], second[1])


def _cells(scanlines: list[list[tuple[float, float]]]) -> list[list[tuple[int, float, float]]]:
    """Group the lane pieces into cells that can each be flown as one unbroken lawnmower.

    `polygon_segments` already returns both pieces of a lane that crosses a notch. What
    went wrong is that the pieces were then flattened into one list and flown end to
    end, straight across the gap. Here a piece joins the cell of the piece on the lane
    before only when the two overlap one-to-one; wherever a lane splits in two, or two
    merge into one, the cells on either side close and new ones start -- a boustrophedon
    decomposition done on the scanlines themselves. Within a cell every lane is a single
    piece, so no lane turn has a gap to jump.
    """
    cells: list[list[tuple[int, float, float]]] = []
    previous: list[tuple[tuple[float, float], int]] = []
    for lane_index, segments in enumerate(scanlines):
        # A clipped share of a concave area can carry a zero-width bridge along the cut,
        # which a scanline meets as a zero-length piece. There is nothing there to scan.
        segments = [segment for segment in segments if segment[1] - segment[0] > 1e-9]
        current = []
        for segment in segments:
            above = [(piece, cell) for piece, cell in previous if _overlap(piece, segment)]
            if len(above) == 1 and sum(_overlap(above[0][0], other) for other in segments) == 1:
                cell_index = above[0][1]
            else:
                cell_index = len(cells)
                cells.append([])
            cells[cell_index].append((lane_index, *segment))
            current.append((segment, cell_index))
        previous = current
    return cells


def _sweep(cell: list[tuple[int, float, float]], lane_ys: list[float], from_last: bool, first_forward: bool) -> list[tuple[Point, Point]]:
    """One cell as lanes in flying order: boustrophedon from either end, either way round."""
    lanes = []
    for position, (lane_index, start, end) in enumerate(cell[::-1] if from_last else cell):
        y = lane_ys[lane_index]
        forward = first_forward == (position % 2 == 0)
        lanes.append((Point(start, y), Point(end, y)) if forward else (Point(end, y), Point(start, y)))
    return lanes


def _lawnmower_route(polygon: list[Point], spacing: float,
                     boundary: list[Point] | None = None) -> tuple[list[Point], list[bool]]:
    """Boustrophedon fill of one sub-area, with lanes along its own long axis.

    Returns the points and, alongside, whether the router inserted each one: those are
    the corners beside an excluded notch, and they get flown tight (corner_radii).

    The polygon is rotated so its minimum-width direction is vertical — lanes then
    run horizontally, which is the fewest and longest passes the shape allows. A
    long thin strip therefore gets a handful of full-length passes instead of many
    short turn-heavy ones. Points are rotated back to the local metre frame on the
    way out, so callers never see the working frame.

    A concave area can cut a lane in two. The pieces are grouped into cells (`_cells`),
    each cell is flown as its own lawnmower, and every connector -- the lane turns and
    the hop from one cell to the next -- is routed inside `boundary` (default: the
    polygon itself), so it follows the edge round an excluded notch instead of flying
    across it. Pass the whole OpsArea as `boundary` when `polygon` is one drone's share
    of it: a share can come out as two blocks either side of a notch, joined only
    through a neighbour's share.
    """
    if len(polygon) < 3:
        return [], []

    angle = min_width_angle(polygon)
    local = [rotate(point, -angle) for point in polygon]
    _, min_y, _, max_y = bounds(local)
    extent = max_y - min_y

    # Lanes are centred within the extent rather than started on the boundary: the
    # camera swath is wider than the lane pitch, so a centred block still covers both
    # edges, and no lane degenerates onto the boundary itself.
    lanes_needed = extent / spacing
    # Rotating the polygon perturbs `extent` in its last bits. Without the tolerance an
    # exact multiple of the spacing tips over the ceil() boundary and adds a spurious
    # lane, which then collapses the centring offset and puts lanes on the two edges,
    # where a scanline degenerates and covers nothing.
    lane_count = max(1, math.ceil(lanes_needed - 1e-9 * max(1.0, lanes_needed)))
    first_y = min_y + (extent - (lane_count - 1) * spacing) / 2

    lane_ys = [first_y + lane_index * spacing for lane_index in range(lane_count)]
    cells = _cells([polygon_segments(local, lane_y) for lane_y in lane_ys])
    if not cells:
        return [], []
    router = BoundaryRouter([rotate(point, -angle) for point in (polygon if boundary is None else boundary)])

    # The first cell starts where the single sweep always started, so an area with no
    # split lanes flies the same lanes in the same order as before this change.
    lanes = _sweep(cells[0], lane_ys, False, cells[0][0][0] % 2 == 0)
    remaining = cells[1:]
    while remaining:
        # Nearest cell next, by routed distance, entered at whichever of its four
        # corners (either end lane, either direction) is closest.
        _, index, sweep = min(
            (
                (router.length(lanes[-1][1], sweep[0][0]), index, sweep)
                for index, cell in enumerate(remaining)
                for from_last in (False, True)
                for first_forward in (True, False)
                for sweep in (_sweep(cell, lane_ys, from_last, first_forward),)
            ),
            key=lambda option: option[:2],
        )
        lanes += sweep
        remaining.pop(index)

    points: list[Point] = []
    routed: list[bool] = []
    for lane_start, lane_end in lanes:
        if points:
            detour = router.route(points[-1], lane_start)
            points.extend(detour)
            routed.extend([True] * len(detour))
        points.extend((lane_start, lane_end))
        routed.extend((False, False))
    return [rotate(point, angle) for point in points], routed


def _lawnmower_points(polygon: list[Point], spacing: float, boundary: list[Point] | None = None) -> list[Point]:
    """The waypoints of `_lawnmower_route`, without the router flags."""
    return _lawnmower_route(polygon, spacing, boundary)[0]


def _waypoints(area: list[Point], boundary: list[Point], origin: Coordinate, config: PlannerConfig,
               reverse: bool = False) -> list[Waypoint]:
    """One drone's lawnmower over `area`, at that drone's own altitude, lane spacing and corner radii.

    turn_style "circle": every corner drawn as a circle for PX4 to round (rounded_turns) --
    flown 14 Sep, PX4 still banks once per vertex rather than holding one steady bank.
    turn_style "pre_yaw": corners left sharp, every waypoint carrying the heading of the leg
    leaving it, so the nose is on the next leg before the corner (pre_yaw_turns).
    Either way the router's notch corners stay sharp and tight.

    `reverse` flies the same lanes from the other end. It is free -- the route is identical,
    so the survey costs the same and only the transit out and the return swap over -- but it
    moves where this cell is entered, which is what a shared dispersal point is placed
    against. Reversal happens here, before the corners are shaped, because a pre_yaw
    waypoint carries the heading of the leg *leaving* it: reversing finished waypoints would
    leave every nose pointing the way it used to go.
    """
    points, routed = _lawnmower_route(area, config.lane_spacing_m, boundary)
    if reverse:
        points, routed = points[::-1], routed[::-1]
    if config.turn_style == "pre_yaw":
        points, sharp, headings = pre_yaw_turns(points, routed, config.pre_yaw_lead_s * config.cruise_speed_m_s)
        fixed = [TIGHT_CORNER_RADIUS_M if keep else None for keep in sharp]
    else:
        # The operator's radius, or half this drone's lane spacing: the widest turn that fits
        # between neighbouring lanes. A turn may cut no deeper inside its corner than a 90 deg
        # turn of that radius does, and never past TURN_CUT_FOOTPRINT_FRACTION of the camera's
        # along-track half-footprint -- the corner it cuts off must still be imaged.
        radius = config.turn_radius_m or config.lane_spacing_m / 2
        max_cut = min((math.sqrt(2) - 1) * radius, TURN_CUT_FOOTPRINT_FRACTION * config.profile.along_track_m / 2)
        points, turn_radii, sharp = rounded_turns(points, routed, boundary, radius, max_cut,
                                                  min(MIN_TURN_RADIUS_M, radius))
        fixed = [TIGHT_CORNER_RADIUS_M if keep else turn_radius for turn_radius, keep in zip(turn_radii, sharp)]
        headings = [None] * len(points)
    waypoints = []
    for point, acceptance, heading in zip(points, corner_radii(points, fixed, boundary), headings):
        coordinate = unproject(point, origin, config.scan_altitude_m)
        waypoints.append(Waypoint(coordinate.longitude, coordinate.latitude, coordinate.altitude, acceptance, heading))
    return waypoints


def _partition_polygon(polygon: list[Point], count: int) -> list[list[Point]]:
    """Split a polygon into `count` contiguous axis-aligned strips of equal width along its longer axis."""
    if count <= 1:
        return [polygon[:] for _ in range(max(count, 1))]
    if not polygon:
        return [[] for _ in range(count)]
    min_x, min_y, max_x, max_y = bounds(polygon)
    if (max_x - min_x) >= (max_y - min_y):
        axis, low_bound, high_bound = Point(1.0, 0.0), min_x, max_x
    else:
        axis, low_bound, high_bound = Point(0.0, 1.0), min_y, max_y
    step = (high_bound - low_bound) / count
    strips = []
    for index in range(count):
        low = low_bound + index * step
        high = low_bound + (index + 1) * step
        strip = clip_polygon(polygon, axis, high, True)
        strip = clip_polygon(strip, axis, low, False)
        strips.append(strip)
    return strips


def lawnmower_scan(region: MissionRegion, origin: Coordinate, config: PlannerConfig) -> list[list[Coordinate]]:
    """Split the OpsArea into one contiguous strip per drone and lawnmower-scan each strip.

    Each drone flies its own altitude layer (PlannerConfig.drone_altitude_m), so its lane
    spacing re-derives from that altitude through the camera model.
    """
    cells, polygon = partition_cells(region, origin, config)
    return [scan_cell(cell, polygon, origin, config, index) for index, cell in enumerate(cells)]


def _distinct(points: list[Point], tolerance: float = 1e-6) -> list[Point]:
    seen: set[tuple[int, int]] = set()
    unique: list[Point] = []
    for point in points:
        key = (round(point.x / tolerance), round(point.y / tolerance))
        if key not in seen:
            seen.add(key)
            unique.append(point)
    return unique


def _spread(candidates: list[Point], count: int) -> list[Point]:
    """Greedy farthest-point sampling, so seeds taken from a dense grid stay spread out."""
    if len(candidates) <= count:
        return candidates
    centre_x = sum(point.x for point in candidates) / len(candidates)
    centre_y = sum(point.y for point in candidates) / len(candidates)
    chosen = [min(candidates, key=lambda p: (p.x - centre_x) ** 2 + (p.y - centre_y) ** 2)]
    remaining = [point for point in candidates if point is not chosen[0]]
    while len(chosen) < count and remaining:
        farthest = max(remaining, key=lambda p: min((p.x - c.x) ** 2 + (p.y - c.y) ** 2 for c in chosen))
        chosen.append(farthest)
        remaining.remove(farthest)
    return chosen


def _seed_points(polygon: list[Point], count: int) -> list[Point]:
    """`count` distinct seeds inside the polygon, one per drone.

    Two drones sharing a seed is not a cosmetic problem: the half-plane clip between
    identical seeds is a no-op, so both would inherit the whole OpsArea and fly the
    same mission. The grid is refined until enough seeds land inside the shape, and
    any padding needed for a degenerate area is kept distinct by construction.
    """
    min_x, min_y, max_x, max_y = bounds(polygon)
    base_columns = max(1, round(count ** 0.5))
    base_rows = max(1, math.ceil(count / base_columns))

    inside: list[Point] = []
    for refinement in range(1, 9):
        columns, rows = base_columns * refinement, base_rows * refinement
        grid = [
            Point(
                min_x + (column + 0.5) * (max_x - min_x) / columns,
                min_y + (row + 0.5) * (max_y - min_y) / rows,
            )
            for row in range(rows)
            for column in range(columns)
        ]
        inside = _distinct([point for point in grid if point_in_polygon(point, polygon)])
        if len(inside) >= count:
            return _spread(inside, count)

    seeds = list(inside)
    index = 0
    while len(seeds) < count:
        index += 1
        fraction = index / (count + 1)
        # The nudge grows with the index, so padded seeds stay distinct from each other
        # even when the bounding box has collapsed to a point.
        candidate = Point(
            min_x + fraction * (max_x - min_x) + index * 1e-3,
            min_y + fraction * (max_y - min_y),
        )
        if len(_distinct(seeds + [candidate])) > len(seeds):
            seeds.append(candidate)
    return seeds[:count]


def _voronoi_cells(polygon: list[Point], count: int) -> list[list[Point]]:
    """One cell per drone, grown by half-plane clipping around spread seed points."""
    seeds = _seed_points(polygon, count)
    cells: list[list[Point]] = []
    for index, seed in enumerate(seeds):
        cell = polygon[:]
        for other_index, other in enumerate(seeds):
            if index == other_index:
                continue
            # Keep the half-plane closer to this seed than to the other seed.
            axis = Point(other.x - seed.x, other.y - seed.y)
            offset = (other.x * other.x + other.y * other.y - seed.x * seed.x - seed.y * seed.y) / 2
            cell = clip_polygon(cell, axis, offset, True)
        cells.append(cell)
    return cells


def partition_cells(region: MissionRegion, origin: Coordinate,
                    config: PlannerConfig) -> tuple[list[list[Point]], list[Point]]:
    """The OpsArea cut into one cell per drone, plus the OpsArea itself, in local metres.

    Cells are geometry alone -- they do not depend on which drone flies them. Separating
    this from the scan is what lets the planner price every drone-cell pairing before it
    commits to one (see energy.assign_cells).
    """
    polygon = [project(point, origin) for point in region.coordinates]
    if config.scan_type == "voronoi":
        return _voronoi_cells(polygon, config.number_of_drones), polygon
    return _partition_polygon(polygon, config.number_of_drones), polygon


def scan_cell(cell: list[Point], polygon: list[Point], origin: Coordinate, config: PlannerConfig,
              drone_index: int, reverse: bool = False) -> list[Waypoint]:
    """`cell` scanned by one particular drone: its altitude layer sets its lane spacing."""
    return _waypoints(cell, polygon, origin, config.for_drone(drone_index), reverse)


def voronoi_scan(region: MissionRegion, origin: Coordinate, config: PlannerConfig) -> list[list[Coordinate]]:
    cells, polygon = partition_cells(region, origin, config)
    return [scan_cell(cell, polygon, origin, config, index) for index, cell in enumerate(cells)]
