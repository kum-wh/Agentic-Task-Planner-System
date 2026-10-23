from __future__ import annotations

import heapq
import math
from typing import Any, Callable, Iterable, NamedTuple

from .models import CORNER_ACCEPTANCE_RADIUS_M, TIGHT_CORNER_RADIUS_M, Coordinate

EARTH_RADIUS_M = 6_378_137.0

# Survey lanes end exactly ON the OpsArea boundary, and the lat/lon round trip moves them
# by ~1e-8 m. A point this close to the boundary counts as on it, so a lane end cannot
# flip between inside and outside on rounding alone.
BOUNDARY_TOLERANCE_M = 1e-3

# How far a routed connector stands off each corner of an excluded notch it goes round:
# the survey cross-track max measured in flight (4.95 m, Pulau Ubin, 08 Sep 2026) plus
# the 0.83 m a multicopter cuts inside a 90 deg corner flown at TIGHT_CORNER_RADIUS_M,
# rounded up. Router corners are always flown tight (see corner_radii).
ROUTE_CLEARANCE_M = 6.0

# Heading change per vertex of a drawn turn: at 90 deg a right-angle corner is a single
# vertex and a lane-end U-turn is two. Each vertex is one waypoint switch for PX4, and on
# 14 Sep every switch cost a roll swing; the continuous-circle drawing (rounded_turns)
# makes the switches seamless, and fewer is better. 22.5 until the evening of 14 Sep 2026,
# then 45 until 18 Sep 2026.
#
# Why it went 45 -> 90 (18 Sep 2026): each vertex carries param2 = radius * tan(step / 2),
# and PX4 caps the speed through a waypoint at sqrt(MPC_ACC_HOR * param2 * tan(bend / 2)).
# A finer arc therefore writes a SMALLER param2 and slows the aircraft down at every
# vertex -- drawing the turn more precisely made it fly worse. Measured on toa-payoh
# drone-3 at 45 deg: param2 4.83 m, a 2.28 m/s cap per vertex, 12 m/s down each lane
# braking to ~3.5 m/s at every turn, 8.13 m/s achieved against 12.2 m/s planned and 40 %
# of the survey below 6 m/s. At 90 deg param2 is the full turn radius.
ARC_STEP_DEG = 90.0


class Point(NamedTuple):
    x: float
    y: float


def project(coordinate: Coordinate, origin: Coordinate) -> Point:
    scale = math.pi / 180 * EARTH_RADIUS_M
    return Point((coordinate.longitude - origin.longitude) * scale * math.cos(math.radians(origin.latitude)), (coordinate.latitude - origin.latitude) * scale)


def unproject(point: Point, origin: Coordinate, altitude: float) -> Coordinate:
    scale = math.pi / 180 * EARTH_RADIUS_M
    return Coordinate(origin.longitude + point.x / (scale * math.cos(math.radians(origin.latitude))), origin.latitude + point.y / scale, altitude)


def point_in_polygon(point: Point, polygon: list[Point]) -> bool:
    inside = False
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        if (start.y > point.y) != (end.y > point.y):
            crossing_x = (end.x - start.x) * (point.y - start.y) / (end.y - start.y) + start.x
            if point.x < crossing_x:
                inside = not inside
    return inside


def clip_polygon(polygon: list[Point], axis: Point, offset: float, keep_less: bool) -> list[Point]:
    """Clip polygon against axis.x*x + axis.y*y <= offset (or >= offset)."""
    if not polygon:
        return []

    def value(point: Point) -> float:
        return axis.x * point.x + axis.y * point.y - offset

    def inside(point: Point) -> bool:
        return value(point) <= 1e-8 if keep_less else value(point) >= -1e-8

    result: list[Point] = []
    previous = polygon[-1]
    previous_inside = inside(previous)
    for current in polygon:
        current_inside = inside(current)
        if current_inside != previous_inside:
            previous_value = value(previous)
            current_value = value(current)
            fraction = previous_value / (previous_value - current_value)
            result.append(Point(previous.x + fraction * (current.x - previous.x), previous.y + fraction * (current.y - previous.y)))
        if current_inside:
            result.append(current)
        previous, previous_inside = current, current_inside
    return result


def polygon_segments(polygon: list[Point], lane_y: float) -> list[tuple[float, float]]:
    intersections: list[float] = []
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        if (start.y <= lane_y < end.y) or (end.y <= lane_y < start.y):
            fraction = (lane_y - start.y) / (end.y - start.y)
            intersections.append(start.x + fraction * (end.x - start.x))
    intersections.sort()
    return list(zip(intersections[::2], intersections[1::2]))


def bounds(polygon: Iterable[Point]) -> tuple[float, float, float, float]:
    points = list(polygon)
    return min(p.x for p in points), min(p.y for p in points), max(p.x for p in points), max(p.y for p in points)


def rotate(point: Point, angle_rad: float) -> Point:
    """Rotate a point about the origin (positive = counter-clockwise)."""
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    return Point(point.x * cos_a - point.y * sin_a, point.x * sin_a + point.y * cos_a)


def cross(origin: Point, a: Point, b: Point) -> float:
    """Z-component of (a-origin) x (b-origin): >0 if origin->a->b turns left."""
    return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x)


def convex_hull(points: list[Point]) -> list[Point]:
    """Andrew's monotone chain, counter-clockwise, without the repeated last point."""
    unique = sorted(set(points))
    if len(unique) < 3:
        return unique

    def half(sequence: list[Point]) -> list[Point]:
        chain: list[Point] = []
        for point in sequence:
            while len(chain) >= 2 and cross(chain[-2], chain[-1], point) <= 0:
                chain.pop()
            chain.append(point)
        return chain[:-1]

    return half(unique) + half(unique[::-1])


def min_width_angle(polygon: list[Point]) -> float:
    """Direction (radians) of the polygon's long axis — the axis to run scan lanes along.

    The minimum-width direction of a convex polygon is always perpendicular to one of
    its edges, so it is enough to test every hull edge: rotate the hull to put that
    edge on the x-axis and measure the remaining y-extent. The edge giving the
    smallest extent is the minimum width, and lanes run along it — the fewest, longest
    passes, which is the swath-count objective Fields2Cover optimised for in drone-dev.
    """
    hull = convex_hull(polygon)
    if len(hull) < 3:
        return 0.0

    best_angle, best_extent = 0.0, float("inf")
    for start, end in zip(hull, hull[1:] + hull[:1]):
        edge_angle = math.atan2(end.y - start.y, end.x - start.x)
        rotated_ys = [rotate(point, -edge_angle).y for point in hull]
        extent = max(rotated_ys) - min(rotated_ys)
        if extent < best_extent:
            best_angle, best_extent = edge_angle, extent
    return best_angle


def segments_properly_intersect(a1: Point, a2: Point, b1: Point, b2: Point, eps: float = 1e-9) -> bool:
    """True when segments a and b cross at an interior point of both.

    Deliberately proper-only: a leg that merely touches the fence at a vertex or runs
    along a boundary edge is not a breach, and treating it as one would reject valid
    missions whose waypoints sit exactly on the boundary.
    """
    d1, d2 = cross(b1, b2, a1), cross(b1, b2, a2)
    d3, d4 = cross(a1, a2, b1), cross(a1, a2, b2)
    straddles_b = (d1 > eps and d2 < -eps) or (d1 < -eps and d2 > eps)
    straddles_a = (d3 > eps and d4 < -eps) or (d3 < -eps and d4 > eps)
    return straddles_a and straddles_b


def segment_in_polygon(a: Point, b: Point, polygon: list[Point]) -> bool:
    """True when the whole segment a->b stays inside the polygon.

    Both endpoints inside is not sufficient: across a concave boundary a leg between
    two interior corners can still leave the polygon and come back.
    """
    if not (point_in_polygon(a, polygon) and point_in_polygon(b, polygon)):
        return False
    return not any(
        segments_properly_intersect(a, b, start, end)
        for start, end in zip(polygon, polygon[1:] + polygon[:1])
    )


def distance_to_boundary(point: Point, polygon: list[Point]) -> float:
    best = float("inf")
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = end.x - start.x, end.y - start.y
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq))
        best = min(best, math.hypot(point.x - (start.x + t * dx), point.y - (start.y + t * dy)))
    return best


def point_within(point: Point, polygon: list[Point], tolerance: float = BOUNDARY_TOLERANCE_M) -> bool:
    """Inside the polygon or on its boundary."""
    return point_in_polygon(point, polygon) or distance_to_boundary(point, polygon) <= tolerance


def segment_within(a: Point, b: Point, polygon: list[Point], tolerance: float = BOUNDARY_TOLERANCE_M) -> bool:
    """True when no part of a->b is outside the polygon. Running along the boundary is allowed.

    `segment_in_polygon` is not enough for the OpsArea. Survey lanes start and end ON its
    boundary, and a leg from one wall of a notch straight to the opposite wall touches
    the boundary only at its two ends -- no proper crossing anywhere -- while flying
    across the whole excluded zone. So the leg is cut at every point where it meets the
    boundary and each piece is tested at its midpoint: between two contacts a piece is
    wholly inside or wholly outside.
    """
    if not (point_within(a, polygon, tolerance) and point_within(b, polygon, tolerance)):
        return False
    rx, ry = b.x - a.x, b.y - a.y
    length_sq = rx * rx + ry * ry
    if length_sq <= tolerance * tolerance:
        return True
    cuts = [0.0, 1.0]
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        sx, sy = end.x - start.x, end.y - start.y
        denominator = rx * sy - ry * sx
        if abs(denominator) <= 1e-12 * math.sqrt(length_sq * (sx * sx + sy * sy)):
            # Parallel. Only a collinear edge can touch the leg, and it does so between its ends.
            for vertex in (start, end):
                t = ((vertex.x - a.x) * rx + (vertex.y - a.y) * ry) / length_sq
                if 0.0 < t < 1.0:
                    cuts.append(t)
            continue
        qx, qy = start.x - a.x, start.y - a.y
        t = (qx * sy - qy * sx) / denominator
        u = (qx * ry - qy * rx) / denominator
        if 0.0 < t < 1.0 and -1e-9 <= u <= 1.0 + 1e-9:
            cuts.append(t)
    cuts.sort()
    return all(
        point_within(Point(a.x + (low + high) / 2 * rx, a.y + (low + high) / 2 * ry), polygon, tolerance)
        for low, high in zip(cuts, cuts[1:])
        if high > low
    )


def _signed_area(polygon: list[Point]) -> float:
    return sum(p.x * q.y - q.x * p.y for p, q in zip(polygon, polygon[1:] + polygon[:1])) / 2


class BoundaryRouter:
    """Shortest path between two points that never leaves a polygon.

    Inside a simple polygon the shortest path can only bend at reflex vertices -- the
    corners that point into the area, such as the two inner corners of a notch -- so
    they are the only waypoints a detour ever needs. Each is pulled `clearance` metres
    into the polygon along its bisector so the route rounds the corner rather than
    grazing it, with less clearance where the gap is too narrow for the full amount.

    Visibility between the corners is worked out once per polygon; a query adds only its
    two endpoints. If no inside path exists the straight leg comes back unchanged, and
    the guardrail's OpsArea check rejects the plan -- loudly, not silently.
    """

    def __init__(self, polygon: list[Point], clearance: float = ROUTE_CLEARANCE_M):
        # A KML ring repeats its first vertex at the end; a zero-length edge has no direction.
        ring = [p for p, q in zip(polygon, polygon[1:] + polygon[:1]) if p != q]
        self.polygon = ring
        orientation = 1.0 if _signed_area(ring) >= 0 else -1.0
        self.corners = [
            self._pulled_in(before, vertex, after, clearance)
            for before, vertex, after in zip(ring[-1:] + ring[:-1], ring, ring[1:] + ring[:1])
            if cross(before, vertex, after) * orientation < -1e-9
        ]
        self.links: list[list[tuple[int, float]]] = [[] for _ in self.corners]
        for i, first in enumerate(self.corners):
            for j in range(i + 1, len(self.corners)):
                if segment_within(first, self.corners[j], ring):
                    distance = math.dist(first, self.corners[j])
                    self.links[i].append((j, distance))
                    self.links[j].append((i, distance))

    def _pulled_in(self, before: Point, vertex: Point, after: Point, clearance: float) -> Point:
        # At a reflex vertex the two edges enclose the exterior, so the interior lies
        # opposite the sum of the two edge directions.
        to_before = math.dist(before, vertex)
        to_after = math.dist(after, vertex)
        dx = -((before.x - vertex.x) / to_before + (after.x - vertex.x) / to_after)
        dy = -((before.y - vertex.y) / to_before + (after.y - vertex.y) / to_after)
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return vertex
        for standoff in (clearance, clearance / 2, clearance / 4):
            candidate = Point(vertex.x + dx / norm * standoff, vertex.y + dy / norm * standoff)
            if point_in_polygon(candidate, self.polygon) and distance_to_boundary(candidate, self.polygon) >= 0.99 * standoff:
                return candidate
        return vertex

    def route(self, a: Point, b: Point) -> list[Point]:
        """The waypoints to insert between a and b; empty when the straight leg stays inside."""
        if segment_within(a, b, self.polygon):
            return []
        nodes = self.corners + [a, b]
        start, goal = len(self.corners), len(self.corners) + 1
        links = [list(corner_links) for corner_links in self.links] + [[], []]
        for end in (start, goal):
            for index, corner in enumerate(self.corners):
                if segment_within(nodes[end], corner, self.polygon):
                    distance = math.dist(nodes[end], corner)
                    links[end].append((index, distance))
                    links[index].append((end, distance))

        best = {start: 0.0}
        previous: dict[int, int] = {}
        queue = [(0.0, start)]
        while queue:
            distance, node = heapq.heappop(queue)
            if node == goal:
                break
            if distance > best[node]:
                continue
            for neighbour, step in links[node]:
                if distance + step < best.get(neighbour, float("inf")):
                    best[neighbour] = distance + step
                    previous[neighbour] = node
                    heapq.heappush(queue, (distance + step, neighbour))
        if goal not in previous:
            return []
        path, node = [], previous[goal]
        while node != start:
            path.append(nodes[node])
            node = previous[node]
        return path[::-1]

    def length(self, a: Point, b: Point) -> float:
        points = [a, *self.route(a, b), b]
        return sum(math.dist(p, q) for p, q in zip(points, points[1:]))


def segment_distance(a1: Point, a2: Point, b1: Point, b2: Point) -> float:
    """Closest distance between segments a1-a2 and b1-b2; 0 when they touch or cross."""
    if segments_properly_intersect(a1, a2, b1, b2):
        return 0.0

    def to_segment(p: Point, s: Point, e: Point) -> float:
        dx, dy = e.x - s.x, e.y - s.y
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((p.x - s.x) * dx + (p.y - s.y) * dy) / length_sq))
        return math.hypot(p.x - (s.x + t * dx), p.y - (s.y + t * dy))

    return min(to_segment(a1, b1, b2), to_segment(a2, b1, b2), to_segment(b1, a1, a2), to_segment(b2, a1, a2))


def rounded_corner(before: Point, corner: Point, after: Point, radius: float, steps: int = 8) -> list[Point]:
    """The path a multicopter flies through `corner` when it may start turning `radius` metres out.

    PX4 passes a waypoint at sqrt(accel * radius * tan(alpha / 2)) (TrajMath::
    computeMaxSpeedInWaypoint): a circular turn tangent to both legs `radius` metres either
    side of the corner. Returned as points from the first tangent point to the second. A
    straight-through corner, or a full reversal, is just the corner itself.
    """
    in_length, out_length = math.dist(before, corner), math.dist(corner, after)
    if in_length == 0 or out_length == 0:
        return [corner]
    ux, uy = (corner.x - before.x) / in_length, (corner.y - before.y) / in_length
    vx, vy = (after.x - corner.x) / out_length, (after.y - corner.y) / out_length
    turn = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)      # signed heading change
    if abs(turn) < 1e-6 or abs(turn) > math.pi - 1e-6:
        return [corner]
    # The turn cannot start further back than half of either leg.
    reach = min(radius, in_length / 2, out_length / 2)
    arc_radius = reach * math.tan((math.pi - abs(turn)) / 2)
    side = 1.0 if turn > 0 else -1.0
    start = Point(corner.x - ux * reach, corner.y - uy * reach)
    centre = Point(start.x - uy * arc_radius * side, start.y + ux * arc_radius * side)
    ox, oy = start.x - centre.x, start.y - centre.y
    points = []
    for step in range(steps + 1):
        angle = side * abs(turn) * step / steps
        points.append(Point(centre.x + ox * math.cos(angle) - oy * math.sin(angle),
                            centre.y + ox * math.sin(angle) + oy * math.cos(angle)))
    return points


def corner_radii(points: list[Point], fixed: list[float | None], polygon: list[Point],
                 loose: float = CORNER_ACCEPTANCE_RADIUS_M, tight: float = TIGHT_CORNER_RADIUS_M) -> list[float]:
    """Acceptance radius to write on each waypoint; 0 leaves the corner to NAV_ACC_RAD.

    `fixed[index]` is a radius the waypoint must carry whatever its geometry: the router's
    corners beside an excluded notch (only ROUTE_CLEARANCE_M to spare, so always tight),
    and the points of a drawn turn (rounded_turns). Any other corner is left to
    NAV_ACC_RAD unless the turn that radius allows would leave the polygon.
    """
    radii = []
    for index, corner in enumerate(points):
        if index == 0 or index == len(points) - 1:
            radii.append(0.0)
        elif fixed[index] is not None:
            radii.append(fixed[index])
        else:
            arc = rounded_corner(points[index - 1], corner, points[index + 1], loose)
            inside = all(segment_within(a, b, polygon) for a, b in zip(arc, arc[1:]))
            radii.append(0.0 if inside else tight)
    return radii


def _circumscribed_turn(before: Point, corner: Point, after: Point, turn_radius: float,
                        max_cut: float, min_radius: float, step_deg: float) -> tuple[list[Point], float, float] | None:
    """The polygon around a circular turn at `corner`: its vertices, their acceptance radius,
    and the tangent distance of the circle. None for a straight-through corner or a reversal."""
    in_length, out_length = math.dist(before, corner), math.dist(corner, after)
    if in_length == 0 or out_length == 0:
        return None
    ux, uy = (corner.x - before.x) / in_length, (corner.y - before.y) / in_length
    vx, vy = (after.x - corner.x) / out_length, (after.y - corner.y) / out_length
    signed = math.atan2(ux * vy - uy * vx, ux * vx + uy * vy)
    turn = abs(signed)
    if not 1e-3 < turn < math.pi - 1e-3:
        return None
    # Tangent distance: the requested radius, capped by half of each leg (neighbouring turns
    # meet, never overlap) and by the cut depth -- a cut of depth c needs c / tan(turn / 4).
    reach = min(turn_radius * math.tan(turn / 2), max_cut / math.tan(turn / 4), in_length / 2, out_length / 2)
    radius = reach / math.tan(turn / 2)
    if radius < min_radius:
        # A hairpin, or legs too short: its vertices would carry a near-zero acceptance
        # radius and PX4 would crawl round. Left as a corner, PX4's own rounding is no slower.
        return None
    steps = max(1, math.ceil(math.degrees(turn) / step_deg - 1e-9))
    step = turn / steps
    half_leg = radius * math.tan(step / 2)       # vertex to where the circle touches each leg
    side = 1.0 if signed > 0 else -1.0
    heading = math.atan2(uy, ux)
    # First vertex: on the incoming leg, half a polygon leg past where the circle begins.
    point = Point(corner.x - ux * (reach - half_leg), corner.y - uy * (reach - half_leg))
    vertices = [point]
    for _ in range(steps - 1):
        heading += side * step
        point = Point(point.x + 2 * half_leg * math.cos(heading), point.y + 2 * half_leg * math.sin(heading))
        vertices.append(point)
    return vertices, half_leg, reach


def rounded_turns(points: list[Point], keep_sharp: list[bool], polygon: list[Point], turn_radius: float,
                  max_cut: float, min_radius: float = 0.0,
                  step_deg: float = ARC_STEP_DEG) -> tuple[list[Point], list[float | None], list[bool]]:
    """Turn every corner into a circle of `turn_radius` that PX4 flies as one steady bank.

    PX4 rounds each waypoint itself: with acceptance radius d at a bend of delta it flies a
    circular fillet of radius d / tan(delta / 2), tangent to both legs d either side of the
    waypoint (TrajMath::computeMaxSpeedInWaypoint). So a turn is drawn as the polygon
    *around* the circle -- every leg grazing it, a vertex every `step_deg` -- and each vertex
    carries d = turn_radius * tan(step / 2), half its leg. Every vertex's fillet is then a
    piece of the same circle, starting exactly where the last one ended: one continuous turn
    at one bank angle, as an aircraft flies it, with nowhere for PX4 to re-steer. A 90 deg
    corner with one step is just the corner with d = turn_radius.

    This replaced the first drawing, flown 14 Sep 2026, whose points sat *on* the circle
    4.6 m apart with a 2 m radius. PX4's fillets there were too short to meet, the curvature
    switched on and off at every point, and the roll swung +-11-14 deg once per waypoint.

    The circle never reaches back more than half of either leg and never cuts deeper than
    `max_cut` inside the corner, so the camera still sees the corner; a sharp corner gets a
    tighter circle. Corners flagged `keep_sharp` (the router's notch corners), turns whose
    polygon or circle would leave `polygon`, and turns tighter than `min_radius` stay as
    they were.
    Returns the points, each with its acceptance radius (None where no turn was drawn) and
    whether it was kept sharp.
    """
    if len(points) < 3:
        return list(points), [None] * len(points), list(keep_sharp)
    out, radii, sharp = [points[0]], [None], [keep_sharp[0]]

    def add(point: Point, radius: float | None, sharp_point: bool) -> None:
        if math.dist(point, out[-1]) > 1e-6:
            out.append(point)
            radii.append(radius)
            sharp.append(sharp_point)

    for index in range(1, len(points) - 1):
        before, corner, after = points[index - 1], points[index], points[index + 1]
        drawn = None if keep_sharp[index] else _circumscribed_turn(before, corner, after, turn_radius,
                                                                   max_cut, min_radius, step_deg)
        if drawn is not None:
            vertices, half_leg, reach = drawn
            circle = rounded_corner(before, corner, after, reach, 8)
            if all(segment_within(a, b, polygon) for path in (vertices, circle) for a, b in zip(path, path[1:])):
                for vertex in vertices:
                    add(vertex, half_leg, False)
                continue
        add(corner, None, keep_sharp[index])
    add(points[-1], None, keep_sharp[-1])
    return out, radii, sharp


def heading_deg(a: Point, b: Point) -> float:
    """Compass heading from a to b, degrees clockwise from north (local x = east, y = north)."""
    return math.degrees(math.atan2(b.x - a.x, b.y - a.y)) % 360.0


def pre_yaw_turns(points: list[Point], keep_sharp: list[bool], lead: float) -> tuple[list[Point], list[bool], list[float]]:
    """"Anti-centripetal" corners: the nose is turned to the next leg before the corner.

    A multicopter can point one way and fly another. So every waypoint is given the heading
    of the leg that leaves it: while flying to a corner, PX4 turns the nose to face the next
    leg, and on reaching the corner the heading change is already done -- only the direction
    of travel is left to change. The path itself stays sharp.

    A heading on every waypoint alone would crab each whole lane with the nose already on the
    next cross-leg, so a pass-through waypoint is added `lead` metres before each corner,
    carrying the lane's own heading: the rotation happens only on that last stretch. It must
    finish in time -- PX4 counts a waypoint with a heading as reached only once the nose is
    within MIS_YAW_ERR (12 deg) of it, and with MIS_YAW_TMT at -1 hovers until it is. A leg
    too short for the lead gets no extra point and turns the nose over its whole length.
    Returns the points, whether each was kept sharp, and each point's heading.
    """
    if len(points) < 2:
        return list(points), list(keep_sharp), [0.0] * len(points)
    out, sharp = [points[0]], [keep_sharp[0]]
    for index in range(1, len(points)):
        before, corner = points[index - 1], points[index]
        leg = math.dist(before, corner)
        if index < len(points) - 1 and leg > lead + 1.0:
            after = points[index + 1]
            bend = abs((math.atan2(after.y - corner.y, after.x - corner.x)
                        - math.atan2(corner.y - before.y, corner.x - before.x) + math.pi) % (2 * math.pi) - math.pi)
            if bend > math.radians(1):
                fraction = (leg - lead) / leg
                out.append(Point(before.x + fraction * (corner.x - before.x), before.y + fraction * (corner.y - before.y)))
                sharp.append(False)
        out.append(corner)
        sharp.append(keep_sharp[index])
    headings = [heading_deg(a, b) for a, b in zip(out, out[1:])]
    return out, sharp, headings + headings[-1:]


# --- where the swarm splits up ------------------------------------------------------

# Grid pitch for the first pass of the dispersal search, then two refinement passes at a
# quarter of the previous pitch. 5 m resolves a gate to well inside a waypoint acceptance
# radius; the objective surfaces here are smooth, so a coarse sweep plus refinement finds
# the same point a finer sweep would, for a fraction of the evaluations.
DISPERSAL_GRID_STEP_M = 5.0


def circumcentre(a: Point, b: Point, c: Point) -> Point | None:
    """The one point equidistant from all three, or None if they are collinear.

    For exactly three drones this *is* the perfectly equitable dispersal point -- there is
    only one, so "fair" is not a matter of taste. It can land anywhere, including outside
    the fence, which is why callers must still check it.
    """
    d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y))
    if abs(d) < 1e-9:
        return None
    sa, sb, sc = a.x ** 2 + a.y ** 2, b.x ** 2 + b.y ** 2, c.x ** 2 + c.y ** 2
    return Point(
        (sa * (b.y - c.y) + sb * (c.y - a.y) + sc * (a.y - b.y)) / d,
        (sa * (c.x - b.x) + sb * (a.x - c.x) + sc * (b.x - a.x)) / d,
    )


def search_in_polygon(polygon: list[Point], objective: Callable[[Point], Any],
                      step: float = DISPERSAL_GRID_STEP_M) -> Point | None:
    """Lowest-scoring point inside `polygon`, by coarse grid then two refinements.

    Deliberately not a gradient method: the objectives this serves (spread of distance,
    spread of arrival time, worst-case battery load) are minima of piecewise-smooth
    functions with flat valleys, where a local method stalls or lands anywhere along the
    valley floor depending on where it started. A sweep is reproducible, and at these sizes
    it costs milliseconds.

    The objective may return anything that compares -- a tuple ranks by its first element
    and settles ties on the rest, which is how "fairest, and cheapest among equally fair"
    is expressed without inventing a weight between seconds and metres.

    Returns None when the polygon contains no candidate at all -- the caller is expected to
    fall back rather than fly to a point that was never checked.
    """
    if not polygon:
        return None
    min_x, min_y, max_x, max_y = bounds(polygon)
    best_point, best_score = None, None

    def sweep(x0: float, x1: float, y0: float, y1: float, pitch: float) -> None:
        nonlocal best_point, best_score
        steps_x = max(int((x1 - x0) / pitch), 0)
        steps_y = max(int((y1 - y0) / pitch), 0)
        for ix in range(steps_x + 1):
            for iy in range(steps_y + 1):
                candidate = Point(x0 + ix * pitch, y0 + iy * pitch)
                if not point_in_polygon(candidate, polygon):
                    continue
                score = objective(candidate)
                if best_score is None or score < best_score:
                    best_point, best_score = candidate, score

    sweep(min_x, max_x, min_y, max_y, step)
    pitch = step
    for _ in range(2):
        if best_point is None:
            return None
        pitch /= 4
        sweep(best_point.x - 4 * pitch, best_point.x + 4 * pitch,
              best_point.y - 4 * pitch, best_point.y + 4 * pitch, pitch)
    return best_point
