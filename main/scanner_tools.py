"""Standalone search-area scanner: split an area between drones, lawnmower-sweep each share.

One file, standard library only. Simplified from Drone-dev's flight_planner (scanner_tools,
geometry and the camera model), keeping the search pattern itself and dropping everything
that only matters when flying a PX4 (circle corners, pre-yaw turns, acceptance radii).

    from scanner_tools import plan_scan
    paths = plan_scan([(1.400, 103.960), (1.400, 103.965), ...], drones=3, altitude_m=30)
    # -> one list of (lat, lon, altitude_m) per drone, in flying order

    python scanner_tools.py                      # built-in U-shaped demo area
    python scanner_tools.py area.kml --drones 2 --alt 40 --scan-type voronoi

The pattern:
  1. Split the area into one share per drone: equal strips along the longer axis
     ("lawnmower"), or one cell per drone around spread-out seed points ("voronoi").
  2. Sweep each share boustrophedon-style, lanes along the share's own long axis,
     centred in the share, alternating direction.
  3. A lane cut in two by a notch (road, rail line) is split into separate blocks that are
     flown one after another, joined by connectors that follow the area's edge round the
     notch instead of crossing it.
  4. Each drone flies its own altitude layer (base, +2 m, -2 m, +4 m ...), and its lane
     spacing follows from that altitude and the camera's field of view.
"""

from __future__ import annotations

import argparse
import heapq
import math
import xml.etree.ElementTree as ET
from typing import Iterable, NamedTuple

EARTH_RADIUS_M = 6_378_137.0
HFOV_DEG = 58.1                  # SIYI ZR30 at 1x zoom; the camera the spacing is derived from
ROUTE_CLEARANCE_M = 6.0          # how far a notch detour stands off the notch's corners
BOUNDARY_TOLERANCE_M = 1e-3      # a point this close to the edge counts as on it
SCAN_TYPES = ("lawnmower", "voronoi")


class Point(NamedTuple):
    x: float
    y: float


# ---------------------------------------------------------------- lat/lon <-> local metres

def _project(lat: float, lon: float, origin: tuple[float, float]) -> Point:
    """Equirectangular, metres east/north of `origin` (lat, lon)."""
    scale = math.pi / 180 * EARTH_RADIUS_M
    return Point((lon - origin[1]) * scale * math.cos(math.radians(origin[0])), (lat - origin[0]) * scale)


def _unproject(point: Point, origin: tuple[float, float]) -> tuple[float, float]:
    scale = math.pi / 180 * EARTH_RADIUS_M
    return (origin[0] + point.y / scale, origin[1] + point.x / (scale * math.cos(math.radians(origin[0]))))


# ---------------------------------------------------------------- polygon geometry

def _cross(origin: Point, a: Point, b: Point) -> float:
    """>0 if origin->a->b turns left."""
    return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x)


def _bounds(polygon: Iterable[Point]) -> tuple[float, float, float, float]:
    points = list(polygon)
    return min(p.x for p in points), min(p.y for p in points), max(p.x for p in points), max(p.y for p in points)


def _rotate(point: Point, angle_rad: float) -> Point:
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    return Point(point.x * cos_a - point.y * sin_a, point.x * sin_a + point.y * cos_a)


def _point_in_polygon(point: Point, polygon: list[Point]) -> bool:
    inside = False
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        if (start.y > point.y) != (end.y > point.y):
            crossing_x = (end.x - start.x) * (point.y - start.y) / (end.y - start.y) + start.x
            if point.x < crossing_x:
                inside = not inside
    return inside


def _clip_polygon(polygon: list[Point], axis: Point, offset: float, keep_less: bool) -> list[Point]:
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
            previous_value, current_value = value(previous), value(current)
            fraction = previous_value / (previous_value - current_value)
            result.append(Point(previous.x + fraction * (current.x - previous.x),
                                previous.y + fraction * (current.y - previous.y)))
        if current_inside:
            result.append(current)
        previous, previous_inside = current, current_inside
    return result


def _polygon_segments(polygon: list[Point], lane_y: float) -> list[tuple[float, float]]:
    """The x-ranges of the horizontal line y=lane_y that lie inside the polygon."""
    intersections: list[float] = []
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        if (start.y <= lane_y < end.y) or (end.y <= lane_y < start.y):
            fraction = (lane_y - start.y) / (end.y - start.y)
            intersections.append(start.x + fraction * (end.x - start.x))
    intersections.sort()
    return list(zip(intersections[::2], intersections[1::2]))


def _convex_hull(points: list[Point]) -> list[Point]:
    """Andrew's monotone chain, counter-clockwise."""
    unique = sorted(set(points))
    if len(unique) < 3:
        return unique

    def half(sequence: list[Point]) -> list[Point]:
        chain: list[Point] = []
        for point in sequence:
            while len(chain) >= 2 and _cross(chain[-2], chain[-1], point) <= 0:
                chain.pop()
            chain.append(point)
        return chain[:-1]

    return half(unique) + half(unique[::-1])


def _min_width_angle(polygon: list[Point]) -> float:
    """Direction (radians) to run lanes along: perpendicular to the polygon's minimum width.

    The minimum width of a convex shape is perpendicular to one of its hull edges, so
    testing every hull edge is enough. Lanes along that direction are the fewest and longest.
    """
    hull = _convex_hull(polygon)
    if len(hull) < 3:
        return 0.0
    best_angle, best_extent = 0.0, float("inf")
    for start, end in zip(hull, hull[1:] + hull[:1]):
        edge_angle = math.atan2(end.y - start.y, end.x - start.x)
        ys = [_rotate(point, -edge_angle).y for point in hull]
        extent = max(ys) - min(ys)
        if extent < best_extent:
            best_angle, best_extent = edge_angle, extent
    return best_angle


def _distance_to_boundary(point: Point, polygon: list[Point]) -> float:
    best = float("inf")
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        dx, dy = end.x - start.x, end.y - start.y
        length_sq = dx * dx + dy * dy
        t = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq))
        best = min(best, math.hypot(point.x - (start.x + t * dx), point.y - (start.y + t * dy)))
    return best


def _point_within(point: Point, polygon: list[Point]) -> bool:
    """Inside the polygon or on its edge."""
    return _point_in_polygon(point, polygon) or _distance_to_boundary(point, polygon) <= BOUNDARY_TOLERANCE_M


def _segment_within(a: Point, b: Point, polygon: list[Point]) -> bool:
    """True when no part of a->b is outside the polygon (running along the edge is fine).

    Both ends being inside is not enough: a leg from one wall of a notch to the opposite
    wall touches the edge only at its ends yet crosses the excluded zone. So the leg is cut
    wherever it meets the edge and each piece is tested at its midpoint.
    """
    tol = BOUNDARY_TOLERANCE_M
    if not (_point_within(a, polygon) and _point_within(b, polygon)):
        return False
    rx, ry = b.x - a.x, b.y - a.y
    length_sq = rx * rx + ry * ry
    if length_sq <= tol * tol:
        return True
    cuts = [0.0, 1.0]
    for start, end in zip(polygon, polygon[1:] + polygon[:1]):
        sx, sy = end.x - start.x, end.y - start.y
        denominator = rx * sy - ry * sx
        if abs(denominator) <= 1e-12 * math.sqrt(length_sq * (sx * sx + sy * sy)):
            for vertex in (start, end):            # parallel: only a collinear edge can touch
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
        _point_within(Point(a.x + (low + high) / 2 * rx, a.y + (low + high) / 2 * ry), polygon)
        for low, high in zip(cuts, cuts[1:])
        if high > low
    )


def _signed_area(polygon: list[Point]) -> float:
    return sum(p.x * q.y - q.x * p.y for p, q in zip(polygon, polygon[1:] + polygon[:1])) / 2


class _BoundaryRouter:
    """Shortest path between two points that never leaves a polygon.

    Inside a simple polygon a shortest path only bends at reflex vertices (corners pointing
    into the area, like the inner corners of a notch), so those are the only waypoints a
    detour needs. Each is pulled `clearance` metres inward so the route rounds the corner.
    If no path inside exists the straight leg comes back unchanged.
    """

    def __init__(self, polygon: list[Point], clearance: float = ROUTE_CLEARANCE_M):
        ring = [p for p, q in zip(polygon, polygon[1:] + polygon[:1]) if p != q]
        self.polygon = ring
        orientation = 1.0 if _signed_area(ring) >= 0 else -1.0
        self.corners = [
            self._pulled_in(before, vertex, after, clearance)
            for before, vertex, after in zip(ring[-1:] + ring[:-1], ring, ring[1:] + ring[:1])
            if _cross(before, vertex, after) * orientation < -1e-9
        ]
        self.links: list[list[tuple[int, float]]] = [[] for _ in self.corners]
        for i, first in enumerate(self.corners):
            for j in range(i + 1, len(self.corners)):
                if _segment_within(first, self.corners[j], ring):
                    distance = math.dist(first, self.corners[j])
                    self.links[i].append((j, distance))
                    self.links[j].append((i, distance))

    def _pulled_in(self, before: Point, vertex: Point, after: Point, clearance: float) -> Point:
        to_before, to_after = math.dist(before, vertex), math.dist(after, vertex)
        dx = -((before.x - vertex.x) / to_before + (after.x - vertex.x) / to_after)
        dy = -((before.y - vertex.y) / to_before + (after.y - vertex.y) / to_after)
        norm = math.hypot(dx, dy)
        if norm < 1e-9:
            return vertex
        for standoff in (clearance, clearance / 2, clearance / 4):
            candidate = Point(vertex.x + dx / norm * standoff, vertex.y + dy / norm * standoff)
            if _point_in_polygon(candidate, self.polygon) and _distance_to_boundary(candidate, self.polygon) >= 0.99 * standoff:
                return candidate
        return vertex

    def route(self, a: Point, b: Point) -> list[Point]:
        """Waypoints to insert between a and b; empty when the straight leg stays inside."""
        if _segment_within(a, b, self.polygon):
            return []
        nodes = self.corners + [a, b]
        start, goal = len(self.corners), len(self.corners) + 1
        links = [list(corner_links) for corner_links in self.links] + [[], []]
        for end in (start, goal):
            for index, corner in enumerate(self.corners):
                if _segment_within(nodes[end], corner, self.polygon):
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


# ---------------------------------------------------------------- the lawnmower sweep

def _overlap(first: tuple[float, float], second: tuple[float, float]) -> bool:
    return max(first[0], second[0]) < min(first[1], second[1])


def _cells(scanlines: list[list[tuple[float, float]]]) -> list[list[tuple[int, float, float]]]:
    """Group lane pieces into blocks that can each be flown as one unbroken lawnmower.

    A lane crossing a notch comes back as two pieces. A piece joins the block of the piece
    on the lane before only when the two overlap one-to-one; where a lane splits in two or
    two merge into one, the blocks on either side close and new ones start.
    """
    cells: list[list[tuple[int, float, float]]] = []
    previous: list[tuple[tuple[float, float], int]] = []
    for lane_index, segments in enumerate(scanlines):
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


def _sweep(cell: list[tuple[int, float, float]], lane_ys: list[float], from_last: bool,
           first_forward: bool) -> list[tuple[Point, Point]]:
    """One block as lanes in flying order: boustrophedon from either end, either way round."""
    lanes = []
    for position, (lane_index, start, end) in enumerate(cell[::-1] if from_last else cell):
        y = lane_ys[lane_index]
        forward = first_forward == (position % 2 == 0)
        lanes.append((Point(start, y), Point(end, y)) if forward else (Point(end, y), Point(start, y)))
    return lanes


def _lawnmower_route(polygon: list[Point], spacing: float, boundary: list[Point]) -> list[Point]:
    """Boustrophedon fill of one share, in local metres.

    The share is rotated so its minimum-width direction is vertical, which makes the lanes
    horizontal along its long axis: the fewest, longest passes. Lanes are centred in the
    share (the swath is wider than the lane pitch, so both edges are still covered). Every
    connector between lanes or blocks is routed inside `boundary`, the whole area, so it
    goes round a notch instead of across it.
    """
    if len(polygon) < 3:
        return []

    angle = _min_width_angle(polygon)
    local = [_rotate(point, -angle) for point in polygon]
    _, min_y, _, max_y = _bounds(local)
    extent = max_y - min_y

    # The tolerance stops an exact multiple of the spacing tipping over ceil() into a
    # spurious extra lane, which would put lanes on the share's edges where they scan nothing.
    lanes_needed = extent / spacing
    lane_count = max(1, math.ceil(lanes_needed - 1e-9 * max(1.0, lanes_needed)))
    first_y = min_y + (extent - (lane_count - 1) * spacing) / 2
    lane_ys = [first_y + index * spacing for index in range(lane_count)]

    cells = _cells([_polygon_segments(local, y) for y in lane_ys])
    if not cells:
        return []
    router = _BoundaryRouter([_rotate(point, -angle) for point in boundary])

    lanes = _sweep(cells[0], lane_ys, False, cells[0][0][0] % 2 == 0)
    remaining = cells[1:]
    while remaining:
        # Nearest block next, by routed distance, entered at whichever of its four corners is closest.
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
    for lane_start, lane_end in lanes:
        if points:
            points.extend(router.route(points[-1], lane_start))
        points.extend((lane_start, lane_end))
    return [_rotate(point, angle) for point in points]


# ---------------------------------------------------------------- splitting the area

def _strips(polygon: list[Point], count: int) -> list[list[Point]]:
    """`count` equal-width strips along the polygon's longer axis."""
    if count <= 1:
        return [polygon[:]]
    min_x, min_y, max_x, max_y = _bounds(polygon)
    if (max_x - min_x) >= (max_y - min_y):
        axis, low_bound, high_bound = Point(1.0, 0.0), min_x, max_x
    else:
        axis, low_bound, high_bound = Point(0.0, 1.0), min_y, max_y
    step = (high_bound - low_bound) / count
    strips = []
    for index in range(count):
        strip = _clip_polygon(polygon, axis, low_bound + (index + 1) * step, True)
        strips.append(_clip_polygon(strip, axis, low_bound + index * step, False))
    return strips


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
    centre_x = sum(p.x for p in candidates) / len(candidates)
    centre_y = sum(p.y for p in candidates) / len(candidates)
    chosen = [min(candidates, key=lambda p: (p.x - centre_x) ** 2 + (p.y - centre_y) ** 2)]
    remaining = [p for p in candidates if p is not chosen[0]]
    while len(chosen) < count and remaining:
        farthest = max(remaining, key=lambda p: min((p.x - c.x) ** 2 + (p.y - c.y) ** 2 for c in chosen))
        chosen.append(farthest)
        remaining.remove(farthest)
    return chosen


def _seed_points(polygon: list[Point], count: int) -> list[Point]:
    """`count` distinct seeds inside the polygon, one per drone (a grid, refined until enough fit)."""
    min_x, min_y, max_x, max_y = _bounds(polygon)
    base_columns = max(1, round(count ** 0.5))
    base_rows = max(1, math.ceil(count / base_columns))

    inside: list[Point] = []
    for refinement in range(1, 9):
        columns, rows = base_columns * refinement, base_rows * refinement
        grid = [
            Point(min_x + (column + 0.5) * (max_x - min_x) / columns,
                  min_y + (row + 0.5) * (max_y - min_y) / rows)
            for row in range(rows)
            for column in range(columns)
        ]
        inside = _distinct([p for p in grid if _point_in_polygon(p, polygon)])
        if len(inside) >= count:
            return _spread(inside, count)

    seeds = list(inside)                      # degenerate area: pad with distinct points
    index = 0
    while len(seeds) < count:
        index += 1
        fraction = index / (count + 1)
        candidate = Point(min_x + fraction * (max_x - min_x) + index * 1e-3,
                          min_y + fraction * (max_y - min_y))
        if len(_distinct(seeds + [candidate])) > len(seeds):
            seeds.append(candidate)
    return seeds[:count]


def _voronoi_cells(polygon: list[Point], count: int) -> list[list[Point]]:
    """One cell per drone: the part of the area closer to its seed than to any other seed."""
    seeds = _seed_points(polygon, count)
    cells: list[list[Point]] = []
    for index, seed in enumerate(seeds):
        cell = polygon[:]
        for other_index, other in enumerate(seeds):
            if index == other_index:
                continue
            axis = Point(other.x - seed.x, other.y - seed.y)
            offset = (other.x * other.x + other.y * other.y - seed.x * seed.x - seed.y * seed.y) / 2
            cell = _clip_polygon(cell, axis, offset, True)
        cells.append(cell)
    return cells


# ---------------------------------------------------------------- public API

def lane_spacing_m(altitude_m: float, overlap_pct: float = 25.0, hfov_deg: float = HFOV_DEG) -> float:
    """Lane pitch for a camera at this altitude: the ground swath minus the side overlap."""
    swath = 2 * altitude_m * math.tan(math.radians(hfov_deg) / 2)
    return max(swath * (1 - overlap_pct / 100), 0.1)


def drone_altitude_m(index: int, base_m: float, layer_spacing_m: float) -> float:
    """Altitude of the drone at 0-based `index`: the base, one layer up, one down, two up, ..."""
    step = (index + 1) // 2
    return base_m + (step if index % 2 else -step) * layer_spacing_m


def plan_scan(ops_area: list[tuple[float, float]], drones: int = 3, altitude_m: float = 30.0,
              scan_type: str = "lawnmower", overlap_pct: float = 25.0, hfov_deg: float = HFOV_DEG,
              layer_spacing_m: float = 2.0,
              origin: tuple[float, float] | None = None) -> list[list[tuple[float, float, float]]]:
    """Search path for each drone over `ops_area`, a polygon of (lat, lon) vertices.

    Returns one list of (lat, lon, altitude_m) per drone, in flying order. A drone whose
    share comes out empty gets an empty list. `origin` is the (lat, lon) the area is
    flattened around; it defaults to the centre of the vertices and barely matters.
    """
    if scan_type not in SCAN_TYPES:
        raise ValueError(f"scan_type must be one of {SCAN_TYPES}")
    if drones < 1:
        raise ValueError("drones must be at least 1")
    if altitude_m <= 0:
        raise ValueError("altitude_m must be positive")
    ring = list(ops_area)
    if len(ring) > 1 and ring[0] == ring[-1]:
        ring.pop()                            # a KML ring repeats its first vertex
    if len(ring) < 3:
        raise ValueError("ops_area needs at least 3 vertices")
    origin = origin or (sum(lat for lat, _ in ring) / len(ring), sum(lon for _, lon in ring) / len(ring))

    area = [_project(lat, lon, origin) for lat, lon in ring]
    shares = _voronoi_cells(area, drones) if scan_type == "voronoi" else _strips(area, drones)

    paths = []
    for index, share in enumerate(shares):
        altitude = drone_altitude_m(index, altitude_m, layer_spacing_m)
        if altitude <= 0:
            raise ValueError("an altitude layer is at or below the ground; lower layer_spacing_m or raise altitude_m")
        points = _lawnmower_route(share, lane_spacing_m(altitude, overlap_pct, hfov_deg), area)
        paths.append([(*_unproject(point, origin), altitude) for point in points])
    return paths


# ---------------------------------------------------------------- command line

def read_kml_area(path: str) -> list[tuple[float, float]]:
    """The (lat, lon) vertices of the placemark named OpsArea in a KML file."""
    name_of = lambda tag: tag.rsplit("}", 1)[-1]
    for placemark in ET.parse(path).getroot().iter():
        if name_of(placemark.tag) != "Placemark":
            continue
        name = next((child.text or "" for child in placemark if name_of(child.tag) == "name"), "")
        if name.strip().lower() != "opsarea":
            continue
        for element in placemark.iter():
            if name_of(element.tag) == "coordinates":
                points = []
                for raw in (element.text or "").split():
                    values = raw.split(",")
                    if len(values) >= 2:
                        points.append((float(values[1]), float(values[0])))
                if len(points) >= 3:
                    return points
    raise ValueError(f"no OpsArea polygon found in {path}")


def _demo_area() -> list[tuple[float, float]]:
    """A 600 x 400 m U with a 200 m wide corridor cut out of its top edge."""
    origin = (1.4000, 103.9600)
    u_shape = [(100, 0), (700, 0), (700, 400), (500, 400), (500, 150), (300, 150), (300, 400), (100, 400)]
    return [_unproject(Point(x, y), origin) for x, y in u_shape]


def main() -> None:
    parser = argparse.ArgumentParser(description="Split an area between drones and lawnmower-sweep each share.")
    parser.add_argument("kml", nargs="?", help="KML with an OpsArea polygon (default: built-in U-shape demo)")
    parser.add_argument("--drones", type=int, default=3)
    parser.add_argument("--alt", type=float, default=30.0, help="base altitude, metres")
    parser.add_argument("--overlap", type=float, default=25.0, help="side overlap, percent")
    parser.add_argument("--scan-type", choices=SCAN_TYPES, default="lawnmower")
    args = parser.parse_args()

    area = read_kml_area(args.kml) if args.kml else _demo_area()
    paths = plan_scan(area, args.drones, args.alt, args.scan_type, args.overlap)

    print(f"{args.scan_type}, {args.drones} drones, base altitude {args.alt:g} m, {args.overlap:g}% overlap")
    for number, path in enumerate(paths, start=1):
        if not path:
            print(f"  drone {number}: no waypoints")
            continue
        local = [_project(lat, lon, path[0][:2]) for lat, lon, _ in path]
        length = sum(math.dist(a, b) for a, b in zip(local, local[1:]))
        altitude = path[0][2]
        print(f"  drone {number}: {len(path):3d} waypoints, {altitude:4.1f} m, "
              f"lanes {lane_spacing_m(altitude, args.overlap):4.1f} m apart, {length:6.0f} m of path")


if __name__ == "__main__":
    main()
