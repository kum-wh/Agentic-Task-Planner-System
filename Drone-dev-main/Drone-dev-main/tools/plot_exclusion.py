#!/usr/bin/env python3
"""Highlight flight legs that leave the OpsArea — i.e. cross an exclusion zone.

    python tools/plot_exclusion.py ops/x.kml missions/y/mission.plan out.png

Until 10 Sep 2026 the guardrail checked only the GeoFence, so a notch cut out of the
search area (a road, a rail corridor, a no-fly zone) stopped the SCANNING but not the
FLYING. Survey legs are now held to the OpsArea too; this still draws exactly which legs
of a given .plan cross it and how far -- which is what you want for a plan written
before that fix.
"""
from __future__ import annotations

import argparse, json, math, sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from tools_basemap import fetch_basemap                       # noqa: E402
from flight_planner.geometry import Point, point_in_polygon, project   # noqa: E402
from flight_planner.kml_parser import parse_kml               # noqa: E402
from flight_planner.models import Coordinate                  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("kml", type=Path); ap.add_argument("plan", type=Path)
    ap.add_argument("out", type=Path); ap.add_argument("--zoom", type=int, default=17)
    args = ap.parse_args()

    mission = parse_kml(args.kml)
    plan = json.loads(args.plan.read_text())
    ops_local = [project(c, mission.gcs_location) for c in mission.ops_area.coordinates]
    edges = list(zip(ops_local, ops_local[1:] + ops_local[:1]))

    def dist_to_boundary(p: Point) -> float:
        best = float("inf")
        for a, b in edges:
            dx, dy = b.x - a.x, b.y - a.y
            length_sq = dx * dx + dy * dy
            t = 0.0 if length_sq == 0 else max(0.0, min(1.0, ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq))
            best = min(best, math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy)))
        return best

    inside_legs, outside_legs, breach_m = [], [], 0.0
    by_drone: dict[str, list] = {}
    for item in plan["mission"]["items"]:
        if item["command"] == 16:
            by_drone.setdefault(item.get("comment", "drone-1"), []).append(item)
    for items in by_drone.values():
        geo = [(i["params"][5], i["params"][4]) for i in items]
        loc = [project(Coordinate(lon, lat), mission.gcs_location) for lon, lat in geo]
        for (ga, gb), (la, lb) in zip(zip(geo, geo[1:]), zip(loc, loc[1:])):
            mid = Point((la.x + lb.x) / 2, (la.y + lb.y) / 2)
            # Interior to a hole, not merely running along the boundary.
            if not point_in_polygon(mid, ops_local) and dist_to_boundary(mid) > 1.0:
                outside_legs.append((ga, gb)); breach_m += math.dist(la, lb)
            else:
                inside_legs.append((ga, gb))

    fence = [(c.longitude, c.latitude) for c in mission.geo_fence.coordinates]
    ops = [(c.longitude, c.latitude) for c in mission.ops_area.coordinates]
    lons = [p[0] for p in fence + ops]; lats = [p[1] for p in fence + ops]
    dlon = (max(lons) - min(lons)) * 0.06; dlat = (max(lats) - min(lats)) * 0.06
    bounds = (min(lats) - dlat, min(lons) - dlon, max(lats) + dlat, max(lons) + dlon)

    fig, ax = plt.subplots(figsize=(12, 11), dpi=130)
    basemap, extent = fetch_basemap(*bounds, args.zoom)
    if basemap is not None:
        ax.imshow(basemap, extent=extent, aspect="auto", zorder=0)
    else:
        ax.set_facecolor("#101418")

    ax.add_patch(MplPolygon(fence, closed=True, fill=False, edgecolor="#ff3b30", lw=2.0, ls="--",
                            label="GeoFence", zorder=3))
    ax.add_patch(MplPolygon(ops, closed=True, facecolor="#ffd400", alpha=0.16, edgecolor="#ffd400",
                            lw=2.0, label="OpsArea (note the notch)", zorder=2))
    for n, (a, b) in enumerate(inside_legs):
        ax.plot([a[0], b[0]], [a[1], b[1]], "-", color="#00e5ff", lw=1.3, zorder=4,
                label="legs inside the search area" if n == 0 else None)
    for n, (a, b) in enumerate(outside_legs):
        ax.plot([a[0], b[0]], [a[1], b[1]], "-", color="#ff2d55", lw=3.4, zorder=6,
                label="LEGS CROSSING THE EXCLUSION ZONE" if n == 0 else None)

    home = mission.gcs_location
    ax.plot(home.longitude, home.latitude, "*", color="white", ms=20, mec="black", mew=1.0,
            label="GCS / home", zorder=7)
    ax.set_title(f"{args.kml.stem} — {len(outside_legs)} legs cross the excluded corridor, "
                 f"{breach_m:.0f} m flown over it\nguardrail verdict: "
                 f"{plan['planner']['guardrailPassed']} (as recorded when the plan was written)", fontsize=12, pad=12)
    ax.set_xlim(bounds[1], bounds[3]); ax.set_ylim(bounds[0], bounds[2])
    ax.set_xlabel("longitude"); ax.set_ylabel("latitude")
    ax.legend(loc="upper right", framealpha=0.88, fontsize=9).set_zorder(10)
    fig.tight_layout(); fig.savefig(args.out, bbox_inches="tight")
    print(f"wrote {args.out}: {len(outside_legs)} breaching legs, {breach_m:.0f} m")


if __name__ == "__main__":
    main()
