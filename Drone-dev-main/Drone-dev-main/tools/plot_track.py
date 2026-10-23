#!/usr/bin/env python3
"""Overlay a flown track (from tools/fly_mission.py) on the plan it came from.

    python tools/plot_track.py ops/x.kml missions/ubin/mission_drones/drone-1.plan \
        missions/ubin/track.json out.png
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from tools_basemap import fetch_basemap  # noqa: E402
from flight_planner.kml_parser import parse_kml  # noqa: E402

R = 6378137.0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("kml", type=Path); ap.add_argument("plan", type=Path)
    ap.add_argument("track", type=Path); ap.add_argument("out", type=Path)
    ap.add_argument("--zoom", type=int, default=16)
    args = ap.parse_args()

    mission = parse_kml(args.kml)
    plan = json.loads(args.plan.read_text())
    track = json.loads(args.track.read_text())

    planned = [(i["params"][5], i["params"][4]) for i in plan["mission"]["items"] if i["command"] == 16]
    flown = [(s["lon"], s["lat"]) for s in track]
    fence = [(c.longitude, c.latitude) for c in mission.geo_fence.coordinates]
    ops = [(c.longitude, c.latitude) for c in mission.ops_area.coordinates]

    lons = [p[0] for p in planned + flown + fence]
    lats = [p[1] for p in planned + flown + fence]
    dlon = (max(lons) - min(lons)) * 0.10 or 1e-4
    dlat = (max(lats) - min(lats)) * 0.10 or 1e-4
    bounds = (min(lats) - dlat, min(lons) - dlon, max(lats) + dlat, max(lons) + dlon)

    fig, ax = plt.subplots(figsize=(12, 11), dpi=130)
    basemap, extent = fetch_basemap(*bounds, args.zoom)
    if basemap is not None:
        ax.imshow(basemap, extent=extent, aspect="auto", zorder=0)
    else:
        ax.set_facecolor("#101418")

    ax.add_patch(MplPolygon(fence, closed=True, fill=False, edgecolor="#ff3b30", lw=2.2, ls="--", label="GeoFence", zorder=2))
    ax.add_patch(MplPolygon(ops, closed=True, facecolor="#ffd400", alpha=0.10, edgecolor="#ffd400", lw=1.6, label="OpsArea", zorder=2))
    ax.plot([p[0] for p in planned], [p[1] for p in planned], "-o", color="#ffd400", lw=2.4, ms=4,
            label=f"planned ({len(planned)} wp)", zorder=4)
    ax.plot([p[0] for p in flown], [p[1] for p in flown], "-", color="#00e5ff", lw=1.5, alpha=0.9,
            label=f"flown ({len(flown)} samples)", zorder=5)

    lat0 = math.radians(plan["mission"]["plannedHomePosition"][0])
    def xy(lon, lat): return (math.radians(lon) * R * math.cos(lat0), math.radians(lat) * R)
    def point_to_segment(px, py, ax, ay, bx, by):
        """Distance from a point to a segment -- mid-lane samples are on track, and
        measuring to the nearest waypoint instead would call them half a lane off."""
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq == 0:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    legs = [(xy(*a), xy(*b)) for a, b in zip(planned, planned[1:])]
    first, last = xy(*planned[0]), xy(*planned[-1])
    # Neither the climb-out transit (home -> waypoint 0) nor the RTL return is a planned
    # leg, so scoring them as cross-track error reads as a huge excursion -- the RTL alone
    # showed 482 m on the Toa Payoh flight. Bound the window to the survey itself: from
    # first arrival at waypoint 0 to last departure from the final waypoint.
    projected = [xy(flon, flat) for flon, flat in flown]
    starts = [i for i, p in enumerate(projected) if math.dist(p, first) < 5.0]
    ends = [i for i, p in enumerate(projected) if math.dist(p, last) < 5.0]
    lo = starts[0] if starts else 0
    hi = ends[-1] if ends and ends[-1] > lo else len(projected) - 1
    dev = [min(point_to_segment(px, py, a[0], a[1], b[0], b[1]) for a, b in legs)
           for px, py in projected[lo:hi + 1]] or [0.0]
    flown_m = sum(math.dist(xy(*a), xy(*b)) for a, b in zip(flown, flown[1:]))
    planned_m = sum(math.dist(xy(*a), xy(*b)) for a, b in zip(planned, planned[1:]))

    ax.set_title(f"Flown vs planned — {args.plan.stem}\n"
                 f"planned {planned_m/1000:.2f} km · flown {flown_m/1000:.2f} km · "
                 f"survey cross-track: max {max(dev):.2f} m, mean {sum(dev)/len(dev):.2f} m "
                 f"(excl. climb-out transit)", fontsize=11, pad=12)
    ax.set_xlim(bounds[1], bounds[3]); ax.set_ylim(bounds[0], bounds[2])
    ax.set_xlabel("longitude"); ax.set_ylabel("latitude")
    ax.legend(loc="upper right", framealpha=0.85, fontsize=9).set_zorder(10)
    fig.tight_layout(); fig.savefig(args.out, bbox_inches="tight")
    print(f"wrote {args.out}")
    print(f"  planned {planned_m/1000:.3f} km | flown {flown_m/1000:.3f} km | "
          f"survey cross-track max {max(dev):.2f} m, mean {sum(dev)/len(dev):.2f} m")


if __name__ == "__main__":
    main()
