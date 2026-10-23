#!/usr/bin/env python3
"""Render a KML sector + its generated .plan over satellite imagery.

    python tools/plot_mission.py ops/pulau-ubin-west.kml missions/ubin/mission.plan out.png

Basemap tiles come from ESRI World Imagery. With no network the overlay still renders
on a plain background, so this is safe to run on the field laptop.

A plan with a dispersal gate (ingress_style "gate") also gets the shared ingress leg, the
gate itself, and the fan-out to each drone's first waypoint -- the part of the flight that
the waypoint list alone does not show.
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

DRONE_COLOURS = ["#00e5ff", "#ffd400", "#ff5ec4", "#7cff6b", "#ff8c42"]

def mission_paths(plan: dict) -> dict[str, list[tuple[float, float]]]:
    """(lon, lat) per drone tag, waypoints only."""
    paths: dict[str, list[tuple[float, float]]] = {}
    for item in plan["mission"]["items"]:
        if item["command"] != 16:
            continue
        paths.setdefault(item.get("comment", "drone-1"), []).append((item["params"][5], item["params"][4]))
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("kml", type=Path)
    parser.add_argument("plan", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--zoom", type=int, default=17)
    parser.add_argument("--title", default=None)
    args = parser.parse_args()

    mission = parse_kml(args.kml)
    plan = json.loads(args.plan.read_text())

    fence = [(c.longitude, c.latitude) for c in mission.geo_fence.coordinates]
    ops = [(c.longitude, c.latitude) for c in mission.ops_area.coordinates]
    lons = [p[0] for p in fence + ops]
    lats = [p[1] for p in fence + ops]
    pad_lon = (max(lons) - min(lons)) * 0.12
    pad_lat = (max(lats) - min(lats)) * 0.12
    bounds = (min(lats) - pad_lat, min(lons) - pad_lon, max(lats) + pad_lat, max(lons) + pad_lon)

    figure, axes = plt.subplots(figsize=(12, 10), dpi=130)
    basemap, extent = fetch_basemap(*bounds, args.zoom)
    if basemap is not None:
        axes.imshow(basemap, extent=extent, aspect="auto", zorder=0)
    else:
        axes.set_facecolor("#101418")

    axes.add_patch(MplPolygon(fence, closed=True, fill=False, edgecolor="#ff3b30", lw=2.6, ls="--",
                              label="GeoFence (hard boundary)", zorder=3))
    axes.add_patch(MplPolygon(ops, closed=True, facecolor="#ffd400", alpha=0.13, edgecolor="#ffd400",
                              lw=2.2, label="OpsArea (search sector)", zorder=2))

    # The dispersal gate, when the swarm flies in as a group (planner.ingress, qgc_plan.py).
    ingress = plan.get("planner", {}).get("ingress") or {}
    gate = ingress.get("gate")
    home = mission.gcs_location

    for index, (tag, points) in enumerate(sorted(mission_paths(plan).items())):
        colour = DRONE_COLOURS[index % len(DRONE_COLOURS)]
        axes.plot([p[0] for p in points], [p[1] for p in points], "-", color=colour, lw=1.5, alpha=0.95,
                  label=f"{tag} ({len(points)} wp)", zorder=4)
        axes.plot(points[0][0], points[0][1], "^", color=colour, ms=11, mec="black", mew=0.8, zorder=6)
        # Where this drone leaves the formation and runs out to its own cell.
        split = (gate["longitude"], gate["latitude"]) if gate else (home.longitude, home.latitude)
        axes.plot([split[0], points[0][0]], [split[1], points[0][1]], ":", color=colour, lw=1.2,
                  alpha=0.9, zorder=4)

    if gate:
        # One leg, flown by every drone, each on its own transit layer.
        axes.plot([home.longitude, gate["longitude"]], [home.latitude, gate["latitude"]], "-",
                  color="white", lw=5, alpha=0.5, zorder=3)
        axes.plot(gate["longitude"], gate["latitude"], "*", color="#ff3b30", ms=26, mec="white", mew=1.4,
                  label=f"dispersal gate ({ingress.get('objective', 'n/a')})", zorder=8)

    axes.plot(home.longitude, home.latitude, "*", color="white", ms=24, mec="black", mew=1.2,
              label="GCS / home", zorder=7)

    imaging = plan.get("planner", {}).get("imaging", {})
    axes.set_title(
        args.title or f"{args.kml.stem} — {plan['x-multiDrone']['droneCount']} drones, "
        f"{imaging.get('altitude_m')} m AGL, {imaging.get('lane_spacing_m')} m lanes, "
        f"{imaging.get('gsd_cm_px')} cm/px, {imaging.get('cruise_speed_m_s')} m/s",
        fontsize=12, pad=12,
    )
    axes.set_xlabel("longitude"); axes.set_ylabel("latitude")
    axes.set_xlim(bounds[1], bounds[3]); axes.set_ylim(bounds[0], bounds[2])
    legend = axes.legend(loc="upper right", framealpha=0.85, fontsize=9)
    legend.set_zorder(10)
    figure.tight_layout()
    figure.savefig(args.out, bbox_inches="tight")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
