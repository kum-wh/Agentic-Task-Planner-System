#!/usr/bin/env python3
"""Plan a KML scenario in-process and draw what the search pattern does.

    python tools/draw_scenario.py scenarios/u_shape_notch.kml
    python tools/draw_scenario.py scenarios/rectangle.kml --drones 2 --alt 40
    python tools/draw_scenario.py scenarios/diagonal_field.kml --scan-type voronoi

Offline: no basemap, no simulator and no .plan file. It runs the same planner and guardrail
the CLI runs (MultiAgentPlanner) and draws in local metres around the GCS location.

The pale band under each path is that drone's real camera swath at its own altitude, so
gaps and overlap between lanes are visible. Dashed red is the GeoFence, grey is the OpsArea,
a hollow circle is a drone's first waypoint and a hollow square its last.

It draws even when the guardrail rejects the mission, since that is when you most want to
see it. Needs matplotlib (pip install matplotlib).
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import replace
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from flight_planner.agents import MultiAgentPlanner  # noqa: E402
from flight_planner.geometry import project  # noqa: E402
from flight_planner.kml_parser import parse_kml  # noqa: E402
from flight_planner.models import PlannerConfig  # noqa: E402

DRONE_COLOURS = ["#00a6c8", "#e0a800", "#d6308f", "#3aa84a", "#e8702a", "#7a5cd6", "#555555", "#a0522d"]


def survey_stats(result, config):
    """Per-drone numbers worth eyeballing next to the picture."""
    rows = []
    for drone in result.drones:
        if not drone.waypoints:
            rows.append((drone.drone_id, 0, drone.altitude_m, 0.0, 0.0, 0.0))
            continue
        pts = [project(w, result.mission.gcs_location) for w in drone.waypoints]
        length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        layer = replace(config, scan_altitude_m=drone.altitude_m)
        rows.append((drone.drone_id, len(pts), drone.altitude_m, layer.lane_spacing_m,
                     length, length / drone.cruise_speed_m_s / 60))
    return rows


def draw_panel(ax, result, config, scan_type):
    mission = result.mission
    origin = mission.gcs_location
    fence = [project(c, origin) for c in mission.geo_fence.coordinates]
    ops = [project(c, origin) for c in mission.ops_area.coordinates]

    ax.add_patch(MplPolygon(fence, closed=True, fill=False, ec="#cc3333", ls="--", lw=1.2, label="GeoFence", zorder=1))
    ax.add_patch(MplPolygon(ops, closed=True, fc="#ececec", ec="#444444", lw=1.3, label="OpsArea", zorder=1))
    ax.plot(0, 0, "*", ms=15, color="black", label="home / GCS", zorder=6)

    bands = []
    for drone in result.drones:
        if not drone.waypoints:
            continue
        colour = DRONE_COLOURS[(drone.drone_id - 1) % len(DRONE_COLOURS)]
        pts = [project(w, origin) for w in drone.waypoints]
        xs, ys = [p.x for p in pts], [p.y for p in pts]
        swath = replace(config, scan_altitude_m=drone.altitude_m).profile.swath_w_m
        band, = ax.plot(xs, ys, color=colour, alpha=0.25, solid_capstyle="butt", solid_joinstyle="round", zorder=2)
        bands.append((band, swath))
        ax.plot(xs, ys, color=colour, lw=1.3, zorder=3, label=f"drone-{drone.drone_id}  {drone.altitude_m:.0f} m")
        ax.plot(xs, ys, ".", ms=3, color=colour, zorder=3)
        ax.plot(xs[0], ys[0], "o", mfc="white", mec=colour, mew=2, ms=9, zorder=5)
        ax.plot(xs[-1], ys[-1], "s", mfc="white", mec=colour, mew=2, ms=9, zorder=5)

    corners = fence + ops + [project(origin, origin)]
    pad = 0.04 * max(max(p.x for p in corners) - min(p.x for p in corners),
                     max(p.y for p in corners) - min(p.y for p in corners))
    ax.set_xlim(min(p.x for p in corners) - pad, max(p.x for p in corners) + pad)
    ax.set_ylim(min(p.y for p in corners) - pad, max(p.y for p in corners) + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("metres east of home")
    ax.set_ylabel("metres north of home")
    ax.grid(alpha=0.25)

    status = "guardrail PASS" if result.guardrail_passed else "guardrail REJECTED"
    ax.set_title(f"{scan_type}  ·  {config.number_of_drones} drones  ·  {status}",
                 color="#1a7f37" if result.guardrail_passed else "#cc3333", fontsize=11)
    stats = ["drone  wpts   alt  lane   survey   est.time"]
    stats += [f"  {i}   {n:4d}  {a:4.0f}m {s:4.1f}m  {l:6.0f}m   {t:5.1f} min"
              for i, n, a, s, l, t in survey_stats(result, config)]
    ax.text(0.0, -0.13, "\n".join(stats), transform=ax.transAxes, va="top", fontsize=8, family="monospace")
    ax.legend(loc="best", fontsize=7, framealpha=0.85)
    return bands


def fit_swath_widths(fig, ax, bands):
    """Make each band as wide in the picture as the camera swath is on the ground.

    Line widths are in points, not metres, so convert using the axes' scale once its
    aspect has settled.
    """
    fig.canvas.draw()
    (x0, _), (x1, _) = ax.transData.transform([(0, 0), (1, 0)])
    pixels_per_metre = x1 - x0
    for band, swath_m in bands:
        band.set_linewidth(swath_m * pixels_per_metre * 72 / fig.dpi)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("kml", type=Path)
    ap.add_argument("--drones", type=int, default=3)
    ap.add_argument("--alt", type=float, default=30.0, help="base scan altitude, metres")
    ap.add_argument("--overlap", type=float, default=25.0, help="side/forward overlap, percent")
    ap.add_argument("--scan-type", choices=["lawnmower", "voronoi", "both"], default="both")
    ap.add_argument("--out", type=Path, help="PNG to write (default: missions/scenarios/<kml name>.png)")
    args = ap.parse_args()

    mission = parse_kml(args.kml)
    kinds = ["lawnmower", "voronoi"] if args.scan_type == "both" else [args.scan_type]
    out = args.out or ROOT / "missions" / "scenarios" / f"{args.kml.stem}.png"
    out.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, len(kinds), figsize=(8 * len(kinds), 8.6), squeeze=False)
    fig.suptitle(f"{args.kml.name}  ·  base altitude {args.alt:g} m  ·  {args.overlap:g}% overlap", fontsize=12)
    exit_code = 0
    for ax, kind in zip(axes[0], kinds):
        config = PlannerConfig(number_of_drones=args.drones, scan_altitude_m=args.alt,
                               scan_overlap_percent=args.overlap, scan_type=kind)
        config.validate()
        result = MultiAgentPlanner().run(mission, config)
        print(f"\n== {kind}: guardrail {'PASS' if result.guardrail_passed else 'REJECTED'}")
        for row in survey_stats(result, config):
            print("   drone %d: %d waypoints, %.0f m, lanes %.1f m apart, survey %.0f m, ~%.1f min" % (
                row[0], row[1], row[2], row[3], row[4], row[5]))
        for warning in result.warnings:
            print("   warning:", warning)
        exit_code |= 0 if result.guardrail_passed else 1
        bands = draw_panel(ax, result, config, kind)
        fit_swath_widths(fig, ax, bands)

    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    fig.subplots_adjust(bottom=0.2)
    fig.savefig(out, dpi=130)
    print(f"\nwrote {out}")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
