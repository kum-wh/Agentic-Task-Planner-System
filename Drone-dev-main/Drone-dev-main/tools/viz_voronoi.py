"""Render the four stages of voronoi_scan() using the real functions from scanner_tools."""
import math, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPoly

sys.path.insert(0, "src")
from flight_planner.kml_parser import parse_kml
from flight_planner.config import load_config
from flight_planner.geometry import Point, project, bounds, clip_polygon, point_in_polygon, min_width_angle
from flight_planner.scanner_tools import _seed_points, _lawnmower_points

mission = parse_kml("ops/pulau-ubin-west.kml")
config  = load_config("config.example.json")
origin  = mission.gcs_location
poly    = [project(p, origin) for p in mission.ops_area.coordinates]
count   = config.number_of_drones
COL     = ["#00b3d6", "#e8a33d", "#e84393"]

seeds = _seed_points(poly, count)
xy    = lambda pts: ([p.x for p in pts], [p.y for p in pts])

fig, axes = plt.subplots(2, 2, figsize=(15, 13))
for ax in axes.flat:
    ax.set_aspect("equal"); ax.grid(alpha=.15, lw=.5)
    ax.set_xlabel("east (m)"); ax.set_ylabel("north (m)")

# ---- A: seeding ---------------------------------------------------------------
ax = axes[0][0]
ax.add_patch(MplPoly([(p.x, p.y) for p in poly], closed=True, fc="#f5e9a8", ec="#8a7d2a", lw=2, alpha=.55))
min_x, min_y, max_x, max_y = bounds(poly)
base_c = max(1, round(count ** .5)); base_r = max(1, math.ceil(count / base_c))
for refine in (1, 2):
    cols, rows = base_c * refine, base_r * refine
    grid = [Point(min_x + (c + .5) * (max_x - min_x) / cols, min_y + (r + .5) * (max_y - min_y) / rows)
            for r in range(rows) for c in range(cols)]
    ins  = [p for p in grid if point_in_polygon(p, poly)]
    outs = [p for p in grid if not point_in_polygon(p, poly)]
    m = "o" if refine == 1 else "s"
    if ins:  ax.scatter(*xy(ins),  s=48, marker=m, fc="none", ec="#2d7d46", lw=1.6, zorder=3,
                        label=f"grid {cols}x{rows} — inside" if refine == 1 else None)
    if outs: ax.scatter(*xy(outs), s=40, marker="x", c="#c0392b", lw=1.4, alpha=.65, zorder=3,
                        label="rejected (outside polygon)" if refine == 1 else None)
for i, s in enumerate(seeds):
    ax.scatter([s.x], [s.y], s=340, marker="*", c=COL[i], ec="k", lw=1.2, zorder=6)
    ax.annotate(f"seed {i+1}", (s.x, s.y), textcoords="offset points", xytext=(12, 10),
                fontsize=11, weight="bold", color=COL[i])
ax.set_title("A. _seed_points() — grid, refine until `count` land inside,\nthen _spread() farthest-point sampling", fontsize=12, weight="bold")
ax.legend(loc="lower left", fontsize=8)

# ---- B: half-plane clipping ---------------------------------------------------
ax = axes[0][1]
ax.add_patch(MplPoly([(p.x, p.y) for p in poly], closed=True, fc="none", ec="#8a7d2a", lw=2, ls="--"))
span = max(max_x - min_x, max_y - min_y)
focus = 0
for j, other in enumerate(seeds):
    if j == focus: continue
    s = seeds[focus]
    axis = Point(other.x - s.x, other.y - s.y)
    off  = (other.x**2 + other.y**2 - s.x**2 - s.y**2) / 2
    n = math.hypot(axis.x, axis.y); ux, uy = axis.x / n, axis.y / n
    px, py = -uy, ux                      # along the bisector
    cx, cy = ux * off / n, uy * off / n   # foot of the perpendicular
    ax.plot([cx - px*span, cx + px*span], [cy - py*span, cy + py*span],
            color=COL[j], lw=2.4, ls="--", zorder=4,
            label=f"bisector: seed{focus+1} | seed{j+1}")
    ax.annotate("", xy=(cx - ux*span*.16, cy - uy*span*.16), xytext=(cx, cy),
                arrowprops=dict(arrowstyle="-|>", lw=2.2, color=COL[j]), zorder=5)
cell0 = poly[:]
for j, other in enumerate(seeds):
    if j == focus: continue
    s = seeds[focus]
    cell0 = clip_polygon(cell0, Point(other.x - s.x, other.y - s.y),
                         (other.x**2 + other.y**2 - s.x**2 - s.y**2) / 2, True)
ax.add_patch(MplPoly([(p.x, p.y) for p in cell0], closed=True, fc=COL[focus], ec=COL[focus], lw=2.5, alpha=.35, zorder=2))
for i, s in enumerate(seeds):
    ax.scatter([s.x], [s.y], s=300, marker="*", c=COL[i], ec="k", lw=1.2, zorder=6)
ax.set_title("B. clip_polygon() — keep the side nearer seed 1 than each rival.\nArrows point into the kept half-plane; shaded = what survives", fontsize=12, weight="bold")
ax.legend(loc="lower left", fontsize=8)

# ---- C: the three cells -------------------------------------------------------
ax = axes[1][0]
cells = []
for i, s in enumerate(seeds):
    cell = poly[:]
    for j, other in enumerate(seeds):
        if i == j: continue
        cell = clip_polygon(cell, Point(other.x - s.x, other.y - s.y),
                            (other.x**2 + other.y**2 - s.x**2 - s.y**2) / 2, True)
    cells.append(cell)
    ax.add_patch(MplPoly([(p.x, p.y) for p in cell], closed=True, fc=COL[i], ec=COL[i], lw=2.2, alpha=.4))
    cxx = sum(p.x for p in cell)/len(cell); cyy = sum(p.y for p in cell)/len(cell)
    a = abs(sum(cell[k].x*cell[(k+1) % len(cell)].y - cell[(k+1) % len(cell)].x*cell[k].y for k in range(len(cell))))/2
    ax.annotate(f"drone-{i+1}\n{a/10000:.2f} ha", (cxx, cyy), ha="center", fontsize=11, weight="bold")
    ax.scatter([seeds[i].x], [seeds[i].y], s=300, marker="*", c=COL[i], ec="k", lw=1.2, zorder=6)
ax.add_patch(MplPoly([(p.x, p.y) for p in poly], closed=True, fc="none", ec="#8a7d2a", lw=2, ls="--"))
ax.set_title("C. One cell per drone — the L-shape is cut along its own\ngeometry, not into equal-width vertical strips", fontsize=12, weight="bold")

# ---- D: lawnmower inside each cell --------------------------------------------
ax = axes[1][1]
for i, cell in enumerate(cells):
    ax.add_patch(MplPoly([(p.x, p.y) for p in cell], closed=True, fc=COL[i], ec=COL[i], lw=1.2, alpha=.14))
    pts = _lawnmower_points(cell, config.lane_spacing_m)
    if pts:
        ax.plot(*xy(pts), color=COL[i], lw=1.7, zorder=4)
        L = sum(math.hypot(pts[k+1].x-pts[k].x, pts[k+1].y-pts[k].y) for k in range(len(pts)-1))
        ang = math.degrees(min_width_angle(cell))
        cxx = sum(p.x for p in cell)/len(cell); cyy = sum(p.y for p in cell)/len(cell)
        ax.annotate(f"{L/1000:.2f} km\nlanes @ {ang:.0f}°", (cxx, cyy), ha="center",
                    fontsize=10, weight="bold", bbox=dict(fc="white", alpha=.75, ec="none", pad=1.5))
ax.add_patch(MplPoly([(p.x, p.y) for p in poly], closed=True, fc="none", ec="#8a7d2a", lw=2, ls="--"))
ax.set_title("D. _lawnmower_points() per cell — each cell rotated to its OWN\nmin-width angle, so every cell gets the fewest, longest passes", fontsize=12, weight="bold")

fig.suptitle("voronoi_scan() on Pulau Ubin West — what the code actually does", fontsize=16, weight="bold")
fig.tight_layout(rect=[0, 0, 1, .975])
fig.savefig(sys.argv[1], dpi=110, bbox_inches="tight")
print("wrote", sys.argv[1])
