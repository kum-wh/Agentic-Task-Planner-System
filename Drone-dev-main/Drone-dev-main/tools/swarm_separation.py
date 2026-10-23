#!/usr/bin/env python3
"""Measure how close three SITL vehicles actually came to each other.

    python tools/swarm_separation.py missions/toapayoh/swarm-2026-09-23/common-clock-tracks.json

The planner's guardrail refuses a fleet plan unless every pair of drones is
separated leg-by-leg -- MIN_VERTICAL_SEPARATION_M 2.0 or MIN_HORIZONTAL_SEPARATION_M
20.0. That is a claim about the *plan*. This measures the claim against what the
vehicles flew.

Input is the common-clock recording (one wall clock for all vehicles). Per-flight
tracks from fly_mission.py cannot be used: each stamps time from its own arming
moment, and a 2 s offset is 24 m of error at survey speed.

A pair is "in conflict" at a sample only if it fails BOTH limits at once -- drones
stacked in altitude layers are meant to overfly one another.
"""

from __future__ import annotations

import json
import math
import sys
from itertools import combinations
from pathlib import Path

MIN_VERTICAL_SEPARATION_M = 2.0
MIN_HORIZONTAL_SEPARATION_M = 20.0
EARTH_R = 6378137.0

# Vehicles parked on the ground a few metres apart break both limits trivially and
# for the whole time they sit there. Only judge a pair while BOTH are airborne.
AIRBORNE_M = 2.0


def horizontal_m(a: dict, b: dict) -> float:
    """Equirectangular distance -- exact enough over the hundreds of metres here."""
    lat = math.radians((a["lat"] + b["lat"]) / 2.0)
    dx = math.radians(b["lon"] - a["lon"]) * math.cos(lat) * EARTH_R
    dy = math.radians(b["lat"] - a["lat"]) * EARTH_R
    return math.hypot(dx, dy)


def resample(track: list[dict], step: float) -> dict[int, dict]:
    """Bucket samples onto a fixed grid so pairs are compared at the same instant."""
    grid: dict[int, dict] = {}
    for s in track:
        grid[int(s["t"] / step)] = s
    return grid


def main() -> int:
    path = Path(sys.argv[1])
    step = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    data = json.loads(path.read_text())
    tracks = {int(k): v for k, v in data["tracks"].items() if v}

    print(f"{path.name}: " + ", ".join(
        f"drone-{k} {len(v)} samples, {v[-1]['t']:.0f} s" for k, v in sorted(tracks.items())))
    print(f"common clock, resampled to {step} s buckets\n")

    grids = {k: resample(v, step) for k, v in tracks.items()}
    worst_overall = None

    for a, b in combinations(sorted(grids), 2):
        shared = sorted(set(grids[a]) & set(grids[b]))
        if not shared:
            print(f"  drone-{a} vs drone-{b}: no overlapping samples")
            continue

        min_h = math.inf
        min_v = math.inf
        worst = None          # smallest horizontal gap while also inside the vertical limit
        conflicts = 0
        airborne = 0
        for bucket in shared:
            sa, sb = grids[a][bucket], grids[b][bucket]
            if sa["rel_alt"] < AIRBORNE_M or sb["rel_alt"] < AIRBORNE_M:
                continue
            airborne += 1
            h = horizontal_m(sa, sb)
            v = abs(sa["rel_alt"] - sb["rel_alt"])
            min_h = min(min_h, h)
            min_v = min(min_v, v)
            if h < MIN_HORIZONTAL_SEPARATION_M and v < MIN_VERTICAL_SEPARATION_M:
                conflicts += 1
                if worst is None or h < worst[1]:
                    worst = (bucket * step, h, v, sa["rel_alt"], sb["rel_alt"])

        if not airborne:
            print(f"  drone-{a} vs drone-{b}: never airborne at the same time")
            continue

        secs = conflicts * step
        verdict = "CLEAR" if conflicts == 0 else f"CONFLICT {secs:.1f} s"
        print(f"  drone-{a} vs drone-{b}: {airborne} samples both airborne "
              f"({airborne * step:.0f} s)  "
              f"min horizontal {min_h:7.1f} m   min vertical {min_v:5.2f} m   {verdict}")
        if worst:
            t, h, v, za, zb = worst
            print(f"      closest breach t={t:6.1f} s  {h:.1f} m apart, "
                  f"{v:.2f} m vertically ({za:.1f} m vs {zb:.1f} m)")
            if worst_overall is None or h < worst_overall[1]:
                worst_overall = worst

    print()
    if worst_overall is None:
        print("  No pair was ever inside both limits at the same instant.")
    else:
        print(f"  Closest approach that broke both limits: {worst_overall[1]:.1f} m "
              f"horizontally and {worst_overall[2]:.2f} m vertically.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
