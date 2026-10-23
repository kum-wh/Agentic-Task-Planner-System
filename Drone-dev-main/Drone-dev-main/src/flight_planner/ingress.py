"""Where the swarm flies in as a group, and where it splits up.

The **gate** is one shared waypoint out in front of home. Every drone flies to it together,
stacked on its own transit layer, and only there does each peel off to its own cell. It
answers a question geometry alone cannot: a swarm that leaves home on three different
headings never flies as a formation, and its drones begin searching up to 39 s apart
(Toa Payoh, measured 16 Sep 2026).

Two things are worth knowing before reading the code:

- **A gate always costs distance.** home -> gate -> start is never shorter than
  home -> start. Ask for the `energy` objective and the search will quite correctly push
  the gate back towards home, because the cheapest gate is no gate. What a gate buys is
  formation flying and a simultaneous start.
- **Equal distance is not equal arrival.** Each drone transits at its own layer's cruise
  speed (12.2 / 13.0 / 13.9 m/s at 28 / 30 / 32 m), so a gate that is equidistant still
  lands them 7-8 s apart. `arrival` is the objective that actually gets them there together.

The free lever is which end of its cell each drone enters from. Reversing a route changes
nothing about what it costs -- the survey is identical and the transit out and the return
merely swap -- but it moves the start points, and so moves the gate. At Toa Payoh it took
the equal-distance gate from 769 m of ingress each down to 681 m.
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from typing import Callable, Sequence

from .energy import DroneSpec
from .geometry import (
    DISPERSAL_GRID_STEP_M,
    Point,
    circumcentre,
    point_in_polygon,
    search_in_polygon,
)

# Entry-end combinations double with every drone. Past this many the search stops paying
# for itself and every cell is simply entered from the end the scanner produced.
ENTRY_END_SEARCH_LIMIT = 5

# Fairness finer than these is not real: arrival times move by seconds with wind and with
# the corner model, and a metre is inside GNSS noise. Scores are bucketed to this
# resolution so that everything "equally fair" can then be ranked on what it costs --
# without it, a gate 700 m out that happens to be fair to nine decimal places beats a
# sensible one that is fair to four.
ARRIVAL_TOLERANCE_S = 0.5
DISTANCE_TOLERANCE_M = 1.0
LOAD_TOLERANCE = 1.0

# Ranking pass runs on a grid this many times coarser; only the winner is refined.
COARSE_STEP_MULTIPLIER = 4.0


@dataclass(frozen=True)
class CellEntry:
    """One drone's cell, seen from outside: where it can be entered, and what it costs."""

    ends: tuple[Point, Point]        # first and last waypoint of the route as scanned
    survey_m: float                  # path length of the survey itself, orientation-independent

    def start(self, reverse: bool) -> Point:
        return self.ends[1] if reverse else self.ends[0]

    def finish(self, reverse: bool) -> Point:
        return self.ends[0] if reverse else self.ends[1]


def _ingress_distances(home: Point, gate: Point, starts: Sequence[Point]) -> list[float]:
    common = math.dist(home, gate)
    return [common + math.dist(gate, start) for start in starts]


def objective_for(kind: str, home: Point, starts: Sequence[Point], speeds: Sequence[float],
                  specs: Sequence[DroneSpec], fixed_m: Sequence[float]) -> Callable[[Point], tuple]:
    """Score a candidate gate: lower is better, and every objective is minimised the same way.

    Each returns a **pair**, because fairness alone does not pick a point. The set of gates
    where all three drones arrive together is a curve, not a point, and most of that curve
    is absurd -- at Toa Payoh (voronoi) the first zero-spread gate found sat 708 m out and
    cost 1417 m of ingress each, perfectly fairly. So the second element is what the swarm
    actually spends, and ties on fairness are settled by cost. A tuple says that exactly,
    where a weighted sum would need an invented exchange rate between seconds and metres.

    `fixed_m` is the part of each drone's flight a gate cannot change -- its survey and its
    return home -- which only the energy objective needs, because a battery does not care
    which leg drained it.
    """
    if kind == "distance":
        def spread(gate: Point) -> tuple[int, float]:
            distances = _ingress_distances(home, gate, starts)
            return round((max(distances) - min(distances)) / DISTANCE_TOLERANCE_M), sum(distances)
        return spread
    if kind == "arrival":
        def arrival_spread(gate: Point) -> tuple[int, float]:
            distances = _ingress_distances(home, gate, starts)
            times = [d / v for d, v in zip(distances, speeds)]
            return round((max(times) - min(times)) / ARRIVAL_TOLERANCE_S), sum(distances)
        return arrival_spread

    def worst_battery(gate: Point) -> tuple[int, float]:
        distances = _ingress_distances(home, gate, starts)
        worst = max(spec.load(d + fixed) for spec, d, fixed in zip(specs, distances, fixed_m))
        return round(worst / LOAD_TOLERANCE), sum(distances)
    return worst_battery


def choose_gate(home: Point, entries: Sequence[CellEntry], fence: list[Point],
                speeds: Sequence[float], specs: Sequence[DroneSpec], kind: str = "arrival",
                step: float = DISPERSAL_GRID_STEP_M,
                min_gate_m: float = 0.0) -> tuple[Point | None, tuple[bool, ...]]:
    """Pick the dispersal point, and which end each drone enters its cell from.

    Ranks every entry-end combination on a coarse grid, then refines only the winner --
    the alternative, refining all of them, is the same answer for many times the work.

    `min_gate_m` is how far out the gate has to be to count as a rendezvous. Without it the
    cost tie-break drags the gate home whenever the entry ends alone can equalise the
    ingress: at Bedok the arrival objective returned a gate 55 m out, fair to 0.2 s, with a
    four-second formation leg. Fair, and not what a gate is for.

    Returns (gate, reverse-per-drone). A gate of None means no legal point was found and
    the caller should fly the swarm out directly, as it always did.
    """
    unreachable = (float("inf"), float("inf"))

    def far_enough(score: Callable[[Point], tuple]) -> Callable[[Point], tuple]:
        if min_gate_m <= 0:
            return score
        return lambda gate: score(gate) if math.dist(home, gate) >= min_gate_m else unreachable
    count = len(entries)
    if count == 0 or not fence:
        return None, tuple(False for _ in entries)
    combinations = (list(itertools.product([False, True], repeat=count))
                    if count <= ENTRY_END_SEARCH_LIMIT else [tuple([False] * count)])

    def setup(combo: Sequence[bool]) -> tuple[list[Point], list[float]]:
        starts = [entry.start(reverse) for entry, reverse in zip(entries, combo)]
        fixed = [entry.survey_m + math.dist(entry.finish(reverse), home)
                 for entry, reverse in zip(entries, combo)]
        return starts, fixed

    best_combo, best_score = None, None
    for combo in combinations:
        starts, fixed = setup(combo)
        score = far_enough(objective_for(kind, home, starts, speeds, specs, fixed))
        coarse = search_in_polygon(fence, score, step * COARSE_STEP_MULTIPLIER)
        if coarse is None:
            continue
        value = score(coarse)
        if value == unreachable:          # nowhere in the fence is far enough out
            continue
        if best_score is None or value < best_score:
            best_combo, best_score = combo, value
    if best_combo is None:
        return None, tuple(False for _ in entries)

    starts, fixed = setup(best_combo)
    score = far_enough(objective_for(kind, home, starts, speeds, specs, fixed))
    gate = search_in_polygon(fence, score, step)
    if gate is not None and score(gate) == unreachable:
        return None, tuple(False for _ in entries)
    # Three drones have exactly one equidistant point, and it is analytic. A swept grid can
    # only land near it, so when it is legal and no worse, use the exact answer.
    if gate is not None and kind == "distance" and len(starts) == 3:
        exact = circumcentre(*starts)
        if exact is not None and point_in_polygon(exact, fence) and score(exact) <= score(gate):
            gate = exact
    return gate, tuple(bool(reverse) for reverse in best_combo)
