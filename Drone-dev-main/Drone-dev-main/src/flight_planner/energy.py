"""What a flight costs a particular drone, and which drone should fly which cell.

The airframe is the same for every drone in the swarm, but the payloads are not: mass and
weight distribution differ, and no two aircraft launch at the same state of charge. So the
cheapest plan is not the one with the shortest total path -- it is the one that keeps the
*worst-off* drone furthest from its reserve.

Two things follow from the geometry, and they are the reason this module exists:

- **A dispersal gate can never save energy.** home -> gate -> cell start is never shorter
  than home -> cell start, so grouping the swarm up costs every drone something. It buys
  formation flying and a simultaneous start, not endurance.
- **Which drone flies which cell is free.** Handing the cells out differently cannot change
  the total distance flown -- a permutation cannot change the sum -- it only changes who
  pays. On Toa Payoh (lawnmower, an illustrative 2.3/2.6/3.0 kg fleet at 100/90/75 % charge)
  that alone took 16.9 % off the worst drone's battery draw for zero extra metres.

ESTIMATE, NOT MEASUREMENT. Energy per metre is modelled as (mass / reference)^1.5, the
hover-power scaling, because nothing on this project has yet measured the real thing. What
would replace it: hover current against all-up mass, and cruise power against speed, on the
airframe as flown. Until then, prefer the worst-case objective over a precise total -- it
stays safe when the model is wrong -- and treat the numbers as ordering, not quantity.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import permutations

# Mass the scaling is written against: the bare x500 in SITL. Only ratios matter.
REFERENCE_MASS_KG = 2.0

# Above this many drones the exact search stops being free (n! assignments), and a greedy
# hand-out takes over: heaviest-loaded drone first, cheapest cell left.
EXACT_ASSIGNMENT_LIMIT = 8


@dataclass(frozen=True)
class DroneSpec:
    """One aircraft as the planner sees it: what it weighs and what it has left.

    `payload_class` is carried for provenance only and changes nothing. The classes and
    their masses are the operator's to define -- see parked item 4 in history.md -- and
    inventing them here would bake a guess into every plan.
    """

    mass_kg: float = REFERENCE_MASS_KG
    battery_fraction: float = 1.0
    payload_class: str = "unspecified"

    def validate(self) -> None:
        if self.mass_kg <= 0:
            raise ValueError("mass_kg must be positive")
        if not 0 < self.battery_fraction <= 1:
            raise ValueError("battery_fraction must be in (0, 1]")

    @property
    def energy_per_metre(self) -> float:
        """Relative energy to cover one metre. 1.0 is the reference airframe carrying nothing."""
        return (self.mass_kg / REFERENCE_MASS_KG) ** 1.5

    def load(self, distance_m: float) -> float:
        """Fraction of this drone's *available* energy that `distance_m` would consume.

        Relative units: comparable between drones on one sortie, not convertible to watt-hours.
        Dividing by the state of charge is what makes a half-empty drone look expensive.
        """
        return self.energy_per_metre * distance_m / self.battery_fraction


def assign_cells(costs: list[list[float]], specs: list[DroneSpec]) -> tuple[int, ...]:
    """Hand out cells so the worst-off drone is as far from its reserve as possible.

    `costs[drone][cell]` is the whole flight in metres -- transit out, survey, and the
    return -- for that pairing, because a cell's cost depends on who flies it: each drone
    scans at its own altitude layer, so its lane spacing and path length differ.

    Returns the cell index for each drone. Ties break on total load, so the result is
    deterministic rather than whichever permutation came first.
    """
    count = len(specs)
    if count == 0 or not costs:
        return ()
    if count == 1:
        return (0,)

    def score(order: tuple[int, ...]) -> tuple[float, float]:
        loads = [specs[drone].load(costs[drone][cell]) for drone, cell in enumerate(order)]
        return max(loads), sum(loads)

    if count <= EXACT_ASSIGNMENT_LIMIT:
        return min(permutations(range(count)), key=score)

    # Bottleneck assignment, greedily: the drone that would suffer most goes first and
    # takes the cheapest cell still free. Not optimal, but monotone and quick.
    order: list[int | None] = [None] * count
    free = set(range(count))
    for drone in sorted(range(count), key=lambda d: -specs[d].load(min(costs[d]))):
        cell = min(free, key=lambda c: specs[drone].load(costs[drone][c]))
        order[drone] = cell
        free.discard(cell)
    return tuple(int(cell) for cell in order)  # type: ignore[arg-type]


def worst_load(costs: list[list[float]], specs: list[DroneSpec], order: tuple[int, ...]) -> float:
    """The binding drone's load under `order` -- what assign_cells is minimising."""
    if not order:
        return 0.0
    return max(specs[drone].load(costs[drone][cell]) for drone, cell in enumerate(order))
