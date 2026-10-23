#!/usr/bin/env python3
"""Decide when a flight has to stop, and how hard to stop it.

Pure logic: no MAVLink, no sockets, and no clock of its own -- every entry point is handed
the time, so the rules are deterministic under test. `drone_agent.py` feeds this telemetry
and acts on the verdict; `tests/test_safety_rules.py` pins the behaviour.

THE LADDER. Four rungs, and the monitor only ever climbs. A condition that has fired once
does not get to un-fire because the next second looked better -- recovery is a human
decision, not a timeout.

    WARN    something is wrong; say so, keep flying
    HOLD    stop the mission, hover where you are
    LAND    descend and land in place, now
    DISARM  cut the motors -- only ever once the aircraft is on the ground

WHY A RATE AND NOT A COUNT. PX4 talks constantly and a healthy aircraft still emits the odd
warning, so a count would either fire on the first grumble or never. One "EKF ..." message
means nothing; ten in twenty seconds means the estimator is coming apart. Errors are
therefore weighted by MAVLink severity and summed over a sliding 60 s window -- the metric
is accumulated error *per minute*, and it falls on its own as the window slides forward.

WHAT THIS IS NOT. None of this is the authoritative manual override. The RC transmitter is:
it talks to the flight controller directly, over a link that does not pass through the
companion, the relay, WiFi or this process. Everything here can only help while that whole
chain is intact, which is exactly the chain an emergency may have broken.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

NONE, WARN, HOLD, LAND, DISARM = "none", "warn", "hold", "land", "disarm"
RUNG = {NONE: 0, WARN: 1, HOLD: 2, LAND: 3, DISARM: 4}

# MAV_SEVERITY 0-4: EMERGENCY, ALERT, CRITICAL, ERROR, WARNING. NOTICE (5) and below are
# routine chatter -- "Ready for takeoff", "Armed by external command" -- and score nothing.
SEVERITY_WEIGHT = {0: 8.0, 1: 6.0, 2: 4.0, 3: 2.0, 4: 1.0}


@dataclass
class Limits:
    """Every threshold, in one place, so a run can be tuned without touching the rules."""

    window_s: float = 60.0            # the sliding window the error rate is measured over
    warn_rate: float = 3.0            # weighted error units per minute
    hold_rate: float = 8.0
    land_rate: float = 14.0

    battery_hold_pct: float = 25.0
    battery_land_pct: float = 15.0

    tilt_land_deg: float = 60.0       # past this a multicopter is not recoverable by us

    # A 1 Hz heartbeat over a radio with 25% loss drops three in a row about 1.6% of the
    # time, so 3 s of silence is not evidence of anything. Twelve seconds is, and it still
    # leaves PX4's own datalink-loss failsafe to act first if it is configured tighter.
    telemetry_gap_hold_s: float = 5.0
    telemetry_gap_land_s: float = 12.0

    innovation_ratio: float = 1.0     # PX4 calls >1.0 a failing innovation test
    innovation_sustain_s: float = 3.0 # one spike is noise; three seconds is a trend


@dataclass
class Verdict:
    action: str
    reasons: list[str] = field(default_factory=list)
    error_rate: float = 0.0
    climbed: bool = False             # this evaluate() moved us up a rung


class Watchdog:
    """Accumulates telemetry, hands back the rung the flight has fallen to."""

    def __init__(self, limits: Limits | None = None, now: float = 0.0):
        self.limits = limits or Limits()
        self._events: deque[tuple[float, float, str]] = deque()   # (t, weight, text)
        self._action = NONE
        self._reasons: list[str] = []
        self._last_heartbeat = now
        self._battery_pct: float | None = None
        self._tilt_deg: float | None = None
        self._attitude_lost = False
        self._innovation_since: float | None = None
        self._manual: tuple[str, str] | None = None

    # ---- telemetry in -------------------------------------------------------------

    def statustext(self, severity: int, text: str, now: float) -> None:
        weight = SEVERITY_WEIGHT.get(int(severity))
        if weight:
            self._events.append((now, weight, text.strip()))

    def heartbeat(self, now: float) -> None:
        self._last_heartbeat = now

    def battery(self, pct: float | None, now: float) -> None:
        # PX4 sends -1 when it has no estimate; that is not a flat battery.
        if pct is not None and pct >= 0:
            self._battery_pct = float(pct)

    def attitude(self, roll_deg: float, pitch_deg: float, now: float) -> None:
        self._tilt_deg = max(abs(roll_deg), abs(pitch_deg))

    def estimator(self, attitude_ok: bool, worst_innovation: float | None,
                  now: float) -> None:
        if not attitude_ok:
            self._attitude_lost = True
        if worst_innovation is None:
            return
        if worst_innovation > self.limits.innovation_ratio:
            if self._innovation_since is None:
                self._innovation_since = now
        else:
            self._innovation_since = None

    def manual(self, action: str, reason: str) -> None:
        """An operator command. Outranks nothing -- it joins the ladder like any rule."""
        if action not in RUNG:
            raise ValueError(f"unknown action {action!r}")
        self._manual = (action, reason)

    # ---- the metric ---------------------------------------------------------------

    def error_rate(self, now: float) -> float:
        """Weighted error units per minute, over the sliding window."""
        cutoff = now - self.limits.window_s
        while self._events and self._events[0][0] < cutoff:
            self._events.popleft()
        if not self._events:
            return 0.0
        total = sum(w for _, w, _ in self._events)
        return total * 60.0 / self.limits.window_s

    def recent_errors(self, now: float, limit: int = 3) -> list[str]:
        self.error_rate(now)      # prunes the window
        return [t for _, _, t in list(self._events)[-limit:]]

    # ---- the verdict --------------------------------------------------------------

    def evaluate(self, now: float) -> Verdict:
        lim = self.limits
        rate = self.error_rate(now)
        found: list[tuple[str, str]] = []

        if rate >= lim.land_rate:
            found.append((LAND, f"error rate {rate:.1f}/min >= {lim.land_rate}/min"))
        elif rate >= lim.hold_rate:
            found.append((HOLD, f"error rate {rate:.1f}/min >= {lim.hold_rate}/min"))
        elif rate >= lim.warn_rate:
            found.append((WARN, f"error rate {rate:.1f}/min >= {lim.warn_rate}/min"))

        if self._battery_pct is not None:
            if self._battery_pct <= lim.battery_land_pct:
                found.append((LAND, f"battery {self._battery_pct:.0f}% <= {lim.battery_land_pct}%"))
            elif self._battery_pct <= lim.battery_hold_pct:
                found.append((HOLD, f"battery {self._battery_pct:.0f}% <= {lim.battery_hold_pct}%"))

        if self._tilt_deg is not None and self._tilt_deg >= lim.tilt_land_deg:
            found.append((LAND, f"tilt {self._tilt_deg:.0f} deg >= {lim.tilt_land_deg} deg"))

        gap = now - self._last_heartbeat
        if gap >= lim.telemetry_gap_land_s:
            found.append((LAND, f"no telemetry for {gap:.1f}s"))
        elif gap >= lim.telemetry_gap_hold_s:
            found.append((HOLD, f"no telemetry for {gap:.1f}s"))

        if self._attitude_lost:
            found.append((LAND, "estimator lost the attitude solution"))

        if self._innovation_since is not None:
            held = now - self._innovation_since
            if held >= lim.innovation_sustain_s:
                found.append((LAND, f"innovation test failing for {held:.1f}s"))

        if self._manual:
            action, reason = self._manual
            found.append((action, f"operator: {reason}"))

        climbed = False
        for action, reason in found:
            if RUNG[action] > RUNG[self._action]:
                self._action, climbed = action, True
            if reason not in self._reasons:
                self._reasons.append(reason)

        return Verdict(action=self._action, reasons=list(self._reasons),
                       error_rate=rate, climbed=climbed)


def on_ground_action(action: str, airborne: bool | None) -> str:
    """What to actually do, given where the aircraft is.

    LAND on an aircraft that is already on the ground means cut the motors. DISARM on one
    that is still flying means drop it out of the sky, so that is downgraded to LAND --
    and when we cannot tell, we land rather than guess.
    """
    if action == LAND and airborne is False:
        return DISARM
    if action == DISARM and airborne is not False:
        return LAND
    return action
