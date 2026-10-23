#!/usr/bin/env python3
"""Can the pilot take the aircraft back with a switch on the RadioMaster Pocket?

    python3 tools/rc_takeover.py --fleet fleet-artex-pair.yaml

Read-only. It changes nothing on any aircraft -- it reports what PX4 is currently
configured to do, and prints the exact values to change if the answer is no.

TWO WAYS A PILOT TAKES OVER, AND THIS PROJECT USES ONLY ONE. A mode SWITCH requests a
flight mode outright (RC_MAP_FLTMODE + COM_FLTMODE1..6). Separately, STICK movement can
override, governed by `COM_RC_OVERRIDE` -- a bitmask whose bit 0 covers AUTO and bit 1
covers OFFBOARD, shipped as 1, so bit 1 is clear.

Decided 6 Oct 2026: **leave COM_RC_OVERRIDE at 1.** A knocked stick must not be able to
drop an aircraft out of OFFBOARD mid-run; taking over should be a deliberate act. So the
mode switch is the takeover path, and this check treats the override as a note rather than
a fault. Set it to 3 only if you later decide you want the sticks to override too.

THE THREE LAYERS, AND WHY THIS ONE IS DIFFERENT. The guard on the companion
(tools/safety_rules.py) and the ground station's abort switch (tools/takeover.py) both run
on our side of a long chain: WiFi, the ELRS backpack, a relay, ssh, Python. The RC switch
is the only override that reaches the flight controller directly over the RF link, with no
companion in the path -- so it is the only one that still works when the companion has
crashed, the relay has died, or the Jetson has fallen off its network. That is why it is
worth configuring properly even though it needs a human holding the transmitter.

WHAT HAS TO BE SET, AND WHERE:

  On the handset (EdgeTX, on the Pocket itself) -- not something this repo can do:
    assign a physical switch to a channel, and give it the positions you want. A
    6-position mode channel is the usual arrangement; a 2-position channel is enough for
    a kill switch.

  On the aircraft (QGroundControl, or `drone_agent.py prepare --set NAME=VALUE`):
    COM_RC_IN_MODE    0 (RC only), 2 (RC and joystick) or 3 (keep first) -- 4 is OFF
    COM_RC_OVERRIDE   left at 1 here on purpose; 3 would let the sticks override too
    COM_RC_STICK_OV   how far a stick must move to count, in % (PX4 default 30)
    RC_MAP_FLTMODE    the channel carrying the mode switch
    COM_FLTMODE1..6   the mode in each of that channel's six slots. At least one must be
                      a hand-flown mode: Manual(0), Altitude(1), Position(2), Acro(6),
                      Stabilized(8), Position Slow(9). Hold(4), Mission(3) and Return(5)
                      are still the autopilot flying, not the pilot.
    RC_MAP_KILL_SW    the channel carrying the kill switch. This one cuts the motors
                      without asking the flight stack's permission -- and will drop the
                      aircraft out of the air, so it is for a crash in progress.

QGroundControl's Vehicle Setup > Flight Modes page is the friendliest way to fill in
RC_MAP_FLTMODE and the six slots, because it shows the switch moving as you flip it. Use
it, then re-run this check to confirm what actually landed on the aircraft.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from swarm_deploy import host_argv  # noqa: E402

WANTED = {
    "COM_RC_IN_MODE": "0, 2 or 3 -- anything that accepts RC",
    "COM_RC_OVERRIDE": "left at 1 on this project, on purpose (see notes)",
    "RC_MAP_FLTMODE": "the channel your mode switch is on",
    "RC_MAP_KILL_SW": "the channel your kill switch is on",
}


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def last_json(text: str) -> dict:
    for line in reversed(text.strip().splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return {}


def check(d: dict, timeout: float) -> tuple[dict, dict]:
    argv, cwd = host_argv(d, ["python3", "tools/drone_agent.py", "rc-check",
                              "--link", d["link"], "--slot", str(d.get("slot", 0)),
                              "--expect-sysid", str(d["sysid"])])
    try:
        r = subprocess.run(argv, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return d, {"error": "rc-check timed out"}
    out = last_json(r.stdout)
    if not out:
        out = {"error": (r.stderr.strip().splitlines() or ["no output"])[-1]}
    return d, out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fleet", required=True)
    ap.add_argument("--timeout-s", type=float, default=120.0)
    args = ap.parse_args()

    drones = yaml.safe_load(Path(args.fleet).read_text())["drones"]
    with ThreadPoolExecutor(len(drones)) as ex:
        results = list(ex.map(lambda d: check(d, args.timeout_s), drones))

    bad = 0
    for d, r in results:
        head = f"slot {d.get('slot')} sysid {d.get('sysid')} on {d.get('host', 'local')}"
        if r.get("error"):
            log(f"[rc] {head}: COULD NOT CHECK -- {r['error']}")
            bad += 1
            continue
        p = r.get("params", {})
        log(f"[rc] {head}: RC heard={r.get('rc_seen')} rssi={r.get('rssi')}")
        log(f"[rc]   " + "  ".join(f"{k}={p.get(k)}" for k in WANTED))
        if r.get("mode_slots"):
            log(f"[rc]   mode slots: " + ", ".join(f"{n}:{m}" for n, m
                                                   in sorted(r["mode_slots"].items())))
        for f in r.get("findings", []):
            log(f"[rc]   PROBLEM: {f}")
        for n in r.get("notes", []):
            log(f"[rc]   note: {n}")
        if r.get("ready"):
            log(f"[rc]   the RC switch can take this aircraft")
        else:
            bad += 1

    if bad:
        log("")
        log("[rc] NOT every aircraft can be taken over by its RC switch. Set these:")
        for k, v in WANTED.items():
            log(f"[rc]   {k:17} -> {v}")
        log("[rc] COM_FLTMODE1..6 need at least one hand-flown mode: Manual(0), "
            "Altitude(1), Position(2), Acro(6), Stabilized(8), Position Slow(9).")
        log("[rc] COM_RC_OVERRIDE is NOT in that list: this project leaves it at PX4's "
            "default 1 on purpose, so a knocked stick cannot drop an aircraft out of "
            "OFFBOARD. The mode switch is the takeover path.")
        log("[rc] QGroundControl > Vehicle Setup > Flight Modes is the easiest way; it "
            "shows the switch moving as you flip it. Then re-run this check.")
        return 1
    log("[rc] every aircraft can be taken over from the transmitter")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
