#!/usr/bin/env python3
"""The ground operator's abort switch: stop every aircraft in a fleet, now.

    python3 tools/takeover.py --fleet fleet-artex-pair.yaml            # press ENTER to abort
    python3 tools/takeover.py --fleet fleet-artex-pair.yaml --now      # abort immediately
    python3 tools/takeover.py --fleet fleet-artex-pair.yaml --action hold
    python3 tools/takeover.py --fleet fleet-artex-pair.yaml --clear    # before the next run
    python3 tools/takeover.py --fleet fleet-artex-pair.yaml --check    # is a flag set?

THE RC TRANSMITTER IS THE REAL OVERRIDE. It talks to the flight controller directly, over a
link that does not pass through the companion, the relay, WiFi, ssh or this script. Reach
for the sticks first. This exists for the case where the operator is at the ground station
rather than on the sticks, and every link in that chain is still healthy.

WHY A FILE AND NOT A MESSAGE. The agent on each companion also takes ABORT on stdin, which
is faster -- but only the process that launched the agent holds that pipe. If the
orchestrator is wedged, has been killed, or was started from a terminal that is now gone,
that path is gone with it. This one needs nothing but ssh: it writes a flag file next to
the agent, which the agent polls several times a second. It therefore survives the
orchestrator dying, which is exactly when an operator reaches for an abort switch.

WHAT EACH ACTION MEANS (the agent's ladder decides the rest -- see tools/safety_rules.py):
    hold     stop the mission, hover in place
    land     descend and land in place, then disarm once it is down   [default]
    disarm   cut the motors -- the agent downgrades this to `land` if it is still flying

None of these can drop an aircraft out of the sky. The one that can -- KILL, a force-disarm
-- is deliberately not here: it needs the stdin path and an agent started with --allow-kill.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from swarm_deploy import host_argv  # noqa: E402

ACTIONS = ("hold", "land", "disarm")


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def run_on(drone: dict, shell: str, timeout: float) -> tuple[bool, str]:
    argv, cwd = host_argv(drone, ["sh", "-c", shell])
    try:
        r = subprocess.run(argv, cwd=cwd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "timed out"
    out = (r.stdout + r.stderr).strip()
    return r.returncode == 0, out


def label(d: dict) -> str:
    return f"slot {d.get('slot')} sysid {d.get('sysid')} on {d.get('host', 'local')}"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fleet", required=True)
    ap.add_argument("--action", choices=ACTIONS, default="land")
    ap.add_argument("--flag", default="ABORT", help="flag path, relative to the agent's dir")
    ap.add_argument("--now", action="store_true", help="fire without waiting for a keypress")
    ap.add_argument("--clear", action="store_true", help="remove the flag on every companion")
    ap.add_argument("--check", action="store_true", help="report the flag, change nothing")
    ap.add_argument("--timeout-s", type=float, default=15.0)
    args = ap.parse_args()

    drones = yaml.safe_load(Path(args.fleet).read_text())["drones"]
    flag = args.flag

    if args.check:
        def check(d):
            ok, out = run_on(d, f"cat {flag} 2>/dev/null || echo __none__", args.timeout_s)
            return d, ok, out
        with ThreadPoolExecutor(len(drones)) as ex:
            for d, ok, out in ex.map(check, drones):
                set_to = "no flag" if "__none__" in out else f"FLAG SET: {out!r}"
                log(f"[check] {label(d)}: {set_to}" + ("" if ok else " (unreachable)"))
        return 0

    if args.clear:
        with ThreadPoolExecutor(len(drones)) as ex:
            results = list(ex.map(lambda d: (d, *run_on(d, f"rm -f {flag}", args.timeout_s)),
                                  drones))
        bad = 0
        for d, ok, out in results:
            log(f"[clear] {label(d)}: {'cleared' if ok else 'FAILED -- ' + out}")
            bad += not ok
        return 1 if bad else 0

    if not args.now:
        log(f"[takeover] fleet {args.fleet}: {len(drones)} aircraft, action {args.action.upper()}")
        for d in drones:
            log(f"  {label(d)}")
        log("[takeover] RC transmitter is the real override -- use the sticks if you have them.")
        try:
            input("Press ENTER to send it, or Ctrl-C to leave the aircraft alone: ")
        except (KeyboardInterrupt, EOFError):
            log("\n[takeover] nothing sent")
            return 1

    # One write, then one read-back, so a silent ssh failure cannot look like success.
    shell = f"printf '%s\\n' {args.action} > {flag} && cat {flag}"
    with ThreadPoolExecutor(len(drones)) as ex:
        results = list(ex.map(lambda d: (d, *run_on(d, shell, args.timeout_s)), drones))

    sent = 0
    for d, ok, out in results:
        confirmed = ok and args.action in out
        log(f"[takeover] {label(d)}: "
            + (f"{args.action.upper()} set" if confirmed else f"NOT SET -- {out or 'no output'}"))
        sent += confirmed
    log(f"[takeover] {sent}/{len(drones)} companions carry the flag")
    if sent < len(drones):
        log("[takeover] AT LEAST ONE AIRCRAFT DID NOT GET IT -- use the RC transmitter")
        return 1
    log("[takeover] clear it with --clear before the next run; the agent refuses to arm "
        "while it is set")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
