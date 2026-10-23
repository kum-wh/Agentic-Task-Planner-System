#!/usr/bin/env python3
"""Arm every drone in a fleet at the same moment, hold, disarm -- all or nobody.

    python3 tools/pair_arm_test.py --fleet fleet-artex-pair.yaml [--hold-s 10]

PROPS MUST BE OFF. This spins motors on every aircraft in the fleet file.

Phases (the same barrier idea as swarm_deploy.py):
  0. link    each Jetson's radio link to its backpack must be good enough that the DISARM
             will certainly arrive -- a lost ARM is a non-event, a lost DISARM is not
  1. status  each aircraft must answer as its expected sysid and be DISARMED
  2. ready   each Jetson streams a level/zero-thrust setpoint, puts its aircraft in OFFBOARD
             and reports READY; nothing is armed yet
  3. arm     only when EVERY drone is READY does the workstation send ARM to all of them
  4. hold, then each agent disarms; a final status confirms every aircraft is disarmed
If any drone is not READY in time, nobody arms. No parameters are written.
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from swarm_deploy import LOCAL_HOSTS, host_argv  # noqa: E402


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def abort_console(sessions: list) -> None:
    """Type 'a' + ENTER to stop everything. Runs on the pipes this process already owns.

    The out-of-band path (tools/takeover.py, a flag file over ssh) does not depend on this
    process being alive, which is the right tool if it has wedged. This one is faster.
    """
    while True:
        try:
            word = sys.stdin.readline()
        except (ValueError, OSError):
            return
        if not word:
            return
        if word.strip().lower() in ("a", "abort", "land", "stop"):
            log("[takeover] OPERATOR ABORT -- telling every agent to land and disarm")
            for s in sessions:
                s.send("ABORT")


def last_json(text: str) -> dict:
    for line in reversed(text.strip().splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return {}


def link_quality(d: dict, gw: str, count: int) -> dict:
    """Ping the ELRS backpack from the Jetson that owns it, as that Jetson sees it."""
    if d.get("host", "local") in LOCAL_HOSTS:
        return {"skipped": "local host -- no radio in the path"}
    argv, cwd = host_argv(d, ["ping", "-c", str(count), "-W", "1", gw])
    try:
        r = subprocess.run(argv, cwd=cwd, capture_output=True, text=True,
                           timeout=2 * count + 30)
    except subprocess.TimeoutExpired:
        return {"error": f"ping {gw} timed out"}
    loss = re.search(r"([\d.]+)% packet loss", r.stdout)
    avg = re.search(r"rtt min/avg/max/mdev = [\d.]+/([\d.]+)/", r.stdout)
    if not loss:
        return {"error": (r.stderr.strip().splitlines() or ["no ping output"])[-1]}
    return {"loss_pct": float(loss.group(1)),
            "avg_ms": float(avg.group(1)) if avg else None}


def status_of(d: dict) -> dict:
    argv, cwd = host_argv(d, ["python3", "tools/drone_agent.py", "status", "--link", d["link"],
                              "--expect-sysid", str(d["sysid"])])
    try:
        r = subprocess.run(argv, cwd=cwd, capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        return {"error": "status timed out"}
    return last_json(r.stdout) or {"error": (r.stderr.strip().splitlines() or ["no output"])[-1]}


class Gated:
    def __init__(self, d: dict, hold_s: float):
        self.d = d
        self.ready = threading.Event()
        self.done = threading.Event()
        self.armed_at: float | None = None      # this workstation's clock
        self.armed_epoch: float | None = None   # the companion's clock, often unsynced
        self.final: dict = {}
        argv, cwd = host_argv(d, ["python3", "tools/drone_agent.py", "offboard-arm", "--link", d["link"],
                                  "--expect-sysid", str(d["sysid"]), "--slot", str(d["slot"]),
                                  "--hold-s", str(hold_s), "--gate"])
        self.p = subprocess.Popen(argv, cwd=cwd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, text=True, bufsize=1)
        threading.Thread(target=self._out, daemon=True).start()
        threading.Thread(target=self._err, daemon=True).start()

    def _out(self) -> None:
        for line in self.p.stdout:
            try:
                obj = json.loads(line)
            except ValueError:
                continue
            if obj.get("event") == "ready":
                self.ready.set()
            elif obj.get("event") == "safety":
                log(f"  [slot {self.d['slot']} sysid {self.d['sysid']}] SAFETY "
                    f"{obj.get('action', '?').upper()} -> {obj.get('carried_out', '?').upper()}: "
                    + "; ".join(obj.get("reasons", [])[-2:]))
            elif obj.get("event") == "armed":
                self.armed_at = time.monotonic()
                self.armed_epoch = obj.get("armed_epoch")
            else:
                self.final = obj
        self.done.set()

    def _err(self) -> None:
        for line in self.p.stderr:
            log(f"  [slot {self.d['slot']} sysid {self.d['sysid']}] {line.rstrip()}")

    def send(self, word: str) -> None:
        try:
            self.p.stdin.write(word + "\n")
            self.p.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    def close(self) -> None:
        try:
            self.p.stdin.close()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fleet", required=True)
    ap.add_argument("--hold-s", type=float, default=10.0)
    ap.add_argument("--ready-timeout-s", type=float, default=75.0)
    ap.add_argument("--skip-link-check", action="store_true",
                    help="arm even if a radio link is poor -- you accept that a DISARM may be lost")
    ap.add_argument("--link-gw", default="10.0.0.1", help="backpack address each Jetson pings")
    ap.add_argument("--link-pings", type=int, default=10)
    ap.add_argument("--max-loss-pct", type=float, default=10.0)
    ap.add_argument("--max-rtt-ms", type=float, default=150.0)
    ap.add_argument("--skip-status", action="store_true",
                    help="skip the separate status handshake (SITL latches its first client)")
    args = ap.parse_args()
    drones = yaml.safe_load(Path(args.fleet).read_text())["drones"]

    if not args.skip_link_check:
        log(f"[link] each Jetson pings its backpack at {args.link_gw} "
            f"(need loss <= {args.max_loss_pct}%, avg RTT <= {args.max_rtt_ms} ms)")
        with ThreadPoolExecutor(len(drones)) as ex:
            qs = list(ex.map(lambda d: link_quality(d, args.link_gw, args.link_pings), drones))
        weak = False
        for d, q in zip(drones, qs):
            if q.get("skipped"):
                log(f"[link] slot {d['slot']} sysid {d['sysid']}: skipped -- {q['skipped']}")
                continue
            loss, avg = q.get("loss_pct"), q.get("avg_ms")
            ok = (loss is not None and loss <= args.max_loss_pct
                  and (avg is None or avg <= args.max_rtt_ms))
            log(f"[link] slot {d['slot']} sysid {d['sysid']} on {d['host']}: "
                + (f"{'OK' if ok else 'TOO WEAK'} -- {loss}% loss, avg {avg} ms"
                   if loss is not None else f"TOO WEAK -- {q}"))
            weak |= not ok
        if weak:
            log("[abort] a radio link cannot be trusted to deliver the DISARM; nothing was started")
            log("[abort] fix the link (antenna, position, range) -- or accept the risk with "
                "--skip-link-check")
            return 1

    sts = [{}] * len(drones)
    if not args.skip_status:
        log("[status] checking every aircraft answers and is disarmed")
        with ThreadPoolExecutor(len(drones)) as ex:
            sts = list(ex.map(status_of, drones))
    bad = False
    for d, st in ([] if args.skip_status else zip(drones, sts)):
        ok = st.get("armed") is False and st.get("sysid") == d["sysid"]
        log(f"[status] slot {d['slot']} sysid {d['sysid']} on {d['host']}: "
            + (f"OK disarmed, battery {st.get('battery_pct')}%" if ok else f"NOT OK -- {st}"))
        bad |= not ok
    if bad:
        log("[abort] not every aircraft is ready for the handshake; nothing was started")
        return 1

    log("[ready] each Jetson takes its aircraft into OFFBOARD (no arming yet)")
    sessions = [Gated(d, args.hold_s) for d in drones]
    threading.Thread(target=abort_console, args=(sessions,), daemon=True).start()
    log("[takeover] type 'a' + ENTER at any time to abort every aircraft "
        "(out-of-band: tools/takeover.py --fleet ... --now)")
    rc = 1
    try:
        end = time.time() + args.ready_timeout_s
        while time.time() < end and not all(s.ready.is_set() for s in sessions):
            if any(s.done.is_set() for s in sessions):
                break
            time.sleep(0.2)
        for s in sessions:
            log(f"[ready] slot {s.d['slot']} sysid {s.d['sysid']}: "
                + ("READY" if s.ready.is_set() else "NOT READY"))
        if not all(s.ready.is_set() for s in sessions):
            log("[abort] barrier failed: nobody arms")
            for s in sessions:
                s.send("ABORT")
        else:
            log("[authorize] workstation authorises ARM on every drone")
            for s in sessions:
                s.send("ARM")
        for s in sessions:
            s.done.wait(args.hold_s + 120)
        finals = [s.final for s in sessions]
        for s, f in zip(sessions, finals):
            log(f"[result] slot {s.d['slot']} sysid {s.d['sysid']}: armed={f.get('armed')} "
                f"held={f.get('held_s')}s disarmed={f.get('disarmed')}"
                + (f" error={f.get('error')}" if f.get("error") else "")
                + (f" vehicle={f.get('vehicle_text')}" if f.get("error") and f.get("vehicle_text") else ""))
        stamps = [s.armed_at for s in sessions if s.armed_at is not None]
        if len(stamps) == len(sessions) and len(sessions) > 1:
            log(f"[result] arm instants differ by {1000 * (max(stamps) - min(stamps)):.0f} ms "
                "(stamped on this workstation, so it includes ssh/link latency -- an upper bound)")
        elif len(sessions) > 1:
            log("[result] arm spread unknown: not every agent reported the instant it armed")
        rc = 0 if all(f.get("armed") and f.get("disarmed") and "error" not in f for f in finals) else 1
    finally:
        for s in sessions:
            s.send("ABORT")
            s.close()
        time.sleep(1)
        if not args.skip_status:
            log("[final] confirming every aircraft is disarmed")
            with ThreadPoolExecutor(len(drones)) as ex:
                sts = list(ex.map(status_of, drones))
            for d, st in zip(drones, sts):
                log(f"[final] slot {d['slot']} sysid {d['sysid']}: armed={st.get('armed')}")
    log("[verdict] " + ("BOTH ARMED TOGETHER AND DISARMED" if rc == 0 else "NOT A CLEAN PASS"))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
