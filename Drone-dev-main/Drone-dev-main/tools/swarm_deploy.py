#!/usr/bin/env python3
"""Plan a sector, hand each drone its share, and launch them together.

    python3 tools/swarm_deploy.py --fleet fleet.yaml --kml ops/pulau-ubin-west.kml
    python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir missions/toapayoh/mission_drones
    python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir ... --dry-run
    python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir ... --upload-only
    python3 tools/swarm_deploy.py --fleet fleet.yaml --plans-dir ... --arm-only [--qgc]
    python3 tools/swarm_deploy.py --fleet fleet.yaml --abort

The sequence, and why each step is there:

  1. PLAN     one global computation. The separation guardrail is a property of the whole
              fleet, so it cannot be run per drone. A guardrail refusal stops the run here.
  2. PUSH     each drone's .plan reaches its agent (a copy for a remote host, nothing for
              a local one).
  3. PREPARE  in parallel. Each agent verifies identity, clears any stale mission, loads
              params, uploads, resets the mission index, runs the preflight contract --
              then REPORTS AND STAYS RESIDENT, holding the link open.
  4. BARRIER  every drone READY within the timeout, or NOBODY arms.
  5. START    a token is sent back to each resident agent over its own stdin. The token
              binds the launch to the plan that was actually verified.
  6. WATCH    poll each agent until the vehicles disarm. The polling is also what keeps
              each agent's failsafe from firing.

Agents stay alive for the whole run. That is deliberate: PX4 refuses to arm for a ground
station it has not heard from recently, so an agent that exits after `prepare` has to
re-establish the link from cold at exactly the wrong moment. One connection, prepare
through landing.

The `link` in fleet.yaml is the only thing that changes between a simulated vehicle, a
board on USB (maintenance only) and a flight controller behind an ELRS radio.
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import shlex
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
AGENT_REL = "tools/drone_agent.py"
PREPARE_TIMEOUT_S = 180.0
START_TIMEOUT_S = 60.0
UPLOAD_TIMEOUT_S = 900.0      # upload + read-back; ~1 s/item each way over ELRS
STATUS_TIMEOUT_S = 20.0
START_SKEW_LEAD_S = 3.0
WATCH_POLL_S = 10.0
LOCAL_HOSTS = {"local", "localhost", None, ""}


def quote_remote(path: str) -> str:
    """Quote a remote path without killing the shell's tilde expansion.

    shlex.quote("~/FlightPlanner") yields '~/FlightPlanner', which the remote shell takes
    literally: `mkdir -p` then creates a directory actually named "~" in $HOME while scp,
    which expands the tilde itself, writes somewhere else entirely and reports "No such
    file or directory". Found 28 Sep 2026 on the first run against real Jetsons.
    """
    if path == "~":
        return "~"
    if path.startswith("~/"):
        return "~/" + shlex.quote(path[2:])
    return shlex.quote(path)


def log(msg: str) -> None:
    print(msg, flush=True)


def open_qgc() -> None:
    """Launch the workstation's QGC detached, unless one is already running."""
    appimage = ROOT.parent / "QGroundControl.AppImage"
    if subprocess.run(["pgrep", "-f", "[Q]GroundControl"],
                      capture_output=True).returncode == 0:
        log("[qgc] already running")
        return
    if not appimage.exists():
        log(f"[qgc] {appimage} not found -- not opened")
        return
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        log("[qgc] no display in this session -- not opened")
        return
    subprocess.Popen([str(appimage)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     start_new_session=True)
    log("[qgc] opened")


def host_argv(drone: dict, argv: list[str]) -> tuple[list[str], str | None]:
    """The command to run an agent verb where this drone's agent lives, and its cwd."""
    host = drone.get("host", "local")
    if host in LOCAL_HOSTS:
        return argv, str(ROOT)
    remote = drone.get("remote_dir", "~/FlightPlanner")
    inner = " ".join(shlex.quote(a) for a in argv)
    return (["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
             host, f"cd {quote_remote(remote)} && {inner}"], None)


def arm_only(sessions: list, hold_s: float) -> int:
    """The barrier passed: authorise ARM everywhere, hold, DISARM. No mission start."""
    log("[authorize] workstation authorises ARM on every drone")
    for s in sessions:
        s.send(f"ARM {s.token}")
    arms = [s.await_verb("arm", 40.0) for s in sessions]
    for s, r in zip(sessions, arms):
        log(f"[arm] slot {s.slot} sysid {r.get('sysid')}: "
            + ("ARMED" if r.get("armed") else f"NOT ARMED -- {r.get('error')}"))
    ok = all(r.get("armed") for r in arms)
    if ok:
        log(f"[arm] every drone armed -- holding {hold_s:.0f}s")
        time.sleep(hold_s)
    log("[disarm] disarming every drone")
    for s in sessions:
        s.send("DISARM")
    downs = [s.await_verb("disarm", 30.0) for s in sessions]
    for s, r in zip(sessions, downs):
        log(f"[disarm] slot {s.slot}: " + ("DISARMED" if r.get("disarmed")
                                           else f"STILL ARMED -- {r.get('error')}"))
    return 0 if ok and all(r.get("disarmed") for r in downs) else 1


def upload_one(drone: dict, plan: Path) -> dict:
    """One-shot `upload` verb on the drone's agent. Returns its json reply or an error."""
    slot = drone["slot"]
    argv = ["python3", AGENT_REL, "upload", "--link", str(drone["link"]),
            "--slot", str(slot), "--plan", str(plan)]
    if drone.get("sysid") is not None:
        argv += ["--expect-sysid", str(drone["sysid"])]
    popen_argv, cwd = host_argv(drone, argv)
    try:
        r = subprocess.run(popen_argv, cwd=cwd, capture_output=True, text=True,
                           timeout=UPLOAD_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return dict(slot=slot, verified=False, error=f"no result within {UPLOAD_TIMEOUT_S:.0f}s")
    for line in reversed(r.stdout.splitlines()):
        if line.startswith("{"):
            try:
                return json.loads(line)
            except json.JSONDecodeError:
                break
    tail = " | ".join((r.stderr or "").strip().splitlines()[-3:]) or "no output"
    return dict(slot=slot, verified=False, error=f"agent exited ({r.returncode}): {tail}")


class AgentSession:
    """One long-lived agent, local or over ssh, spoken to in json lines."""

    def __init__(self, drone: dict, plan: Path, params: str | None, run_id: str,
                 failsafe_s: float):
        self.drone = drone
        self.slot = drone["slot"]
        self.host = drone.get("host", "local")
        self.token: str | None = None
        self.replies: queue.Queue = queue.Queue()
        self.errtail: deque = deque(maxlen=20)

        argv = ["python3", AGENT_REL, "prepare",
                "--link", str(drone["link"]),
                "--slot", str(self.slot),
                "--plan", str(plan),
                "--run-id", run_id,
                "--failsafe-s", str(failsafe_s),
                "--resident"]
        if drone.get("sysid") is not None:
            argv += ["--expect-sysid", str(drone["sysid"])]
        if params:
            argv += ["--params", params]
        for name, value in (drone.get("set") or {}).items():
            argv += ["--set", f"{name}={value}"]

        popen_argv, cwd = host_argv(drone, argv)

        self.proc = subprocess.Popen(popen_argv, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                     text=True, bufsize=1, cwd=cwd)
        threading.Thread(target=self._pump_stdout, daemon=True).start()
        threading.Thread(target=self._pump_stderr, daemon=True).start()

    def _pump_stdout(self) -> None:
        for line in self.proc.stdout:
            line = line.strip()
            if not line:
                continue
            try:
                self.replies.put(json.loads(line))
            except json.JSONDecodeError:
                self.errtail.append(line)

    def _pump_stderr(self) -> None:
        for line in self.proc.stderr:
            self.errtail.append(line.rstrip())

    def send(self, cmd: str) -> None:
        try:
            self.proc.stdin.write(cmd + "\n")
            self.proc.stdin.flush()
        except (BrokenPipeError, ValueError):
            pass

    def await_verb(self, verb: str, timeout: float) -> dict:
        """Wait for a reply of this kind, surfacing anything unsolicited on the way."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                msg = self.replies.get(timeout=min(1.0, max(0.1, deadline - time.time())))
            except queue.Empty:
                if self.proc.poll() is not None:
                    return dict(slot=self.slot, error=self._died())
                continue
            if msg.get("verb") == verb:
                return msg
            if msg.get("verb") == "failsafe":
                log(f"[failsafe] slot {self.slot} returned to launch on its own "
                    f"(orchestrator went silent)")
            else:
                log(f"[agent] slot {self.slot}: unsolicited {msg}")
        return dict(slot=self.slot, error=f"no {verb} reply within {timeout:.0f}s")

    def _died(self) -> str:
        tail = " | ".join(list(self.errtail)[-3:]) or "no output"
        return f"agent exited ({self.proc.returncode}): {tail}"

    def close(self) -> None:
        self.send("QUIT")
        try:
            self.proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            self.proc.kill()


# ---------------------------------------------------------------- steps

def plan_from_kml(kml: Path, out: Path) -> Path:
    log(f"[plan] {kml.name} -> {out}")
    out.parent.mkdir(parents=True, exist_ok=True)
    proc = subprocess.run(["python3", "-m", "flight_planner.cli", str(kml),
                           "--output", str(out)],
                          capture_output=True, text=True, cwd=ROOT,
                          env={**os.environ, "PYTHONPATH": "src"})
    sys.stderr.write(proc.stderr)
    if proc.returncode != 0:
        raise SystemExit("[plan] the planner refused this mission -- nothing was pushed")
    return out.parent / "mission_drones"


def push(drone: dict, plan: Path) -> str | None:
    """Copy a plan to a remote agent. Returns an error string, never raises.

    An unreachable host must fail the barrier like any other not-ready drone, not crash
    the orchestrator with a traceback -- found by tools/test_handshake.py.
    """
    host = drone.get("host", "local")
    if host in LOCAL_HOSTS:
        return None
    remote = drone.get("remote_dir", "~/FlightPlanner")
    try:
        mk = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
                             host, f"mkdir -p {quote_remote(remote)}/{plan.parent}"],
                            capture_output=True, text=True, timeout=30)
        if mk.returncode != 0:
            return f"ssh {host}: {(mk.stderr or '').strip().splitlines()[-1:] or ['failed']}"[:200]
        cp = subprocess.run(["scp", "-q", "-o", "ConnectTimeout=10", str(ROOT / plan),
                             f"{host}:{remote}/{plan.parent}/"],
                            capture_output=True, text=True, timeout=60)
        if cp.returncode != 0:
            return f"scp to {host}: {(cp.stderr or '').strip()[:160]}"
    except subprocess.TimeoutExpired:
        return f"ssh/scp to {host} timed out"
    except OSError as exc:
        return f"ssh/scp to {host}: {exc}"
    log(f"[push] drone-{drone['slot']} -> {host}:{remote}/{plan}")
    return None


def abort_all(sessions: list[AgentSession], why: str) -> None:
    log(f"[abort] {why} -- returning every drone to launch")
    for s in sessions:
        s.send("ABORT rtl")
    for s in sessions:
        r = s.await_verb("abort", 20)
        log(f"[abort] slot {s.slot}: {'accepted' if r.get('accepted') else r}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fleet", required=True)
    ap.add_argument("--kml")
    ap.add_argument("--plans-dir")
    ap.add_argument("--output", default="missions/run/mission.plan")
    ap.add_argument("--dry-run", action="store_true",
                    help="prepare and barrier only -- never arms. Use this on a bench.")
    ap.add_argument("--upload-only", action="store_true",
                    help="push each plan, upload it to its aircraft, read it back, stop. "
                         "No params, no preflight, never arms.")
    ap.add_argument("--arm-only", action="store_true",
                    help="after the barrier, authorise ARM on every drone, hold, then "
                         "DISARM. Never starts a mission.")
    ap.add_argument("--arm-hold-s", type=float, default=5.0,
                    help="with --arm-only: how long to stay armed before disarming")
    ap.add_argument("--qgc", action="store_true",
                    help="open QGroundControl on this workstation if it is not running")
    ap.add_argument("--abort", action="store_true",
                    help="one-shot RTL to every drone, then exit")
    ap.add_argument("--at-in", type=float, default=None,
                    help="seconds from now for the shared launch instant")
    ap.add_argument("--failsafe-s", type=float, default=60.0,
                    help="each agent RTLs if this orchestrator goes silent for this long")
    ap.add_argument("--watch-timeout", type=float, default=1200.0)
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.fleet).read_text())
    fleet = cfg["drones"]
    params = cfg.get("params")
    run_id = f"run{uuid.uuid4().hex[:6]}"
    log(f"[fleet] {run_id}: " + ", ".join(
        f"slot {d['slot']}->sysid {d.get('sysid')} @ {d['link']} ({d.get('host','local')})"
        for d in fleet))

    if args.abort:
        for d in fleet:
            argv = ["python3", AGENT_REL, "abort", "--link", str(d["link"]),
                    "--slot", str(d["slot"]), "--action", "rtl"]
            subprocess.run(argv, cwd=ROOT)
        return 0

    # 1. plan
    if args.kml:
        plans_dir = plan_from_kml(Path(args.kml), Path(args.output))
    elif args.plans_dir:
        plans_dir = Path(args.plans_dir)
    else:
        plans_dir = Path(cfg.get("plans_dir") or ap.error("need --kml or --plans-dir"))

    plans = {}
    for d in fleet:
        p = plans_dir / f"drone-{d['slot']}.plan"
        if not p.exists():
            raise SystemExit(f"[plan] missing {p}")
        plans[d["slot"]] = p

    # 2. push -- a drone we cannot even reach fails the barrier like any other
    push_errors = {}
    for d in fleet:
        err = push(d, plans[d["slot"]])
        if err:
            push_errors[d["slot"]] = err
            log(f"[push] slot {d['slot']}: NOT REACHABLE -- {err}")
    if push_errors:
        log("[barrier] at least one drone is not ready -- NOBODY is armed")
        return 1

    if args.upload_only:
        log("[upload] uploading and reading back")
        with ThreadPoolExecutor(max_workers=len(fleet)) as pool:
            results = list(pool.map(lambda d: upload_one(d, plans[d["slot"]]), fleet))
        for d, r in zip(fleet, results):
            if r.get("verified"):
                log(f"[upload] slot {d['slot']} sysid {r.get('sysid')}: VERIFIED "
                    f"{r.get('onboard')}/{r.get('uploaded')} items in {r.get('upload_s')} s "
                    f"(plan {r.get('plan_sha')})")
            else:
                why = r.get("error") or "; ".join(r.get("problems") or ["unknown"])
                log(f"[upload] slot {d['slot']}: FAILED -- {why}")
        return 0 if all(r.get("verified") for r in results) else 1

    if args.qgc:
        open_qgc()

    # 3. prepare -- agents stay resident from here on
    log("[prepare] starting agents")
    sessions = [AgentSession(d, plans[d["slot"]], params, run_id, args.failsafe_s)
                for d in fleet]
    try:
        reports = [s.await_verb("prepare", PREPARE_TIMEOUT_S) for s in sessions]
        for s, r in zip(sessions, reports):
            s.token = r.get("token")
            if r.get("ready"):
                log(f"[prepare] slot {s.slot} sysid {r.get('sysid')}: READY "
                    f"({r.get('items')} items, top {r.get('top_alt_m')} m, token {s.token})")
            else:
                why = r.get("reasons") or [r.get("error", "unknown")]
                log(f"[prepare] slot {s.slot}: NOT READY -- {', '.join(map(str, why))}")
                for c in r.get("checks", []):
                    if not c["ok"]:
                        log(f"           {c['name']}: {c['detail']}")

        # 4. barrier
        if not all(r.get("ready") for r in reports):
            log("[barrier] at least one drone is not ready -- NOBODY is armed")
            return 1
        log("[barrier] all drones READY")

        if args.dry_run:
            log("[dry-run] stopping before arming, as asked")
            return 0

        if args.arm_only:
            return arm_only(sessions, args.arm_hold_s)

        # 5. start
        at = time.time() + (args.at_in if args.at_in is not None else START_SKEW_LEAD_S)
        log(f"[start] shared launch instant in {at - time.time():.1f}s")
        for s in sessions:
            s.send(f"START {s.token} {at:.3f}")
        starts = [s.await_verb("start", START_TIMEOUT_S) for s in sessions]
        for s, r in zip(sessions, starts):
            if not r.get("started"):
                log(f"[start] slot {s.slot} FAILED: {r.get('error')}")
        if not all(r.get("started") for r in starts):
            abort_all(sessions, "a drone failed to start")
            return 1
        spread = max(r["t_epoch"] for r in starts) - min(r["t_epoch"] for r in starts)
        log(f"[start] all away, launch spread {spread * 1000:.0f} ms")

        # 6. watch
        deadline = time.time() + args.watch_timeout
        while time.time() < deadline:
            time.sleep(WATCH_POLL_S)
            for s in sessions:
                s.send("STATUS")
            st = [s.await_verb("status", STATUS_TIMEOUT_S) for s in sessions]
            log("[watch] " + "  ".join(
                f"slot {s.slot}: wp {r.get('waypoint')} alt {r.get('rel_alt_m')}m "
                f"{'armed' if r.get('armed') else 'disarmed'}"
                for s, r in zip(sessions, st)))
            if all(r.get("armed") is False for r in st):
                log("[watch] every drone has disarmed")
                return 0
        abort_all(sessions, "watch timeout")
        return 1
    finally:
        for s in sessions:
            s.close()


if __name__ == "__main__":
    raise SystemExit(main())
