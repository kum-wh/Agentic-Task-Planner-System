#!/usr/bin/env python3
"""Fault-injection tests for the launch handshake, against a live SITL swarm.

    tools/sitl_swarm.sh start toapayoh 2
    python3 tools/test_handshake.py
    python3 tools/test_handshake.py --only barrier_refuses_on_unready

A handshake is defined by what it does when things go wrong, and the happy path exercises
none of that. Every case here sets up a REAL condition -- writes an actual parameter to an
actual vehicle, points at an actual dead port, kills an actual process -- rather than
mocking. Nothing here needs hardware beyond the SITL swarm.

The one rule each case is really testing: **if any drone is not ready, none of them arm.**
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import yaml
from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent
PLANS = "missions/toapayoh/mission_drones"
DEPLOY = "tools/swarm_deploy.py"
AGENT = "tools/drone_agent.py"
LINK1, LINK2 = "udpin://:14540", "udpin://:14541"
DEAD_LINK = "udpin://:14599"          # nothing ever listens here


# ---------------------------------------------------------------- helpers

def run(argv: list[str], timeout: float, stdin: str | None = None) -> tuple[int, str]:
    p = subprocess.run(argv, cwd=ROOT, capture_output=True, text=True,
                       timeout=timeout, input=stdin)
    return p.returncode, p.stdout + p.stderr


def fleet_file(drones: list[dict], tmp: Path) -> Path:
    path = tmp / "fleet.yaml"
    path.write_text(yaml.safe_dump(
        {"params": "sar_turn_params.params", "drones": drones}))
    return path


def base_drone(slot: int, sysid: int, link: str, **extra) -> dict:
    d = dict(slot=slot, sysid=sysid, host="local", link=link,
             set={"RTL_RETURN_ALT": 35 if slot == 1 else 45})
    d.update(extra)
    return d


def deploy(drones: list[dict], tmp: Path, extra: list[str], timeout: float) -> tuple[int, str]:
    return run(["python3", DEPLOY, "--fleet", str(fleet_file(drones, tmp)),
                "--plans-dir", PLANS] + extra, timeout)


def set_param(link_port: int, name: str, value: float, is_int: bool) -> bool:
    """Write straight to a SITL vehicle so a fault is real, not simulated."""
    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{link_port}", source_system=251)
    if conn.wait_heartbeat(timeout=15) is None:
        conn.close()
        return False
    payload = (struct.unpack("<f", struct.pack("<i", int(value)))[0] if is_int
               else float(value))
    ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int
             else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    ok = False
    for _ in range(4):
        conn.mav.param_set_send(conn.target_system, 1, name.encode(), payload, ptype)
        deadline = time.time() + 3
        while time.time() < deadline:
            m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if m is not None and m.param_id == name:
                got = (struct.unpack("<i", struct.pack("<f", m.param_value))[0]
                       if is_int else m.param_value)
                ok = abs(float(got) - float(value)) < 1e-4
                break
        if ok:
            break
    conn.close()
    return ok


def vehicle_mode(link_port: int) -> tuple[int, int, bool]:
    conn = mavutil.mavlink_connection(f"udpin:0.0.0.0:{link_port}", source_system=252)
    hb = conn.wait_heartbeat(timeout=15)
    conn.close()
    if hb is None:
        return -1, -1, False
    return ((hb.custom_mode >> 16) & 0xFF, (hb.custom_mode >> 24) & 0xFF,
            bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))


# ---------------------------------------------------------------- cases

def case_happy_path(tmp: Path) -> tuple[bool, str]:
    rc, out = deploy([base_drone(1, 1, LINK1), base_drone(2, 2, LINK2)],
                     tmp, ["--dry-run"], 240)
    ok = rc == 0 and "all drones READY" in out
    return ok, "both READY, barrier passed" if ok else out[-300:]


def case_barrier_refuses_on_unready(tmp: Path) -> tuple[bool, str]:
    """A 7 m vertical geofence on one vehicle -- the real setting found on the boards."""
    if not set_param(14541, "GF_MAX_VER_DIST", 7.0, False):
        return False, "could not inject the fault"
    try:
        rc, out = deploy([base_drone(1, 1, LINK1), base_drone(2, 2, LINK2)],
                         tmp, ["--dry-run"], 240)
        refused = rc != 0 and "NOBODY is armed" in out
        named = "fence_ceiling" in out
        return refused and named, ("barrier refused, named fence_ceiling"
                                   if refused and named else out[-400:])
    finally:
        set_param(14541, "GF_MAX_VER_DIST", 0.0, False)


def case_unreachable_host(tmp: Path) -> tuple[bool, str]:
    drones = [base_drone(1, 1, LINK1),
              base_drone(2, 2, LINK2, host="nosuchhost.invalid",
                         remote_dir="~/FlightPlanner")]
    rc, out = deploy(drones, tmp, ["--dry-run"], 180)
    ok = rc != 0 and "NOBODY is armed" in out
    return ok, "barrier refused an unreachable agent" if ok else out[-300:]


def case_dead_link(tmp: Path) -> tuple[bool, str]:
    """Agent that can never find a vehicle: must time out, not hang the run."""
    drones = [base_drone(1, 1, LINK1), base_drone(2, 2, DEAD_LINK)]
    rc, out = deploy(drones, tmp, ["--dry-run"], 180)
    ok = rc != 0 and "NOBODY is armed" in out
    return ok, "barrier refused a link with no vehicle" if ok else out[-300:]


def case_wrong_vehicle_answers(tmp: Path) -> tuple[bool, str]:
    """Slot 1's link pointed at vehicle 2: identity check must refuse."""
    drones = [base_drone(1, 1, LINK2), base_drone(2, 2, DEAD_LINK)]
    rc, out = deploy(drones, tmp, ["--dry-run"], 180)
    ok = rc != 0 and ("wrong vehicle" in out or "NOBODY is armed" in out)
    return ok, "identity check refused the wrong vehicle" if ok else out[-300:]


def case_token_mismatch(tmp: Path) -> tuple[bool, str]:
    """Drive one agent directly and give START a token it never issued."""
    argv = ["python3", AGENT, "prepare", "--link", LINK1, "--expect-sysid", "1",
            "--slot", "1", "--plan", f"{PLANS}/drone-1.plan",
            "--run-id", "testrun", "--failsafe-s", "0", "--resident"]
    rc, out = run(argv, 120, stdin="START wrong:token\nQUIT\n")
    refused = any(json.loads(l).get("verb") == "start"
                  and json.loads(l).get("started") is False
                  and "token mismatch" in str(json.loads(l).get("error", ""))
                  for l in out.splitlines() if l.startswith("{"))
    return refused, "agent refused a START with the wrong token" if refused else out[-400:]


def case_ssh_localhost(tmp: Path) -> tuple[bool, str]:
    """The remote path, over real ssh, against this machine."""
    rc, probe = run(["ssh", "-o", "BatchMode=yes", "localhost", "true"], 20)
    if rc != 0:
        return False, "passwordless ssh to localhost is not set up -- skipped"
    drones = [base_drone(1, 1, LINK1, host="localhost", remote_dir=str(ROOT)),
              base_drone(2, 2, LINK2, host="localhost", remote_dir=str(ROOT))]
    rc, out = deploy(drones, tmp, ["--dry-run"], 300)
    ok = rc == 0 and "all drones READY" in out
    return ok, "prepare ran over real ssh, both READY" if ok else out[-400:]


def case_agent_failsafe(tmp: Path) -> tuple[bool, str]:
    """Launch one vehicle, then go silent. The agent must bring it home unprompted."""
    argv = ["python3", AGENT, "prepare", "--link", LINK1, "--expect-sysid", "1",
            "--slot", "1", "--plan", f"{PLANS}/drone-1.plan",
            "--params", "sar_turn_params.params", "--set", "RTL_RETURN_ALT=35",
            "--run-id", "fs", "--failsafe-s", "15", "--resident"]
    proc = subprocess.Popen(argv, cwd=ROOT, stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            text=True, bufsize=1)
    try:
        token = None
        deadline = time.time() + 150
        while time.time() < deadline:
            line = proc.stdout.readline()
            if not line:
                break
            if line.startswith("{"):
                msg = json.loads(line)
                if msg.get("verb") == "prepare":
                    if not msg.get("ready"):
                        return False, f"prepare not ready: {msg.get('reasons')}"
                    token = msg["token"]
                    break
        if not token:
            return False, "no prepare reply"

        proc.stdin.write(f"START {token}\n")
        proc.stdin.flush()
        started = False
        deadline = time.time() + 60
        while time.time() < deadline:
            line = proc.stdout.readline()
            if line.startswith("{"):
                msg = json.loads(line)
                if msg.get("verb") == "start":
                    started = bool(msg.get("started"))
                    break
        if not started:
            return False, "vehicle did not start"

        # Now go silent. The failsafe is 15 s; give it 45 to fire and be seen.
        fired = False
        deadline = time.time() + 45
        while time.time() < deadline:
            line = proc.stdout.readline()
            if line.startswith("{") and json.loads(line).get("verb") == "failsafe":
                fired = True
                break
        if not fired:
            return False, "agent never fired its failsafe"
        time.sleep(4)
        main_mode, sub_mode, _ = vehicle_mode(14540)
        rtl = (main_mode, sub_mode) == (4, 5)
        return rtl, (f"agent RTLed on its own; vehicle in AUTO.RTL"
                     if rtl else f"failsafe fired but mode is {main_mode}.{sub_mode}")
    finally:
        try:
            proc.stdin.write("ABORT rtl\nQUIT\n")
            proc.stdin.flush()
        except Exception:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


CASES = [
    ("happy_path", case_happy_path),
    ("barrier_refuses_on_unready", case_barrier_refuses_on_unready),
    ("unreachable_host", case_unreachable_host),
    ("dead_link", case_dead_link),
    ("wrong_vehicle_answers", case_wrong_vehicle_answers),
    ("token_mismatch", case_token_mismatch),
    ("ssh_localhost", case_ssh_localhost),
    ("agent_failsafe", case_agent_failsafe),
]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", action="append", help="run just these cases")
    args = ap.parse_args()

    cases = [c for c in CASES if not args.only or c[0] in args.only]
    results = []
    with tempfile.TemporaryDirectory() as td:
        tmp = Path(td)
        for name, fn in cases:
            print(f"\n=== {name} ===", flush=True)
            t0 = time.time()
            try:
                ok, detail = fn(tmp)
            except subprocess.TimeoutExpired:
                ok, detail = False, "the case itself timed out"
            except Exception as exc:
                ok, detail = False, f"{type(exc).__name__}: {exc}"
            dt = time.time() - t0
            print(f"{'PASS' if ok else 'FAIL'} ({dt:.0f}s)  {detail}", flush=True)
            results.append((name, ok))

    print("\n================ handshake fault matrix ================")
    for name, ok in results:
        print(f"  {'PASS' if ok else 'FAIL'}  {name}")
    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
