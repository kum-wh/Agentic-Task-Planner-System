#!/usr/bin/env python3
"""One vehicle's agent: prepare a mission, report readiness, start, watch, abort.

    python3 tools/drone_agent.py prepare --link udpin://:14540 --expect-sysid 1 \
        --plan missions/toapayoh/mission_drones/drone-1.plan --params sar_turn_params.params
    python3 tools/drone_agent.py upload  --link udpin://:14540 --expect-sysid 1 \
        --plan missions/toapayoh/mission_drones/drone-1.plan
    python3 tools/drone_agent.py start   --link udpin://:14540 --expect-sysid 1
    python3 tools/drone_agent.py status  --link udpin://:14540 --expect-sysid 1
    python3 tools/drone_agent.py abort   --link udpin://:14540 --expect-sysid 1

This is the piece that runs next to a single aircraft — on a Jetson in the field, on this
workstation against SITL. It owns exactly one vehicle and speaks only MAVLink, so the link
is the only thing that changes between a simulated vehicle, a board on USB (maintenance
only) and a flight controller at the far end of an ELRS radio.

No GUI. QGroundControl has no CLI, no IPC and no headless mode, so scripting it means
synthesising mouse clicks -- not something to put on the critical path of a live flight.
QGC also uploads to one vehicle at a time; agents upload to every vehicle at once.

Every verb prints ONE json object to stdout for the orchestrator. Human-readable progress
goes to stderr, so the two never mix.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import select
import struct
import sys
import threading
import time
from pathlib import Path

from pymavlink import mavutil

sys.path.insert(0, str(Path(__file__).resolve().parent))
import safety_rules  # noqa: E402

INT_PARAM_TYPES = {1, 2, 3, 4, 5, 6, 7, 8}
HEARTBEAT_TIMEOUT_S = 20.0
MISSION_UPLOAD_TIMEOUT_S = 120.0
UPLOAD_S_PER_ITEM = 4.0      # ELRS measured ~1 s/item; leave 4x headroom for a bad link
# PX4 stores MISSION_ITEM_INT, so a plan's local-relative frame 3 reads back as 6 and 0 as 5.
FRAME_EQUIV = {3: 6, 0: 5}
PARAM_READ_ATTEMPTS = 3
NAV_COMMANDS_WITH_ALTITUDE = {16, 22, 21, 82}      # waypoint, takeoff, land, spline
MIN_BATTERY_FRACTION = 0.30
FENCE_MARGIN_M = 5.0


def log(msg: str) -> None:
    print(msg, file=sys.stderr, flush=True)


def emit(obj: dict) -> None:
    """The single machine-readable line. Orchestrator reads stdout, nothing else."""
    print(json.dumps(obj), flush=True)


# ---------------------------------------------------------------- link + parameters

def normalise_link(url: str) -> str:
    """Accept MAVSDK-style URLs as well as pymavlink's own spelling.

    The link is the ONLY thing that differs between SITL, a USB board and a radio, so it
    is worth accepting whatever spelling the caller has to hand.
    """
    u = url.replace("://", ":")
    if u.startswith("udpin:") and u.count(":") == 2 and u.split(":")[1] == "":
        return "udpin:0.0.0.0:" + u.rsplit(":", 1)[1]        # udpin://:14540
    if u.startswith("serial:"):
        rest = u[len("serial:"):]
        return rest.lstrip("/") if rest.startswith("///") else rest
    return u


def start_heartbeat(conn) -> None:
    """Announce ourselves as a ground station, once a second, forever.

    PX4 refuses to arm with "No connection to the ground control station" if no GCS
    heartbeat has arrived -- the command comes back MAV_RESULT_TEMPORARILY_REJECTED with
    no explanation. MAVSDK does this for you; pymavlink does not. Measured 28 Sep 2026.

    Sends are serialised behind a lock because this thread and the main one share one
    link, and on a serial link two interleaved writes corrupt both frames.
    """
    lock = threading.Lock()
    raw_write = conn.write

    def locked_write(buf):
        with lock:
            return raw_write(buf)

    conn.write = locked_write

    def beat():
        while True:
            try:
                conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            except Exception:
                return
            time.sleep(1.0)

    threading.Thread(target=beat, daemon=True).start()


def connect(link: str, expect_sysid: int | None) -> mavutil.mavlink_connection:
    conn = mavutil.mavlink_connection(normalise_link(link), source_system=250,
                                      source_component=190)
    start_heartbeat(conn)
    log(f"[link] {link} -- waiting for heartbeat")
    hb = conn.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT_S)
    if hb is None:
        raise TimeoutError(f"no heartbeat on {link} within {HEARTBEAT_TIMEOUT_S:.0f}s")
    log(f"[link] connected, vehicle is sysid {conn.target_system}")
    if expect_sysid is not None and conn.target_system != expect_sysid:
        raise ValueError(f"wrong vehicle: link {link} answers as sysid "
                         f"{conn.target_system}, expected {expect_sysid}")
    return conn


def decode_param(msg) -> float | int:
    """PX4 advertises PARAM_ENCODE_BYTEWISE: an int arrives as the bits of a float."""
    if msg.param_type in INT_PARAM_TYPES:
        return int(struct.unpack("<i", struct.pack("<f", msg.param_value))[0])
    return msg.param_value


def read_param(conn, name: str, timeout: float = 3.0):
    for _ in range(PARAM_READ_ATTEMPTS):
        conn.mav.param_request_read_send(conn.target_system, 1, name.encode(), -1)
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg is not None and msg.param_id == name:
                return decode_param(msg)
    return None


def write_param(conn, name: str, value: float, is_int: bool) -> bool:
    payload = (struct.unpack("<f", struct.pack("<i", int(value)))[0] if is_int
               else float(value))
    ptype = (mavutil.mavlink.MAV_PARAM_TYPE_INT32 if is_int
             else mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    conn.mav.param_set_send(conn.target_system, 1, name.encode(), payload, ptype)
    deadline = time.time() + 4
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            got = decode_param(msg)
            return abs(float(got) - float(value)) < 1e-4
    return False


def write_param_typed(conn, name: str, value: float) -> bool:
    """Write a parameter whose type we look up on the vehicle first.

    Per-drone overrides come from fleet.yaml as plain numbers, with no type information.
    Guessing int-vs-float wrong writes nonsense, because PX4 encodes integers bytewise.
    """
    conn.mav.param_request_read_send(conn.target_system, 1, name.encode(), -1)
    deadline = time.time() + 4
    ptype = None
    while time.time() < deadline:
        msg = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None and msg.param_id == name:
            ptype = msg.param_type
            break
    if ptype is None:
        return False
    return write_param(conn, name, value, ptype in INT_PARAM_TYPES)


def load_param_file(path: Path) -> list[tuple[str, float, bool]]:
    """QGC .params -> [(name, value, is_int)]. Columns: sysid compid name value type."""
    out = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        f = line.split()
        if len(f) >= 5:
            out.append((f[2], float(f[3]), f[4] == "6"))
    return out


# ---------------------------------------------------------------- mission

def plan_to_items(plan_path: Path) -> tuple[list[dict], float]:
    """QGC .plan -> MISSION_ITEM_INT fields, plus the mission's highest commanded altitude.

    The .plan's items map 1:1 onto MAVLink mission items; no home item is prepended.
    `params` is [p1, p2, p3, p4, x, y, z] and any of them may be null.
    """
    plan = json.loads(plan_path.read_text())
    raw = plan["mission"]["items"]
    items, top_alt = [], 0.0
    for seq, it in enumerate(raw):
        p = [0.0 if v is None else float(v) for v in it["params"]]
        cmd = int(it["command"])
        if cmd in NAV_COMMANDS_WITH_ALTITUDE:
            top_alt = max(top_alt, p[6])
        items.append(dict(
            seq=seq, frame=int(it["frame"]), command=cmd,
            current=1 if seq == 0 else 0,
            autocontinue=1 if it.get("autoContinue", True) else 0,
            p1=p[0], p2=p[1], p3=p[2], p4=p[3],
            x=int(round(p[4] * 1e7)), y=int(round(p[5] * 1e7)), z=p[6],
        ))
    return items, top_alt


def upload_mission(conn, items: list[dict], timeout: float = MISSION_UPLOAD_TIMEOUT_S) -> None:
    """Mission protocol: COUNT, serve each request, wait for ACK.

    PX4 keeps the mission AND the current item index in dataman across restarts, and
    uploading does not reset the index. A vehicle whose last run ended on the RTL item
    will otherwise resume *there* -- "return to launch" from 1 m, then disarm on the
    ground, having never taken off. Measured 24 Sep 2026. Hence the clear and the
    explicit set-current below.
    """
    MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    conn.mav.mission_clear_all_send(conn.target_system, 1, MT)
    conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)

    def send_item(seq: int) -> None:
        i = items[seq]
        conn.mav.mission_item_int_send(
            conn.target_system, 1, i["seq"], i["frame"], i["command"],
            i["current"], i["autocontinue"], i["p1"], i["p2"], i["p3"], i["p4"],
            i["x"], i["y"], i["z"], MT)

    conn.mav.mission_count_send(conn.target_system, 1, len(items), MT)
    sent, deadline = set(), time.time() + timeout
    last_seq, quiet_resends = None, 0
    while time.time() < deadline:
        msg = conn.recv_match(type=["MISSION_REQUEST", "MISSION_REQUEST_INT",
                                    "MISSION_ACK"], blocking=True, timeout=2.5)
        if msg is None:
            # a radio drops packets: nudge the vehicle by repeating our last message
            quiet_resends += 1
            if quiet_resends > 6:
                raise TimeoutError(
                    f"vehicle stopped requesting items at {len(sent)}/{len(items)}")
            log(f"[plan] silence at {len(sent)}/{len(items)}; resend {quiet_resends}/6")
            if last_seq is None:
                conn.mav.mission_count_send(conn.target_system, 1, len(items), MT)
            else:
                send_item(last_seq)
            continue
        quiet_resends = 0
        if msg.get_type() == "MISSION_ACK":
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f"vehicle rejected the mission: MAV_MISSION result {msg.type}")
            break
        send_item(msg.seq)
        last_seq = msg.seq
        sent.add(msg.seq)
    else:
        raise TimeoutError("no MISSION_ACK before timeout")

    conn.mav.mission_set_current_send(conn.target_system, 1, 0)
    log(f"[plan] uploaded {len(items)} items, current index reset to 0")


def mission_count_onboard(conn) -> int | None:
    conn.mav.mission_request_list_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    msg = conn.recv_match(type="MISSION_COUNT", blocking=True, timeout=6)
    if msg is None:
        return None
    conn.mav.mission_ack_send(conn.target_system, 1,
                              mavutil.mavlink.MAV_MISSION_ACCEPTED,
                              mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    return msg.count


def download_mission(conn, attempts: int = 4) -> list[dict] | None:
    """Read the whole mission back from the vehicle, item by item. None if it cannot."""
    MT = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    count = None
    for _ in range(attempts):
        conn.mav.mission_request_list_send(conn.target_system, 1, MT)
        msg = conn.recv_match(type="MISSION_COUNT", blocking=True, timeout=6)
        if msg is not None:
            count = msg.count
            break
    if count is None:
        return None

    def fetch(seq: int):
        for _ in range(attempts):
            conn.mav.mission_request_int_send(conn.target_system, 1, seq, MT)
            deadline = time.time() + 5
            while time.time() < deadline:
                m = conn.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=1)
                if m is not None and m.seq == seq:
                    return m
        return None

    items = []
    for seq in range(count):
        m = fetch(seq)
        if m is None:
            log(f"[readback] item {seq}/{count} never arrived")
            return None
        items.append(dict(seq=m.seq, frame=m.frame, command=m.command,
                          p1=m.param1, p2=m.param2, p3=m.param3, p4=m.param4,
                          x=m.x, y=m.y, z=m.z))
    conn.mav.mission_ack_send(conn.target_system, 1,
                              mavutil.mavlink.MAV_MISSION_ACCEPTED, MT)
    return items


def diff_items(sent: list[dict], got: list[dict]) -> list[str]:
    """Human-readable differences between what we sent and what the vehicle holds."""
    if len(sent) != len(got):
        return [f"{len(got)} items on board, {len(sent)} sent"]
    norm = lambda f: FRAME_EQUIV.get(f, f)
    out = []
    for a, b in zip(sent, got):
        bad = []
        if a["command"] != b["command"]:
            bad.append("command")
        if norm(a["frame"]) != norm(b["frame"]):
            bad.append("frame")
        if a["x"] != b["x"] or a["y"] != b["y"]:
            bad.append("position")
        bad += [k for k in ("p1", "p2", "p3", "p4", "z") if abs(a[k] - b[k]) > 1e-3]
        if bad:
            out.append(f"item {a['seq']}: {'/'.join(bad)} differ")
    return out


def vehicle_armed(conn) -> bool | None:
    """Armed state from the VEHICLE's own heartbeat (not ours, not another GCS's)."""
    deadline = time.time() + 8
    while time.time() < deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if msg is None or msg.get_srcSystem() != conn.target_system:
            continue
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            continue
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return None


# ---------------------------------------------------------------- preflight contract

def gather_state(conn, seconds: float = 6.0) -> dict:
    state, deadline = {}, time.time() + seconds
    while time.time() < deadline:
        msg = conn.recv_match(blocking=True, timeout=2)
        if msg is None or msg.get_type() == "BAD_DATA":
            continue
        state[msg.get_type()] = msg
    return state


def preflight_contract(conn, expect_sysid, top_alt, uploaded, onboard) -> list[dict]:
    """The checks that a .plan cannot express.

    Every one of these came from something that actually bit us: a 7 m vertical geofence
    with GF_ACTION=Land that put the aircraft back on the ground during climb-out, an
    RTL_RETURN_ALT of 5 m that would transit home below the rooftops, NAV_RCL_ACT=Land on
    a link where RC loss is also C2 loss.
    """
    checks = []

    def add(name, ok, detail):
        checks.append(dict(name=name, ok=bool(ok), detail=detail))

    add("identity", conn.target_system == expect_sysid,
        f"vehicle answers as sysid {conn.target_system}, slot expects {expect_sysid}")
    add("mission_uploaded", onboard == uploaded,
        f"{onboard} items on board, {uploaded} in the plan")

    gf_dist = read_param(conn, "GF_MAX_VER_DIST")
    gf_act = read_param(conn, "GF_ACTION")
    fence_ok = gf_dist is not None and (gf_dist == 0 or gf_dist > top_alt + FENCE_MARGIN_M)
    add("fence_ceiling", fence_ok,
        f"GF_MAX_VER_DIST={gf_dist} (0=off), mission tops out at {top_alt:.1f} m"
        + ("" if fence_ok else f"; GF_ACTION={gf_act}"))

    rtl_alt = read_param(conn, "RTL_RETURN_ALT")
    add("rtl_altitude", rtl_alt is not None and rtl_alt >= top_alt,
        f"RTL_RETURN_ALT={rtl_alt}, mission tops out at {top_alt:.1f} m")

    rcl = read_param(conn, "NAV_RCL_ACT")
    add("rc_loss_action", rcl is not None and rcl != 3,
        f"NAV_RCL_ACT={rcl} (3=Land in place, 2=Return)")

    state = gather_state(conn)
    gps = state.get("GPS_RAW_INT")
    fix = gps.fix_type if gps else 0
    add("gps_fix", fix >= 3, f"fix_type={fix}, satellites={gps.satellites_visible if gps else 0}")

    pos = state.get("GLOBAL_POSITION_INT")
    add("position_estimate", pos is not None,
        "global position published" if pos else "no GLOBAL_POSITION_INT")

    batt = state.get("BATTERY_STATUS")
    remaining = batt.battery_remaining if batt else -1
    add("battery", remaining < 0 or remaining >= MIN_BATTERY_FRACTION * 100,
        f"battery_remaining={remaining}% (-1 = not reported)")

    return checks


# ---------------------------------------------------------------- verbs

def run_token(run_id: str, plan_path: Path) -> str:
    """Binds a START to the exact plan that was prepared.

    Without it, `start` has no memory that `prepare` ever happened: it would arm a vehicle
    that was never verified, or one whose plan was replaced in between. The barrier checks
    that every drone *said* ready; the token checks that the thing about to arm is the
    thing that was checked.
    """
    digest = hashlib.sha256(plan_path.read_bytes()).hexdigest()[:12]
    return f"{run_id}:{digest}"


def resident_loop(conn, args, token: str) -> int:
    """Hold the link and take commands on stdin until told to stop.

    The link staying up is the point. PX4 will not arm for a ground station it has not
    heard from recently, so a one-shot `start` that reconnects has to re-establish that
    relationship from cold -- cheap on localhost UDP, expensive on a radio whose
    round-trip is tens of milliseconds. One connection from prepare through landing.
    """
    state = dict(failsafe_armed=False, last_cmd=time.time(), armed_only=False)
    guard = Guard(conn, args)
    # pymavlink's connection is not safe to read from two threads. Every command handler
    # below runs under this lock, and the monitor only reads while holding it, so there is
    # exactly one reader at any instant.
    conn_lock = threading.Lock()

    def monitor():
        while True:
            time.sleep(0.2)
            with conn_lock:
                msg = conn.recv_match(blocking=False)
                while msg is not None:
                    guard.observe(msg)
                    msg = conn.recv_match(blocking=False)
                acted = guard.check()
            if acted:
                state["failsafe_armed"] = False
                if acted.get("carried_out") in (safety_rules.LAND, safety_rules.DISARM):
                    state["armed_only"] = False

    def disarm_if_left_armed():
        # We armed it for an arm-only test and the orchestrator is gone: do not leave
        # motors armed with nobody watching.
        if state["armed_only"]:
            log("[resident] leaving while armed by ARM -- disarming")
            with conn_lock:
                do_disarm(conn)
            state["armed_only"] = False

    def watchdog():
        while True:
            time.sleep(1.0)
            if guard.pilot_has_it:
                state["failsafe_armed"] = False
            if (state["failsafe_armed"] and args.failsafe_s > 0
                    and time.time() - state["last_cmd"] > args.failsafe_s):
                log(f"[failsafe] no command for {args.failsafe_s:.0f}s -- returning to launch")
                with conn_lock:
                    r = do_abort(conn, "rtl")
                emit(dict(verb="failsafe", slot=args.slot, sysid=conn.target_system,
                          t_epoch=time.time(), **r))
                state["failsafe_armed"] = False

    threading.Thread(target=watchdog, daemon=True).start()
    if guard.enabled:
        threading.Thread(target=monitor, daemon=True).start()
        log(f"[resident] safety guard on: HOLD at {guard.wd.limits.hold_rate}/min, "
            f"LAND at {guard.wd.limits.land_rate}/min, ABORT flag {guard.flag}")
    log(f"[resident] holding link, token {token}, "
        f"failsafe {'off' if args.failsafe_s <= 0 else f'{args.failsafe_s:.0f}s'}")

    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        state["last_cmd"] = time.time()
        parts = line.split()
        verb = parts[0].upper()

        # Emergency words outrank the token dance: no run id, no plan, no argument.
        if verb in ("ABORT", "LAND", "HOLD", "KILL") and len(parts) == 1:
            with conn_lock:
                if verb == "KILL":
                    guard.kill("operator said KILL on stdin")
                else:
                    guard.operator(verb)
                    acted = guard.check()
            state["failsafe_armed"] = False
            continue

        if verb == "START":
            given = parts[1] if len(parts) > 1 else ""
            if given != token:
                emit(dict(verb="start", slot=args.slot, sysid=conn.target_system,
                          started=False,
                          error=f"token mismatch: given {given!r}, prepared {token!r}"))
                continue
            at = float(parts[2]) if len(parts) > 2 else None
            with conn_lock:
                r = do_start(conn, at)
            emit(dict(verb="start", slot=args.slot, sysid=conn.target_system, **r))
            state["last_cmd"] = time.time()
            state["failsafe_armed"] = bool(r.get("started"))
            if r.get("started"):
                state["armed_only"] = False

        elif verb == "ARM":
            given = parts[1] if len(parts) > 1 else ""
            if given != token:
                emit(dict(verb="arm", slot=args.slot, sysid=conn.target_system,
                          armed=False,
                          error=f"token mismatch: given {given!r}, prepared {token!r}"))
                continue
            at = float(parts[2]) if len(parts) > 2 else None
            with conn_lock:
                r = do_arm(conn, at)
            state["armed_only"] = bool(r.get("armed"))
            emit(dict(verb="arm", slot=args.slot, sysid=conn.target_system, **r))
            state["last_cmd"] = time.time()

        elif verb == "DISARM":
            with conn_lock:
                r = do_disarm(conn)
            if r.get("disarmed"):
                state["armed_only"] = False
            emit(dict(verb="disarm", slot=args.slot, sysid=conn.target_system, **r))

        elif verb == "STATUS":
            with conn_lock:
                st = do_status(conn)
            emit(dict(verb="status", slot=args.slot, sysid=conn.target_system, **st))

        elif verb == "ABORT":
            state["failsafe_armed"] = False
            action = parts[1] if len(parts) > 1 else "rtl"
            with conn_lock:
                ab = do_abort(conn, action)
            emit(dict(verb="abort", slot=args.slot, sysid=conn.target_system, **ab))

        elif verb == "QUIT":
            disarm_if_left_armed()
            emit(dict(verb="quit", slot=args.slot, sysid=conn.target_system))
            return 0

        else:
            emit(dict(verb="error", slot=args.slot, error=f"unknown command {verb!r}"))

    log("[resident] stdin closed")
    disarm_if_left_armed()
    return 0


def cmd_upload(args) -> int:
    """Put a plan on the aircraft and prove it is there. Never arms, never starts.

    Replaces whatever mission the vehicle holds. Refuses an armed vehicle. Success means
    the autopilot read the same mission back, not merely that it said ACK.
    """
    conn = connect(args.link, args.expect_sysid)
    plan_path = Path(args.plan)
    items, top_alt = plan_to_items(plan_path)
    log(f"[plan] {plan_path.name}: {len(items)} items, top altitude {top_alt:.1f} m")

    armed = vehicle_armed(conn)
    if armed is not False:
        raise RuntimeError("refusing to upload: vehicle is ARMED" if armed else
                           "refusing to upload: no heartbeat from the vehicle to check "
                           "its arming state")

    t0 = time.time()
    upload_mission(conn, items, timeout=max(MISSION_UPLOAD_TIMEOUT_S,
                                            UPLOAD_S_PER_ITEM * len(items)))
    upload_s = time.time() - t0
    log(f"[upload] {len(items)} items in {upload_s:.1f} s ({upload_s / len(items):.2f} s/item)")

    got = download_mission(conn)
    problems = (["could not read the mission back from the vehicle"] if got is None
                else diff_items(items, got))
    for pr in problems:
        log(f"[readback] {pr}")
    verified = not problems
    log(f"[readback] {'VERIFIED' if verified else 'FAILED'}: "
        f"{len(got) if got else 0} of {len(items)} items on the vehicle")
    emit(dict(verb="upload", slot=args.slot, sysid=conn.target_system, verified=verified,
              uploaded=len(items), onboard=None if got is None else len(got),
              problems=problems, upload_s=round(upload_s, 1), top_alt_m=top_alt,
              plan=str(plan_path),
              plan_sha=hashlib.sha256(plan_path.read_bytes()).hexdigest()[:12]))
    return 0 if verified else 1


def cmd_prepare(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    plan_path = Path(args.plan)
    items, top_alt = plan_to_items(plan_path)
    log(f"[plan] {plan_path.name}: {len(items)} items, top altitude {top_alt:.1f} m")

    if args.params:
        applied = failed = 0
        for name, value, is_int in load_param_file(Path(args.params)):
            if write_param(conn, name, value, is_int):
                applied += 1
            else:
                failed += 1
        log(f"[param] {applied} applied, {failed} failed")

    # Per-drone overrides. This is where RTL separation lives: each aircraft returns at
    # its own altitude, so two drones coming home never collapse onto one layer.
    for pair in args.set or []:
        name, _, raw = pair.partition("=")
        if write_param_typed(conn, name.strip(), float(raw)):
            log(f"[param] override {name.strip()} = {raw}")
        else:
            log(f"[param] override {name.strip()} FAILED")

    upload_mission(conn, items)
    onboard = mission_count_onboard(conn)
    checks = preflight_contract(conn, args.expect_sysid, top_alt, len(items), onboard)

    ready = all(c["ok"] for c in checks)
    for c in checks:
        log(f"[check] {'PASS' if c['ok'] else 'FAIL'}  {c['name']}: {c['detail']}")
    token = run_token(args.run_id, plan_path)
    emit(dict(verb="prepare", slot=args.slot, sysid=conn.target_system, ready=ready,
              plan=str(plan_path), items=len(items), top_alt_m=top_alt,
              token=token, checks=checks,
              reasons=[c["name"] for c in checks if not c["ok"]]))
    if args.resident:
        return resident_loop(conn, args, token)
    return 0 if ready else 1


def hold_for(at: float | None, what: str) -> None:
    if at:
        wait = at - time.time()
        if wait > 0:
            log(f"[{what}] holding {wait:.2f}s for the shared instant")
            time.sleep(wait)


def do_arm(conn, at: float | None = None) -> dict:
    """Arm and wait to SEE the armed heartbeat. Never starts the mission."""
    hold_for(at, "arm")
    reason, deadline, last_try = None, time.time() + 20, 0.0
    while time.time() < deadline:
        if time.time() - last_try > 3.0:
            conn.mav.command_long_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 0, 0, 0, 0, 0, 0)
            last_try = time.time()
        ack = conn.recv_match(type="COMMAND_ACK", blocking=False)
        if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            reason = f"arm command result {ack.result}"
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.get_srcSystem() == conn.target_system \
                and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("[arm] armed")
            return dict(armed=True, t_epoch=time.time())
    return dict(armed=False, error="vehicle did not arm" + (f" ({reason})" if reason else ""))


def do_disarm(conn) -> dict:
    """Disarm and wait to SEE the disarmed heartbeat."""
    deadline, last_try = time.time() + 15, 0.0
    while time.time() < deadline:
        if time.time() - last_try > 3.0:
            conn.mav.command_long_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            last_try = time.time()
        hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
        if hb and hb.get_srcSystem() == conn.target_system \
                and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("[disarm] disarmed")
            return dict(armed=False, disarmed=True)
    return dict(armed=True, disarmed=False, error="vehicle did not disarm")


def do_start(conn, at: float | None) -> dict:
    """Arm, then MISSION_START. Returns the report; the caller decides how to emit it."""
    arm = do_arm(conn, at)
    if not arm["armed"]:
        return dict(started=False, armed=False, error=arm["error"])

    log("[start] armed")
    conn.mav.command_long_send(conn.target_system, 1,
                               mavutil.mavlink.MAV_CMD_MISSION_START,
                               0, 0, 0, 0, 0, 0, 0, 0)
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    ok = ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    log(f"[start] MISSION_START {'accepted' if ok else 'not accepted'}")
    return dict(started=ok, armed=True, t_epoch=time.time())


def do_status(conn) -> dict:
    state = gather_state(conn, seconds=3)
    hb = state.get("HEARTBEAT")
    pos = state.get("GLOBAL_POSITION_INT")
    cur = state.get("MISSION_CURRENT")
    batt = state.get("BATTERY_STATUS")
    return dict(
        armed=bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) if hb else None,
        main_mode=(hb.custom_mode >> 16) & 0xFF if hb else None,
        sub_mode=(hb.custom_mode >> 24) & 0xFF if hb else None,
        waypoint=cur.seq if cur else None,
        rel_alt_m=round(pos.relative_alt / 1000.0, 2) if pos else None,
        lat=pos.lat / 1e7 if pos else None, lon=pos.lon / 1e7 if pos else None,
        battery_pct=batt.battery_remaining if batt else None)


def do_abort(conn, action: str) -> dict:
    cmd = (mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH if action == "rtl"
           else mavutil.mavlink.MAV_CMD_NAV_LAND)
    conn.mav.command_long_send(conn.target_system, 1, cmd, 0, 0, 0, 0, 0, 0, 0, 0)
    ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    ok = ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    log(f"[abort] {action} {'accepted' if ok else 'NOT accepted'}")
    return dict(action=action, accepted=ok)


MAV_LANDED_ON_GROUND = 1
MAV_LANDED_AIRBORNE = (2, 3, 4)       # IN_AIR, TAKEOFF, LANDING
ESTIMATOR_ATTITUDE_FLAG = 1
FORCE_DISARM_MAGIC = 21196            # MAVLink's "yes, I mean it" for disarming in flight
EMERGENCY_WORDS = ("ABORT", "LAND", "HOLD", "DISARM", "KILL")

# PX4 main modes a human is flying by hand: MANUAL, ALTCTL, POSCTL, ACRO, STABILIZED,
# RATTITUDE. Not AUTO (4) and not OFFBOARD (6), which are ours.
PILOT_MODES = (1, 2, 3, 5, 7, 8)
AUTOMATIC_MODES = (4, 6)
INNOVATION_FIELDS = ("vel_ratio", "pos_horiz_ratio", "pos_vert_ratio",
                     "mag_ratio", "hagl_ratio")


def statustext_of(msg) -> str:
    text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
    return text.strip()


def limits_from_args(args) -> safety_rules.Limits:
    lim = safety_rules.Limits()
    for attr, name in (("hold_rate", "hold_rate"), ("land_rate", "land_rate"),
                       ("battery_land_pct", "battery_land_pct"),
                       ("tilt_land_deg", "tilt_land_deg")):
        given = getattr(args, attr, None)
        if given is not None:
            setattr(lim, name, given)
    return lim


class Guard:
    """Watches one vehicle and carries out the abort ladder. See tools/safety_rules.py.

    Three ways a flight can be stopped, in rising order of how much has to still be
    working for them to help:

      1. the RC transmitter -- the authoritative one, and nothing here can match it
      2. this guard, on the companion: autonomous, needs only the companion-to-aircraft link
      3. the operator on the ground: a word on stdin, or the out-of-band ABORT flag file

    Non-blocking by design. carry_out() sends a command and returns; the caller's normal
    telemetry loop keeps running, and the landing is finished across later check() calls.
    A guard that blocked would stop feeding itself the telemetry it is judging.
    """

    def __init__(self, conn, args):
        self.conn = conn
        self.slot = getattr(args, "slot", None)
        self.enabled = not getattr(args, "no_safety", False)
        self.wd = safety_rules.Watchdog(limits_from_args(args), now=time.time())
        self.flag = Path(getattr(args, "abort_flag", "ABORT") or "ABORT")
        self.allow_kill = getattr(args, "allow_kill", False)
        self.armed: bool | None = None
        self.airborne: bool | None = None
        self._seen_landed_state = False
        self.stop_streaming = False          # the offboard streamer checks this
        self.acted: str | None = None
        self.landing_since: float | None = None
        self._last_disarm = 0.0
        self.killed = False
        self.pilot_has_it = False       # the RC switch has been thrown: hands off
        self._prev_mode: int | None = None
        # A flag that predates the run is an abort nobody cleared. Refuse to fly rather
        # than guess which it was, and say which case this is -- the operator needs to
        # know whether the abort happened just now or last time.
        self._flag_at_start = self.flag.exists()

    # ---- telemetry in -------------------------------------------------------------

    def observe(self, msg) -> None:
        now, kind = time.time(), msg.get_type()
        if kind == "HEARTBEAT":
            if msg.get_srcSystem() == self.conn.target_system \
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                self.wd.heartbeat(now)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self._note_mode(vehicle_main_mode(msg))
        elif kind == "STATUSTEXT":
            self.wd.statustext(msg.severity, statustext_of(msg), now)
        elif kind == "BATTERY_STATUS":
            self.wd.battery(getattr(msg, "battery_remaining", None), now)
        elif kind == "ATTITUDE":
            self.wd.attitude(math.degrees(msg.roll), math.degrees(msg.pitch), now)
        elif kind == "EXTENDED_SYS_STATE":
            if msg.landed_state == MAV_LANDED_ON_GROUND:
                self.airborne, self._seen_landed_state = False, True
            elif msg.landed_state in MAV_LANDED_AIRBORNE:
                self.airborne, self._seen_landed_state = True, True
        elif kind == "GLOBAL_POSITION_INT" and not self._seen_landed_state:
            self.airborne = (msg.relative_alt / 1000.0) > 1.0
        elif kind == "ESTIMATOR_STATUS":
            worst = max((getattr(msg, f, 0.0) or 0.0) for f in INNOVATION_FIELDS)
            self.wd.estimator(bool(msg.flags & ESTIMATOR_ATTITUDE_FLAG), worst, now)

    def _note_mode(self, mode: int) -> None:
        """Spot the pilot taking the aircraft, and get out of the way for good.

        The signal is the TRANSITION out of one of our modes into a hand-flown one -- not
        the mode itself. An aircraft sitting in POSCTL before we ask for OFFBOARD has not
        been taken over; one that was in OFFBOARD a moment ago and is now in POSCTL has.
        """
        prev, self._prev_mode = self._prev_mode, mode
        if self.pilot_has_it or prev is None:
            return
        if prev in AUTOMATIC_MODES and mode in PILOT_MODES:
            self.pilot_has_it = True
            self.stop_streaming = True      # stop arguing with the sticks
            log(f"[safety] PILOT HAS IT: main mode {prev} -> {mode}. "
                "Standing down -- no setpoints, no mode changes, no disarm while airborne.")
            emit(dict(event="rc_takeover", slot=self.slot, from_mode=prev, to_mode=mode,
                      airborne=self.airborne, t_epoch=round(time.time(), 3)))

    def operator(self, word: str) -> bool:
        """A command from the ground. Returns False if it was not understood."""
        word = word.strip().upper()
        if word in ("ABORT", "LAND"):
            self.wd.manual(safety_rules.LAND, f"operator said {word}")
        elif word == "HOLD":
            self.wd.manual(safety_rules.HOLD, "operator said HOLD")
        elif word == "DISARM":
            self.wd.manual(safety_rules.DISARM, "operator said DISARM")
        elif word == "KILL":
            if not self.allow_kill:
                log("[safety] KILL refused: the agent was not started with --allow-kill")
                return False
            self.kill("operator said KILL")
        else:
            return False
        return True

    # ---- the verdict, and acting on it --------------------------------------------

    def check(self) -> dict | None:
        """Evaluate, act if the ladder has climbed, and nurse a landing to its end."""
        if not self.enabled or self.pilot_has_it:
            # A human is flying it. Two aborts fighting each other is worse than either.
            return None
        if self.flag.exists():
            want = ""
            try:
                want = self.flag.read_text().strip().lower()
            except OSError:
                pass
            action = want if want in safety_rules.RUNG else safety_rules.LAND
            how = " left over from before this run" if self._flag_at_start else ""
            self.wd.manual(action, f"ABORT flag {self.flag}{how}"
                                   + (f" ({want})" if want else ""))

        v = self.wd.evaluate(time.time())
        if v.climbed:
            log(f"[safety] {v.action.upper()}: " + "; ".join(v.reasons[-2:])
                + f" (error rate {v.error_rate:.1f}/min)")
        self._finish_landing()
        if safety_rules.RUNG[v.action] >= safety_rules.RUNG[safety_rules.HOLD] \
                and v.action != self.acted:
            return self.carry_out(v.action, v.reasons)
        return None

    def carry_out(self, action: str, reasons: list[str]) -> dict:
        real = safety_rules.on_ground_action(action, self.airborne)
        where = {True: "airborne", False: "on the ground", None: "position unknown"}[self.airborne]
        log(f"[safety] acting: {action.upper()} -> {real.upper()} ({where})")
        self.acted = action
        if real == safety_rules.HOLD:
            # AUTO.LOITER: main mode 4, sub mode 3. Stops the mission, holds position.
            self.conn.mav.command_long_send(self.conn.target_system, 1,
                                            mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                            1, 4, 3, 0, 0, 0, 0)
        elif real == safety_rules.LAND:
            self.stop_streaming = True      # offboard setpoints would fight AUTO.LAND
            self.conn.mav.command_long_send(self.conn.target_system, 1,
                                            mavutil.mavlink.MAV_CMD_NAV_LAND,
                                            0, 0, 0, 0, 0, 0, 0, 0)
            self.landing_since = time.time()
        elif real == safety_rules.DISARM:
            self.stop_streaming = True
            self.landing_since = time.time()
            self._send_disarm()
        ev = dict(event="safety", slot=self.slot, action=action, carried_out=real,
                  airborne=self.airborne, reasons=reasons,
                  error_rate=round(self.wd.error_rate(time.time()), 2),
                  t_epoch=round(time.time(), 3))
        emit(ev)
        return ev

    def _send_disarm(self) -> None:
        self.conn.mav.command_long_send(self.conn.target_system, 1,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 0, 0, 0, 0, 0, 0, 0)
        self._last_disarm = time.time()

    def _finish_landing(self) -> None:
        """Once it is down, cut the motors. Never while it is still flying."""
        if self.landing_since is None or self.armed is False:
            return
        if self.airborne is not False:
            return
        if time.time() - self._last_disarm > 3.0:
            log("[safety] on the ground -- disarming")
            self._send_disarm()

    def kill(self, reason: str) -> dict:
        """Force-disarm, in flight if need be. This drops the aircraft. Last resort."""
        log(f"[safety] KILL ({reason}) -- force disarm, the aircraft will fall")
        self.stop_streaming = self.killed = True
        self.conn.mav.command_long_send(self.conn.target_system, 1,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                        0, 0, FORCE_DISARM_MAGIC, 0, 0, 0, 0, 0)
        ev = dict(event="safety", slot=self.slot, action="kill", carried_out="kill",
                  reasons=[reason], t_epoch=round(time.time(), 3))
        emit(ev)
        return ev


def cmd_start(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    time.sleep(2.0)   # one-shot mode: PX4 must see a few GCS heartbeats before arming
    r = do_start(conn, args.at)
    emit(dict(verb="start", slot=args.slot, sysid=conn.target_system, **r))
    return 0 if r.get("started") else 1


def cmd_status(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    emit(dict(verb="status", slot=args.slot, sysid=conn.target_system, **do_status(conn)))
    return 0


def stream_level_zero_thrust(conn) -> None:
    """One SET_ATTITUDE_TARGET: level, zero thrust, rates ignored. Offboard needs >2 Hz."""
    kw = dict(thrust_body=[0.0, 0.0, 0.0]) if _ATT_HAS_BODY_THRUST else {}
    conn.mav.set_attitude_target_send(int(time.monotonic() * 1000) & 0xFFFFFFFF,
                                      conn.target_system, 1, 0b00000111,
                                      [1.0, 0.0, 0.0, 0.0], 0.0, 0.0, 0.0, 0.0, **kw)


_ATT_HAS_BODY_THRUST = "thrust_body" in mavutil.mavlink.MAVLink_set_attitude_target_message.fieldnames


def vehicle_main_mode(hb) -> int:
    return (hb.custom_mode >> 16) & 0xFF


def cmd_offboard_arm(args) -> int:
    """Bench test, no GPS needed: stream level/zero-thrust, enter OFFBOARD, arm, hold, disarm.

    Props must be off. Always ends with a disarm attempt, whatever happened."""
    OFFBOARD = 6
    conn = connect(args.link, args.expect_sysid)
    result = dict(verb="offboard-arm", slot=args.slot, sysid=conn.target_system,
                  offboard=False, armed=False, held_s=0.0, disarmed=False)
    armed_now = vehicle_armed(conn)
    if armed_now is not False:
        result["error"] = "refusing: vehicle is ARMED or silent"
        emit(result)
        return 2
    wo_gps = read_param(conn, "COM_ARM_WO_GPS")
    result["com_arm_wo_gps"] = wo_gps
    log(f"[offboard] COM_ARM_WO_GPS={wo_gps}")

    state = dict(armed=False, mode=None)
    stdin = dict(pending=[], eof=False)
    guard = Guard(conn, args)

    def pump(seconds: float, until=None) -> bool:
        end = time.time() + seconds
        while time.time() < end:
            if not guard.stop_streaming:
                stream_level_zero_thrust(conn)
            msg = conn.recv_match(blocking=True, timeout=0.05)
            while msg is not None:
                kind = msg.get_type()
                if kind == "HEARTBEAT" and msg.get_srcSystem() == conn.target_system \
                        and msg.type != mavutil.mavlink.MAV_TYPE_GCS:
                    state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    state["mode"] = vehicle_main_mode(msg)
                elif kind == "STATUSTEXT":
                    text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="replace")
                    log(f"[vehicle] {text.strip()}")
                    result.setdefault("vehicle_text", []).append(text.strip())
                elif kind == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    result["arm_result"] = msg.result
                    log(f"[vehicle] arm/disarm ack result {msg.result}")
                guard.observe(msg)
                msg = conn.recv_match(blocking=False)
            # The operator can speak at any time, not only at the arming gate. Words the
            # guard does not own (the gate's own ARM) are queued for whoever is waiting,
            # never swallowed here.
            while select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line:
                    stdin["eof"] = True
                    break
                word = line.strip().upper()
                if not word:
                    continue
                if word in EMERGENCY_WORDS:
                    guard.operator(word)
                    result.setdefault("operator", []).append(word)
                else:
                    stdin["pending"].append(word)
            acted = guard.check()
            if acted:
                result.setdefault("safety", []).append(acted)
                result["error"] = "safety abort: " + "; ".join(acted["reasons"][-2:])
            if until and until():
                return True
            time.sleep(0.05)
        return until is None

    try:
        log("[offboard] streaming setpoints")
        pump(2.0)
        log("[offboard] requesting OFFBOARD")
        last = 0.0
        deadline = time.time() + 10
        while time.time() < deadline and state["mode"] != OFFBOARD and not guard.pilot_has_it:
            if time.time() - last > 2.0:
                conn.mav.command_long_send(conn.target_system, 1,
                                           mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                           1, OFFBOARD, 0, 0, 0, 0, 0)
                last = time.time()
            pump(0.5, lambda: state["mode"] == OFFBOARD)
        result["offboard"] = state["mode"] == OFFBOARD
        if not result["offboard"]:
            result["error"] = f"vehicle did not enter OFFBOARD (main mode {state['mode']})"
            return 2
        if args.gate:
            emit(dict(event="ready", slot=args.slot, sysid=conn.target_system))
            log("[offboard] READY -- waiting for workstation authorisation (ARM)")
            end, verdict = time.time() + args.gate_timeout_s, None
            while time.time() < end and verdict is None:
                pump(0.3)
                if stdin["pending"]:
                    verdict = stdin["pending"].pop(0)
                elif stdin["eof"]:
                    verdict = "EOF"
                elif guard.acted or guard.killed:
                    verdict = "ABORTED"
            if verdict != "ARM":
                result["error"] = f"not authorised to arm ({verdict or 'timeout'})"
                return 2
            if guard.acted or guard.killed:
                result["error"] = "refusing to arm: the safety guard has already aborted"
                return 2
            log("[offboard] authorised")
        log("[offboard] in OFFBOARD; arming")
        last = 0.0
        deadline = time.time() + 20
        while time.time() < deadline and not state["armed"]:
            if time.time() - last > 3.0:
                conn.mav.command_long_send(conn.target_system, 1,
                                           mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 1, 0, 0, 0, 0, 0, 0)
                last = time.time()
            pump(0.5, lambda: state["armed"])
        result["armed"] = state["armed"]
        if not state["armed"]:
            result["error"] = "vehicle did not arm"
            return 2
        t0 = time.time()
        result["armed_epoch"] = round(t0, 3)
        # Emitted the instant arming is observed, so the workstation can stamp it against
        # its own clock. Companion clocks are not trustworthy: a Jetson with no usable
        # default route never syncs NTP (jetson-1 sat at 1970-01-01 on 6 Oct 2026), which
        # makes any spread computed from armed_epoch meaningless.
        emit(dict(event="armed", slot=args.slot, sysid=conn.target_system,
                  armed_epoch=result["armed_epoch"]))
        log(f"[offboard] ARMED; holding {args.hold_s:.0f} s")
        while time.time() - t0 < args.hold_s:
            pump(0.5)
            if guard.acted or guard.killed:
                break
            if not state["armed"]:
                result["error"] = "vehicle disarmed itself during hold"
                break
            if state["mode"] != OFFBOARD:
                result["error"] = ("pilot took over with the RC switch"
                                   if guard.pilot_has_it else
                                   f"left OFFBOARD during hold (main mode {state['mode']})")
                break
        result["held_s"] = round(time.time() - t0, 1)
        return 0 if "error" not in result else 2
    finally:
        result["rc_takeover"] = guard.pilot_has_it
        if guard.pilot_has_it and guard.airborne is not False:
            log("[offboard] NOT disarming: the pilot has the aircraft and it is airborne")
            result["disarmed"] = False
            emit(result)
            return 2 if "error" in result else 0
        log("[offboard] disarming")
        for _ in range(6):
            conn.mav.command_long_send(conn.target_system, 1,
                                       mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 0, 0, 0, 0, 0, 0, 0)
            pump(2.0, lambda: not state["armed"])
            if not state["armed"]:
                break
        result["disarmed"] = not state["armed"]
        emit(result)


# COM_FLTMODE<n> values, from PX4 1.16's own parameter metadata
# (build/px4_sitl_default/parameters.json), not from memory.
FLTMODE_NAMES = {-1: "Unassigned", 0: "Manual", 1: "Altitude", 2: "Position", 3: "Mission",
                 4: "Hold", 5: "Return", 6: "Acro", 7: "Offboard", 8: "Stabilized",
                 9: "Position Slow", 10: "Takeoff", 11: "Land", 12: "Follow Me",
                 13: "Precision Land"}
# The slots that hand the aircraft to a human. Hold/Mission/Return are still autopilot.
PILOT_FLTMODES = (0, 1, 2, 6, 8, 9)
RC_PARAMS = ("COM_RC_IN_MODE", "COM_RC_OVERRIDE", "COM_RC_STICK_OV", "RC_MAP_FLTMODE",
             "RC_MAP_KILL_SW", "COM_RCL_EXCEPT", "COM_FLTMODE1", "COM_FLTMODE2",
             "COM_FLTMODE3", "COM_FLTMODE4", "COM_FLTMODE5", "COM_FLTMODE6")


def rc_takeover_findings(p: dict, rc_seen: bool) -> list[str]:
    """What STOPS the RC switch from taking this aircraft. Empty = nothing does.

    Only blocking faults belong here. Policy choices live in rc_takeover_notes() -- see
    the note there about COM_RC_OVERRIDE, which is deliberately left at PX4's default on
    this project. Pure, so tests/test_rc_takeover.py can pin it without an aircraft.
    """
    out = []
    # 0 RC only, 2 RC and joystick, 3 whichever is seen first (so RC works if RC is first --
    # and the QGC mirror is observe-only, so no joystick can get in ahead of it).
    # 1 is joystick only and 4 disables stick input entirely.
    in_mode = p.get("COM_RC_IN_MODE")
    if in_mode not in (0, 2, 3):
        out.append(f"COM_RC_IN_MODE={in_mode}: this aircraft does not accept RC stick input "
                   f"(need 0 'RC only', 2 'RC and joystick', or 3 'keep first'; "
                   f"1 is joystick only and 4 is disabled)")

    if not p.get("RC_MAP_FLTMODE"):
        out.append("RC_MAP_FLTMODE=0: no channel carries the flight-mode switch, so no "
                   "switch position can request a mode")
    slots = {n: p.get(f"COM_FLTMODE{n}") for n in range(1, 7)}
    if not any(v is not None and int(v) in PILOT_FLTMODES for v in slots.values()):
        out.append("no COM_FLTMODE1..6 slot holds a hand-flown mode (Manual/Altitude/"
                   "Position/Acro/Stabilized), so no switch position hands over control")
    if not rc_seen:
        out.append("no RC_CHANNELS heard: the transmitter is off, unbound, or its link to "
                   "this aircraft is down. Nothing above can work until it is heard")
    return out


def rc_takeover_notes(p: dict) -> list[str]:
    """True but not blocking: what the current configuration does and does not cover.

    COM_RC_OVERRIDE is the big one. Bit 1 enables stick override during OFFBOARD, and this
    project deliberately leaves it CLEAR (6 Oct 2026) so a knocked stick cannot drop an
    aircraft out of offboard mid-run. The consequence is that the mode switch is the only
    way to take over, which is the intent -- a deliberate act, not an accidental one.
    """
    out = []
    ov = p.get("COM_RC_OVERRIDE")
    if ov is None:
        out.append("COM_RC_OVERRIDE could not be read")
    else:
        ov = int(ov)
        if not ov & 2:
            out.append(f"COM_RC_OVERRIDE={ov}: stick movement will NOT break OFFBOARD "
                       f"(deliberate here -- the mode switch is the takeover path). "
                       f"Set 3 only if you want the sticks to override too")
        if not ov & 1:
            out.append(f"COM_RC_OVERRIDE={ov}: stick movement will not break AUTO either")
    if not p.get("RC_MAP_KILL_SW"):
        out.append("RC_MAP_KILL_SW=0: no kill switch mapped -- the only override that does "
                   "not need the autopilot to cooperate")
    return out


def cmd_rc_check(args) -> int:
    """Read-only: can the RC switch take this aircraft away from us?"""
    conn = connect(args.link, args.expect_sysid)
    params = {}
    for name in RC_PARAMS:
        params[name] = read_param(conn, name)
    rc_seen, deadline = False, time.time() + 6.0
    rssi = None
    while time.time() < deadline and not rc_seen:
        m = conn.recv_match(type="RC_CHANNELS", blocking=True, timeout=1)
        if m is not None:
            rc_seen, rssi = True, getattr(m, "rssi", None)
    slots = {n: params.get(f"COM_FLTMODE{n}") for n in range(1, 7)}
    findings = rc_takeover_findings(params, rc_seen)
    notes = rc_takeover_notes(params)
    for f in findings:
        log(f"[rc] {f}")
    for n in notes:
        log(f"[rc] note: {n}")
    if not findings:
        log("[rc] the RC switch can take this aircraft")
    emit(dict(verb="rc-check", slot=args.slot, sysid=conn.target_system,
              rc_seen=rc_seen, rssi=rssi, params=params,
              mode_slots={n: FLTMODE_NAMES.get(int(v), v) for n, v in slots.items()
                          if v is not None},
              findings=findings, notes=notes, ready=not findings))
    return 0 if not findings else 1


def cmd_abort(args) -> int:
    conn = connect(args.link, args.expect_sysid)
    emit(dict(verb="abort", slot=args.slot, sysid=conn.target_system,
              **do_abort(conn, args.action)))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("verb", choices=["prepare", "upload", "start", "status", "abort",
                                        "offboard-arm", "rc-check"])
    ap.add_argument("--hold-s", type=float, default=10.0, help="offboard-arm: seconds to stay armed")
    ap.add_argument("--gate", action="store_true",
                    help="offboard-arm: enter OFFBOARD, print a ready event, arm only on 'ARM' from stdin")
    ap.add_argument("--gate-timeout-s", type=float, default=90.0)
    ap.add_argument("--link", required=True,
                    help="udpin://:14540, serial:///dev/ttyACM0, udpout://host:port")
    ap.add_argument("--expect-sysid", type=int, default=None,
                    help="refuse the vehicle if it answers as anything else")
    ap.add_argument("--slot", type=int, default=None, help="which drone-N this is")
    ap.add_argument("--plan")
    ap.add_argument("--params")
    ap.add_argument("--at", type=float, default=None,
                    help="epoch seconds: hold, then arm and start (shared clock launch)")
    ap.add_argument("--set", action="append", metavar="NAME=VALUE",
                    help="per-drone parameter override, repeatable")
    ap.add_argument("--resident", action="store_true",
                    help="after prepare, hold the link and take commands on stdin")
    ap.add_argument("--run-id", default="run", help="run id, part of the token")
    ap.add_argument("--failsafe-s", type=float, default=60.0,
                    help="once flying, RTL if the orchestrator goes silent this long "
                         "(0 disables)")
    ap.add_argument("--action", choices=["rtl", "land"], default="rtl")
    ap.add_argument("--no-safety", action="store_true",
                    help="run without the abort ladder (bench diagnosis only)")
    ap.add_argument("--abort-flag", default="ABORT",
                    help="path polled for an out-of-band abort; its contents may name the "
                         "rung (hold/land/disarm). tools/takeover.py writes this over ssh")
    ap.add_argument("--allow-kill", action="store_true",
                    help="permit KILL: a force-disarm that drops the aircraft out of the air")
    ap.add_argument("--hold-rate", type=float, default=None,
                    help="weighted vehicle errors per minute that stop the mission")
    ap.add_argument("--land-rate", type=float, default=None,
                    help="weighted vehicle errors per minute that force a landing")
    ap.add_argument("--battery-land-pct", type=float, default=None)
    ap.add_argument("--tilt-land-deg", type=float, default=None)
    args = ap.parse_args()

    if args.verb in ("prepare", "upload") and not args.plan:
        ap.error(f"{args.verb} needs --plan")

    try:
        return {"prepare": cmd_prepare, "upload": cmd_upload, "start": cmd_start,
                "status": cmd_status, "abort": cmd_abort,
                "offboard-arm": cmd_offboard_arm,
                "rc-check": cmd_rc_check}[args.verb](args)
    except Exception as exc:
        log(f"[error] {type(exc).__name__}: {exc}")
        emit(dict(verb=args.verb, slot=args.slot, ready=False, error=str(exc)))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
