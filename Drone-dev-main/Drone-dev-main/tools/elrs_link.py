#!/usr/bin/env python3
"""Straddle the ELRS backpack's asymmetric UDP ports so the agent sees a normal link.

    python3 tools/elrs_link.py --agent-port 14540
    python3 tools/elrs_link.py --stats            # one-shot, prints module counters

WHY THIS EXISTS. The RadioMaster Pocket's TX backpack (firmware 1.5.5) bridges MAVLink
over UDP, but not symmetrically -- measured 28 Sep 2026 from jetson-1:

    GET http://10.0.0.1/mavlink
    {"enabled":true,"ports":{"listen":14555,"send":14550},"ip":{"gcs":"IP UNSET"},...}

It accepts GCS traffic on 14555 and emits vehicle traffic to 14550, and it learns which
host to emit to from the first packet that arrives on 14555 -- until then `gcs` reads
"IP UNSET" and nothing comes back however long you listen. pymavlink has no connection
string for "bind one port, send to another": `udpin` binds and can only reply to a peer
it has already heard from (so it can never speak first, which a mission upload must),
and `udpout` sends from an ephemeral port the module will never target. This relay owns
one socket bound to 14550 that sends to 10.0.0.1:14555, and presents the agent an
ordinary endpoint on localhost, so drone_agent.py stays unaware of any of it.

Uplink is proven: 100 MISSION_ITEM_INT/s (4.9 kB/s) accepted with zero loss, 5.8% loss at
200/s. Downlink is NOT yet proven -- it needs a powered aircraft with a bound receiver,
and the RF hop is far narrower than the WiFi hop measured here.
"""

from __future__ import annotations

import argparse
import json
import select
import socket
import sys
import time
import urllib.request

MODULE_IP = "10.0.0.1"
MODULE_LISTEN = 14555      # module's ear  -- we send here
MODULE_SEND = 14550        # module's mouth -- we bind here


def counters(ip: str = MODULE_IP) -> dict:
    try:
        with urllib.request.urlopen(f"http://{ip}/mavlink", timeout=5) as r:
            return json.load(r)
    except Exception as exc:
        return {"error": str(exc)}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--module-ip", default=MODULE_IP)
    ap.add_argument("--module-listen", type=int, default=MODULE_LISTEN)
    ap.add_argument("--module-send", type=int, default=MODULE_SEND)
    ap.add_argument("--agent-port", type=int, default=14540,
                    help="localhost port the agent connects to with udpout://")
    ap.add_argument("--stats", action="store_true", help="print module counters and exit")
    ap.add_argument("--report-s", type=float, default=10.0)
    ap.add_argument("--mirror", action="append", metavar="HOST:PORT", default=[],
                    help="send a copy of every downlink packet here. For QGroundControl: "
                         "--mirror 127.0.0.1:14551. Repeatable. Observe-only unless "
                         "--mirror-control is given.")
    ap.add_argument("--mirror-control", action="store_true",
                    help="ALSO forward what the mirror sends to the aircraft, making it a "
                         "full ground station. It can then arm, disarm and change modes "
                         "alongside the agent -- two commanders on one airframe. Without "
                         "this, QGC's mission/geofence/rally/parameter requests all fail, "
                         "because every one of them needs a packet to reach the vehicle.")
    ap.add_argument("--agent-ttl-s", type=float, default=30.0,
                    help="how long an agent address keeps receiving downlink after its "
                         "last packet")
    args = ap.parse_args()

    if args.stats:
        print(json.dumps(counters(args.module_ip), indent=2))
        return 0

    radio = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    radio.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    radio.bind(("0.0.0.0", args.module_send))

    agent = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    agent.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    agent.bind(("127.0.0.1", args.agent_port))

    mirrors = []
    for spec in args.mirror:
        host, _, port = spec.rpartition(":")
        if not host or not port.isdigit():
            print(f"[elrs] bad --mirror {spec!r}, expected HOST:PORT", flush=True)
            return 2
        mirrors.append((host, int(port)))

    # Its own socket. By default we never act on what arrives here: QGroundControl talks
    # back the moment it sees a vehicle, and forwarding that puts a second commander on the
    # aircraft alongside the agent. --mirror-control opts into exactly that, deliberately,
    # because a QGC that cannot send is a QGC whose every request fails -- mission,
    # geofence, rally points and parameters are all request/response.
    mirror_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if mirrors else None

    module = (args.module_ip, args.module_listen)
    agents: dict[tuple[str, int], float] = {}
    up = down = dropped = unreachable = 0
    mirror_up = mirror_blocked = 0
    last = time.time()

    print(f"[elrs] agent udp/{args.agent_port} <-> module {module[0]}:{module[1]} "
          f"(downlink on {args.module_send})", flush=True)
    if mirrors:
        where = ", ".join(f"{h}:{p}" for h, p in mirrors)
        if args.mirror_control:
            print(f"[elrs] mirroring downlink to {where} -- TWO-WAY: that station can "
                  f"command this aircraft, alongside the agent", flush=True)
        else:
            print(f"[elrs] mirroring downlink to {where} (observe only, nothing it sends "
                  f"is forwarded)", flush=True)

    while True:
        watch = [radio, agent] + ([mirror_sock] if mirror_sock else [])
        ready, _, _ = select.select(watch, [], [], 1.0)
        for s in ready:
            try:
                data, addr = s.recvfrom(4096)
            except OSError:
                continue
            if s is mirror_sock:
                if not args.mirror_control:
                    mirror_blocked += 1    # drained and discarded on purpose -- see above
                    continue
                try:
                    radio.sendto(data, module)
                except OSError:
                    unreachable += 1
                else:
                    mirror_up += 1
                continue
            if s is agent:
                agents[addr] = time.time()
                try:
                    radio.sendto(data, module)
                except OSError as exc:
                    # The backpack is not associated yet (ENETUNREACH). Staying alive and
                    # counting the loss beats dying: the relay is often started before the
                    # transmitter is powered, and it recovers by itself once WiFi joins.
                    unreachable += 1
                    if unreachable == 1:
                        print(f"[elrs] uplink unreachable ({exc}) -- waiting for the backpack",
                              flush=True)
                else:
                    up += 1
            else:
                down += 1
                # Every agent heard from recently, not just the last one. One relay used to
                # serve exactly one address, so a second reader silently stole the replies.
                cutoff = time.time() - args.agent_ttl_s
                live = [a for a, seen in agents.items() if seen >= cutoff]
                for stale in [a for a in agents if a not in live]:
                    del agents[stale]
                if not live:
                    dropped += 1       # vehicle spoke before any agent attached
                for a in live:
                    agent.sendto(data, a)
                for m in mirrors:
                    try:
                        mirror_sock.sendto(data, m)
                    except OSError:
                        pass           # QGC not up yet, or no route; never fatal

        now = time.time()
        if now - last >= args.report_s:
            c = counters(args.module_ip).get("counters", {})
            print(f"[elrs] up={up} down={down} no-agent-drop={dropped} "
                  f"uplink-unreachable={unreachable} agents={len(agents)} "
                  f"mirrors={len(mirrors)} mirror-up={mirror_up} "
                  f"mirror-blocked={mirror_blocked} module={c}", flush=True)
            last = now


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n[elrs] stopped", flush=True)
