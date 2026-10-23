#!/usr/bin/env python3
"""Record every SITL vehicle's position on ONE wall clock.

    python tools/swarm_record.py out.json --vehicles 3 --seconds 900

Why this exists: `fly_mission.py` stamps each track from that flight's own arming
moment, so two tracks cannot be compared against each other -- 2 s of skew is 24 m at
survey speed, which is larger than the separation limits the guardrail promises. This
attaches to every instance's GCS link and stamps each sample with `time.time()`.

Output feeds `tools/swarm_separation.py`:

    {"t0_epoch": 1758...,
     "tracks": {"1": [{"t": 0.02, "lat": .., "lon": .., "rel_alt": .., "amsl": ..}, ...]}}

**This competes with QGroundControl.** PX4 SITL's GCS link announces to 127.0.0.1:14550
only until something talks to it on 18570+i, and then speaks to that partner alone --
permanently, even after the partner exits. Attaching this recorder therefore makes QGC
go blind, and the only cure is restarting the swarm. Run the recorder *or* QGC, not both,
until a MAVLink router is in the loop.

The file is written when recording ends, so let it run to completion; killing it loses
everything gathered so far.
"""

from __future__ import annotations

import argparse
import json
import threading
import time

from pymavlink import mavutil

GCS_PORT_BASE = 18570          # PX4 SITL: instance i listens on 18570+i
HEARTBEAT_INTERVAL_S = 1.0


def watch(instance: int, duration: float, t0: float, out: list) -> None:
    port = GCS_PORT_BASE + instance
    link = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}",
                                      source_system=200 + instance, source_component=190)
    # PX4 only streams once a GCS has spoken to it, and it drops us if we go quiet.
    link.mav.heartbeat_send(6, 8, 0, 0, 0)
    last_hb = time.time()
    end = t0 + duration

    while time.time() < end:
        if time.time() - last_hb > HEARTBEAT_INTERVAL_S:
            link.mav.heartbeat_send(6, 8, 0, 0, 0)
            last_hb = time.time()
        msg = link.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            continue
        out.append({
            "t": round(time.time() - t0, 3),
            "lat": msg.lat / 1e7,
            "lon": msg.lon / 1e7,
            "rel_alt": round(msg.relative_alt / 1000.0, 3),
            "amsl": round(msg.alt / 1000.0, 3),
        })


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("output")
    ap.add_argument("--vehicles", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=900.0)
    args = ap.parse_args()

    t0 = time.time()
    tracks = {i + 1: [] for i in range(args.vehicles)}
    threads = [threading.Thread(target=watch, args=(i, args.seconds, t0, tracks[i + 1]),
                                daemon=True)
               for i in range(args.vehicles)]
    print(f"recording {args.vehicles} vehicles for {args.seconds:.0f} s "
          f"(ports {GCS_PORT_BASE}..{GCS_PORT_BASE + args.vehicles - 1}) -- "
          f"QGroundControl will not see the swarm while this runs", flush=True)
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    with open(args.output, "w") as fh:
        json.dump({"t0_epoch": t0, "tracks": tracks}, fh, indent=1)
    print(f"wrote {args.output}: "
          + ", ".join(f"drone-{k} {len(v)} samples" for k, v in tracks.items()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
