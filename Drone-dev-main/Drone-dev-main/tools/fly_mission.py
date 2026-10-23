#!/usr/bin/env python3
"""Upload a generated QGC .plan to a running PX4 (SITL or real) and fly it.

    python tools/fly_mission.py missions/ubin/mission_drones/drone-1.plan \
        --params ../sar_turn_params.params --track missions/ubin/track.json

The .plan is imported through MAVSDK's QGroundControl importer, so this exercises the
real file format rather than a hand-built item list. The flown track is recorded so it
can be compared against the plan.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import time
from pathlib import Path

from mavsdk import System


def read_params(path: Path) -> list[tuple[str, float, bool]]:
    """QGC .params -> [(name, value, is_int)]. Columns: sysid compid name value type."""
    entries = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) >= 5:
            entries.append((fields[2], float(fields[3]), fields[4] == "6"))
    return entries


async def connect(address: str, server_port: int = 50051, gcs_sysid: int = 245) -> System:
    # Each System() spawns its own mavsdk_server on `server_port`. Flying several
    # vehicles at once needs a distinct port and a distinct GCS sysid per flight,
    # or the second one fails to bind and the vehicles share a ground-station id.
    drone = System(port=server_port, sysid=gcs_sysid)
    await drone.connect(system_address=address)
    print(f"[link] waiting for vehicle on {address} ...", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[link] connected", flush=True)
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("[link] global position + home OK", flush=True)
            break
    return drone


async def run(args) -> int:
    drone = await connect(args.url, args.server_port, args.gcs_sysid)

    if args.params:
        for name, value, is_int in read_params(Path(args.params)):
            try:
                if is_int:
                    await drone.param.set_param_int(name, int(value))
                else:
                    await drone.param.set_param_float(name, value)
                print(f"[param] {name} = {value}", flush=True)
            except Exception as exc:
                print(f"[param] {name} FAILED: {exc}", flush=True)

    print(f"[plan]  importing {args.plan}", flush=True)
    data = await drone.mission_raw.import_qgroundcontrol_mission(str(Path(args.plan).resolve()))
    items = data.mission_items
    print(f"[plan]  {len(items)} mission items; commands={[i.command for i in items]}", flush=True)

    # PX4 persists the mission AND the current item index in dataman, across restarts.
    # Without clearing and resetting the index, a vehicle whose last run ended on the RTL
    # item resumes *there*: it "returns to launch" from 1 m and disarms without ever taking
    # off. Measured 24 Sep 2026 on SITL instances 1 and 2.
    await drone.mission_raw.clear_mission()
    await drone.mission_raw.upload_mission(items)
    await drone.mission_raw.set_current_mission_item(0)
    print("[plan]  uploaded to PX4 and accepted; current item reset to 0", flush=True)

    track: list[dict] = []
    stop = asyncio.Event()

    async def record():
        async for position in drone.telemetry.position():
            track.append({
                "t": round(time.time() - start, 2),
                "lat": position.latitude_deg,
                "lon": position.longitude_deg,
                "rel_alt": round(position.relative_altitude_m, 2),
            })
            if stop.is_set():
                return

    async def progress():
        async for item in drone.mission_raw.mission_progress():
            print(f"[fly]   waypoint {item.current}/{item.total}"
                  f"  t={time.time()-start:6.1f}s  alt={track[-1]['rel_alt'] if track else 0:5.1f} m", flush=True)
            if item.total and item.current >= item.total:
                stop.set()
                return

    start = time.time()
    print("[fly]   arming", flush=True)
    await drone.action.arm()
    print("[fly]   starting mission", flush=True)
    await drone.mission_raw.start_mission()

    recorder = asyncio.ensure_future(record())
    watcher = asyncio.ensure_future(progress())
    try:
        await asyncio.wait_for(stop.wait(), timeout=args.timeout)
        print(f"[fly]   mission items complete at t={time.time()-start:.1f}s", flush=True)
    except asyncio.TimeoutError:
        print(f"[fly]   TIMEOUT after {args.timeout}s", flush=True)
    stop.set()

    # RTL is the last mission item; wait for the vehicle to land and disarm.
    deadline = time.time() + args.land_timeout
    while time.time() < deadline:
        armed = await anext_value(drone.telemetry.armed())
        if not armed:
            print(f"[fly]   disarmed (landed) at t={time.time()-start:.1f}s", flush=True)
            break
        await asyncio.sleep(2)
    else:
        print("[fly]   still armed when land timeout expired", flush=True)

    for task in (recorder, watcher):
        task.cancel()

    if args.track:
        Path(args.track).write_text(json.dumps(track, indent=1))
        print(f"[fly]   wrote {len(track)} track samples -> {args.track}", flush=True)
    return 0


async def anext_value(generator):
    async for value in generator:
        return value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("plan")
    parser.add_argument("--url", default="udpin://0.0.0.0:14540")
    parser.add_argument("--server-port", type=int, default=50051,
                        help="gRPC port for this flight's mavsdk_server; must differ per concurrent vehicle")
    parser.add_argument("--gcs-sysid", type=int, default=245,
                        help="MAVLink system id this ground station presents as")
    parser.add_argument("--params")
    parser.add_argument("--track")
    parser.add_argument("--timeout", type=float, default=900.0)
    parser.add_argument("--land-timeout", type=float, default=240.0)
    args = parser.parse_args()
    sys.exit(asyncio.get_event_loop().run_until_complete(run(args)))


if __name__ == "__main__":
    main()
