"""Upload a .plan to the vehicle without flying it, so QGC renders it on the map."""
import asyncio, sys
from pathlib import Path
from mavsdk import System

async def main(plan, params=None):
    d = System()
    await d.connect(system_address="udpin://0.0.0.0:14540")
    async for s in d.core.connection_state():
        if s.is_connected: break
    print("[link] connected", flush=True)
    async for h in d.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok: break
    print("[link] position OK", flush=True)
    if params:
        for line in Path(params).read_text().splitlines():
            line = line.strip()
            if not line or line.startswith("#"): continue
            f = line.split()
            if len(f) >= 5:
                try:
                    if f[4] == "6": await d.param.set_param_int(f[2], int(float(f[3])))
                    else: await d.param.set_param_float(f[2], float(f[3]))
                    print(f"[param] {f[2]} = {f[3]}", flush=True)
                except Exception as e: print(f"[param] {f[2]} FAILED: {e}", flush=True)
    data = await d.mission_raw.import_qgroundcontrol_mission(str(Path(plan).resolve()))
    items = data.mission_items
    print(f"[plan] {len(items)} items", flush=True)
    await d.mission_raw.upload_mission(items)
    print("[plan] uploaded and accepted by PX4", flush=True)

asyncio.get_event_loop().run_until_complete(main(*sys.argv[1:]))
