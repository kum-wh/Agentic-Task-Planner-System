"""Frame QGC's Fly view for a .plan.

With a vehicle connected QGC forces the map centre onto the vehicle, so centring is
not ours to choose - but the zoom IS restored from the ini. So centre on home and
pick a zoom wide enough that the farthest waypoint still fits in the half-window.
"""
import json, math, re, sys
from pathlib import Path

plan = json.loads(Path(sys.argv[1]).read_text())
mi = plan["mission"]
home = mi["plannedHomePosition"]
hlat, hlon = home[0], home[1]
pts = [(i["params"][4], i["params"][5]) for i in mi["items"]
       if i.get("params") and isinstance(i["params"][4], float) and isinstance(i["params"][5], float)]

def metres(lat, lon):
    return ((lon-hlon)*111320*math.cos(math.radians(hlat)), (lat-hlat)*110574)

radius = max((math.hypot(*metres(la, lo)) for la, lo in pts), default=100.0)
R = 156543.03392 * math.cos(math.radians(hlat))
HALF_PX = 400                      # half the usable window height, minus chrome
z = math.log2(HALF_PX * R / radius)
z = max(10.0, min(20.0, z))

ini = Path.home()/".config/QGroundControl.org/QGroundControl.ini"
txt = ini.read_text()
txt = re.sub(r"FlightMapZoom=[\d.]+", f"FlightMapZoom={z:.6f}", txt)
txt = re.sub(r"Latitude=[-\d.]+",     f"Latitude={hlat:.8f}", txt)
txt = re.sub(r"Longitude=[-\d.]+",    f"Longitude={hlon:.8f}", txt)
ini.write_text(txt)
print(f"  home {hlat:.6f},{hlon:.6f}  farthest wp {radius:.0f} m  zoom {z:.2f}  ({R/2**z:.2f} m/px)")
