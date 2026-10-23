"""Shared ESRI World Imagery basemap fetcher for the tools/ plotters."""
from __future__ import annotations

import io
import math
import sys
import time
import urllib.request

from PIL import Image

TILE_URL = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
TILE_SIZE = 256
DRONE_COLOURS = ["#00e5ff", "#ffd400", "#ff5ec4", "#7cff6b", "#ff8c42"]


def deg2xy(lat: float, lon: float, zoom: int) -> tuple[float, float]:
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2.0 * n
    return x, y


def fetch_basemap(min_lat, min_lon, max_lat, max_lon, zoom):
    """Stitch the tiles covering the bounds; returns (image, extent) or (None, None)."""
    x0, y1 = deg2xy(min_lat, min_lon, zoom)
    x1, y0 = deg2xy(max_lat, max_lon, zoom)
    tx0, tx1 = int(math.floor(x0)), int(math.floor(x1))
    ty0, ty1 = int(math.floor(y0)), int(math.floor(y1))
    mosaic = Image.new("RGB", ((tx1 - tx0 + 1) * TILE_SIZE, (ty1 - ty0 + 1) * TILE_SIZE))
    for tx in range(tx0, tx1 + 1):
        for ty in range(ty0, ty1 + 1):
            url = TILE_URL.format(z=zoom, x=tx, y=ty)
            tile = None
            for attempt in range(3):                       # DNS/tile service can blip
                try:
                    request = urllib.request.Request(url, headers={"User-Agent": "drone-dev-sar-planner"})
                    tile = Image.open(io.BytesIO(urllib.request.urlopen(request, timeout=20).read()))
                    break
                except Exception as exc:
                    if attempt == 2:                       # genuinely offline / rate-limited
                        print(f"  basemap tile {zoom}/{tx}/{ty} unavailable ({exc}); "
                              "rendering overlay without imagery", file=sys.stderr)
                    else:
                        time.sleep(1.5 * (attempt + 1))
            if tile is None:
                return None, None
            mosaic.paste(tile, ((tx - tx0) * TILE_SIZE, (ty - ty0) * TILE_SIZE))

    def lon_of(x):
        return x / 2.0 ** zoom * 360.0 - 180.0

    def lat_of(y):
        return math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / 2.0 ** zoom))))

    return mosaic, (lon_of(tx0), lon_of(tx1 + 1), lat_of(ty1 + 1), lat_of(ty0))


