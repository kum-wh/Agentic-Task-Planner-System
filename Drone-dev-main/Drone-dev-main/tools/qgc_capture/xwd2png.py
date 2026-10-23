"""Minimal XWD -> PNG. No ImageMagick on this box, so parse the format directly."""
import struct, sys
from PIL import Image

raw = open(sys.argv[1], "rb").read()
f = struct.unpack(">25I", raw[:100])
(hdr, ver, fmt, depth, w, h, xoff, border, bunit, bbo, bpad, bpp, bpl,
 vclass, rmask, gmask, bmask, bprgb, cmap_ent, ncolors) = f[:20]
off = hdr + ncolors * 12
px = raw[off:]
print(f"  {w}x{h} depth={depth} bpp={bpp} bytes_per_line={bpl} ncolors={ncolors} masks={rmask:#x}/{gmask:#x}/{bmask:#x}")

def shift(m):
    s = 0
    while m and not (m & 1): m >>= 1; s += 1
    return s
rs, gs, bs = shift(rmask), shift(gmask), shift(bmask)
img = Image.new("RGB", (w, h))
out = img.load()
step = bpp // 8
for y in range(h):
    row = px[y * bpl:(y + 1) * bpl]
    for x in range(w):
        c = row[x * step:x * step + step]
        if len(c) < step: break
        v = int.from_bytes(c, "big" if bunit and False else "little")
        out[x, y] = ((v & rmask) >> rs, (v & gmask) >> gs, (v & bmask) >> bs)
img.save(sys.argv[2])
print("  wrote", sys.argv[2])
