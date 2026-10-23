"""Click inside a window using WINDOW coordinates (same frame as the screenshots).

The window's absolute origin is resolved at run time - QGC restores its saved
MainWindowState position, so a hardcoded offset silently sends every click to the
wrong place.
"""
import sys, time
from Xlib import display, X
from Xlib.ext import xtest

d = display.Display(":1")
root = d.screen().root
wid = int(sys.argv[1], 16)
w = d.create_resource_object("window", wid)

t = w.translate_coords(root, 0, 0)
ox, oy = -t.x, -t.y
print(f"  window {hex(wid)} origin at root ({ox},{oy})")

w.configure(stack_mode=X.Above)
w.set_input_focus(X.RevertToParent, X.CurrentTime)
d.sync(); time.sleep(1.0)

def click(wx, wy):
    x, y = ox + wx, oy + wy
    xtest.fake_input(d, X.MotionNotify, x=x, y=y); d.sync(); time.sleep(.35)
    xtest.fake_input(d, X.ButtonPress, 1); d.sync(); time.sleep(.12)
    xtest.fake_input(d, X.ButtonRelease, 1); d.sync(); time.sleep(.9)

for arg in sys.argv[2:]:
    p = arg.split(",")
    if p[0] == "click":
        click(int(p[1]), int(p[2])); print(f"  click win({p[1]},{p[2]})")
    elif p[0] == "rep":
        for _ in range(int(p[3])): click(int(p[1]), int(p[2]))
        print(f"  click win({p[1]},{p[2]}) x{p[3]}")
d.sync()
