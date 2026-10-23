"""Scroll-zoom a window out N steps. Window origin is resolved at run time:
QGC restores its saved MainWindowState position, so a fixed offset misses."""
import sys, time
from Xlib import display, X
from Xlib.ext import xtest
d = display.Display(":1"); root = d.screen().root
w = d.create_resource_object("window", int(sys.argv[1], 16))
n = int(sys.argv[2])
t = w.translate_coords(root, 0, 0); ox, oy = -t.x, -t.y
w.configure(stack_mode=X.Above); w.set_input_focus(X.RevertToParent, X.CurrentTime)
d.sync(); time.sleep(1.0)
xtest.fake_input(d, X.MotionNotify, x=ox+910, y=oy+500); d.sync(); time.sleep(.4)
for _ in range(n):
    xtest.fake_input(d, X.ButtonPress, 5); d.sync(); time.sleep(.1)
    xtest.fake_input(d, X.ButtonRelease, 5); d.sync(); time.sleep(.65)
print(f"  zoomed out {n} steps (origin {ox},{oy})")
