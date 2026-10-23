# tools/qgc_capture — screenshot a `.plan` in QGroundControl

Built 2026-09-10 while capturing the Toa Payoh urban plans. Driving QGC from a script
is more awkward than it looks; the gotchas below each cost real time, so read them
before changing anything here.

## Use

    # from ~/drone-dev/FlightPlanner, with SITL already running on the right world
    tools/qgc_capture/qgc_shot.sh <abs-path-to.plan> <abs-path-out.png> [zoom_steps]

Default `zoom_steps` is 10, which frames a ~900 m survey at 150 m scale. It uploads the
plan to the vehicle, restarts QGC, dismisses the update dialog, scroll-zooms out, and
captures the window.

| file | what it does |
|---|---|
| `qgc_shot.sh`   | the pipeline |
| `upload_only.py`| upload a `.plan` to PX4 via MAVSDK **without** flying it (port 14540) |
| `frame_ini.py`  | rewrite `[FlightMapPosition]` in QGC's ini to frame a plan |
| `xdrive.py`     | click inside a window using *window* coordinates |
| `zoomout.py`    | scroll-wheel zoom out N steps |
| `xwd2png.py`    | XWD → PNG (there is no ImageMagick or ffmpeg on this box) |

## Gotchas, each learned the hard way

- **QGC's Fly view only draws the *vehicle's* mission**, never a `.plan` passed on the
  command line. So the plan must be uploaded to a running PX4 first, and QGC must be
  restarted afterwards — it syncs the mission at connect and does not re-pull later.
- **The window origin is not fixed.** QGC restores `MainWindowState x=209 y=130` from
  its ini, and `xwininfo` has also reported `+70+64` for the same window. Hardcode the
  offset and every synthetic click lands ~140 px off with no error. `xdrive.py` and
  `zoomout.py` resolve it at run time via `translate_coords`; keep it that way.
- **The "New Version Available" dialog appears intermittently and grabs input.** Any
  click or scroll after it is silently swallowed. Dismiss it first (harmless if absent).
- **`FlightMapPosition` is ignored while a vehicle is connected** — QGC forces both
  centre and zoom onto the vehicle. `frame_ini.py` is therefore only useful with no
  vehicle attached; with one, scroll-zoom after launch is the only lever that works.
- **QGC must be down before the ini is written**, because it rewrites the ini on exit.
- **The QGC binary ignores SIGTERM** and needs `kill -9`. The AppRun wrapper and the
  real binary under `/tmp/.mount_*/` are separate PIDs; both must go.
- **`xwd -root` fails with `BadColor`** on this display. Capture by window id instead.
- **Never `pgrep -f` a pattern that appears in your own command line** — `pgrep -f
  "gz sim"` matched the shell running it and killed the command mid-script. Use
  `pgrep -x`.
