#!/bin/bash
# qgc_shot.sh <abs-plan> <abs-out.png>
# Upload, relaunch QGC, scroll-zoom out, capture.
set -e
PLAN="$1"; OUT="$2"
SP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /home/kumwh/drone-dev/venv/bin/activate

echo "[1/6] upload $(basename "$PLAN")"
timeout 90 python $SP/upload_only.py "$PLAN" /home/kumwh/drone-dev/sar_turn_params.params 2>&1 | tail -1

# NB: match on exact process names only. A -f pattern would also match this script's
# own command line and kill the shell running it.
echo "[2/6] stop QGC (it rewrites the ini on exit)"
for P in $(pgrep -x QGroundControl || true) $(pgrep -x QGroundControl.AppImage || true); do
  kill -9 "$P" 2>/dev/null || true
done
sleep 3

echo "[3/6] frame"
python $SP/frame_ini.py "$PLAN"

echo "[4/6] launch"
(DISPLAY=:1 nohup ./QGroundControl.AppImage "$PLAN" >/dev/null 2>&1 &)
sleep 36
WID=$(DISPLAY=:1 xwininfo -root -tree 2>/dev/null | grep -m1 '"QGroundControl": ("QGroundControl"' | awk '{print $1}')

# The "New Version Available" dialog appears intermittently and grabs input, so any
# zoom that follows it is silently swallowed. Dismiss it first; harmless if absent.
echo "[5/6] dismiss dialog + zoom out to fit the survey"
python $SP/xdrive.py "$WID" "click,1181,483" || true
python $SP/zoomout.py "$WID" "${3:-10}"
sleep 2

echo "[6/6] capture -> $(basename "$OUT")"
DISPLAY=:1 xwd -id "$WID" -out $SP/shot.xwd
python $SP/xwd2png.py $SP/shot.xwd "$OUT"
