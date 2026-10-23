#!/usr/bin/env bash
# Launch N PX4 SITL instances into ONE Gazebo world (multi-vehicle SITL).
#
#     tools/sitl_swarm.sh start toapayoh 3        # 3 x500 over Toa Payoh
#     tools/sitl_swarm.sh status
#     tools/sitl_swarm.sh stop
#
# Environment:
#   HEADLESS=1   (default) no 3D GUI. Set HEADLESS= (empty) to watch it.
#   SPACING=5    metres between spawn points along the world Y axis.
#   MODEL=gz_x500
#
# How PX4 derives everything from the instance index (read out of
# ROMFS/px4fmu_common/init.d-posix/rcS and px4-rc.mavlink, v1.16.0):
#
#   instance i  ->  MAV_SYS_ID      = i + 1
#               ->  working dir     = build/px4_sitl_default/rootfs/<i>   (own dataman)
#               ->  GCS  UDP local  = 18570 + i
#               ->  MAVSDK/API port = 14540 + i        ("udpin://:1454<i>")
#               ->  gz model name   = <model>_<i>      e.g. x500_0
#
# The FIRST instance boots the gz server; the rest detect the running world and
# attach to it. That ordering is not optional -- two instances racing to start
# `gz sim -s` leaves you with a half-built world -- so this script waits for the
# world's /clock topic before starting instance 1.
#
# Golden rules honoured here: px4 runs with -d (without it the pxh> prompt
# redraws into a non-TTY forever and the log grows GB/min), and stop kills by
# numeric PID, never by a pkill pattern that would match this script itself.

set -euo pipefail

DRONE_HOME="${DRONE_HOME:-$HOME/drone-dev}"
PX4_DIR="$DRONE_HOME/PX4-Autopilot"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
RUN_DIR="$BUILD_DIR/rootfs/swarm"
PID_FILE="$RUN_DIR/pids"

MODEL="${MODEL:-gz_x500}"
SPACING="${SPACING:-5}"

die() { echo "ERROR: $*" >&2; exit 1; }

world_up() {
    gz topic -l 2>/dev/null | grep -qE "^/world/$1/clock$"
}

cmd_start() {
    local world="${1:-toapayoh}" count="${2:-3}"

    [ -x "$BUILD_DIR/bin/px4" ] || die "no px4 binary at $BUILD_DIR/bin/px4 -- build SITL first"
    [ -f "$PX4_DIR/Tools/simulation/gz/worlds/$world.sdf" ] || die "no world $world.sdf"
    [ -f "$BUILD_DIR/rootfs/gz_env.sh" ] || die "no rootfs/gz_env.sh -- gz build missing"

    if [ -s "$PID_FILE" ] && kill -0 "$(head -1 "$PID_FILE")" 2>/dev/null; then
        die "a swarm is already running (see '$0 status'); stop it first"
    fi

    mkdir -p "$RUN_DIR"
    : > "$PID_FILE"

    echo "world   : $world"
    echo "vehicles: $count x $MODEL, ${SPACING} m apart"
    # Note the '-' not ':-' below and in the launch: HEADLESS= (set but empty) is
    # how you ask for the GUI, so an unset variable must still default to 1.
    echo "headless: ${HEADLESS-1}${HEADLESS+ }$([ -z "${HEADLESS-1}" ] && echo '(GUI will open)')"
    echo

    for i in $(seq 0 $((count - 1))); do
        # Each instance gets its own rootfs/<i>, so each has its own dataman and
        # a stale mission from one vehicle cannot leak into another.
        mkdir -p "$BUILD_DIR/rootfs/$i"

        local pose="0,$((i * SPACING)),0"
        local log="$RUN_DIR/sitl-$i.log"

        (
            cd "$BUILD_DIR"
            PX4_SIM_MODEL="$MODEL" \
            PX4_GZ_WORLD="$world" \
            PX4_GZ_MODEL_POSE="$pose" \
            GZ_IP=127.0.0.1 \
            HEADLESS="${HEADLESS-1}" \
            ./bin/px4 -i "$i" -d < /dev/null > "$log" 2>&1 &
            echo $! >> "$PID_FILE"
        )

        local pid
        pid=$(tail -1 "$PID_FILE")
        printf "  instance %d  pid %-7s sysid %d  api udpin://:%d  gcs %d  model %s_%d  pose %s\n" \
            "$i" "$pid" "$((i + 1))" "$((14540 + i))" "$((18570 + i))" "${MODEL#gz_}" "$i" "$pose"

        if [ "$i" -eq 0 ]; then
            printf "  waiting for gz world to come up "
            local waited=0
            until world_up "$world"; do
                sleep 2; waited=$((waited + 2)); printf "."
                [ "$waited" -ge 90 ] && { echo; die "world $world never appeared -- see $log"; }
                kill -0 "$pid" 2>/dev/null || { echo; die "instance 0 died -- see $log"; }
            done
            echo " up after ${waited}s"
        else
            sleep 3   # let each attach and spawn its model before the next
        fi
    done

    echo
    echo "logs: $RUN_DIR/sitl-<i>.log      stop: $0 stop"
}

cmd_status() {
    [ -s "$PID_FILE" ] || { echo "no swarm recorded"; return 0; }
    local i=0
    while read -r pid; do
        if kill -0 "$pid" 2>/dev/null; then
            printf "  instance %d  pid %-7s RUNNING  sysid %d  api udpin://:%d\n" \
                "$i" "$pid" "$((i + 1))" "$((14540 + i))"
        else
            printf "  instance %d  pid %-7s gone\n" "$i" "$pid"
        fi
        i=$((i + 1))
    done < "$PID_FILE"
    echo
    echo "  gz models: $(gz model --list 2>/dev/null | grep -c "${MODEL#gz_}_" || echo 0)"
}

cmd_stop() {
    [ -s "$PID_FILE" ] || { echo "no swarm recorded"; return 0; }
    # Numeric PIDs only. A pkill pattern here would match this script's own
    # command line and kill the shell running it.
    while read -r pid; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "  TERM $pid"; kill "$pid" 2>/dev/null || true
        fi
    done < "$PID_FILE"
    sleep 3
    while read -r pid; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "  KILL $pid"; kill -9 "$pid" 2>/dev/null || true
        fi
    done < "$PID_FILE"
    # The gz server is a child of instance 0 and does not always follow it down.
    for gzpid in $(pgrep -x "ruby" 2>/dev/null || true) $(pgrep -x "gz" 2>/dev/null || true); do
        if tr '\0' ' ' < "/proc/$gzpid/cmdline" 2>/dev/null | grep -q "sim"; then
            echo "  KILL gz $gzpid"; kill -9 "$gzpid" 2>/dev/null || true
        fi
    done
    : > "$PID_FILE"
    echo "stopped"
}

case "${1:-start}" in
    start)  shift || true; cmd_start "$@" ;;
    status) cmd_status ;;
    stop)   cmd_stop ;;
    *)      die "usage: $0 {start [world] [count] | status | stop}" ;;
esac
