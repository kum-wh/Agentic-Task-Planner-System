#!/usr/bin/env bash
# Make the ELRS relay a boot-time service on an Orin NX, so the link is up before
# anyone asks for it.
#
# Run ON the Jetson, as root (writing to /etc/systemd/system needs it):
#     sudo bash tools/elrs_service.sh              # install, enable, start
#     tools/elrs_service.sh --status               # verify only, changes nothing
#     sudo bash tools/elrs_service.sh --remove     # stop, disable, delete the unit
#
# WHY THIS EXISTS. tools/elrs_link.py was started by hand in a login shell, so every
# Jetson reboot silently left the aircraft unreachable: the workstation's ssh still works
# (that is the ground LAN), drone_agent.py connects to udpout://127.0.0.1:14540 and simply
# never sees a heartbeat. The failure looks like a dead aircraft, not a missing process.
#
# The relay binds only unprivileged UDP ports (14550 from the module, 14540 for the
# agent), so it runs as the login user, not root. It does NOT need the backpack to be
# associated in order to start -- it binds, then waits -- so the unit deliberately does
# not depend on the WiFi profile being up. Restart=always covers the backpack blinking.
#
# ONE RELAY AT A TIME. The relay forwards downlink to the last address it heard from, so
# two of them on 14540 (both bind: SO_REUSEADDR) split the vehicle's replies between them
# and uploads stall with no error. Installing therefore stops any hand-started relay
# first -- by numeric PID, never `pkill -f`, which would match this script itself.

set -uo pipefail

UNIT=/etc/systemd/system/elrs-link.service
SERVICE=elrs-link.service
AGENT_PORT="${ELRS_AGENT_PORT:-14540}"

note() { echo "[elrs-service] $*"; }
die()  { echo "ERROR: $*" >&2; exit 1; }

status() {
    note "enabled: $(systemctl is-enabled "$SERVICE" 2>/dev/null || echo no)"
    note "active:  $(systemctl is-active  "$SERVICE" 2>/dev/null || echo no)"
    systemctl --no-pager --lines=5 status "$SERVICE" 2>/dev/null | tail -n 8
    echo
    note "relay processes:"
    pgrep -af "tools/elrs_link\.py" || echo "  (none)"
    echo
    note "backpack reachability:"
    ping -c3 -W1 10.0.0.1 2>/dev/null | tail -2 || echo "  10.0.0.1 unreachable"
}

case "${1:-}" in
    --status) status; exit 0 ;;
    --remove)
        [ "$(id -u)" -eq 0 ] || die "--remove needs root"
        systemctl disable --now "$SERVICE" 2>/dev/null
        rm -f "$UNIT"
        systemctl daemon-reload
        note "removed $UNIT"
        exit 0 ;;
    "") : ;;
    *) die "unknown argument '$1' (expected --status or --remove)" ;;
esac

[ "$(id -u)" -eq 0 ] || die "installing needs root: sudo bash tools/elrs_service.sh"

# The unit must run as the human user that owns the checkout, not as root.
RUN_USER="${SUDO_USER:-}"
[ -n "$RUN_USER" ] || die "cannot tell which user to run as -- set SUDO_USER"
RUN_HOME=$(getent passwd "$RUN_USER" | cut -d: -f6)
[ -n "$RUN_HOME" ] || die "no home directory for $RUN_USER"
WORKDIR="$RUN_HOME/FlightPlanner"
[ -f "$WORKDIR/tools/elrs_link.py" ] || die "no relay at $WORKDIR/tools/elrs_link.py"

PY=$(command -v python3) || die "python3 not found"

# Stop a hand-started relay before the unit claims the ports. Our own PIDs are excluded
# so this can never kill the shell running the script.
for pid in $(pgrep -f "tools/elrs_link\.py" || true); do
    [ "$pid" = "$$" ] && continue
    [ "$pid" = "${PPID:-0}" ] && continue
    note "stopping hand-started relay pid $pid"
    kill "$pid" 2>/dev/null
done

cat > "$UNIT" <<UNITEOF
[Unit]
Description=ELRS backpack <-> MAVLink agent relay (FlightPlanner)
Documentation=file://$WORKDIR/tools/elrs_link.py
After=network.target
Wants=network-online.target

[Service]
Type=simple
User=$RUN_USER
WorkingDirectory=$WORKDIR
ExecStart=$PY $WORKDIR/tools/elrs_link.py --agent-port $AGENT_PORT --report-s 15
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNITEOF

note "wrote $UNIT (user $RUN_USER, agent port $AGENT_PORT)"
systemctl daemon-reload
systemctl enable --now "$SERVICE" >/dev/null || die "could not enable $SERVICE"
sleep 2
echo
status
