#!/usr/bin/env bash
# Put an Orin NX into the two-network role it holds during a live run.
#
#     ETH  -> the ground LAN. The workstation drives the agent over ssh on this.
#     WiFi -> the RadioMaster Pocket's ELRS backpack AP, 10.0.0.0/24.
#
# Run ON the Jetson. NetworkManager will not create a system profile for a non-console
# ssh session -- polkit refuses with "Insufficient privileges" -- so configuring needs root:
#     echo <pw> | sudo -S -p '' env ELRS_BSSID=02:70:07:61:E4:CB ELRS_PSK='...' \
#         bash tools/jetson_net.sh
#     tools/jetson_net.sh --status                # verify only, changes nothing
#     tools/jetson_net.sh --down                  # leave the backpack, keep the profile
#
# The PSK is read from the environment and never written to the repo -- only into
# NetworkManager's own store, root-readable, same as every other saved network.
#
# THE TRAP THIS SCRIPT EXISTS TO AVOID. The backpack AP serves DHCP *with a default
# route*: measured from the workstation 28 Sep 2026, `default via 10.0.0.1 dev wlp129s0
# proto dhcp metric 20600`. Accept it on the Jetson and the default route points at a
# radio that routes nowhere -- apt, DNS and NTP all stop. ssh from the ground LAN still
# works, because that is a connected route, so the box looks healthy right up until
# something needs the internet. `ipv4.never-default yes` keeps the 10.0.0.0/24 subnet
# route and discards the default, which is the only part of the lease we want.

set -uo pipefail

# ELRS_BSSID IS NOT OPTIONAL when more than one backpack can be powered at once.
# Every ExpressLRS backpack broadcasts the identical ssid "ExpressLRS TX Backpack 000000"
# -- the 000000 is an unset uid, not a serial -- so the name identifies nothing. Measured
# 28 Sep 2026: both Jetsons had been configured against 02:70:07:61:73:D0 and came back
# attached to 02:70:07:61:E4:CB, a different physical board, with jetson-2's lease moving
# .102 -> .100. Neither box reported an error; NetworkManager simply took the loudest AP.
# Two aircraft sharing one transmitter is the specific outcome this pin exists to prevent.
SSID="${ELRS_SSID:-ExpressLRS TX Backpack 000000}"
BSSID="${ELRS_BSSID:-}"
PROFILE="${ELRS_PROFILE:-elrs-backpack}"
BACKPACK_GW="${ELRS_GW:-10.0.0.1}"

die()  { echo "ERROR: $*" >&2; exit 1; }
note() { echo "[jetson-net] $*"; }

wifi_dev() { nmcli -t -f DEVICE,TYPE dev status | awk -F: '$2=="wifi"{print $1; exit}'; }
eth_dev()  { nmcli -t -f DEVICE,TYPE dev status | awk -F: '$2=="ethernet"{print $1; exit}'; }

status() {
    local w e
    w="$(wifi_dev)"; e="$(eth_dev)"
    echo "=== interfaces ==="
    nmcli -f DEVICE,TYPE,STATE,CONNECTION dev status | grep -Ev "^(lo|docker|veth)"
    echo
    echo "=== ground LAN (${e:-none}) ==="
    [ -n "$e" ] && ip -br addr show "$e" || echo "  no ethernet device"
    echo
    echo "=== backpack (${w:-none}) ==="
    if [ -n "$w" ]; then
        ip -br addr show "$w"
        echo "  attached BSSID: $(iw dev "$w" link 2>/dev/null | awk '/Connected to/{print $3}')"
        echo "  pinned  BSSID: $(nmcli -g 802-11-wireless.bssid con show "$PROFILE" 2>/dev/null)"
    else
        echo "  no wifi device"
    fi
    echo
    echo "=== default route ==="
    ip route show default || true
    local via
    via="$(ip route show default | awk '{print $3; exit}')"
    if [ "$via" = "$BACKPACK_GW" ]; then
        echo "  !! default route points at the backpack -- apt/DNS/NTP will fail."
        echo "     fix: nmcli con modify '$PROFILE' ipv4.never-default yes && \\"
        echo "          nmcli con up '$PROFILE'"
    else
        echo "  ok: default route is not the backpack"
    fi
    echo
    echo "=== backpack reachability ==="
    if ping -c 3 -W 2 "$BACKPACK_GW" 2>/dev/null | tail -2; then :; else
        echo "  $BACKPACK_GW unreachable"
    fi
}

case "${1:-}" in
    --status) status; exit 0 ;;
    --down)   nmcli con down "$PROFILE"; exit $? ;;
esac

command -v nmcli >/dev/null || die "nmcli not found"
WIFI="$(wifi_dev)"
[ -n "$WIFI" ] || die "no wifi device -- the Orin NX needs its M.2 WiFi card seated"

ETH="$(eth_dev)"
note "wifi device: $WIFI"
note "ground LAN device: ${ETH:-none}"

[ "$(id -u)" -eq 0 ] || die "must run as root -- nmcli cannot add a profile from an ssh session otherwise"

# The AP is 2.4 GHz only; a card parked on a 5 GHz-only band will never see it.
note "scanning for '$SSID' ..."
nmcli dev wifi rescan ifname "$WIFI" 2>/dev/null
sleep 3
if [ -n "$BSSID" ]; then
    # nmcli -t escapes the colons inside a BSSID as "\:", so the raw terse output never
    # matches a plain MAC. Strip the backslashes before comparing.
    if ! nmcli -t -f BSSID dev wifi list ifname "$WIFI" | tr -d '\\' | grep -qiF "$BSSID"; then
        echo "visible ELRS APs:" >&2
        nmcli -t -f SSID,BSSID,SIGNAL dev wifi list ifname "$WIFI" | grep -i expresslrs >&2
        die "backpack $BSSID is not in range -- power it, or correct the registry"
    fi
    note "backpack $BSSID is visible"
else
    if ! nmcli -t -f SSID dev wifi list ifname "$WIFI" | grep -qxF "$SSID"; then
        die "'$SSID' not visible. Put the backpack into WiFi mode and retry."
    fi
    note "AP is visible (UNPINNED -- will attach to whichever board is loudest)"
fi

if nmcli -t -f NAME con show | grep -qxF "$PROFILE"; then
    note "profile '$PROFILE' exists; updating it"
else
    [ -n "${ELRS_PSK:-}" ] || die "set ELRS_PSK to the backpack password"
    nmcli con add type wifi ifname "$WIFI" con-name "$PROFILE" ssid "$SSID" \
        >/dev/null || die "could not create the profile"
fi

nmcli con modify "$PROFILE" \
    connection.interface-name "$WIFI" \
    connection.autoconnect yes \
    connection.autoconnect-priority -10 \
    802-11-wireless-security.key-mgmt wpa-psk \
    ipv4.method auto \
    ipv4.never-default yes \
    ipv4.route-metric 800 \
    ipv6.method ignore \
    || die "could not configure the profile"

# The pin itself. Without it the profile matches by ssid alone and roams between boards.
if [ -n "$BSSID" ]; then
    nmcli con modify "$PROFILE" 802-11-wireless.bssid "$BSSID" \
        || die "could not pin the profile to $BSSID"
    note "pinned to $BSSID"
else
    nmcli con modify "$PROFILE" 802-11-wireless.bssid "" || true
fi

# Only set the secret if one was supplied, so a re-run without ELRS_PSK is harmless.
[ -n "${ELRS_PSK:-}" ] && nmcli con modify "$PROFILE" 802-11-wireless-security.psk "$ELRS_PSK"

note "bringing up '$PROFILE'"
nmcli con up "$PROFILE" >/dev/null || die "could not join '$SSID' -- wrong PSK?"
sleep 3

echo
status
