#!/usr/bin/env python3
"""Pin each Jetson to exactly one ELRS backpack, and prove it stuck.

    python3 tools/elrs_pair.py --discover          # what is in range, by BSSID
    python3 tools/elrs_pair.py --verify            # who is attached to what, no changes
    ELRS_PSK=... python3 tools/elrs_pair.py        # apply the registry's pairing

Every ExpressLRS backpack broadcasts the same ssid, so a profile that matches by name will
attach to whichever board is loudest -- and it changes its mind between scans. On 28 Sep
2026 both Jetsons, configured against 02:70:07:61:73:D0, came back attached to
02:70:07:61:E4:CB with no error logged anywhere. Two aircraft on one transmitter is a
collision, so this tool treats a shared BSSID as a hard failure, not a warning.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
REGISTRY = ROOT / "artex-registry.yaml"
WIFI = "wlP1p1s0"
REMOTE = "~/FlightPlanner/tools/jetson_net.sh"


def ssh(host: str, cmd: str, timeout: int = 120) -> tuple[int, str]:
    p = subprocess.run(["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", host, cmd],
                       capture_output=True, text=True, timeout=timeout)
    return p.returncode, (p.stdout + p.stderr).strip()


def attached(host: str) -> str | None:
    """The BSSID this host is on, or None. An unreachable host is None, not its ssh error
    -- letting the error text through prints it in the address column and makes an
    offline board look like it is attached to something."""
    rc, out = ssh(host, f"iw dev {WIFI} link 2>/dev/null | awk '/Connected to/{{print $3}}'")
    out = out.strip().lower()
    if rc != 0 or not out or ":" not in out or len(out) != 17:
        return None
    return out


def scan(host: str) -> list[tuple[str, str, str]]:
    ssh(host, f"nmcli dev wifi rescan ifname {WIFI} 2>/dev/null; sleep 4")
    rc, out = ssh(host, f"nmcli -t -f SSID,BSSID,SIGNAL dev wifi list ifname {WIFI}")
    rows = []
    for line in out.splitlines():
        # nmcli escapes the colons inside a BSSID as \:
        parts = line.replace("\\:", "\x00").split(":")
        if len(parts) >= 3 and "expresslrs" in parts[0].lower():
            rows.append((parts[0], parts[1].replace("\x00", ":").upper(), parts[2]))
    return rows


def load() -> dict:
    return yaml.safe_load(REGISTRY.read_text())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--discover", action="store_true", help="list ELRS APs in range")
    ap.add_argument("--verify", action="store_true", help="check the pairing, change nothing")
    ap.add_argument("--registry", default=str(REGISTRY))
    args = ap.parse_args()

    reg = yaml.safe_load(Path(args.registry).read_text())
    packs, pairing = reg["backpacks"], reg["pairing"]
    known = {v["bssid"].upper(): k for k, v in packs.items() if v.get("bssid")}

    if args.discover:
        for host in pairing:
            print(f"=== {host} ===")
            for ssid, bssid, sig in scan(host):
                print(f"  {bssid}  signal {sig:>3}  {known.get(bssid, 'UNKNOWN -- new board')}")
        return 0

    if not args.verify:
        psk = os.environ.get("ELRS_PSK")
        pw = os.environ.get("JETSON_PW", "jetson")
        if not psk:
            print("set ELRS_PSK (the backpack wifi password)", file=sys.stderr)
            return 2
        for host, name in pairing.items():
            pack = packs.get(name)
            if not pack:
                print(f"[{host}] registry has no entry for {name}"); return 2
            if not pack.get("bssid"):
                # Do NOT leave this Jetson attached to somebody else's transmitter just
                # because its own is not powered. An idle radio is safe; a shared one is
                # two aircraft on one uplink.
                print(f"=== {host} -> {name}: no BSSID known, detaching from the radio ===")
                ssh(host, f"echo {pw} | sudo -S -p '' bash {REMOTE} --down", timeout=60)
                continue
            print(f"=== {host} -> {name} ({pack['bssid']}) ===")
            rc, out = ssh(host,
                          f"cd ~/FlightPlanner && echo {pw} | sudo -S -p '' "
                          f"env ELRS_BSSID={pack['bssid']} ELRS_PSK={psk} bash {REMOTE}",
                          timeout=240)
            for line in out.splitlines():
                if line.startswith("[jetson-net]") or "ERROR" in line or "BSSID" in line:
                    print("  " + line)

    # Verification is the point of the tool, so it runs either way.
    print("\n=== pairing ===")
    seen: dict[str, str] = {}
    bad = False
    for host, name in pairing.items():
        want = (packs.get(name) or {}).get("bssid")
        got = attached(host)
        want_l = want.lower() if want else None
        ok = bool(got) and got == want_l
        print(f"  {host:<10} want {name} {want or '(unknown)':<20} got {got or '(none)':<20} "
              f"{'OK' if ok else 'MISMATCH'}")
        if not ok:
            bad = True
        if got:
            if got in seen:
                print(f"  !! {host} and {seen[got]} are BOTH on {got} -- "
                      f"two aircraft, one transmitter")
                bad = True
            seen[got] = host
    print("\nexclusive pairing holds" if not bad else "\nPAIRING IS NOT SAFE TO FLY")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
