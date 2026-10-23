#!/usr/bin/env python3
"""Bridge one serial flight controller onto UDP, optionally rewriting its system id.

    # pass-through: board on its own UDP port, sysid untouched
    python tools/mavlink_bridge.py --serial /dev/ttyACM0 --udp 127.0.0.1:14551

    # rewrite: board keeps MAV_SYS_ID 1 on the wire, appears as sysid 2 to the GCS
    python tools/mavlink_bridge.py --serial /dev/ttyACM1 --udp 127.0.0.1:14550 --as-sysid 2

Why this exists: three identical MicoAir boards all ship as MAV_SYS_ID 1, and
/dev/serial/by-id/ cannot tell them apart either (all report USB serial '0'). Two ways
out -- give each board a different id, or leave the boards alone and separate them
somewhere else. This tool lets both be measured without touching the hardware.

--as-sysid re-encodes every frame coming off the board with a new source system id, and
translates commands going the other way back to the board's real id, so a GCS can address
the rewritten id and the board still answers.

Stats are printed to stderr every 5 s so a test harness can watch progress; the bridge
runs until interrupted.
"""

from __future__ import annotations

import argparse
import fcntl
import socket
import sys
import termios
import threading
import time

import struct

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# This pymavlink (2.4.49) has no top-level pymavlink.mavcrc; the dialect re-exports it.
x25crc = mavlink2.x25crc

STATS_INTERVAL_S = 5.0


def patch_sysid(buf: bytes, new_sysid: int, crc_extra: int) -> bytes:
    """Rewrite the source system id in a raw frame and fix its checksum.

    Re-encoding through msg.pack() fails on any message pymavlink decoded generically
    ("MAVLink_message cannot be serialized directly"), and a bridge must pass everything
    the board emits. Editing the byte in place works for every message type.
    """
    b = bytearray(buf)
    if b[0] == 0xFD:              # MAVLink 2: magic len incompat compat seq SYSID compid ...
        b[5] = new_sysid
        end = 10 + b[1]
    elif b[0] == 0xFE:            # MAVLink 1: magic len seq SYSID compid msgid ...
        b[3] = new_sysid
        end = 6 + b[1]
    else:
        return buf
    crc = x25crc(b[1:end])
    # The C-accelerated x25crc exposes accumulate(); the pure-python one accumulate_str().
    if hasattr(crc, "accumulate_str"):
        crc.accumulate_str(struct.pack("B", crc_extra))
    else:
        crc.accumulate(bytes([crc_extra]))
    b[end:end + 2] = struct.pack("<H", crc.crc)
    return bytes(b)


class Bridge:
    def __init__(self, serial_dev: str, baud: int, udp_host: str, udp_port: int,
                 as_sysid: int | None, real_sysid: int | None):
        self.master = mavutil.mavlink_connection(serial_dev, baud=baud,
                                                 source_system=255, source_component=190)
        # Claim the tty exclusively. Without this, QGroundControl's serial autoconnect
        # opens the same device, both processes read half the byte stream each, and the
        # bridge dies with "device reports readiness to read but returned no data
        # (multiple access on port?)". TIOCEXCL makes any later open() fail with EBUSY
        # instead. One consumer per link, enforced rather than hoped for.
        try:
            fcntl.ioctl(self.master.port.fileno(), termios.TIOCEXCL)
        except (AttributeError, OSError) as exc:
            print(f"[bridge] warning: could not claim {serial_dev} exclusively: {exc}",
                  file=sys.stderr, flush=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.sock.settimeout(0.2)
        self.peer = (udp_host, udp_port)
        self.as_sysid = as_sysid
        self.real_sysid = real_sysid          # learned from the first heartbeat if unset
        self.dev = serial_dev
        self.up = self.down = self.rewritten = 0
        self.seen_sysids: set[int] = set()
        self.running = True
        # Encoder used only when rewriting; seq/CRC are recomputed on pack().
        self._out = mavlink2.MAVLink(None, srcSystem=as_sysid or 1, srcComponent=1)

    def _to_udp(self, msg) -> None:
        src = msg.get_srcSystem()
        self.seen_sysids.add(src)
        if self.real_sysid is None and msg.get_type() == "HEARTBEAT":
            self.real_sysid = src

        needs_rewrite = (self.as_sysid is not None
                         and src == (self.real_sysid or src)
                         and src != self.as_sysid)
        if not needs_rewrite:
            raw = msg.get_msgbuf()
        else:
            entry = mavlink2.mavlink_map.get(msg.get_msgId())
            raw = patch_sysid(msg.get_msgbuf(), self.as_sysid,
                              entry.crc_extra if entry else 0)
            self.rewritten += 1
        self.sock.sendto(raw, self.peer)
        self.up += 1

    def _from_udp(self, data: bytes) -> None:
        if self.as_sysid is None:
            self.master.write(data)
            self.down += 1
            return
        # Commands are addressed to the advertised id; the board only answers to its own.
        for msg in self.master.mav.parse_buffer(data) or []:
            tgt = getattr(msg, "target_system", None)
            if tgt == self.as_sysid and self.real_sysid is not None:
                msg.target_system = self.real_sysid
                self._out.srcComponent = msg.get_srcComponent()
                out = mavlink2.MAVLink(None, srcSystem=msg.get_srcSystem(),
                                       srcComponent=msg.get_srcComponent())
                self.master.write(msg.pack(out))
            else:
                self.master.write(msg.get_msgbuf())
            self.down += 1

    def _udp_reader(self) -> None:
        while self.running:
            try:
                data, _ = self.sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                self._from_udp(data)
            except Exception:
                pass

    def run(self) -> None:
        threading.Thread(target=self._udp_reader, daemon=True).start()
        label = f"{self.dev} -> {self.peer[0]}:{self.peer[1]}"
        if self.as_sysid:
            label += f" as sysid {self.as_sysid}"
        print(f"[bridge] {label}", file=sys.stderr, flush=True)

        next_stats = time.time() + STATS_INTERVAL_S
        while self.running:
            msg = self.master.recv_match(blocking=True, timeout=1)
            if msg is not None and msg.get_type() != "BAD_DATA":
                self._to_udp(msg)
            if time.time() >= next_stats:
                print(f"[bridge] {self.dev}: {self.up} up, {self.down} down, "
                      f"{self.rewritten} rewritten, board sysid={self.real_sysid}, "
                      f"seen={sorted(self.seen_sysids)}", file=sys.stderr, flush=True)
                next_stats = time.time() + STATS_INTERVAL_S


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--udp", required=True, help="host:port to forward to")
    ap.add_argument("--as-sysid", type=int, default=None,
                    help="advertise the board under this system id instead of its own")
    ap.add_argument("--real-sysid", type=int, default=None,
                    help="the board's actual id; learned from its first heartbeat if omitted")
    args = ap.parse_args()

    host, _, port = args.udp.partition(":")
    Bridge(args.serial, args.baud, host, int(port), args.as_sysid, args.real_sysid).run()


if __name__ == "__main__":
    main()
