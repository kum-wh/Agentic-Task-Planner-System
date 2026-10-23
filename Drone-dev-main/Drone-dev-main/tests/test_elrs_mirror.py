"""The relay must feed every agent AND the QGC mirror, and obey the uplink policy.

Real sockets on loopback, with a fake module standing in for the backpack. Two policies are
pinned here, and which one runs is the operator's call:

  default           QGC sees the vehicle and cannot command it -- safe, but every QGC
                    request (mission, geofence, rally points, parameters) fails
  --mirror-control  QGC can command it, alongside the agent. Chosen 7 Oct 2026 with the
                    two-commander risk accepted deliberately.
"""

import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest

RELAY = Path(__file__).resolve().parent.parent / "tools" / "elrs_link.py"


def _free_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _sock(port=0, timeout=2.0):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.settimeout(timeout)
    return s


def _start_relay(mirror_control: bool):
    """A relay whose 'module' is a loopback socket we control."""
    module_listen = _free_port()        # the relay sends uplink here
    module_send = _free_port()          # the relay binds this for downlink
    agent_port = _free_port()
    mirror_port = _free_port()

    module = _sock(module_listen)       # stands in for the backpack's ear
    mirror = _sock(mirror_port)         # stands in for QGroundControl

    argv = [sys.executable, str(RELAY),
            "--module-ip", "127.0.0.1",
            "--module-listen", str(module_listen),
            "--module-send", str(module_send),
            "--agent-port", str(agent_port),
            "--mirror", f"127.0.0.1:{mirror_port}",
            "--report-s", "3600"]
    if mirror_control:
        argv.append("--mirror-control")
    proc = subprocess.Popen(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True)
    time.sleep(1.5)                     # let it bind
    try:
        yield dict(proc=proc, module=module, mirror=mirror, agent_port=agent_port,
                   module_send=module_send, mirror_port=mirror_port)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        module.close()
        mirror.close()


@pytest.fixture
def relay():
    yield from _start_relay(mirror_control=False)


@pytest.fixture
def relay_rw():
    yield from _start_relay(mirror_control=True)


def _vehicle_says(relay, payload: bytes):
    """The backpack emitting a downlink packet toward the relay."""
    v = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    v.sendto(payload, ("127.0.0.1", relay["module_send"]))
    v.close()


def test_uplink_from_an_agent_reaches_the_module(relay):
    a = _sock()
    a.sendto(b"from-agent", ("127.0.0.1", relay["agent_port"]))
    data, _ = relay["module"].recvfrom(4096)
    assert data == b"from-agent"


def test_downlink_reaches_the_agent(relay):
    a = _sock()
    a.sendto(b"hello", ("127.0.0.1", relay["agent_port"]))      # so the relay knows a
    relay["module"].recvfrom(4096)
    _vehicle_says(relay, b"telemetry")
    data, _ = a.recvfrom(4096)
    assert data == b"telemetry"


def test_every_agent_gets_the_downlink_not_just_the_last(relay):
    a1, a2 = _sock(), _sock()
    for a in (a1, a2):
        a.sendto(b"hello", ("127.0.0.1", relay["agent_port"]))
        relay["module"].recvfrom(4096)
    _vehicle_says(relay, b"telemetry")
    assert a1.recvfrom(4096)[0] == b"telemetry"      # the OLD behaviour starved this one
    assert a2.recvfrom(4096)[0] == b"telemetry"


def test_the_mirror_sees_the_downlink(relay):
    a = _sock()
    a.sendto(b"hello", ("127.0.0.1", relay["agent_port"]))
    relay["module"].recvfrom(4096)
    _vehicle_says(relay, b"telemetry")
    assert relay["mirror"].recvfrom(4096)[0] == b"telemetry"


def test_the_mirror_sees_downlink_even_with_no_agent_attached(relay):
    _vehicle_says(relay, b"telemetry")
    assert relay["mirror"].recvfrom(4096)[0] == b"telemetry"


def test_nothing_the_mirror_sends_reaches_the_aircraft(relay):
    """QGroundControl talks back the instant it sees a vehicle. None of it may be forwarded."""
    a = _sock()
    a.sendto(b"hello", ("127.0.0.1", relay["agent_port"]))
    relay["module"].recvfrom(4096)                   # drain the agent's own uplink
    _vehicle_says(relay, b"telemetry")
    _, relay_mirror_src = relay["mirror"].recvfrom(4096)

    # QGC replying straight back to the port the mirror copy came from
    relay["mirror"].sendto(b"QGC-ARM-COMMAND", relay_mirror_src)
    relay["module"].settimeout(1.5)
    with pytest.raises(socket.timeout):
        relay["module"].recvfrom(4096)               # must never reach the module


def test_the_relay_survives_a_mirror_that_is_not_listening(relay):
    """QGC closed mid-flight must not take the aircraft link down with it."""
    relay["mirror"].close()
    a = _sock()
    a.sendto(b"hello", ("127.0.0.1", relay["agent_port"]))
    relay["module"].recvfrom(4096)
    for _ in range(5):
        _vehicle_says(relay, b"telemetry")
        time.sleep(0.1)
    assert a.recvfrom(4096)[0] == b"telemetry"       # agent unaffected
    assert relay["proc"].poll() is None              # and the relay is still alive


# ---- two-way: the operator has accepted a second commander on the airframe -----------

def test_with_mirror_control_the_station_can_command_the_aircraft(relay_rw):
    """Chosen 7 Oct 2026: QGC must be able to send, or its every request fails."""
    _vehicle_says(relay_rw, b"telemetry")
    _, back_to = relay_rw["mirror"].recvfrom(4096)   # learn the relay's mirror socket
    relay_rw["mirror"].sendto(b"QGC-REQUEST-PARAMS", back_to)
    data, _ = relay_rw["module"].recvfrom(4096)
    assert data == b"QGC-REQUEST-PARAMS"


def test_two_way_still_delivers_downlink_to_the_agent(relay_rw):
    a = _sock()
    a.sendto(b"hello", ("127.0.0.1", relay_rw["agent_port"]))
    relay_rw["module"].recvfrom(4096)
    _vehicle_says(relay_rw, b"telemetry")
    assert a.recvfrom(4096)[0] == b"telemetry"


def test_a_bad_mirror_spec_is_refused_not_ignored():
    r = subprocess.run([sys.executable, str(RELAY), "--mirror", "nonsense"],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 2
    assert "bad --mirror" in r.stdout + r.stderr
