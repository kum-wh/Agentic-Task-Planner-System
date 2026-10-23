"""The guard must send the right MAVLink command for the rung it has reached.

safety_rules decides *what* should happen; these tests pin *what the agent actually sends*,
with a stub connection that records commands instead of an aircraft.
"""

import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import drone_agent as agent  # noqa: E402
import safety_rules as sr  # noqa: E402

ARM_DISARM = 400          # MAV_CMD_COMPONENT_ARM_DISARM
NAV_LAND = 21             # MAV_CMD_NAV_LAND
DO_SET_MODE = 176         # MAV_CMD_DO_SET_MODE


class FakeMav:
    def __init__(self):
        self.sent = []

    def command_long_send(self, target, comp, command, confirm, *params):
        self.sent.append((command, list(params)))


class FakeConn:
    def __init__(self):
        self.target_system = 1
        self.mav = FakeMav()


def _msg(kind, **fields):
    m = types.SimpleNamespace(**fields)
    m.get_type = lambda: kind
    m.get_srcSystem = lambda: fields.get("_src", 1)
    return m


def _guard(tmp_path, **over):
    args = types.SimpleNamespace(slot=1, no_safety=False, allow_kill=False,
                                 abort_flag=str(tmp_path / "ABORT"),
                                 hold_rate=None, land_rate=None,
                                 battery_land_pct=None, tilt_land_deg=None)
    for k, v in over.items():
        setattr(args, k, v)
    conn = FakeConn()
    return agent.Guard(conn, args), conn


def _pile_on_errors(g, n=8, severity=2):
    for i in range(n):
        g.observe(_msg("STATUSTEXT", severity=severity, text=f"EKF fault {i}"))


def _on_ground(g):
    g.observe(_msg("EXTENDED_SYS_STATE", landed_state=1))


def _in_air(g):
    g.observe(_msg("EXTENDED_SYS_STATE", landed_state=2))


def _commands(conn):
    return [c for c, _ in conn.mav.sent]


# ---- the ladder, carried out on the wire ---------------------------------------------

def test_a_quiet_aircraft_is_left_alone(tmp_path):
    g, conn = _guard(tmp_path)
    g.observe(_msg("STATUSTEXT", severity=6, text="Ready for takeoff"))
    assert g.check() is None
    assert conn.mav.sent == []


def test_errors_piling_up_in_flight_command_a_landing(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    _pile_on_errors(g)
    ev = g.check()
    assert ev["action"] == sr.LAND and ev["carried_out"] == sr.LAND
    assert NAV_LAND in _commands(conn)
    assert ARM_DISARM not in _commands(conn)       # never cut motors in the air
    assert g.stop_streaming is True                # offboard must not fight AUTO.LAND


def test_errors_piling_up_on_the_ground_cut_the_motors(tmp_path):
    g, conn = _guard(tmp_path)
    _on_ground(g)
    _pile_on_errors(g)
    ev = g.check()
    assert ev["carried_out"] == sr.DISARM
    cmd, params = conn.mav.sent[0]
    assert cmd == ARM_DISARM
    assert params[0] == 0                          # 0 = disarm
    assert params[1] == 0                          # not the force-disarm magic


def test_a_middling_error_rate_only_holds(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    _pile_on_errors(g, n=3)                        # 3 x CRITICAL(4) = 12/min -> HOLD
    ev = g.check()
    assert ev["carried_out"] == sr.HOLD
    cmd, params = conn.mav.sent[0]
    assert cmd == DO_SET_MODE
    assert params[:3] == [1, 4, 3]                 # custom mode, AUTO, LOITER
    assert g.stop_streaming is False               # still flying the mission


def test_it_escalates_from_hold_to_land_as_things_get_worse(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    _pile_on_errors(g, n=3)
    assert g.check()["carried_out"] == sr.HOLD
    _pile_on_errors(g, n=5)
    assert g.check()["carried_out"] == sr.LAND
    assert NAV_LAND in _commands(conn)


def test_the_same_rung_is_not_commanded_twice(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    _pile_on_errors(g)
    g.check()
    before = len(conn.mav.sent)
    g.check()
    g.check()
    assert len(conn.mav.sent) == before


# ---- finishing the landing ------------------------------------------------------------

def test_once_it_is_down_the_motors_are_cut(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    _pile_on_errors(g)
    g.check()
    g.observe(_msg("HEARTBEAT", base_mode=128, type=2, custom_mode=0))   # armed
    assert ARM_DISARM not in _commands(conn)
    _on_ground(g)                                   # touchdown
    g._last_disarm = 0.0
    g.check()
    assert ARM_DISARM in _commands(conn)


def test_a_disarmed_aircraft_is_not_disarmed_again(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    _pile_on_errors(g)
    g.check()
    g.observe(_msg("HEARTBEAT", base_mode=0, type=2, custom_mode=0))     # disarmed
    _on_ground(g)
    g._last_disarm = 0.0
    g.check()
    assert ARM_DISARM not in _commands(conn)


# ---- the operator's words -------------------------------------------------------------

def test_operator_abort_lands(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    assert g.operator("ABORT") is True
    assert g.check()["carried_out"] == sr.LAND


def test_operator_disarm_in_the_air_is_downgraded(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    g.operator("DISARM")
    ev = g.check()
    assert ev["action"] == sr.DISARM and ev["carried_out"] == sr.LAND
    assert ARM_DISARM not in _commands(conn)


def test_nonsense_from_the_operator_is_refused(tmp_path):
    g, _ = _guard(tmp_path)
    assert g.operator("TAKEOFF") is False
    assert g.check() is None


def test_kill_needs_permission(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    assert g.operator("KILL") is False
    assert conn.mav.sent == []
    assert g.killed is False


def test_kill_force_disarms_when_permitted(tmp_path):
    g, conn = _guard(tmp_path, allow_kill=True)
    _in_air(g)
    assert g.operator("KILL") is True
    cmd, params = conn.mav.sent[0]
    assert cmd == ARM_DISARM
    assert params[0] == 0 and params[1] == agent.FORCE_DISARM_MAGIC
    assert g.killed is True


# ---- the out-of-band flag -------------------------------------------------------------

def test_the_flag_file_lands_the_aircraft(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    (tmp_path / "ABORT").write_text("land\n")
    assert g.check()["carried_out"] == sr.LAND


def test_the_flag_file_can_name_a_lesser_rung(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    (tmp_path / "ABORT").write_text("hold\n")
    assert g.check()["carried_out"] == sr.HOLD


def test_an_unreadable_flag_still_aborts(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    (tmp_path / "ABORT").write_text("gibberish")
    assert g.check()["carried_out"] == sr.LAND      # default to the safe reading


def test_a_flag_left_over_from_last_time_refuses_the_flight(tmp_path):
    (tmp_path / "ABORT").write_text("land\n")
    g, conn = _guard(tmp_path)
    _on_ground(g)
    ev = g.check()
    assert ev["carried_out"] == sr.DISARM
    assert "left over from before this run" in " ".join(ev["reasons"])


# ---- the off switch -------------------------------------------------------------------

def test_no_safety_disables_the_whole_ladder(tmp_path):
    g, conn = _guard(tmp_path, no_safety=True)
    _in_air(g)
    _pile_on_errors(g, n=20)
    assert g.check() is None
    assert conn.mav.sent == []


# ---- the RC switch: a human taking the aircraft ---------------------------------------

def _mode_hb(main_mode, armed=True):
    return _msg("HEARTBEAT", base_mode=128 if armed else 0, type=2,
                custom_mode=main_mode << 16)


def test_sitting_in_a_manual_mode_before_we_start_is_not_a_takeover(tmp_path):
    g, conn = _guard(tmp_path)
    g.observe(_mode_hb(3))              # POSCTL on the bench, before we ask for OFFBOARD
    g.observe(_mode_hb(3))
    assert g.pilot_has_it is False
    g.observe(_mode_hb(6))              # now OFFBOARD -- ours
    assert g.pilot_has_it is False


def test_leaving_offboard_for_a_hand_flown_mode_is_a_takeover(tmp_path):
    g, conn = _guard(tmp_path)
    g.observe(_mode_hb(6))
    g.observe(_mode_hb(2))              # ALTCTL: the switch was thrown
    assert g.pilot_has_it is True
    assert g.stop_streaming is True


def test_leaving_auto_for_a_hand_flown_mode_is_a_takeover(tmp_path):
    g, conn = _guard(tmp_path)
    g.observe(_mode_hb(4))              # flying the mission
    g.observe(_mode_hb(3))              # pilot grabs it in POSCTL
    assert g.pilot_has_it is True


def test_our_own_loiter_is_not_a_takeover(tmp_path):
    g, conn = _guard(tmp_path)
    g.observe(_mode_hb(6))
    g.observe(_mode_hb(4))              # the guard's own HOLD -> AUTO.LOITER
    assert g.pilot_has_it is False


def test_once_the_pilot_has_it_the_guard_commands_nothing(tmp_path):
    g, conn = _guard(tmp_path)
    _in_air(g)
    g.observe(_mode_hb(6))
    g.observe(_mode_hb(2))              # taken over
    _pile_on_errors(g, n=20)            # and then everything goes wrong
    assert g.check() is None
    assert conn.mav.sent == []          # hands off: no LAND, no HOLD, no disarm


def test_a_takeover_latches_even_if_the_mode_comes_back(tmp_path):
    g, conn = _guard(tmp_path)
    g.observe(_mode_hb(6))
    g.observe(_mode_hb(2))
    g.observe(_mode_hb(6))              # offboard again -- we still do not take it back
    assert g.pilot_has_it is True
